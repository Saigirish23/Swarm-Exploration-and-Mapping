"""Main frontier exploration node for a single robot."""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import SaveMap
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import rclpy.duration

from .frontier_detector import detect_frontiers, create_frontier_markers
from .goal_selector import score_frontiers, select_best_frontier
from .map_utils import (
    count_cell_types,
    get_grid_array,
    is_free,
    is_occupied,
    world_to_grid,
)
from .nav2_goal_client import Nav2GoalClient


class RobotExplorer(Node):
    """Frontier-based exploration node."""

    def __init__(self):
        super().__init__('robot_explorer')

        # Parameters
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('map_topic', '/robot1/map')
        self.declare_parameter('base_frame', 'robot1/base_link')
        self.declare_parameter('map_frame', 'robot1/map')
        self.declare_parameter('goal_action', '/robot1/navigate_to_pose')
        self.declare_parameter('goal_pose_topic', '')
        self.declare_parameter('claim_topic', '/swarm/claim_requests')
        self.declare_parameter('claimed_frontiers_topic', '/swarm/claimed_frontiers')
        self.declare_parameter('claim_frame', 'map')
        self.declare_parameter('claim_radius', 0.6)
        self.declare_parameter('exploration_status_topic', '')
        self.declare_parameter('exploration_done_topic', '/swarm/exploration_done')
        self.declare_parameter('frontier_marker_topic', '/robot1/frontier_markers')
        self.declare_parameter('exploration_period', 5.0)
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('distance_weight', 1.0)
        self.declare_parameter('startup_delay_sec', 3.0)
        self.declare_parameter('min_free_cells', 200)
        self.declare_parameter('min_unknown_cells', 0)
        self.declare_parameter('no_frontier_patience', 5)
        self.declare_parameter('goal_clearance_cells', 2)
        self.declare_parameter('goal_unknown_clearance_cells', 0)
        self.declare_parameter('frontier_out_of_bounds_as_unknown', True)
        self.declare_parameter('save_map_on_complete', False)
        self.declare_parameter('save_map_service', '/robot1/map_saver/save_map')
        self.declare_parameter('save_map_topic', '/merged_map')
        self.declare_parameter('save_map_url', 'merged_map')
        self.declare_parameter('save_map_image_format', 'png')
        self.declare_parameter('save_map_mode', 'trinary')
        self.declare_parameter('save_map_free_thresh', 0.25)
        self.declare_parameter('save_map_occupied_thresh', 0.65)
        # World boundary parameters to reject out-of-world frontier goals
        self.declare_parameter('world_min_x', -5.0)
        self.declare_parameter('world_max_x', 5.0)
        self.declare_parameter('world_min_y', -5.0)
        self.declare_parameter('world_max_y', 5.0)

        self.robot_name = self.get_parameter('robot_name').value
        self.map_topic = self.get_parameter('map_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.goal_action = self.get_parameter('goal_action').value
        self.goal_pose_topic = self.get_parameter('goal_pose_topic').value
        if not self.goal_pose_topic:
            self.goal_pose_topic = f'/{self.robot_name}/goal_pose'
        self.claim_topic = self.get_parameter('claim_topic').value
        self.claimed_frontiers_topic = self.get_parameter('claimed_frontiers_topic').value
        self.claim_frame = self.get_parameter('claim_frame').value
        self.claim_radius = self.get_parameter('claim_radius').value
        self.exploration_status_topic = self.get_parameter('exploration_status_topic').value
        if not self.exploration_status_topic:
            self.exploration_status_topic = f'/{self.robot_name}/exploration_status'
        self.exploration_done_topic = self.get_parameter('exploration_done_topic').value
        self.frontier_marker_topic = self.get_parameter('frontier_marker_topic').value
        self.exploration_period = self.get_parameter('exploration_period').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.distance_weight = self.get_parameter('distance_weight').value
        self.startup_delay_sec = self.get_parameter('startup_delay_sec').value
        self.min_free_cells = self.get_parameter('min_free_cells').value
        self.min_unknown_cells = self.get_parameter('min_unknown_cells').value
        self.no_frontier_patience = self.get_parameter('no_frontier_patience').value
        self.goal_clearance_cells = self.get_parameter('goal_clearance_cells').value
        self.goal_unknown_clearance_cells = self.get_parameter('goal_unknown_clearance_cells').value
        self.frontier_out_of_bounds_as_unknown = (
            self.get_parameter('frontier_out_of_bounds_as_unknown').value
        )
        self.save_map_on_complete = self.get_parameter('save_map_on_complete').value
        self.save_map_service = self.get_parameter('save_map_service').value
        self.save_map_topic = self.get_parameter('save_map_topic').value
        self.save_map_url = self.get_parameter('save_map_url').value
        self.save_map_image_format = self.get_parameter('save_map_image_format').value
        self.save_map_mode = self.get_parameter('save_map_mode').value
        self.save_map_free_thresh = self.get_parameter('save_map_free_thresh').value
        self.save_map_occupied_thresh = self.get_parameter('save_map_occupied_thresh').value
        self.world_min_x = self.get_parameter('world_min_x').value
        self.world_max_x = self.get_parameter('world_max_x').value
        self.world_min_y = self.get_parameter('world_min_y').value
        self.world_max_y = self.get_parameter('world_max_y').value

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers — TRANSIENT_LOCAL to match the map publisher QoS
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.on_map, map_qos
        )

        # Publishers
        self.frontier_pub = self.create_publisher(
            MarkerArray, self.frontier_marker_topic, 10
        )
        self.goal_pub = self.create_publisher(
            PoseStamped, self.goal_pose_topic, 10
        )
        self.claim_pub = self.create_publisher(
            PoseStamped, self.claim_topic, 10
        )
        self.status_pub = self.create_publisher(
            String, self.exploration_status_topic, 10
        )

        self.claim_sub = self.create_subscription(
            PoseArray, self.claimed_frontiers_topic, self._on_claims, 10
        )
        self.done_sub = self.create_subscription(
            Bool, self.exploration_done_topic, self._on_exploration_done, 10
        )

        # Nav2 client
        self.nav2_client = Nav2GoalClient(self, self.goal_action)

        # State
        self.current_map = None
        self.current_goal = None
        self.failed_goals = []  # Track recently failed goals
        self.nav2_connected = False
        self.exploration_started = False
        self.no_frontier_cycles = 0
        self.exploration_complete = False
        self.map_save_requested = False
        self.exploration_status = 'IDLE'
        self.claimed_frontiers = []
        self.start_time = self.get_clock().now()

        self.map_saver_client = None
        if self.save_map_on_complete:
            self.map_saver_client = self.create_client(
                SaveMap, self.save_map_service
            )

        self.get_logger().info(
            f'RobotExplorer [{self.robot_name}] initialized. '
            f'Map: {self.map_topic}, Frame: {self.map_frame}, '
            f'Action: {self.goal_action}'
        )

        # Main exploration loop
        self.create_timer(self.exploration_period, self.exploration_loop)

    def on_map(self, msg: OccupancyGrid):
        """Callback for map updates."""
        self.current_map = msg

    def get_robot_pose(self) -> tuple:
        """Get robot pose in map frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return (x, y)
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def is_in_world_bounds(self, x: float, y: float) -> bool:
        """Check if a position is within the known world boundaries."""
        return (self.world_min_x <= x <= self.world_max_x and
                self.world_min_y <= y <= self.world_max_y)

    def is_goal_valid(self, pos: tuple, grid) -> bool:
        """Check that a goal lies within world bounds, on free space, with local clearance."""
        # Reject goals outside world boundaries
        if not self.is_in_world_bounds(pos[0], pos[1]):
            self.get_logger().warn(
                f'[{self.robot_name}] Rejecting out-of-world goal '
                f'({pos[0]:.2f}, {pos[1]:.2f}) — '
                f'world bounds: x=[{self.world_min_x}, {self.world_max_x}], '
                f'y=[{self.world_min_y}, {self.world_max_y}]'
            )
            return False

        cell = world_to_grid(self.current_map, pos[0], pos[1])
        if cell is None:
            return False

        cx, cy = cell
        if not is_free(grid[cy, cx]):
            return False

        clearance = max(0, int(self.goal_clearance_cells))
        unknown_clearance = max(0, int(self.goal_unknown_clearance_cells))
        height, width = grid.shape
        for ny in range(cy - clearance, cy + clearance + 1):
            if ny < 0 or ny >= height:
                continue
            for nx in range(cx - clearance, cx + clearance + 1):
                if nx < 0 or nx >= width:
                    continue
                if is_occupied(grid[ny, nx]):
                    return False

        if unknown_clearance > 0:
            for ny in range(cy - unknown_clearance, cy + unknown_clearance + 1):
                if ny < 0 or ny >= height:
                    continue
                for nx in range(cx - unknown_clearance, cx + unknown_clearance + 1):
                    if nx < 0 or nx >= width:
                        continue
                    if grid[ny, nx] < 0:
                        return False

        return True

    def exploration_loop(self):
        """Main exploration loop called periodically."""
        if self.exploration_complete:
            if self.save_map_on_complete and not self.map_save_requested:
                self._request_map_save()
            return

        now = self.get_clock().now()
        if (now - self.start_time).nanoseconds < int(self.startup_delay_sec * 1e9):
            self.get_logger().debug(
                f'[{self.robot_name}] Startup delay in progress...'
            )
            return

        if self.current_map is None:
            self.get_logger().debug(
                f'[{self.robot_name}] Waiting for map...'
            )
            return

        # Check Nav2 server availability (try once)
        if not self.nav2_connected:
            self.get_logger().info(
                f'[{self.robot_name}] Waiting for Nav2 action server '
                f'{self.goal_action}...'
            )
            if self.nav2_client.wait_for_server(timeout_sec=2.0):
                self.nav2_connected = True
                self.get_logger().info(
                    f'[{self.robot_name}] Nav2 action server connected!'
                )
            else:
                return

        # If a goal is currently active, check its status
        if self.nav2_client.is_active():
            self.get_logger().debug(
                f'[{self.robot_name}] Goal still active, waiting...'
            )
            return

        # If the last goal failed, track it to avoid re-selecting
        if self.nav2_client.state == Nav2GoalClient.STATE_FAILED:
            if self.current_goal is not None:
                self.failed_goals.append(self.current_goal)
                # Keep only last 10 failed goals
                self.failed_goals = self.failed_goals[-10:]
                self.get_logger().warn(
                    f'[{self.robot_name}] Previous goal failed. '
                    f'Will select a different frontier.'
                )
            self.nav2_client.reset()
            self.current_goal = None

        if self.nav2_client.state == Nav2GoalClient.STATE_REJECTED:
            self.get_logger().warn(
                f'[{self.robot_name}] Previous goal rejected.'
            )
            self.nav2_client.reset()
            self.current_goal = None

        if self.nav2_client.state == Nav2GoalClient.STATE_SUCCEEDED:
            self.get_logger().info(
                f'[{self.robot_name}] Previous goal succeeded!'
            )
            self.nav2_client.reset()
            self.current_goal = None
            # Clear failed goals on success (environment may have changed)
            self.failed_goals.clear()

        # Validate map readiness
        grid = get_grid_array(self.current_map)
        free_count, unknown_count, occupied_count = count_cell_types(grid)
        if free_count < self.min_free_cells:
            self.get_logger().info(
                f'[{self.robot_name}] Map not ready yet '
                f'(free={free_count}, unknown={unknown_count}, occupied={occupied_count})'
            )
            return

        if self.min_unknown_cells > 0 and unknown_count < self.min_unknown_cells:
            self.get_logger().info(
                f'[{self.robot_name}] Map not ready yet '
                f'(free={free_count}, unknown={unknown_count}, occupied={occupied_count})'
            )
            return

        # Get robot pose
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        # Detect frontiers
        frontiers = detect_frontiers(
            self.current_map,
            min_frontier_size=self.min_frontier_size,
            treat_out_of_bounds_as_unknown=self.frontier_out_of_bounds_as_unknown,
        )

        # Filter out frontiers that are not valid navigation targets
        valid_frontiers = [
            (pos, size) for (pos, size) in frontiers
            if self.is_goal_valid(pos, grid)
        ]

        filtered_frontiers = [
            (pos, size) for (pos, size) in valid_frontiers
            if not self._is_claimed(pos)
        ]

        # Publish frontier markers (valid only)
        marker_array = create_frontier_markers(
            filtered_frontiers, self.map_frame, self.current_map.header.stamp
        )
        self.frontier_pub.publish(marker_array)

        if not filtered_frontiers:
            if not self.exploration_started:
                self.get_logger().info(
                    f'[{self.robot_name}] No frontiers currently detected, '
                    f'waiting for map growth.'
                )
                return

            self.no_frontier_cycles += 1
            if self.no_frontier_cycles >= self.no_frontier_patience:
                self.get_logger().info(
                    f'[{self.robot_name}] No frontiers found after '
                    f'{self.no_frontier_cycles} cycles. Exploration complete.'
                )
                self.exploration_complete = True
                self._set_status('DONE')
                if self.save_map_on_complete:
                    self._request_map_save()
            else:
                self.get_logger().info(
                    f'[{self.robot_name}] No frontiers currently detected, '
                    f'waiting for map growth.'
                )
            return

        if not self.exploration_started:
            self.exploration_started = True
            self._set_status('ACTIVE')
            self.get_logger().info(
                f'[{self.robot_name}] Exploration started.'
            )

        self.no_frontier_cycles = 0

        self.get_logger().info(
            f'[{self.robot_name}] Detected {len(filtered_frontiers)} frontier(s)'
        )

        # Score frontiers
        scored = score_frontiers(
            filtered_frontiers, robot_pose,
            alpha=1.0,
            distance_weight=self.distance_weight
        )

        # Try to select a frontier that hasn't recently failed
        selected_frontier = None
        selected_size = None
        for _score, _idx, (pos, size) in scored:
            # Check if this frontier is too close to a recently failed goal
            too_close_to_failed = False
            for failed in self.failed_goals:
                dist = ((pos[0] - failed[0])**2 +
                        (pos[1] - failed[1])**2)**0.5
                if dist < 0.5:  # Within 0.5m of failed goal
                    too_close_to_failed = True
                    break
            if not too_close_to_failed:
                if self.is_goal_valid(pos, grid):
                    selected_frontier = pos
                    selected_size = size
                    break
                else:
                    self.get_logger().debug(
                        f'[{self.robot_name}] Skipping invalid goal '
                        f'({pos[0]:.2f}, {pos[1]:.2f})'
                    )

        if selected_frontier is None:
            # All frontiers are near failed goals, clear and try again
            self.failed_goals.clear()
            best_frontier, selected_size = select_best_frontier(scored)
            if best_frontier is not None and self.is_goal_valid(best_frontier, grid):
                selected_frontier = best_frontier

        if selected_frontier is None:
            return

        self.get_logger().info(
            f'[{self.robot_name}] Sending goal to '
            f'({selected_frontier[0]:.2f}, {selected_frontier[1]:.2f}), '
            f'size={selected_size}'
        )

        self.current_goal = selected_frontier
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self.map_frame
        goal_msg.pose.position.x = selected_frontier[0]
        goal_msg.pose.position.y = selected_frontier[1]
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)
        self._publish_claim(selected_frontier)
        self.nav2_client.send_goal(
            selected_frontier[0], selected_frontier[1], self.map_frame
        )

    def _on_claims(self, msg: PoseArray):
        self.claimed_frontiers = [
            (p.position.x, p.position.y) for p in msg.poses
        ]

    def _on_exploration_done(self, msg: Bool):
        if not msg.data:
            return
        if not self.exploration_complete:
            self.get_logger().info(
                f'[{self.robot_name}] Swarm exploration complete. Stopping.'
            )
        self.exploration_complete = True
        self._set_status('DONE')
        self.nav2_client.cancel_goal()

    def _set_status(self, status: str):
        if self.exploration_status == status:
            return
        self.exploration_status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _transform_to_claim_frame(self, pos: tuple) -> tuple:
        if self.claim_frame == self.map_frame:
            return pos

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        try:
            transformed = self.tf_buffer.transform(
                pose,
                self.claim_frame,
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            return (
                transformed.pose.position.x,
                transformed.pose.position.y,
            )
        except Exception as e:
            self.get_logger().warn(
                f'[{self.robot_name}] Claim transform failed: {e}'
            )
            return None

    def _is_claimed(self, pos: tuple) -> bool:
        if not self.claimed_frontiers:
            return False

        transformed = self._transform_to_claim_frame(pos)
        if transformed is None:
            return False

        for cx, cy in self.claimed_frontiers:
            dist = ((transformed[0] - cx) ** 2 + (transformed[1] - cy) ** 2) ** 0.5
            if dist <= self.claim_radius:
                return True
        return False

    def _publish_claim(self, pos: tuple):
        claim_msg = PoseStamped()
        claim_msg.header.stamp = self.get_clock().now().to_msg()
        claim_msg.header.frame_id = self.map_frame
        claim_msg.pose.position.x = pos[0]
        claim_msg.pose.position.y = pos[1]
        claim_msg.pose.position.z = 0.0
        claim_msg.pose.orientation.w = 1.0

        if self.claim_frame != self.map_frame:
            transformed = self._transform_to_claim_frame(pos)
            if transformed is not None:
                claim_msg.header.frame_id = self.claim_frame
                claim_msg.pose.position.x = transformed[0]
                claim_msg.pose.position.y = transformed[1]

        self.claim_pub.publish(claim_msg)

    def _request_map_save(self):
        if self.map_saver_client is None:
            self.get_logger().warn(
                f'[{self.robot_name}] Map saver client not available.'
            )
            return

        if not self.map_saver_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f'[{self.robot_name}] Map saver service not ready: '
                f'{self.save_map_service}'
            )
            return

        req = SaveMap.Request()
        req.map_topic = self.save_map_topic
        req.map_url = self.save_map_url
        req.image_format = self.save_map_image_format
        req.map_mode = self.save_map_mode
        req.free_thresh = float(self.save_map_free_thresh)
        req.occupied_thresh = float(self.save_map_occupied_thresh)

        self.map_save_requested = True
        future = self.map_saver_client.call_async(req)
        future.add_done_callback(self._on_map_save_response)

        self.get_logger().info(
            f'[{self.robot_name}] Saving map to {self.save_map_url} '
            f'from {self.save_map_topic}'
        )

    def _on_map_save_response(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warn(
                f'[{self.robot_name}] Map save failed: {e}'
            )
            self.map_save_requested = False
            return

        if resp.result:
            self.get_logger().info(
                f'[{self.robot_name}] Map saved successfully.'
            )
        else:
            self.get_logger().warn(
                f'[{self.robot_name}] Map save failed (result=false).'
            )
            self.map_save_requested = False


def main(args=None):
    rclpy.init(args=args)
    node = RobotExplorer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
