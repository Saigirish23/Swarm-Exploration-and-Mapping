"""Swarm-level frontier claim coordinator and exploration completion gate."""

import math
import rclpy
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.srv import SaveMap
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class FrontierCoordinator(Node):
    """Coordinate frontier claims and exploration completion for multiple robots."""

    def __init__(self):
        super().__init__('frontier_coordinator')

        self.declare_parameter('claim_requests_topic', '/swarm/claim_requests')
        self.declare_parameter('claimed_frontiers_topic', '/swarm/claimed_frontiers')
        self.declare_parameter('claim_ttl_sec', 12.0)
        self.declare_parameter('claim_merge_radius', 0.3)
        self.declare_parameter('claim_frame', 'map')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('log_status_period_sec', 5.0)
        self.declare_parameter('robot_names', ['robot1', 'robot2'])
        self.declare_parameter('exploration_status_suffix', '/exploration_status')
        self.declare_parameter('exploration_done_topic', '/swarm/exploration_done')
        self.declare_parameter('save_map_on_complete', True)
        self.declare_parameter('save_map_service', '/robot1/map_saver/save_map')
        self.declare_parameter('save_map_topic', '/merged_map')
        self.declare_parameter(
            'save_map_url',
            'file:///home/saigirish/tb3_swarm_ws/src/swarm_explorer/maps/merged_map.yaml'
        )
        self.declare_parameter('save_map_image_format', 'png')
        self.declare_parameter('save_map_mode', 'trinary')
        self.declare_parameter('save_map_free_thresh', 0.25)
        self.declare_parameter('save_map_occupied_thresh', 0.65)

        self.claim_requests_topic = self.get_parameter('claim_requests_topic').value
        self.claimed_frontiers_topic = self.get_parameter('claimed_frontiers_topic').value
        self.claim_ttl_sec = self.get_parameter('claim_ttl_sec').value
        self.claim_merge_radius = self.get_parameter('claim_merge_radius').value
        self.claim_frame = self.get_parameter('claim_frame').value
        self.publish_period_sec = self.get_parameter('publish_period_sec').value
        self.log_status_period_sec = self.get_parameter('log_status_period_sec').value
        self.robot_names = list(self.get_parameter('robot_names').value)
        self.exploration_status_suffix = self.get_parameter('exploration_status_suffix').value
        self.exploration_done_topic = self.get_parameter('exploration_done_topic').value
        self.save_map_on_complete = self.get_parameter('save_map_on_complete').value
        self.save_map_service = self.get_parameter('save_map_service').value
        self.save_map_topic = self.get_parameter('save_map_topic').value
        self.save_map_url = self.get_parameter('save_map_url').value
        self.save_map_image_format = self.get_parameter('save_map_image_format').value
        self.save_map_mode = self.get_parameter('save_map_mode').value
        self.save_map_free_thresh = self.get_parameter('save_map_free_thresh').value
        self.save_map_occupied_thresh = self.get_parameter('save_map_occupied_thresh').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.claim_sub = self.create_subscription(
            PoseStamped, self.claim_requests_topic, self._on_claim, 10
        )
        self.claim_pub = self.create_publisher(
            PoseArray, self.claimed_frontiers_topic, 10
        )
        self.done_pub = self.create_publisher(
            Bool, self.exploration_done_topic, 10
        )

        self.status_subs = []
        self.robot_status = {name: 'UNKNOWN' for name in self.robot_names}
        for name in self.robot_names:
            topic = f'/{name}{self.exploration_status_suffix}'
            self.status_subs.append(
                self.create_subscription(
                    String,
                    topic,
                    lambda msg, n=name: self._on_status(msg, n),
                    10,
                )
            )

        self.claims = []
        self.exploration_done = False
        self.map_save_requested = False
        self.last_status_log = self.get_clock().now()

        self.map_saver_client = None
        if self.save_map_on_complete:
            self.map_saver_client = self.create_client(
                SaveMap, self.save_map_service
            )

        self.create_timer(self.publish_period_sec, self._tick)

        self.get_logger().info(
            f'FrontierCoordinator started. Claims: {self.claim_requests_topic} -> '
            f'{self.claimed_frontiers_topic}, frame={self.claim_frame}, '
            f'robots={self.robot_names}'
        )

    def _on_status(self, msg: String, robot_name: str):
        self.robot_status[robot_name] = msg.data.strip().upper()

    def _on_claim(self, msg: PoseStamped):
        now = self.get_clock().now()
        pose = self._to_claim_frame(msg)
        if pose is None:
            return

        x = pose.pose.position.x
        y = pose.pose.position.y

        merged = False
        for idx, (cx, cy, _stamp) in enumerate(self.claims):
            dist = math.hypot(x - cx, y - cy)
            if dist <= self.claim_merge_radius:
                self.claims[idx] = (cx, cy, now)
                merged = True
                break

        if not merged:
            self.claims.append((x, y, now))
            self.get_logger().info(
                f'Claim accepted at ({x:.2f}, {y:.2f})'
            )

    def _tick(self):
        self._prune_claims()
        self._publish_claims()
        self._check_completion()
        self._log_status()

    def _prune_claims(self):
        now = self.get_clock().now()
        ttl_ns = int(self.claim_ttl_sec * 1e9)
        self.claims = [
            (x, y, stamp)
            for (x, y, stamp) in self.claims
            if (now - stamp).nanoseconds <= ttl_ns
        ]

    def _publish_claims(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.claim_frame
        for x, y, _stamp in self.claims:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose.pose)
        self.claim_pub.publish(msg)

    def _check_completion(self):
        if self.exploration_done:
            return

        if not self.robot_status:
            return

        all_done = all(status == 'DONE' for status in self.robot_status.values())
        if not all_done:
            return

        self.exploration_done = True
        done_msg = Bool()
        done_msg.data = True
        self.done_pub.publish(done_msg)

        self.get_logger().info('All robots DONE. Swarm exploration complete.')

        if self.save_map_on_complete and not self.map_save_requested:
            self._request_map_save()

    def _log_status(self):
        if self.log_status_period_sec <= 0:
            return

        now = self.get_clock().now()
        if (now - self.last_status_log).nanoseconds < int(self.log_status_period_sec * 1e9):
            return

        status_summary = ', '.join(
            f'{name}:{status}' for name, status in self.robot_status.items()
        )
        self.get_logger().info(
            f'Claims={len(self.claims)} | {status_summary}'
        )
        self.last_status_log = now

    def _request_map_save(self):
        if self.map_saver_client is None:
            return

        if not self.map_saver_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f'Map saver service not ready: {self.save_map_service}'
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
            f'Saving map to {self.save_map_url} from {self.save_map_topic}'
        )

    def _on_map_save_response(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warn(f'Map save failed: {e}')
            self.map_save_requested = False
            return

        if resp.result:
            self.get_logger().info('Map saved successfully.')
        else:
            self.get_logger().warn('Map save failed (result=false).')
            self.map_save_requested = False

    def _to_claim_frame(self, msg: PoseStamped):
        if not msg.header.frame_id or msg.header.frame_id == self.claim_frame:
            return msg

        try:
            return self.tf_buffer.transform(
                msg,
                self.claim_frame,
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except Exception as e:
            self.get_logger().warn(f'Claim transform failed: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = FrontierCoordinator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
