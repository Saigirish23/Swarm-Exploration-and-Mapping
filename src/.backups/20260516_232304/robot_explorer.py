"""Main frontier exploration node for a single robot."""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformListener, Buffer
import math

from .frontier_detector import detect_frontiers, create_frontier_markers
from .goal_selector import score_frontiers, select_best_frontier
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
        self.declare_parameter('frontier_marker_topic', '/robot1/frontier_markers')
        self.declare_parameter('exploration_period', 5.0)
        
        self.robot_name = self.get_parameter('robot_name').value
        self.map_topic = self.get_parameter('map_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.goal_action = self.get_parameter('goal_action').value
        self.frontier_marker_topic = self.get_parameter('frontier_marker_topic').value
        self.exploration_period = self.get_parameter('exploration_period').value
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.on_map, 10
        )
        
        # Publishers
        self.frontier_pub = self.create_publisher(
            MarkerArray, self.frontier_marker_topic, 10
        )
        
        # Nav2 client
        self.nav2_client = Nav2GoalClient(self, self.goal_action)
        
        # State
        self.current_map = None
        self.exploring = False
        self.current_goal = None
        
        self.get_logger().info(
            f'RobotExplorer [{self.robot_name}] initialized. '
            f'Map: {self.map_topic}, Frame: {self.map_frame}'
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
    
    def exploration_loop(self):
        """Main exploration loop called periodically."""
        if self.current_map is None:
            return
        
        # Get robot pose
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return
        
        # Detect frontiers
        frontiers = detect_frontiers(self.current_map, min_frontier_size=5)
        
        # Publish frontier markers
        marker_array = create_frontier_markers(
            frontiers, self.map_frame, self.current_map.header.stamp
        )
        self.frontier_pub.publish(marker_array)
        
        if not frontiers:
            self.get_logger().info(f'[{self.robot_name}] No frontiers found. Exploration complete.')
            return
        
        self.get_logger().info(
            f'[{self.robot_name}] Detected {len(frontiers)} frontier(s)'
        )
        
        # Score frontiers
        scored = score_frontiers(frontiers, robot_pose, alpha=1.0)
        
        # Select best frontier
        best_frontier, size = select_best_frontier(scored)
        if best_frontier is None:
            return
        
        self.get_logger().info(
            f'[{self.robot_name}] Selected frontier at ({best_frontier[0]:.2f}, {best_frontier[1]:.2f}), '
            f'size={size}'
        )
        
        # Send goal to Nav2
        if not self.exploring:
            self.exploring = True
            self.current_goal = best_frontier
            self.nav2_client.send_goal(best_frontier[0], best_frontier[1], self.map_frame)
            self.get_logger().info(f'[{self.robot_name}] Goal sent to Nav2.')


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
