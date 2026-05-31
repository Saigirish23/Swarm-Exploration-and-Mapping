"""Nav2 NavigateToPose action client."""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class Nav2GoalClient:
    """Client for sending goals to Nav2 NavigateToPose action server."""
    
    def __init__(self, node, action_name: str):
        """
        Initialize Nav2 goal client.
        
        Args:
            node: ROS2 node
            action_name: full action name (e.g., '/robot1/navigate_to_pose')
        """
        self.node = node
        self.action_client = ActionClient(node, NavigateToPose, action_name)
    
    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        """Wait for action server to be available."""
        return self.action_client.wait_for_server(timeout_sec=timeout_sec)
    
    def send_goal(
        self,
        target_x: float,
        target_y: float,
        map_frame: str,
        yaw: float = 0.0
    ) -> bool:
        """
        Send goal to Nav2.
        
        Args:
            target_x, target_y: goal position in map frame
            map_frame: frame ID (e.g., 'robot1/map')
            yaw: goal orientation
        
        Returns:
            True if goal sent, False otherwise
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = map_frame
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.position.z = 0.0
        
        # Simple quaternion for yaw (only rotation around z)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = (yaw * 0.5).__sin__()
        goal_msg.pose.pose.orientation.w = (yaw * 0.5).__cos__()
        
        self.goal_handle = self.action_client.send_goal_async(goal_msg)
        return True
    
    def is_goal_reached(self) -> bool:
        """Check if goal was reached successfully."""
        if not hasattr(self, 'goal_handle') or self.goal_handle is None:
            return False
        
        if not self.goal_handle.done():
            return False
        
        try:
            result = self.goal_handle.result()
            # If result is not None, goal was completed
            return result is not None
        except Exception:
            return False
