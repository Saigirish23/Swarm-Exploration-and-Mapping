"""Nav2 NavigateToPose action client with proper async lifecycle."""

import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class Nav2GoalClient:
    """Client for sending goals to Nav2 NavigateToPose action server."""

    # Goal states
    STATE_IDLE = 'idle'
    STATE_SENDING = 'sending'
    STATE_ACTIVE = 'active'
    STATE_SUCCEEDED = 'succeeded'
    STATE_FAILED = 'failed'
    STATE_REJECTED = 'rejected'

    def __init__(self, node, action_name: str):
        """
        Initialize Nav2 goal client.

        Args:
            node: ROS2 node
            action_name: full action name (e.g., '/robot1/navigate_to_pose')
        """
        self.node = node
        self.action_name = action_name
        self.action_client = ActionClient(node, NavigateToPose, action_name)
        self.state = self.STATE_IDLE
        self._goal_handle = None
        self._result_future = None

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        """Wait for action server to be available."""
        return self.action_client.wait_for_server(timeout_sec=timeout_sec)

    def send_goal(
        self,
        target_x: float,
        target_y: float,
        map_frame: str,
        yaw: float = 0.0
    ):
        """
        Send goal to Nav2 asynchronously.

        Args:
            target_x, target_y: goal position in map frame
            map_frame: frame ID (e.g., 'robot1/map')
            yaw: goal orientation
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = map_frame
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.position.z = 0.0

        # Quaternion for yaw (rotation around z-axis)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal_msg.pose.pose.orientation.w = math.cos(yaw * 0.5)

        self.state = self.STATE_SENDING
        self._goal_handle = None
        self._result_future = None

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn(
                f'Goal REJECTED by {self.action_name}'
            )
            self.state = self.STATE_REJECTED
            return

        self.node.get_logger().info(
            f'Goal ACCEPTED by {self.action_name}'
        )
        self._goal_handle = goal_handle
        self.state = self.STATE_ACTIVE

        # Request the result
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        """Handle goal result."""
        result = future.result()
        status = result.status

        # GoalStatus values: STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6
        if status == 4:  # SUCCEEDED
            self.node.get_logger().info(
                f'Goal SUCCEEDED on {self.action_name}'
            )
            self.state = self.STATE_SUCCEEDED
        else:
            self.node.get_logger().warn(
                f'Goal FAILED on {self.action_name} (status={status})'
            )
            self.state = self.STATE_FAILED

    def is_active(self) -> bool:
        """Check if a goal is currently being pursued."""
        return self.state in (self.STATE_SENDING, self.STATE_ACTIVE)

    def is_done(self) -> bool:
        """Check if the last goal completed (success or failure)."""
        return self.state in (
            self.STATE_SUCCEEDED, self.STATE_FAILED,
            self.STATE_REJECTED, self.STATE_IDLE
        )

    def is_succeeded(self) -> bool:
        """Check if the last goal succeeded."""
        return self.state == self.STATE_SUCCEEDED

    def cancel_goal(self):
        """Cancel the current goal if active."""
        if self._goal_handle is not None and self.state == self.STATE_ACTIVE:
            self.node.get_logger().info(
                f'Cancelling goal on {self.action_name}'
            )
            self._goal_handle.cancel_goal_async()
            self.state = self.STATE_IDLE

    def reset(self):
        """Reset state to idle."""
        self.state = self.STATE_IDLE
        self._goal_handle = None
        self._result_future = None
