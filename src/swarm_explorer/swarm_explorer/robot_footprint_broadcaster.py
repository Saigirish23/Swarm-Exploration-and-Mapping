"""Publish robot footprint as a PointCloud2 obstacle for teammate costmaps."""

import math
import rclpy
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


class RobotFootprintBroadcaster(Node):
    """Broadcast a circular footprint as a PointCloud2 obstacle."""

    def __init__(self):
        super().__init__('robot_footprint_broadcaster')

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('base_frame', 'robot1/base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('obstacle_topic', '/robot1/robot_obstacle')
        self.declare_parameter('radius', 0.22)
        self.declare_parameter('point_count', 16)
        self.declare_parameter('publish_rate', 5.0)

        self.robot_name = self.get_parameter('robot_name').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.obstacle_topic = self.get_parameter('obstacle_topic').value
        self.radius = float(self.get_parameter('radius').value)
        self.point_count = int(self.get_parameter('point_count').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PointCloud2, self.obstacle_topic, 10)

        period = 1.0 / max(self.publish_rate, 0.1)
        self.create_timer(period, self._publish)

        self.get_logger().info(
            f'FootprintBroadcaster [{self.robot_name}] -> {self.obstacle_topic}, '
            f'frame={self.map_frame}, radius={self.radius}m'
        )

    def _publish(self):
        pose = self._lookup_pose()
        if pose is None:
            return

        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z

        points = []
        for i in range(self.point_count):
            ang = 2.0 * math.pi * (i / self.point_count)
            px = x + self.radius * math.cos(ang)
            py = y + self.radius * math.sin(ang)
            points.append((px, py, z))

        header = pose.header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame

        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.pub.publish(cloud)

    def _lookup_pose(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.base_frame
        pose.pose.orientation.w = 1.0

        try:
            return self.tf_buffer.transform(
                pose,
                self.map_frame,
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except Exception as e:
            self.get_logger().debug(
                f'Footprint transform failed: {e}',
                throttle_duration_sec=2.0,
            )
            return None


def main(args=None):
    rclpy.init(args=args)
    node = RobotFootprintBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
