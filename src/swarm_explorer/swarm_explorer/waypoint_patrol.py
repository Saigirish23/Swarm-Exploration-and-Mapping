"""
Deterministic waypoint patrol node for a single TurtleBot3 robot.

Uses direct cmd_vel control with odometry feedback to navigate through
a list of waypoints in a continuous loop. Independent of Nav2/costmap/map.

Includes LiDAR-based obstacle detection: the robot slows down and stops
when obstacles are detected ahead, preventing wall/furniture collisions.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class WaypointPatrol(Node):
    """Drive a robot through a repeating list of waypoints via cmd_vel."""

    # FSM states
    STATE_ROTATING = 'ROTATING'
    STATE_DRIVING = 'DRIVING'
    STATE_ARRIVED = 'ARRIVED'
    STATE_OBSTACLE = 'OBSTACLE'  # obstacle detected, rotating away

    def __init__(self):
        super().__init__('waypoint_patrol')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter(
            'waypoints',
            [],
        )
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('goal_tolerance', 0.35)
        self.declare_parameter('heading_tolerance', 0.15)   # radians (~8.6°)
        self.declare_parameter('control_rate', 10.0)        # Hz
        self.declare_parameter('startup_delay', 5.0)        # seconds
        self.declare_parameter('obstacle_stop_dist', 0.35)  # stop if obstacle this close
        self.declare_parameter('obstacle_slow_dist', 0.6)   # slow down zone
        self.declare_parameter('obstacle_arc_deg', 50.0)    # scan arc ±degrees ahead

        self.robot_name = self.get_parameter('robot_name').value
        raw_wps = self.get_parameter('waypoints').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.heading_tolerance = self.get_parameter('heading_tolerance').value
        control_rate = self.get_parameter('control_rate').value
        self.startup_delay = self.get_parameter('startup_delay').value
        self.obstacle_stop_dist = self.get_parameter('obstacle_stop_dist').value
        self.obstacle_slow_dist = self.get_parameter('obstacle_slow_dist').value
        self.obstacle_arc_deg = self.get_parameter('obstacle_arc_deg').value

        self.patrol_enabled = True

        # Parse flat list [x1,y1,x2,y2,...] into list of (x,y) tuples
        if len(raw_wps) == 0:
            self.get_logger().info(
                f'[{self.robot_name}] No waypoints provided. Patrol disabled.'
            )
            self.waypoints = []
            self.patrol_enabled = False
        elif len(raw_wps) < 4 or len(raw_wps) % 2 != 0:
            self.get_logger().error(
                f'waypoints must be a flat list of even length >= 4, '
                f'got {len(raw_wps)} values'
            )
            raw_wps = [0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0]
        if self.patrol_enabled:
            self.waypoints = [
                (raw_wps[i], raw_wps[i + 1])
                for i in range(0, len(raw_wps), 2)
            ]

        # ── Topics ──────────────────────────────────────────────
        cmd_vel_topic = f'/{self.robot_name}/cmd_vel'
        odom_topic = f'/{self.robot_name}/odom'
        scan_topic = f'/{self.robot_name}/scan'

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, 10
        )

        # ── State ───────────────────────────────────────────────
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        self.wp_index = 0
        self.state = self.STATE_ROTATING
        self.obstacle_start_time = None
        self.laps = 0
        self.start_time = self.get_clock().now()

        # LiDAR obstacle state
        self.front_min_dist = float('inf')  # min distance in front arc
        self.obstacle_rotate_dir = 1.0      # which way to rotate to avoid
        self.blocked_start_time = None      # for stuck waypoint recovery

        # ── Control timer ───────────────────────────────────────
        dt = 1.0 / control_rate
        self.create_timer(dt, self._control_loop)

        self.get_logger().info(
            f'WaypointPatrol [{self.robot_name}] initialized\n'
            f'  cmd_vel: {cmd_vel_topic}\n'
            f'  odom:    {odom_topic}\n'
            f'  scan:    {scan_topic}\n'
            f'  waypoints ({len(self.waypoints)}): {self.waypoints}\n'
            f'  linear_speed={self.linear_speed}, '
            f'angular_speed={self.angular_speed}\n'
            f'  goal_tolerance={self.goal_tolerance}m\n'
            f'  obstacle_stop={self.obstacle_stop_dist}m, '
            f'obstacle_slow={self.obstacle_slow_dist}m, '
            f'arc=±{self.obstacle_arc_deg}°\n'
            f'  startup_delay={self.startup_delay}s'
        )

    # ── Odometry callback ───────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.odom_received = True

    # ── LiDAR callback ──────────────────────────────────────────

    def _on_scan(self, msg: LaserScan):
        """Extract minimum distance in the forward arc."""
        if not msg.ranges:
            return

        n = len(msg.ranges)
        arc_rad = math.radians(self.obstacle_arc_deg)

        # Indices for ±arc around 0 (front)
        arc_indices = int(arc_rad / msg.angle_increment) if msg.angle_increment > 0 else 0

        min_dist = float('inf')
        left_min = float('inf')
        right_min = float('inf')

        for i in range(arc_indices + 1):
            # Front-right rays: indices 0..arc_indices
            if i < n:
                r = msg.ranges[i]
                if msg.range_min < r < msg.range_max:
                    min_dist = min(min_dist, r)
                    right_min = min(right_min, r)

            # Front-left rays: indices (n-arc_indices)..n-1
            j = n - 1 - i
            if 0 <= j < n:
                r = msg.ranges[j]
                if msg.range_min < r < msg.range_max:
                    min_dist = min(min_dist, r)
                    left_min = min(left_min, r)

        self.front_min_dist = min_dist
        # Rotate toward the side with more space
        self.obstacle_rotate_dir = 1.0 if left_min > right_min else -1.0

    # ── Control loop ────────────────────────────────────────────

    def _control_loop(self):
        """FSM: ROTATING → DRIVING → ARRIVED → next waypoint.
        Obstacles only halt linear driving, allowing free rotation to clear.
        Stuck recovery automatically skips to the next waypoint if blocked for > 5s."""
        # Startup delay
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.startup_delay:
            return

        if not self.odom_received:
            self.get_logger().warn(
                f'[{self.robot_name}] No odom received yet, waiting...',
                throttle_duration_sec=3.0,
            )
            return

        if not self.patrol_enabled:
            return

        # Reset obstacle timer when not in obstacle state
        if self.state != self.STATE_OBSTACLE:
            self.obstacle_start_time = None

        target = self.waypoints[self.wp_index]
        dx = target[0] - self.current_x
        dy = target[1] - self.current_y
        dist = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        heading_error = self._normalize_angle(desired_yaw - self.current_yaw)

        cmd = Twist()

        # ── Obstacle Detection (only during DRIVING) ────────────
        if self.state == self.STATE_DRIVING and self.front_min_dist < self.obstacle_stop_dist:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.obstacle_start_time = self.get_clock().now()
            self.get_logger().warn(
                f'[{self.robot_name}] ⚠ Obstacle at {self.front_min_dist:.2f}m! '
                f'Entering active avoidance mode'
            )
            self.state = self.STATE_OBSTACLE
            return

        # ── OBSTACLE STATE - Active avoidance with timeout ──────
        if self.state == self.STATE_OBSTACLE:
            # Calculate how long we've been stuck
            if self.obstacle_start_time is None:
                self.obstacle_start_time = self.get_clock().now()
            
            elapsed = (self.get_clock().now() - self.obstacle_start_time).nanoseconds / 1e9
            
            # TIMEOUT RECOVERY: Force skip waypoint after 10 seconds
            if elapsed > 10.0:
                self.get_logger().warn(
                    f'[{self.robot_name}] Stuck in obstacle avoidance for {elapsed:.1f}s! '
                    f'Forcing skip to next waypoint'
                )
                self.state = self.STATE_ARRIVED
                self.obstacle_start_time = None
                return
            
            # SUCCESS: Path is now clear
            if self.front_min_dist > self.obstacle_stop_dist * 1.5:
                self.get_logger().info(
                    f'[{self.robot_name}] Path clear (front={self.front_min_dist:.2f}m)! '
                    f'Resuming navigation'
                )
                self.state = self.STATE_ROTATING
                self.obstacle_start_time = None
                return
            
            # Continue rotating in place to find clear path safely
            cmd.linear.x = 0.0
            cmd.angular.z = self.obstacle_rotate_dir * self.angular_speed * 0.5
            self.cmd_pub.publish(cmd)
            
            self.get_logger().info(
                f'[{self.robot_name}] Avoiding obstacle, rotating... '
                f'(blocked {elapsed:.1f}s/10.0s)',
                throttle_duration_sec=2.0
            )
            return

        # ── ARRIVED ─────────────────────────────────────────────
        if self.state == self.STATE_ARRIVED:
            self.wp_index = (self.wp_index + 1) % len(self.waypoints)
            if self.wp_index == 0:
                self.laps += 1
                self.get_logger().info(
                    f'[{self.robot_name}] ── Lap {self.laps} complete ──'
                )
            target = self.waypoints[self.wp_index]
            self.state = self.STATE_ROTATING
            self.blocked_start_time = None
            self.get_logger().info(
                f'[{self.robot_name}] → WP {self.wp_index}: '
                f'({target[0]:.2f}, {target[1]:.2f})'
            )
            return

        # ── Goal reached? ───────────────────────────────────────
        if dist < self.goal_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f'[{self.robot_name}] ✓ Reached WP {self.wp_index} '
                f'({target[0]:.2f}, {target[1]:.2f})  '
                f'pose=({self.current_x:.2f}, {self.current_y:.2f})'
            )
            self.state = self.STATE_ARRIVED
            self.blocked_start_time = None
            return

        # ── ROTATING — face the target ──────────────────────────
        if self.state == self.STATE_ROTATING:
            self.blocked_start_time = None
            if self.front_min_dist < self.obstacle_stop_dist * 0.5:
                # Emergency: obstacle too close even during rotation
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                return
            
            if abs(heading_error) > self.heading_tolerance:
                sign = 1.0 if heading_error > 0 else -1.0
                scale = min(abs(heading_error) / 0.4, 1.0)
                cmd.angular.z = sign * self.angular_speed * scale
                self.cmd_pub.publish(cmd)
            else:
                self.state = self.STATE_DRIVING
                self.get_logger().debug(
                    f'[{self.robot_name}] Heading aligned → driving'
                )
            return

        # ── DRIVING — move toward target ────────────────────────
        if self.state == self.STATE_DRIVING:
            self.blocked_start_time = None
            if abs(heading_error) > self.heading_tolerance * 3:
                self.state = self.STATE_ROTATING
                return

            speed = self.linear_speed
            # Slow in obstacle proximity zone
            if self.front_min_dist < self.obstacle_slow_dist:
                speed *= max(
                    (self.front_min_dist - self.obstacle_stop_dist)
                    / (self.obstacle_slow_dist - self.obstacle_stop_dist),
                    0.2,
                )
            # Slow near waypoint
            if dist < 0.5:
                speed *= max(dist / 0.5, 0.3)

            cmd.linear.x = speed
            cmd.angular.z = 1.2 * heading_error  # proportional steering
            self.cmd_pub.publish(cmd)

            self.get_logger().info(
                f'[{self.robot_name}] Driving → WP {self.wp_index} '
                f'({target[0]:.1f},{target[1]:.1f})  '
                f'dist={dist:.2f}m  scan={self.front_min_dist:.2f}m  '
                f'pose=({self.current_x:.2f},{self.current_y:.2f})',
                throttle_duration_sec=3.0,
            )

    # ── Helpers ─────────────────────────────────────────────────

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Wrap angle to [-π, π]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def destroy_node(self):
        """Stop the robot before shutting down."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
