"""Known-initial-pose occupancy grid merger for dual robots."""

import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import threading
from tf2_ros import Buffer, TransformListener


class MapMerger(Node):
    """Merge occupancy grids from two robots assuming known initial poses."""

    def __init__(self):
        super().__init__('map_merger')

        # Parameters
        self.declare_parameter('map1_topic', '/robot1/map')
        self.declare_parameter('map2_topic', '/robot2/map')
        self.declare_parameter('merged_map_topic', '/merged_map')
        self.declare_parameter('merged_frame_id', 'map')
        self.declare_parameter('robot1_initial_x', -2.0)
        self.declare_parameter('robot1_initial_y', -2.0)
        self.declare_parameter('robot1_initial_yaw', 0.0)
        self.declare_parameter('robot2_initial_x', 2.0)
        self.declare_parameter('robot2_initial_y', -2.0)
        self.declare_parameter('robot2_initial_yaw', 0.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('merge_rate_hz', 1.0)
        self.declare_parameter('require_both_maps', True)

        self.map1_topic = self.get_parameter('map1_topic').value
        self.map2_topic = self.get_parameter('map2_topic').value
        self.merged_map_topic = self.get_parameter('merged_map_topic').value
        self.merged_frame_id = self.get_parameter('merged_frame_id').value
        self.robot1_pos = (
            self.get_parameter('robot1_initial_x').value,
            self.get_parameter('robot1_initial_y').value,
        )
        self.robot2_pos = (
            self.get_parameter('robot2_initial_x').value,
            self.get_parameter('robot2_initial_y').value,
        )
        self.resolution = self.get_parameter('resolution').value
        self.merge_rate_hz = self.get_parameter('merge_rate_hz').value
        self.require_both_maps = self.get_parameter('require_both_maps').value

        # QoS profile for maps (transient-local, reliable for late joiners)
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        # Subscribers with transient-local QoS
        self.map1_sub = self.create_subscription(
            OccupancyGrid, self.map1_topic, self.on_map1, map_qos
        )
        self.map2_sub = self.create_subscription(
            OccupancyGrid, self.map2_topic, self.on_map2, map_qos
        )

        # Publisher with transient-local QoS
        self.merged_pub = self.create_publisher(
            OccupancyGrid, self.merged_map_topic, map_qos
        )

        # State
        self.map1 = None
        self.map2 = None
        self.lock = threading.Lock()

        # TF Buffer and TransformListener for live offset lookup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer-based merge instead of on-every-callback
        merge_period = 1.0 / self.merge_rate_hz
        self.merge_timer = self.create_timer(merge_period, self.merge_tick)

        self.get_logger().info(
            f'MapMerger initialized. '
            f'Map1: {self.map1_topic}, Map2: {self.map2_topic}, '
            f'Robot1 pos: {self.robot1_pos}, Robot2 pos: {self.robot2_pos}, '
            f'Merge rate: {self.merge_rate_hz} Hz'
        )

    def on_map1(self, msg: OccupancyGrid):
        """Callback for robot1 map."""
        with self.lock:
            self.map1 = msg

    def on_map2(self, msg: OccupancyGrid):
        """Callback for robot2 map."""
        with self.lock:
            self.map2 = msg

    def merge_tick(self):
        """Timer callback to merge maps using live TF transforms."""
        with self.lock:
            if self.require_both_maps:
                if self.map1 is None or self.map2 is None:
                    return
            elif self.map1 is None and self.map2 is None:
                return

            # Try dynamic transforms first
            robot1_offset = self._get_robot_offset('robot1')
            robot2_offset = self._get_robot_offset('robot2')

            # Fall back to static initial poses
            if robot1_offset is None:
                robot1_offset = self.robot1_pos
            if robot2_offset is None:
                robot2_offset = self.robot2_pos

            merged = self.merge_occupancy_grids(
                self.map1, self.map2,
                robot1_offset, robot2_offset,
                self.resolution
            )

        merged.header.frame_id = self.merged_frame_id
        merged.header.stamp = self.get_clock().now().to_msg()

        self.merged_pub.publish(merged)

    def _get_robot_offset(self, robot_name: str):
        """Get robot's map->robotX/map transform dynamically."""
        try:
            # Look up map -> robotX/map transform to get the map offset
            transform = self.tf_buffer.lookup_transform(
                'map',
                f'{robot_name}/map',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return (
                transform.transform.translation.x,
                transform.transform.translation.y
            )
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed for {robot_name}: {e}')
            return None

    @staticmethod
    def merge_occupancy_grids(
        map1, map2,
        robot1_pos: tuple, robot2_pos: tuple,
        resolution: float
    ) -> OccupancyGrid:
        """
        Merge two occupancy grids assuming known initial robot poses.

        Dynamically computes bounding box from actual map extents each cycle.
        Uses cell-center coordinate conversion.

        Args:
            map1, map2: OccupancyGrid messages from robots (may be None)
            robot1_pos, robot2_pos: (x, y) initial positions
            resolution: grid resolution

        Returns:
            Merged OccupancyGrid
        """
        maps_with_offsets = []
        if map1 is not None:
            maps_with_offsets.append((map1, robot1_pos))
        if map2 is not None:
            maps_with_offsets.append((map2, robot2_pos))

        if not maps_with_offsets:
            # Return empty grid
            merged = OccupancyGrid()
            merged.info.resolution = resolution
            return merged

        # Compute global bounding box from actual map extents
        global_min_x = float('inf')
        global_min_y = float('inf')
        global_max_x = float('-inf')
        global_max_y = float('-inf')

        for src_map, robot_offset in maps_with_offsets:
            src_origin_x = src_map.info.origin.position.x + robot_offset[0]
            src_origin_y = src_map.info.origin.position.y + robot_offset[1]
            src_max_x = src_origin_x + src_map.info.width * src_map.info.resolution
            src_max_y = src_origin_y + src_map.info.height * src_map.info.resolution

            global_min_x = min(global_min_x, src_origin_x)
            global_min_y = min(global_min_y, src_origin_y)
            global_max_x = max(global_max_x, src_max_x)
            global_max_y = max(global_max_y, src_max_y)

        canvas_width = int(np.ceil((global_max_x - global_min_x) / resolution))
        canvas_height = int(np.ceil((global_max_y - global_min_y) / resolution))

        # Clamp to reasonable size to prevent memory issues
        canvas_width = min(canvas_width, 4000)
        canvas_height = min(canvas_height, 4000)

        if canvas_width <= 0 or canvas_height <= 0:
            merged = OccupancyGrid()
            merged.info.resolution = resolution
            return merged

        # Initialize canvas (unknown = -1)
        canvas = np.full((canvas_height, canvas_width), -1, dtype=np.int8)

        # Place each map on canvas
        for src_map, robot_offset in maps_with_offsets:
            MapMerger._place_map_on_canvas(
                canvas, src_map, robot_offset,
                (global_min_x, global_min_y), resolution
            )

        # Create merged occupancy grid
        merged = OccupancyGrid()
        merged.info.resolution = resolution
        merged.info.width = canvas_width
        merged.info.height = canvas_height
        merged.info.origin.position.x = global_min_x
        merged.info.origin.position.y = global_min_y
        merged.info.origin.position.z = 0.0
        merged.data = canvas.flatten().tolist()

        return merged

    @staticmethod
    def _place_map_on_canvas(
        canvas: np.ndarray,
        src_map: OccupancyGrid,
        robot_offset: tuple,
        canvas_origin: tuple,
        resolution: float
    ):
        """
        Place source map on canvas at robot offset.
        Uses cell-center coordinate conversion.
        Conflict rule: occupied > free > unknown.
        """
        src_data = np.array(src_map.data, dtype=np.int8)
        src_grid = src_data.reshape((src_map.info.height, src_map.info.width))

        src_resolution = src_map.info.resolution
        src_origin_x = src_map.info.origin.position.x + robot_offset[0]
        src_origin_y = src_map.info.origin.position.y + robot_offset[1]

        canvas_origin_x, canvas_origin_y = canvas_origin

        # Vectorized coordinate computation
        cols = np.arange(src_map.info.width)
        rows = np.arange(src_map.info.height)
        col_grid, row_grid = np.meshgrid(cols, rows)

        local_x = src_origin_x + (col_grid + 0.5) * src_resolution
        local_y = src_origin_y + (row_grid + 0.5) * src_resolution

        canvas_col = ((local_x - canvas_origin_x) / resolution).astype(np.int32)
        canvas_row = ((local_y - canvas_origin_y) / resolution).astype(np.int32)

        valid_mask = (
            (canvas_col >= 0) & (canvas_col < canvas.shape[1]) &
            (canvas_row >= 0) & (canvas_row < canvas.shape[0])
        )

        # Flatten arrays for boolean indexing / assignments
        c_rows = canvas_row[valid_mask]
        c_cols = canvas_col[valid_mask]
        s_vals = src_grid[valid_mask]
        c_vals = canvas[c_rows, c_cols]

        new_vals = c_vals.copy()

        # Rule 1: occupied wins over free/unknown (src_value > 50)
        occupied_mask = s_vals > 50
        new_vals[occupied_mask] = s_vals[occupied_mask]

        # Rule 2: free wins over unknown (0 <= src_value <= 50 and canvas_value < 0)
        free_over_unk_mask = (s_vals >= 0) & (s_vals <= 50) & (c_vals < 0)
        new_vals[free_over_unk_mask] = s_vals[free_over_unk_mask]

        # Rule 3: both free, keep lower value (more free)
        both_free_mask = (s_vals >= 0) & (s_vals <= 50) & (c_vals >= 0) & (c_vals <= 50)
        new_vals[both_free_mask] = np.minimum(s_vals[both_free_mask], c_vals[both_free_mask])

        canvas[c_rows, c_cols] = new_vals


def main(args=None):
    rclpy.init(args=args)
    node = MapMerger()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
