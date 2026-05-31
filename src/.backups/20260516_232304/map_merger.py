"""Known-initial-pose occupancy grid merger for dual robots."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
import threading


class MapMerger(Node):
    """Merge occupancy grids from two robots assuming known initial poses."""
    
    def __init__(self):
        super().__init__('map_merger')
        
        # Parameters
        self.declare_parameter('map1_topic', '/robot1/map')
        self.declare_parameter('map2_topic', '/robot2/map')
        self.declare_parameter('merged_map_topic', '/merged_map')
        self.declare_parameter('robot1_initial_x', -2.0)
        self.declare_parameter('robot1_initial_y', -2.0)
        self.declare_parameter('robot1_initial_yaw', 0.0)
        self.declare_parameter('robot2_initial_x', 2.0)
        self.declare_parameter('robot2_initial_y', -2.0)
        self.declare_parameter('robot2_initial_yaw', 0.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('merged_frame', 'map')
        self.declare_parameter('use_dynamic_origin', False)
        
        self.map1_topic = self.get_parameter('map1_topic').value
        self.map2_topic = self.get_parameter('map2_topic').value
        self.merged_map_topic = self.get_parameter('merged_map_topic').value
        self.robot1_pos = (
            self.get_parameter('robot1_initial_x').value,
            self.get_parameter('robot1_initial_y').value,
        )
        self.robot2_pos = (
            self.get_parameter('robot2_initial_x').value,
            self.get_parameter('robot2_initial_y').value,
        )
        self.resolution = self.get_parameter('resolution').value
        self.merged_frame = self.get_parameter('merged_frame').value
        self.use_dynamic_origin = self.get_parameter('use_dynamic_origin').value
        
        # QoS profile for maps (transient-local for late joiners)
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
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
        
        self.get_logger().info(
            f'MapMerger initialized. '
            f'Map1: {self.map1_topic}, Map2: {self.map2_topic}, '
            f'Robot1 pos: {self.robot1_pos}, Robot2 pos: {self.robot2_pos}, '
            f'Dynamic origin: {self.use_dynamic_origin}'
        )
    
    def on_map1(self, msg: OccupancyGrid):
        """Callback for robot1 map."""
        with self.lock:
            self.map1 = msg
            self.try_merge()
    
    def on_map2(self, msg: OccupancyGrid):
        """Callback for robot2 map."""
        with self.lock:
            self.map2 = msg
            self.try_merge()
    
    def try_merge(self):
        """Merge maps if both are available."""
        if self.map1 is None or self.map2 is None:
            return
        
        # Use dynamic origins if available and enabled
        robot1_pos = self.robot1_pos
        robot2_pos = self.robot2_pos
        
        if self.use_dynamic_origin:
            # Use map origins if available (for future map-origin discovery)
            # For now, fall back to static initial poses
            pass
        
        merged = self.merge_occupancy_grids(
            self.map1, self.map2,
            robot1_pos, robot2_pos,
            self.resolution
        )
        
        merged.header.frame_id = self.merged_frame
        merged.header.stamp = self.get_clock().now().to_msg()
        
        self.merged_pub.publish(merged)
    
    @staticmethod
    def merge_occupancy_grids(
        map1: OccupancyGrid, map2: OccupancyGrid,
        robot1_pos: tuple, robot2_pos: tuple,
        resolution: float
    ) -> OccupancyGrid:
        """
        Merge two occupancy grids assuming known initial robot poses.
        
        Args:
            map1, map2: OccupancyGrid messages from robots
            robot1_pos, robot2_pos: (x, y) initial positions
            resolution: grid resolution
        
        Returns:
            Merged OccupancyGrid
        """
        # Create a canvas large enough to contain both maps
        margin = 1.0  # margin in meters
        
        # World bounds
        min_x = min(robot1_pos[0], robot2_pos[0]) - margin
        max_x = max(robot1_pos[0], robot2_pos[0]) + margin
        min_y = min(robot1_pos[1], robot2_pos[1]) - margin
        max_y = max(robot1_pos[1], robot2_pos[1]) + margin
        
        canvas_width = int((max_x - min_x) / resolution) + 1
        canvas_height = int((max_y - min_y) / resolution) + 1
        
        # Initialize canvas (unknown = -1)
        canvas = np.full((canvas_height, canvas_width), -1, dtype=np.int8)
        
        # Place map1
        MapMerger._place_map_on_canvas(
            canvas, map1, robot1_pos, (min_x, min_y), resolution
        )
        
        # Place map2 (occupied cells override free, unknown stays unknown)
        MapMerger._place_map_on_canvas(
            canvas, map2, robot2_pos, (min_x, min_y), resolution
        )
        
        # Create merged occupancy grid
        merged = OccupancyGrid()
        merged.info.resolution = resolution
        merged.info.width = canvas_width
        merged.info.height = canvas_height
        merged.info.origin.position.x = min_x
        merged.info.origin.position.y = min_y
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
        Conflict rule: occupied > free > unknown.
        """
        src_data = np.array(src_map.data, dtype=np.int8)
        src_grid = src_data.reshape((src_map.info.height, src_map.info.width))
        
        src_resolution = src_map.info.resolution
        src_origin_x = src_map.info.origin.position.x + robot_offset[0]
        src_origin_y = src_map.info.origin.position.y + robot_offset[1]
        
        canvas_origin_x, canvas_origin_y = canvas_origin
        
        # Iterate over source map cells
        for src_y in range(src_map.info.height):
            for src_x in range(src_map.info.width):
                src_value = src_grid[src_y, src_x]
                
                # Convert source grid coords to world coords
                world_x = src_origin_x + src_x * src_resolution
                world_y = src_origin_y + src_y * src_resolution
                
                # Convert world coords to canvas grid coords
                canvas_x = int((world_x - canvas_origin_x) / resolution)
                canvas_y = int((world_y - canvas_origin_y) / resolution)
                
                # Check bounds
                if 0 <= canvas_x < canvas.shape[1] and 0 <= canvas_y < canvas.shape[0]:
                    canvas_value = canvas[canvas_y, canvas_x]
                    
                    # Merge rule: occupied wins over free, free wins over unknown
                    if src_value > 50:  # occupied
                        canvas[canvas_y, canvas_x] = src_value
                    elif src_value >= 0 and canvas_value < 0:  # free over unknown
                        canvas[canvas_y, canvas_x] = src_value
                    elif src_value >= 0 and src_value < canvas_value:  # more free
                        canvas[canvas_y, canvas_x] = src_value


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
