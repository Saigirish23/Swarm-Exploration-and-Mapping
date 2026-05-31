"""
Replacement for cartographer_occupancy_grid_node.

Queries Cartographer submaps and publishes a combined OccupancyGrid.
Uses wall-clock timer to avoid the sim-time timer bug in Humble.
"""

import gzip
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid
from cartographer_ros_msgs.msg import SubmapList
from cartographer_ros_msgs.srv import SubmapQuery


class SubmapToMapNode(Node):
    """Query Cartographer submaps and publish OccupancyGrid."""

    def __init__(self):
        super().__init__('submap_to_map_node')

        self.declare_parameter('robot_ns', 'robot1')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('map_frame', '')  # defaults to {robot_ns}/map
        self.declare_parameter('map_padding', 2.0)
        # Cartographer occupancy thresholds (on the 0-100 probability scale)
        # After converting raw intensity to probability via (1 - intensity/255)*100:
        #   occupied_thresh: cells with probability >= this are occupied (default 55)
        #   free_thresh: cells with probability <= this are free (default 19.6)
        self.declare_parameter('occupied_threshold', 55)   # probability >= this → occupied (100)
        self.declare_parameter('free_threshold', 20)       # probability <= this → free (0)
        self.declare_parameter('speckle_filter_enabled', True)
        self.declare_parameter('speckle_max_size', 4)

        self.robot_ns = self.get_parameter('robot_ns').value
        self.resolution = self.get_parameter('resolution').value
        period = self.get_parameter('publish_period_sec').value
        map_frame_param = self.get_parameter('map_frame').value
        self.map_padding = self.get_parameter('map_padding').value
        self.map_frame = map_frame_param if map_frame_param else f'{self.robot_ns}/map'
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.free_threshold = self.get_parameter('free_threshold').value
        self.speckle_filter_enabled = self.get_parameter('speckle_filter_enabled').value
        self.speckle_max_size = self.get_parameter('speckle_max_size').value
        self.publish_count = 0  # Track publishes for debug logging

        # Topics / services
        submap_list_topic = f'/{self.robot_ns}/submap_list'
        submap_query_srv = f'/{self.robot_ns}/submap_query'
        map_topic = f'/{self.robot_ns}/map'

        # QoS for map publisher: RELIABLE + TRANSIENT_LOCAL so late
        # subscribers (Nav2 static_layer, explorers) receive the last map.
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        # Subscriber: submap_list (default QoS, RELIABLE)
        self.submap_list_sub = self.create_subscription(
            SubmapList, submap_list_topic, self._on_submap_list, 10
        )

        # Service client
        self.query_client = self.create_client(SubmapQuery, submap_query_srv)

        # Publisher
        self.map_pub = self.create_publisher(OccupancyGrid, map_topic, map_qos)

        # State
        self.latest_submap_list = None
        self.pending_request = False  # guard against overlapping calls
        self.submap_cache = {}  # (traj_id, submap_index) -> (version, texture_data)

        self.create_timer(period, self._timer_cb)

        self.get_logger().info(
            f'SubmapToMapNode [{self.robot_ns}] started. '
            f'Subscribing to {submap_list_topic}, '
            f'querying {submap_query_srv}, '
            f'publishing {map_topic} at {1.0/period:.1f} Hz '
            f'(occupied_thresh={self.occupied_threshold}, '
            f'free_thresh={self.free_threshold}, '
            f'speckle_filter={self.speckle_filter_enabled}, '
            f'speckle_max_size={self.speckle_max_size})'
        )

    # ── callbacks ───────────────────────────────────────────────

    def _on_submap_list(self, msg: SubmapList):
        self.latest_submap_list = msg

    def _timer_cb(self):
        """Periodically query ALL submaps and build a merged map."""
        if self.latest_submap_list is None:
            return
        if not self.latest_submap_list.submap:
            return

        # 1. Update the poses of all cached submaps instantly with the latest optimized poses!
        for entry in self.latest_submap_list.submap:
            key = (entry.trajectory_id, entry.submap_index)
            cached = self.submap_cache.get(key)
            if cached is not None:
                # Retain the cached version and texture data, but update with the latest optimized entry pose
                self.submap_cache[key] = (cached[0], cached[1], entry)

        # 2. Always publish the map at a steady, reliable 1.0 Hz using current cache contents!
        # This completely eliminates any map delay, lag, or displacement while queries are in-flight.
        self._build_and_publish_map()

        # 3. Handle background texture queries asynchronously without blocking the publish loop
        if self.pending_request:
            return
        if not self.query_client.service_is_ready():
            return

        # Determine which submaps need a texture/version update
        submaps_to_query = []
        for entry in self.latest_submap_list.submap:
            key = (entry.trajectory_id, entry.submap_index)
            cached = self.submap_cache.get(key)
            if cached is None or cached[0] < entry.submap_version:
                submaps_to_query.append(entry)

        if submaps_to_query:
            # Query the submaps in the background
            self._query_next_submap(submaps_to_query, 0)

    def _query_next_submap(self, entries, idx):
        """Chain async submap queries one at a time."""
        if idx >= len(entries):
            # All background queries finished! The newly cached textures
            # will automatically be rendered in the very next 1.0 Hz timer tick.
            return

        entry = entries[idx]
        req = SubmapQuery.Request()
        req.trajectory_id = entry.trajectory_id
        req.submap_index = entry.submap_index

        self.pending_request = True
        future = self.query_client.call_async(req)
        future.add_done_callback(
            lambda f: self._on_submap_response(f, entry, entries, idx)
        )

    def _on_submap_response(self, future, entry, entries, idx):
        """Handle a single SubmapQuery response."""
        self.pending_request = False
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warn(f'SubmapQuery failed: {e}')
            return

        if resp.status.code != 0:
            self.get_logger().warn(
                f'SubmapQuery error code {resp.status.code}: {resp.status.message}'
            )
            return

        if not resp.textures:
            self.get_logger().debug(
                f'No textures for submap {entry.submap_index}'
            )
            return

        # Cache the response with its version
        key = (entry.trajectory_id, entry.submap_index)
        self.submap_cache[key] = (resp.submap_version, resp.textures, entry)

        # Query next
        self._query_next_submap(entries, idx + 1)

    # ── pixel → occupancy conversion ───────────────────────────

    def _pixel_to_occupancy(self, intensity, alpha):
        """Convert Cartographer raw texture pixel to OccupancyGrid value.

        Cartographer's ProbabilityGrid::DrawToSubmapTexture encodes cells as:
          intensity > 0, alpha == 0  → free (intensity encodes free certainty)
          intensity == 0, alpha > 0  → occupied (alpha encodes occupied certainty)

        We map these back to a 0-100 probability scale so we can apply the
        user-defined occupied_threshold (e.g. 55) and free_threshold (e.g. 20).
        This filters out low-confidence sensor noise as "unknown".
        """
        # Unobserved pixel
        if intensity == 0 and (alpha is None or alpha == 0):
            return -1

        if alpha is not None and alpha > 0:
            # Map alpha [1..127] to probability [50..100]
            prob = 50.0 + (alpha / 127.0) * 50.0
        elif intensity > 0:
            # Map intensity [1..127] to probability [0..50]
            prob = 50.0 - (intensity / 127.0) * 50.0
        else:
            prob = 50.0

        if prob >= self.occupied_threshold:
            return 100
        elif prob <= self.free_threshold:
            return 0
        else:
            return -1

    def _vectorized_pixel_to_occupancy(self, intensity_array, alpha_array):
        """Vectorized version of _pixel_to_occupancy for entire submap."""
        h, w = intensity_array.shape
        
        # Start with default unknown probability of 50
        probs = np.full((h, w), 50.0, dtype=np.float32)

        if alpha_array is not None:
            # Occupied pixels: map alpha [1..127] to [50..100]
            occ_mask = alpha_array > 0
            probs[occ_mask] = 50.0 + (alpha_array[occ_mask].astype(np.float32) / 127.0) * 50.0
            
            # Free pixels: map intensity [1..127] to [0..50]
            free_mask = (intensity_array > 0) & (alpha_array == 0)
            probs[free_mask] = 50.0 - (intensity_array[free_mask].astype(np.float32) / 127.0) * 50.0
        else:
            # Without alpha, any intensity > 0 is treated as free
            free_mask = intensity_array > 0
            probs[free_mask] = 50.0 - (intensity_array[free_mask].astype(np.float32) / 127.0) * 50.0

        occ_grid = np.full((h, w), -1, dtype=np.int8)
        occ_grid[probs >= self.occupied_threshold] = 100
        occ_grid[probs <= self.free_threshold] = 0

        return occ_grid

    # ── map building ───────────────────────────────────────────

    def _build_and_publish_map(self):
        """Build OccupancyGrid from all cached submaps and publish."""
        if not self.submap_cache:
            return

        all_grids = []

        for key, (version, textures, entry) in self.submap_cache.items():
            tex = textures[0]  # Use first (finest) texture
            cells_raw = bytes(tex.cells)

            # Decompress gzip if needed
            if len(cells_raw) >= 2 and cells_raw[0] == 0x1f and cells_raw[1] == 0x8b:
                try:
                    cells_raw = gzip.decompress(cells_raw)
                except Exception as e:
                    self.get_logger().warn(f'gzip decompress failed: {e}')
                    continue

            width = tex.width
            height = tex.height
            resolution = tex.resolution

            if len(cells_raw) < width * height:
                self.get_logger().warn(
                    f'Submap {key}: expected {width*height} bytes, '
                    f'got {len(cells_raw)}'
                )
                continue

            # Cartographer texture: usually pairs of (intensity, alpha) bytes.
            # If we have width*height*2 bytes, use intensity.
            if len(cells_raw) == width * height * 2:
                pixels = np.frombuffer(cells_raw, dtype=np.uint8).reshape((height, width, 2))
                intensity_array = pixels[:, :, 0]
                alpha_array = pixels[:, :, 1]
            elif len(cells_raw) == width * height:
                intensity_array = np.frombuffer(cells_raw, dtype=np.uint8).reshape((height, width))
                alpha_array = None
            else:
                self.get_logger().warn(
                    f'Submap {key}: unexpected data size {len(cells_raw)} '
                    f'for {width}x{height}'
                )
                continue

            # Get submap pose from the SubmapList entry (submap-local → map)
            pose = entry.pose
            ox = pose.position.x
            oy = pose.position.y
            q = pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )

            # Get slice_pose from the texture (texture-local → submap-local)
            sp = tex.slice_pose
            sp_ox = sp.position.x
            sp_oy = sp.position.y
            sq = sp.orientation
            sp_yaw = math.atan2(
                2.0 * (sq.w * sq.z + sq.x * sq.y),
                1.0 - 2.0 * (sq.y * sq.y + sq.z * sq.z),
            )

            all_grids.append((intensity_array, alpha_array, width, height, resolution,
                              ox, oy, yaw, sp_ox, sp_oy, sp_yaw))

        if not all_grids:
            return

        # ── Merge all submaps into one OccupancyGrid ──
        # Compute bounding box in world coordinates
        world_min_x = float('inf')
        world_min_y = float('inf')
        world_max_x = float('-inf')
        world_max_y = float('-inf')

        for (intensity_array, alpha_array, w, h, res, ox, oy, yaw,
             sp_ox, sp_oy, sp_yaw) in all_grids:
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            sp_cos = math.cos(sp_yaw)
            sp_sin = math.sin(sp_yaw)

            # Corners in texture space (row-major: X is row, Y is col)
            # cx corresponds to row dimension, cy corresponds to col dimension
            corners = [
                (0, 0), (-h * res, 0), (0, -w * res), (-h * res, -w * res)
            ]
            for cx, cy in corners:
                # texture-local → submap-local
                sx = sp_ox + sp_cos * cx - sp_sin * cy
                sy = sp_oy + sp_sin * cx + sp_cos * cy
                # submap-local → world (map) frame
                wx = ox + cos_y * sx - sin_y * sy
                wy = oy + sin_y * sx + cos_y * sy
                world_min_x = min(world_min_x, wx)
                world_min_y = min(world_min_y, wy)
                world_max_x = max(world_max_x, wx)
                world_max_y = max(world_max_y, wy)

        if self.map_padding > 0.0:
            world_min_x -= self.map_padding
            world_min_y -= self.map_padding
            world_max_x += self.map_padding
            world_max_y += self.map_padding

        # Use the configured resolution for the output map
        out_res = self.resolution
        out_width = int(math.ceil((world_max_x - world_min_x) / out_res)) + 1
        out_height = int(math.ceil((world_max_y - world_min_y) / out_res)) + 1

        # Clamp to avoid absurd sizes
        out_width = min(out_width, 4000)
        out_height = min(out_height, 4000)

        if out_width <= 0 or out_height <= 0:
            return
        # Canvas: -1 = unknown
        canvas = np.full((out_height, out_width), -1, dtype=np.int8)

        # Iterate newest submaps first so their observations take priority.
        for (intensity_array, alpha_array, w, h, res, ox, oy, yaw,
             sp_ox, sp_oy, sp_yaw) in reversed(all_grids):
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            sp_cos = math.cos(sp_yaw)
            sp_sin = math.sin(sp_yaw)

            # Vectorized occupancy conversion for the entire submap
            occ_grid = self._vectorized_pixel_to_occupancy(intensity_array, alpha_array)

            # Find all known (non-unknown) pixels
            known_mask = (occ_grid != -1)
            known_rows, known_cols = np.where(known_mask)

            if len(known_rows) == 0:
                continue

            # Draw this submap onto a clean local canvas
            sub_canvas = np.full((out_height, out_width), -1, dtype=np.int8)

            # Vectorized coordinate transform for all known pixels at once
            # Cartographer grid: X decreasing with row, Y decreasing with col
            tx = -known_rows.astype(np.float64) * res
            ty = -known_cols.astype(np.float64) * res

            # texture-local → submap-local
            sx = sp_ox + sp_cos * tx - sp_sin * ty
            sy = sp_oy + sp_sin * tx + sp_cos * ty

            # submap-local → world (map) frame
            wx = ox + cos_y * sx - sin_y * sy
            wy = oy + sin_y * sx + cos_y * sy

            # World to canvas coordinates
            cx = ((wx - world_min_x) / out_res).astype(np.int32)
            cy = ((wy - world_min_y) / out_res).astype(np.int32)

            # Filter to valid canvas bounds
            valid = (cx >= 0) & (cx < out_width) & (cy >= 0) & (cy < out_height)
            cx_v = cx[valid]
            cy_v = cy[valid]
            occ_v = occ_grid[known_rows[valid], known_cols[valid]]

            # Write to sub_canvas
            sub_canvas[cy_v, cx_v] = occ_v

            # Fill forward-mapping holes in the sub_canvas with 0 (free)
            # Only fill -1 holes that are internal (surrounded by at least 2 free neighbors)
            # This prevents older submaps from leaking '100' (black) into these aliasing holes.
            holes = (sub_canvas == -1)
            if np.any(holes):
                has_free_neighbor = np.zeros(holes.shape, dtype=np.int8)
                has_free_neighbor[:-1, :] += (sub_canvas[1:, :] == 0)
                has_free_neighbor[1:, :] += (sub_canvas[:-1, :] == 0)
                has_free_neighbor[:, :-1] += (sub_canvas[:, 1:] == 0)
                has_free_neighbor[:, 1:] += (sub_canvas[:, :-1] == 0)
                sub_canvas[holes & (has_free_neighbor >= 2)] = 0

            # Composite onto the global canvas without letting older submaps overwrite
            # Since we iterate newest to oldest, we only write where global canvas is still -1
            mask = (sub_canvas != -1) & (canvas == -1)
            canvas[mask] = sub_canvas[mask]

        speckle_removed = 0
        if self.speckle_filter_enabled:
            speckle_removed = self._despeckle(canvas)

        # Build the OccupancyGrid message
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = out_res
        msg.info.width = out_width
        msg.info.height = out_height
        msg.info.origin.position.x = world_min_x
        msg.info.origin.position.y = world_min_y
        msg.info.origin.position.z = 0.0
        msg.data = canvas.flatten().tolist()

        self.map_pub.publish(msg)
        self.publish_count += 1

        # Log stats on first few publishes and then every 10th
        if self.publish_count <= 5 or self.publish_count % 10 == 0:
            flat = canvas.flatten()
            total = len(flat)
            n_free = int(np.sum(flat == 0))
            n_unknown = int(np.sum(flat == -1))
            n_occupied = int(np.sum(flat == 100))
            self.get_logger().info(
                f'Map #{self.publish_count}: '
                f'free={n_free} ({100*n_free/max(total,1):.1f}%), '
                f'occupied={n_occupied} ({100*n_occupied/max(total,1):.1f}%), '
                f'unknown={n_unknown} ({100*n_unknown/max(total,1):.1f}%), '
                f'total={total}, speckle_removed={speckle_removed}, '
                f'submaps={len(self.submap_cache)}, '
                f'size={out_width}x{out_height}'
            )

    def _despeckle(self, grid: np.ndarray) -> int:
        """Remove small occupied clusters that appear as speckle noise."""
        max_size = int(self.speckle_max_size)
        if max_size <= 0:
            return 0

        height, width = grid.shape
        visited = np.zeros((height, width), dtype=bool)
        removed = 0

        for y in range(height):
            for x in range(width):
                if grid[y, x] != 100 or visited[y, x]:
                    continue

                queue = deque([(x, y)])
                visited[y, x] = True
                cluster = [(x, y)]

                while queue:
                    cx, cy = queue.popleft()
                    for dx in (-1, 0, 1):
                        for dy in (-1, 0, 1):
                            if dx == 0 and dy == 0:
                                continue
                            nx = cx + dx
                            ny = cy + dy
                            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                                continue
                            if visited[ny, nx] or grid[ny, nx] != 100:
                                continue
                            visited[ny, nx] = True
                            queue.append((nx, ny))
                            cluster.append((nx, ny))

                if len(cluster) <= max_size:
                    for cx, cy in cluster:
                        grid[cy, cx] = 0
                    removed += len(cluster)

        return removed


def main(args=None):
    rclpy.init(args=args)
    node = SubmapToMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
