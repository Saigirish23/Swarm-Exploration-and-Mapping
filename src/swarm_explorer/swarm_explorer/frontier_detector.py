"""Frontier cell detection and clustering."""

from collections import deque
import numpy as np
import rclpy.logging
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from .map_utils import (
    count_cell_types,
    get_grid_array,
    get_neighbors_8,
    grid_to_world,
    is_free,
    is_unknown,
)


def _nearest_free_cell(
    grid: np.ndarray,
    start_x: int,
    start_y: int,
    max_radius: int = 10
) -> tuple:
    """Find the nearest free cell around a start cell within max_radius."""
    height, width = grid.shape
    if 0 <= start_x < width and 0 <= start_y < height:
        if is_free(grid[start_y, start_x]):
            return (start_x, start_y)

    visited = set()
    queue = deque([(start_x, start_y, 0)])
    visited.add((start_x, start_y))

    while queue:
        x, y, dist = queue.popleft()
        if dist >= max_radius:
            continue
        for nx, ny in get_neighbors_8(x, y, width, height):
            if (nx, ny) in visited:
                continue
            visited.add((nx, ny))
            if is_free(grid[ny, nx]):
                return (nx, ny)
            queue.append((nx, ny, dist + 1))

    return None


def detect_frontiers(
    occupancy_grid: OccupancyGrid,
    min_frontier_size: int = 1,
    treat_out_of_bounds_as_unknown: bool = True
) -> list:
    grid = get_grid_array(occupancy_grid)
    unique_vals = np.unique(grid)

    logger = rclpy.logging.get_logger("frontier_detector")
    logger.info(f"[FrontierDetector] unique map values: {unique_vals[:30]}")

    height, width = grid.shape

    free_count, unknown_count, occupied_count = count_cell_types(grid)
    frontier_cells = set()

    for y in range(height):
        for x in range(width):
            if not is_free(grid[y, x]):
                continue

            has_unknown_neighbor = False
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx = x + dx
                    ny = y + dy
                    if nx < 0 or nx >= width or ny < 0 or ny >= height:
                        if treat_out_of_bounds_as_unknown:
                            has_unknown_neighbor = True
                            break
                        continue
                    if is_unknown(grid[ny, nx]):
                        has_unknown_neighbor = True
                        break
                if has_unknown_neighbor:
                    break

            if has_unknown_neighbor:
                frontier_cells.add((x, y))

    logger.info(
        f"[FrontierDetector] free={free_count}, "
        f"unknown={unknown_count}, occupied={occupied_count}, "
        f"raw_frontiers={len(frontier_cells)}"
    )

    if not frontier_cells:
        return []

    visited = set()
    clusters = []

    for start_cell in frontier_cells:
        if start_cell in visited:
            continue

        cluster = []
        queue = deque([start_cell])
        visited.add(start_cell)

        while queue:
            x, y = queue.popleft()
            cluster.append((x, y))

            for nx, ny in get_neighbors_8(x, y, width, height):
                if (nx, ny) in frontier_cells and (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny))

        if len(cluster) >= min_frontier_size:
            clusters.append(cluster)

    logger.info(f"[FrontierDetector] clusters={len(clusters)}")

    centroids = []
    for cluster in clusters:
        cx = sum(c[0] for c in cluster) / len(cluster)
        cy = sum(c[1] for c in cluster) / len(cluster)
        free_cell = _nearest_free_cell(grid, int(cx), int(cy), max_radius=12)
        if free_cell is None:
            continue
        wx, wy = grid_to_world(occupancy_grid, free_cell[0], free_cell[1])
        centroids.append(((wx, wy), len(cluster)))

    if centroids:
        preview = ", ".join(
            f"({c[0][0]:.2f},{c[0][1]:.2f})" for c in centroids[:3]
        )
        logger.info(
            f"[FrontierDetector] selected_centroids={preview}"
        )

    return centroids


def create_frontier_markers(
    frontiers: list,
    map_frame: str,
    stamp: int
) -> MarkerArray:
    """
    Create RViz markers for frontier visualization.
    
    Args:
        frontiers: list of (world_pos, size) tuples
        map_frame: frame ID (e.g., 'robot1/map')
        stamp: ROS timestamp
    
    Returns:
        MarkerArray message
    """
    marker_array = MarkerArray()

    # Clear all previous markers first
    delete_marker = Marker()
    delete_marker.header.frame_id = map_frame
    delete_marker.header.stamp = stamp
    delete_marker.action = Marker.DELETEALL
    marker_array.markers.append(delete_marker)

    for i, (pos, size) in enumerate(frontiers):
        marker = Marker()
        marker.header.frame_id = map_frame
        marker.header.stamp = stamp
        marker.id = i + 1  # Start from 1 since 0 is DELETEALL
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = 0.1

        # Scale proportional to frontier size (clamped)
        scale = min(0.1 + size * 0.02, 0.5)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        marker_array.markers.append(marker)
    
    return marker_array
