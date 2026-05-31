"""Frontier cell detection and clustering."""

from collections import deque
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from .map_utils import (
    get_grid_array, get_neighbors_8, is_free, is_unknown,
    grid_to_world
)


def detect_frontiers(occupancy_grid: OccupancyGrid, min_frontier_size: int = 3) -> list:
    """
    Detect frontier cells (free cells adjacent to unknown cells).
    Cluster them and return centroids in world coordinates.
    
    Args:
        occupancy_grid: OccupancyGrid message
        min_frontier_size: minimum frontier cluster size (discard smaller)
    
    Returns:
        List of frontier centroids as (wx, wy) tuples
    """
    grid = get_grid_array(occupancy_grid)
    height, width = grid.shape
    
    # Find frontier cells
    frontier_cells = set()
    for y in range(height):
        for x in range(width):
            if is_free(grid[y, x]):
                for nx, ny in get_neighbors_8(x, y, width, height):
                    if is_unknown(grid[ny, nx]):
                        frontier_cells.add((x, y))
                        break
    
    if not frontier_cells:
        return []
    
    # Cluster frontier cells using BFS
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
    
    # Compute centroids in world coordinates
    centroids = []
    for cluster in clusters:
        cx = sum(c[0] for c in cluster) / len(cluster)
        cy = sum(c[1] for c in cluster) / len(cluster)
        wx, wy = grid_to_world(occupancy_grid, int(cx), int(cy))
        centroids.append(((wx, wy), len(cluster)))
    
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
    
    for i, (pos, size) in enumerate(frontiers):
        marker = Marker()
        marker.header.frame_id = map_frame
        marker.header.stamp = stamp
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = 0.1
        
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        marker_array.markers.append(marker)
    
    return marker_array
