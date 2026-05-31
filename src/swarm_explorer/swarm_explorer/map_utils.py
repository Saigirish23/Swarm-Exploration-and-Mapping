"""Utility functions for occupancy grid manipulation and analysis."""

import numpy as np
from nav_msgs.msg import OccupancyGrid


def grid_to_world(occupancy_grid: OccupancyGrid, gx: int, gy: int) -> tuple:
    """
    Convert grid cell (gx, gy) to world coordinates.
    
    Args:
        occupancy_grid: OccupancyGrid message
        gx: grid x index
        gy: grid y index
    
    Returns:
        (wx, wy) world coordinates
    """
    resolution = occupancy_grid.info.resolution
    origin_x = occupancy_grid.info.origin.position.x
    origin_y = occupancy_grid.info.origin.position.y
    
    wx = origin_x + gx * resolution
    wy = origin_y + gy * resolution
    
    return (wx, wy)


def world_to_grid(occupancy_grid: OccupancyGrid, wx: float, wy: float) -> tuple:
    """
    Convert world coordinates (wx, wy) to grid cell indices.
    
    Args:
        occupancy_grid: OccupancyGrid message
        wx: world x coordinate
        wy: world y coordinate
    
    Returns:
        (gx, gy) grid indices, or None if out of bounds
    """
    resolution = occupancy_grid.info.resolution
    origin_x = occupancy_grid.info.origin.position.x
    origin_y = occupancy_grid.info.origin.position.y
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    
    gx = int((wx - origin_x) / resolution)
    gy = int((wy - origin_y) / resolution)
    
    if 0 <= gx < width and 0 <= gy < height:
        return (gx, gy)
    return None


def is_free(value: int) -> bool:
    """Check if cell value represents free space."""
    v = int(value)
    return 0 <= v <= 90

def is_unknown(value: int) -> bool:
    """Check if cell value represents unknown space."""
    return int(value) == -1


def is_occupied(value: int) -> bool:
    """Check if cell value represents occupied space."""
    v = int(value)
    return v > 90


def euclidean_distance(p1: tuple, p2: tuple) -> float:
    """Calculate Euclidean distance between two 2D points."""
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5


def get_neighbors_8(x: int, y: int, width: int, height: int) -> list:
    """
    Get 8-connected neighbors of grid cell (x, y).
    
    Args:
        x, y: grid cell indices
        width, height: grid dimensions
    
    Returns:
        List of valid neighbor coordinates (x, y)
    """
    neighbors = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0:
                continue
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                neighbors.append((nx, ny))
    return neighbors


def get_grid_array(occupancy_grid: OccupancyGrid) -> np.ndarray:
    """
    Convert OccupancyGrid data to 2D numpy array.
    
    Args:
        occupancy_grid: OccupancyGrid message
    
    Returns:
        2D numpy array with shape (height, width)
    """
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    data = np.array(occupancy_grid.data, dtype=np.int8)
    grid = data.reshape((height, width))
    return grid


def count_cell_types(grid: np.ndarray) -> tuple:
    """Count free, unknown, and occupied cells in a grid."""
    free_count = 0
    unknown_count = 0
    occupied_count = 0

    height, width = grid.shape
    for y in range(height):
        for x in range(width):
            val = int(grid[y, x])
            if is_unknown(val):
                unknown_count += 1
            elif is_free(val):
                free_count += 1
            elif is_occupied(val):
                occupied_count += 1

    return free_count, unknown_count, occupied_count
