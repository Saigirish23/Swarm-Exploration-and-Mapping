"""Frontier goal scoring and selection."""

from .map_utils import euclidean_distance


def score_frontiers(
    frontiers: list,
    robot_pos: tuple,
    alpha: float = 1.0,
    info_gain_weight: float = 1.0,
    distance_weight: float = 1.0
) -> list:
    """
    Score frontiers based on information gain and distance.
    
    Args:
        frontiers: list of (world_pos, cluster_size) tuples
        robot_pos: (wx, wy) robot position
        alpha: weight for distance cost (lower = more weight on info gain)
        info_gain_weight: weight for frontier size (information gain proxy)
        distance_weight: weight for distance cost
    
    Returns:
        List of (score, frontier_idx, frontier_data) tuples, sorted by score descending
    """
    if not frontiers:
        return []
    
    scored = []
    for idx, (pos, size) in enumerate(frontiers):
        distance = euclidean_distance(robot_pos, pos)
        
        # Avoid division by zero
        distance = max(distance, 0.1)
        
        # Information gain is proxy for frontier size
        info_gain = size
        
        # Score: maximize information gain, minimize distance
        # score = info_gain - alpha * distance_cost
        distance_cost = distance_weight * distance
        score = info_gain_weight * info_gain - alpha * distance_cost
        
        scored.append((score, idx, (pos, size)))
    
    # Sort by score descending
    scored.sort(key=lambda x: x[0], reverse=True)
    return scored


def select_best_frontier(scored_frontiers: list) -> tuple:
    """
    Select the best frontier from scored list.
    
    Args:
        scored_frontiers: output from score_frontiers()
    
    Returns:
        (frontier_pos, frontier_size) or (None, None) if no frontiers
    """
    if not scored_frontiers:
        return (None, None)
    
    # Best frontier is first (highest score)
    _, _, frontier_data = scored_frontiers[0]
    return frontier_data
