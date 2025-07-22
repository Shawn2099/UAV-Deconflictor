"""
utils.py

This module contains common, reusable helper functions that are used across
the application. By centralizing them here, we ensure they can be
independently tested and consistently applied.
"""

import numpy as np
from typing import List

# Import data models for type hinting
from .data_models import Waypoint

def calculate_etas(mission_waypoints: List[Waypoint], start_time: float, drone_speed_mps: float) -> List[float]:
    """
    Calculates Estimated Times of Arrival (ETAs) for each waypoint in a list,
    based on a constant drone speed.

    Args:
        mission_waypoints: A list of Waypoint objects.
        start_time: The starting time of the mission in seconds.
        drone_speed_mps: The constant speed of the drone in meters per second.

    Returns:
        A list of floats representing the ETA at each waypoint.
    """
    # Handle edge cases for empty or single-point missions
    if not mission_waypoints:
        return []
    if len(mission_waypoints) == 1:
        return [float(start_time)]
        
    # Handle edge case for zero or negative speed
    if drone_speed_mps <= 0:
        return [float(start_time)] * len(mission_waypoints)

    # Calculate ETAs for each segment
    etas = [float(start_time)]
    for p1, p2 in zip(mission_waypoints, mission_waypoints[1:]):
        p1_np = np.array([p1.x, p1.y, p1.z])
        p2_np = np.array([p2.x, p2.y, p2.z])
        segment_dist = np.linalg.norm(p2_np - p1_np)
        
        time_for_segment = segment_dist / drone_speed_mps
        etas.append(etas[-1] + time_for_segment)
        
    return etas
