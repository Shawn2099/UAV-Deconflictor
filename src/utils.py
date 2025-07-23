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
    based on a constant drone speed, using a vectorized approach.

    Args:
        mission_waypoints: A list of Waypoint objects.
        start_time: The starting time of the mission in seconds.
        drone_speed_mps: The constant speed of the drone in meters per second.

    Returns:
        A list of floats representing the ETA at each waypoint.

    Raises:
        ValueError: If drone_speed_mps is not a positive number.
    """
    # --- Input Validation (Tier 1 Change) ---
    # This is a critical fix. A non-positive speed is a physical impossibility
    # that must be flagged as an error, not handled silently.
    if drone_speed_mps <= 0:
        raise ValueError("Drone speed must be a positive number.")

    # --- Edge Case Handling ---
    if not mission_waypoints:
        return []
    if len(mission_waypoints) == 1:
        return [float(start_time)]

    # --- Vectorized Calculation (Tier 2 Change) ---
    # This replaces the inefficient for-loop with a highly optimized NumPy
    # implementation for better performance.
    
    # 1. Convert all waypoints to a single NumPy array.
    coords = np.array([[wp.x, wp.y, wp.z] for wp in mission_waypoints])

    # 2. Calculate vectors between consecutive points (e.g., p2-p1, p3-p2, ...).
    vectors = np.diff(coords, axis=0)

    # 3. Calculate the Euclidean distance (L2 norm) of all segment vectors at once.
    segment_distances = np.linalg.norm(vectors, axis=1)

    # 4. Calculate the travel time for each segment.
    time_for_segments = segment_distances / drone_speed_mps

    # 5. Calculate the cumulative sum of travel times to get the ETA relative to mission start.
    #    We prepend 0 to represent the zero travel time to the first waypoint.
    cumulative_travel_time = np.cumsum(np.insert(time_for_segments, 0, 0))

    # 6. Add the mission start_time to get the final absolute ETAs.
    etas = start_time + cumulative_travel_time
    
    return etas.tolist()


