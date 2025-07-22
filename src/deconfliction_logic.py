"""
deconfliction_logic.py

This module contains the core logic for detecting spatio-temporal conflicts
between a primary drone mission and a set of simulated drone flights.

[CORRECTED VERSION]
"""

import math
import numpy as np
from typing import List, Dict, Any

# Import our custom data models from the sibling file
from .data_models import PrimaryMission, SimulatedFlight, Waypoint

def get_closest_distance_between_segments(p1: Waypoint, p2: Waypoint, q1: Waypoint, q2: Waypoint) -> float:
    """
    Calculates the shortest distance between two 2D line segments (p1-p2 and q1-q2).
    
    This is a robust implementation based on vector projections. It correctly
    handles all cases, including intersecting, parallel, and collinear segments.

    The algorithm finds the points on the infinite lines that are closest to each other.
    It then clamps these points to the boundaries of the line segments.
    
    Args:
        p1, p2: Waypoints defining the first segment.
        q1, q2: Waypoints defining the second segment.

    Returns:
        The minimum distance between the two segments.
    """
    # Convert waypoints to numpy arrays for vector operations
    p1_np = np.array([p1.x, p1.y])
    p2_np = np.array([p2.x, p2.y])
    q1_np = np.array([q1.x, q1.y])
    q2_np = np.array([q2.x, q2.y])

    # Define the segment vectors
    d1 = p2_np - p1_np
    d2 = q2_np - q1_np
    r = p1_np - q1_np

    a = np.dot(d1, d1)  # Squared length of segment 1
    b = np.dot(d1, d2)
    c = np.dot(d2, d2)  # Squared length of segment 2
    e = np.dot(d1, r)
    f = np.dot(d2, r)

    # Check if either or both segments are points
    if a == 0 and c == 0:
        return np.linalg.norm(p1_np - q1_np)
    if a == 0:
        # First segment is a point, find distance to second segment
        t = -f / c
        t = np.clip(t, 0, 1) # Clamp to segment
        closest_point = q1_np + t * d2
        return np.linalg.norm(p1_np - closest_point)
    if c == 0:
        # Second segment is a point, find distance to first segment
        t = e / a
        t = np.clip(t, 0, 1) # Clamp to segment
        closest_point = p1_np + t * d1
        return np.linalg.norm(q1_np - closest_point)

    # The general case: both segments are lines
    # We are solving for s and t in the equations:
    # p1 + s*d1 = q1 + t*d2
    # which gives: s*d1 - t*d2 = q1 - p1
    # This is a linear system for s and t.
    
    det = a * c - b * b
    if det != 0:
        # Lines are not parallel, find intersection point
        s = (b * f - c * e) / det
        t = (a * f - b * e) / det
        
        # Clamp s and t to the range [0, 1] to stay on the segments
        s = np.clip(s, 0, 1)
        t = np.clip(t, 0, 1)
    else:
        # Lines are parallel, s and t are not unique
        # We can set s=0 and find the corresponding t
        s = 0
        t = f / c
        t = np.clip(t, 0, 1)

    # Calculate the closest points on each segment
    closest_p = p1_np + s * d1
    closest_q = q1_np + t * d2
    
    return np.linalg.norm(closest_p - closest_q)


def check_mission_conflicts(
    primary_mission: PrimaryMission, 
    simulated_flights: List[SimulatedFlight], 
    safety_buffer: float
) -> Dict[str, Any]:
    """
    Checks for spatio-temporal conflicts using a brute-force approach.

    This function iterates through every segment of the primary mission and compares
    it against every segment of every simulated flight.

    Args:
        primary_mission: The primary drone's mission plan.
        simulated_flights: A list of other drones' flight plans.
        safety_buffer: The minimum allowable distance between drones.

    Returns:
        A dictionary containing conflict details if a conflict is found,
        otherwise a dictionary indicating no conflict.
    """
    # --- Primary Mission Time Calculation (Based on Constant Velocity Assumption) ---
    total_dist = sum(
        math.hypot(p2.x - p1.x, p2.y - p1.y)
        for p1, p2 in zip(primary_mission.waypoints, primary_mission.waypoints[1:])
    )
    
    mission_duration = primary_mission.end_time - primary_mission.start_time
    if mission_duration <= 0 or total_dist == 0:
        # Avoid division by zero if mission time is instant, invalid, or no movement
        primary_speed = float('inf')
    else:
        primary_speed = total_dist / mission_duration

    # Calculate the estimated time of arrival at each waypoint for the primary mission
    primary_etas = [primary_mission.start_time]
    dist_covered = 0
    if total_dist > 0:
        for p1, p2 in zip(primary_mission.waypoints, primary_mission.waypoints[1:]):
            segment_dist = math.hypot(p2.x - p1.x, p2.y - p1.y)
            time_for_segment = segment_dist / primary_speed
            last_eta = primary_etas[-1]
            primary_etas.append(last_eta + time_for_segment)
    else: # If total_dist is 0, the drone doesn't move.
        for _ in primary_mission.waypoints[1:]:
            primary_etas.append(primary_mission.start_time)


    # --- Brute-Force Conflict Check ---
    for sim_flight in simulated_flights:
        for i in range(len(primary_mission.waypoints) - 1):
            pri_p1 = primary_mission.waypoints[i]
            pri_p2 = primary_mission.waypoints[i+1]
            
            for j in range(len(sim_flight.waypoints) - 1):
                sim_p1 = sim_flight.waypoints[j]
                sim_p2 = sim_flight.waypoints[j+1]

                # 1. Spatial Check
                distance = get_closest_distance_between_segments(pri_p1, pri_p2, sim_p1, sim_p2)

                if distance < safety_buffer:
                    # 2. Temporal Check (only if spatially close)
                    pri_start_t = primary_etas[i]
                    pri_end_t = primary_etas[i+1]
                    
                    sim_start_t = sim_flight.timestamps[j]
                    sim_end_t = sim_flight.timestamps[j+1]
                    
                    # Check for time interval overlap: max(start) < min(end)
                    if max(pri_start_t, sim_start_t) < min(pri_end_t, sim_end_t):
                        # Spatio-temporal conflict found!
                        # For a better location, we can estimate the intersection point
                        # This is a simplification for the MVP
                        conflict_location = {
                            "x": (pri_p1.x + pri_p2.x) / 2,
                            "y": (pri_p1.y + pri_p2.y) / 2
                        }
                        
                        return {
                            "conflict": True,
                            "flight_id": sim_flight.flight_id,
                            "location": conflict_location, # Simplified to midpoint of primary segment
                            "time": max(pri_start_t, sim_start_t)
                        }

    # If all loops complete without returning, no conflicts were found
    return {"conflict": False}
