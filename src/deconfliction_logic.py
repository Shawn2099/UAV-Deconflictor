"""
deconfliction_logic.py

[V2 - 3D Enabled] This module contains the core logic for detecting spatio-temporal
conflicts in 3D space between a primary drone mission and a set of
simulated drone flights.
"""

import math
import numpy as np
from typing import List, Dict, Any

# Import our custom data models
from .data_models import PrimaryMission, SimulatedFlight, Waypoint

def get_closest_distance_between_segments_3d(p1: Waypoint, p2: Waypoint, q1: Waypoint, q2: Waypoint) -> float:
    """
    Calculates the shortest distance between two 3D line segments (p1-p2 and q1-q2).
    
    This is a robust implementation based on vector projections, extended to 3D.
    """
    # Convert waypoints to numpy arrays for 3D vector operations
    p1_np = np.array([p1.x, p1.y, p1.z])
    p2_np = np.array([p2.x, p2.y, p2.z])
    q1_np = np.array([q1.x, q1.y, q1.z])
    q2_np = np.array([q2.x, q2.y, q2.z])

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
        t = np.clip(-f / c, 0, 1) if c != 0 else 0
        return np.linalg.norm(p1_np - (q1_np + t * d2))
    if c == 0:
        s = np.clip(e / a, 0, 1) if a != 0 else 0
        return np.linalg.norm(q1_np - (p1_np + s * d1))

    # General case: both segments are lines
    det = a * c - b * b
    if det != 0:
        s = np.clip((b * f - c * e) / det, 0, 1)
        t = np.clip((a * f - b * e) / det, 0, 1)
    else: # Parallel lines
        s = 0
        t = f / c if c > 0 else 0
        t = np.clip(t, 0, 1)

    closest_p = p1_np + s * d1
    closest_q = q1_np + t * d2
    
    return np.linalg.norm(closest_p - closest_q)

def check_mission_conflicts(
    primary_mission: PrimaryMission, 
    simulated_flights: List[SimulatedFlight], 
    safety_buffer: float
) -> Dict[str, Any]:
    """
    Checks for spatio-temporal conflicts using a brute-force approach in 3D.
    """
    # --- Primary Mission Time Calculation (3D Distance) ---
    total_dist = 0
    for p1, p2 in zip(primary_mission.waypoints, primary_mission.waypoints[1:]):
        total_dist += np.linalg.norm(np.array([p2.x, p2.y, p2.z]) - np.array([p1.x, p1.y, p1.z]))

    mission_duration = primary_mission.end_time - primary_mission.start_time
    if mission_duration <= 0 or total_dist == 0:
        primary_speed = float('inf')
    else:
        primary_speed = total_dist / mission_duration

    primary_etas = [primary_mission.start_time]
    if total_dist > 0:
        for p1, p2 in zip(primary_mission.waypoints, primary_mission.waypoints[1:]):
            segment_dist = np.linalg.norm(np.array([p2.x, p2.y, p2.z]) - np.array([p1.x, p1.y, p1.z]))
            time_for_segment = segment_dist / primary_speed
            primary_etas.append(primary_etas[-1] + time_for_segment)
    else:
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

                # 1. Spatial Check (using the new 3D function)
                distance = get_closest_distance_between_segments_3d(pri_p1, pri_p2, sim_p1, sim_p2)

                if distance < safety_buffer:
                    # 2. Temporal Check (logic remains the same)
                    pri_start_t = primary_etas[i]
                    pri_end_t = primary_etas[i+1]
                    sim_start_t = sim_flight.timestamps[j]
                    sim_end_t = sim_flight.timestamps[j+1]
                    
                    if max(pri_start_t, sim_start_t) < min(pri_end_t, sim_end_t):
                        conflict_location = {
                            "x": (pri_p1.x + pri_p2.x) / 2,
                            "y": (pri_p1.y + pri_p2.y) / 2,
                            "z": (pri_p1.z + pri_p2.z) / 2
                        }
                        return {
                            "conflict": True,
                            "flight_id": sim_flight.flight_id,
                            "location": conflict_location,
                            "time": max(pri_start_t, sim_start_t)
                        }

    return {"conflict": False}
