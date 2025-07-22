"""
deconfliction_logic.py

[V3 - Accurate Location] This module contains the core logic for detecting
spatio-temporal conflicts. It has been upgraded to calculate the precise
point of closest approach for more accurate conflict reporting.
"""

import math
import numpy as np
from typing import List, Dict, Any, Tuple

# Import our custom data models
from .data_models import PrimaryMission, SimulatedFlight, Waypoint

def get_closest_points_and_distance_3d(p1: Waypoint, p2: Waypoint, q1: Waypoint, q2: Waypoint) -> Tuple[float, np.ndarray]:
    """
    Calculates the shortest distance between two 3D line segments and the midpoint
    of the two closest points.
    """
    p1_np, p2_np = np.array([p1.x, p1.y, p1.z]), np.array([p2.x, p2.y, p2.z])
    q1_np, q2_np = np.array([q1.x, q1.y, q1.z]), np.array([q2.x, q2.y, q2.z])

    d1, d2, r = p2_np - p1_np, q2_np - q1_np, p1_np - q1_np
    a, c = np.dot(d1, d1), np.dot(d2, d2)
    b, e, f = np.dot(d1, d2), np.dot(d1, r), np.dot(d2, r)

    if a == 0 and c == 0:
        return np.linalg.norm(p1_np - q1_np), (p1_np + q1_np) / 2
    if a == 0:
        t = np.clip(-f / c, 0, 1) if c != 0 else 0
        closest_q = q1_np + t * d2
        return np.linalg.norm(p1_np - closest_q), (p1_np + closest_q) / 2
    if c == 0:
        s = np.clip(e / a, 0, 1) if a != 0 else 0
        closest_p = p1_np + s * d1
        return np.linalg.norm(q1_np - closest_p), (q1_np + closest_p) / 2

    det = a * c - b * b
    s = np.clip((b * f - c * e) / det, 0, 1) if det != 0 else 0
    t_num = (a * f - b * e)
    
    if det != 0:
        t = np.clip(t_num / det, 0, 1)
    else: # Parallel lines
        t = np.clip(f / c if c > 0 else 0, 0, 1)

    closest_p = p1_np + s * d1
    closest_q = q1_np + t * d2
    
    distance = np.linalg.norm(closest_p - closest_q)
    midpoint_of_closest = (closest_p + closest_q) / 2
    
    return distance, midpoint_of_closest

def check_mission_conflicts(
    primary_mission: PrimaryMission, 
    simulated_flights: List[SimulatedFlight], 
    safety_buffer: float
) -> Dict[str, Any]:
    """
    Checks for spatio-temporal conflicts using a brute-force approach in 3D.
    """
    total_dist = sum(
        np.linalg.norm(np.array([p2.x, p2.y, p2.z]) - np.array([p1.x, p1.y, p1.z]))
        for p1, p2 in zip(primary_mission.waypoints, primary_mission.waypoints[1:])
    )
    mission_duration = primary_mission.end_time - primary_mission.start_time
    primary_speed = total_dist / mission_duration if mission_duration > 0 and total_dist > 0 else float('inf')

    primary_etas = [primary_mission.start_time]
    if total_dist > 0:
        for p1, p2 in zip(primary_mission.waypoints, primary_mission.waypoints[1:]):
            segment_dist = np.linalg.norm(np.array([p2.x, p2.y, p2.z]) - np.array([p1.x, p1.y, p1.z]))
            time_for_segment = segment_dist / primary_speed if primary_speed != float('inf') else 0
            primary_etas.append(primary_etas[-1] + time_for_segment)
    else:
        for _ in primary_mission.waypoints[1:]:
            primary_etas.append(primary_mission.start_time)

    for sim_flight in simulated_flights:
        for i in range(len(primary_mission.waypoints) - 1):
            for j in range(len(sim_flight.waypoints) - 1):
                pri_p1, pri_p2 = primary_mission.waypoints[i], primary_mission.waypoints[i+1]
                sim_p1, sim_p2 = sim_flight.waypoints[j], sim_flight.waypoints[j+1]

                # 1. Spatial Check (now gets accurate point)
                distance, conflict_point = get_closest_points_and_distance_3d(pri_p1, pri_p2, sim_p1, sim_p2)

                if distance < safety_buffer:
                    # 2. Temporal Check
                    pri_start_t, pri_end_t = primary_etas[i], primary_etas[i+1]
                    sim_start_t, sim_end_t = sim_flight.timestamps[j], sim_flight.timestamps[j+1]
                    
                    if max(pri_start_t, sim_start_t) < min(pri_end_t, sim_end_t):
                        conflict_location = {
                            "x": conflict_point[0],
                            "y": conflict_point[1],
                            "z": conflict_point[2]
                        }
                        return {
                            "conflict": True,
                            "flight_id": sim_flight.flight_id,
                            "location": conflict_location, # Now using the accurate point
                            "time": max(pri_start_t, sim_start_t)
                        }

    return {"conflict": False}
