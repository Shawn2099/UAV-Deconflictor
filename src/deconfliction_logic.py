"""
deconfliction_logic.py

[V10 - Hybrid Engine with External Configuration]
This module implements the deconfliction engine. Key parameters like safety buffer
and grid size are no longer hardcoded but are passed in via a configuration object.
This makes the system more flexible and maintainable.
"""

import math
import numpy as np
from typing import List, Dict, Any, Tuple, Set

# Import our custom data models
from .data_models import PrimaryMission, SimulatedFlight, Waypoint

# --- Helper for Path Interpolation ---
def get_position_on_segment_at_time(p1: Waypoint, p2: Waypoint, t1: float, t2: float, current_t: float) -> Waypoint:
    """
    Interpolates the position of a drone at a given time along a segment.
    Clamps current_t to the segment's time range [t1, t2] to prevent extrapolation.
    """
    if t2 == t1: # Handle stationary drone or instantaneous waypoint
        return p1
    
    clamped_t = max(t1, min(t2, current_t))
    progress = (clamped_t - t1) / (t2 - t1)
    x = p1.x + progress * (p2.x - p1.x)
    y = p1.y + progress * (p2.y - p1.y)
    z = p1.z + progress * (p2.z - p1.z)
    return Waypoint(x, y, z)


# --- Phase 2: The Narrow Phase (Precise Analytical Geometry) ---
def get_closest_points_and_distance_3d(p1: Waypoint, p2: Waypoint, q1: Waypoint, q2: Waypoint) -> Tuple[float, np.ndarray]:
    """
    Calculates the shortest distance between two 3D line segments and the midpoint of the two closest points.
    """
    P1 = np.array([p1.x, p1.y, p1.z])
    P2 = np.array([p2.x, p2.y, p2.z])
    Q1 = np.array([q1.x, q1.y, q1.z])
    Q2 = np.array([q2.x, q2.y, q2.z])

    u = P2 - P1
    v = Q2 - Q1
    w = P1 - Q1

    a = np.dot(u, u)
    b = np.dot(u, v)
    c = np.dot(v, v)
    d = np.dot(u, w)
    e = np.dot(v, w)

    denom = a * c - b * b

    if denom < 1e-7:
        def dist_point_to_segment(point, seg_p1, seg_p2):
            seg_vec = seg_p2 - seg_p1
            point_vec = point - seg_p1
            seg_len_sq = np.dot(seg_vec, seg_vec)
            if seg_len_sq < 1e-9:
                return np.linalg.norm(point_vec), seg_p1
            t = max(0.0, min(1.0, np.dot(point_vec, seg_vec) / seg_len_sq))
            closest_point_on_seg = seg_p1 + t * seg_vec
            return np.linalg.norm(point - closest_point_on_seg), closest_point_on_seg

        d1, cp_q_for_p1 = dist_point_to_segment(P1, Q1, Q2)
        d2, cp_q_for_p2 = dist_point_to_segment(P2, Q1, Q2)
        d3, cp_p_for_q1 = dist_point_to_segment(Q1, P1, P2)
        d4, cp_p_for_q2 = dist_point_to_segment(Q2, P1, P2)
        
        distances = [(d1, P1, cp_q_for_p1), (d2, P2, cp_q_for_p2), (d3, cp_p_for_q1, Q1), (d4, cp_p_for_q2, Q2)]
        min_dist, p_final, q_final = min(distances, key=lambda x: x[0])
        midpoint = (p_final + q_final) / 2
        return min_dist, midpoint

    s_candidate = (b * e - c * d) / denom
    t_candidate = (a * e - b * d) / denom
    s = max(0.0, min(1.0, s_candidate))
    t = (b * s + e) / c if c > 1e-6 else 0.0
    t = max(0.0, min(1.0, t))

    if s_candidate < 0.0 or s_candidate > 1.0:
        t_recalc = (b * s + e) / c if c > 1e-6 else 0.0
        t = max(0.0, min(1.0, t_recalc))
        if t == 0.0 or t == 1.0:
            s_recalc = (-d + b * t) / a if a > 1e-6 else 0.0
            s = max(0.0, min(1.0, s_recalc))
            
    closest_p = P1 + s * u
    closest_q = Q1 + t * v
    distance = np.linalg.norm(closest_p - closest_q)
    midpoint = (closest_p + closest_q) / 2
    return distance, midpoint

# --- Phase 1: The Broad Phase (4D Grid Filter) ---

class Grid4D:
    """A class to manage the 4D spatio-temporal grid for broad-phase filtering."""
    def __init__(self, bin_size: Tuple[float, float, float, float]):
        self.x_size, self.y_size, self.z_size, self.t_size = bin_size
        self.grid: Dict[Tuple[int, int, int, int], Set[str]] = {}

    def _get_bin_id(self, x: float, y: float, z: float, t: float) -> Tuple[int, int, int, int]:
        return (math.floor(x / self.x_size), math.floor(y / self.y_size), math.floor(z / self.z_size), math.floor(t / self.t_size))

    def add_flight(self, flight: SimulatedFlight):
        time_step_for_sampling = self.t_size / 2.0
        for i in range(len(flight.waypoints) - 1):
            p1, p2 = flight.waypoints[i], flight.waypoints[i+1]
            t1, t2 = flight.timestamps[i], flight.timestamps[i+1]
            if t1 == t2:
                sample_times = [t1]
            else:
                num_steps = max(2, int(math.ceil(abs(t2 - t1) / time_step_for_sampling)))
                sample_times = list(np.linspace(t1, t2, num_steps))
            for curr_t in sample_times:
                curr_pos = get_position_on_segment_at_time(p1, p2, t1, t2, curr_t)
                bin_id = self._get_bin_id(curr_pos.x, curr_pos.y, curr_pos.z, curr_t)
                if bin_id not in self.grid:
                    self.grid[bin_id] = set()
                self.grid[bin_id].add(flight.flight_id)

    def get_candidate_ids(self, mission: PrimaryMission, etas: List[float]) -> Set[str]:
        candidates = set()
        time_step_for_sampling = self.t_size / 2.0
        for i in range(len(mission.waypoints) - 1):
            p1, p2 = mission.waypoints[i], mission.waypoints[i+1]
            t1, t2 = etas[i], etas[i+1]
            if t1 == t2:
                sample_times = [t1]
            else:
                num_steps = max(2, int(math.ceil(abs(t2 - t1) / time_step_for_sampling)))
                sample_times = list(np.linspace(t1, t2, num_steps))
            for curr_t in sample_times:
                curr_pos = get_position_on_segment_at_time(p1, p2, t1, t2, curr_t)
                bin_id = self._get_bin_id(curr_pos.x, curr_pos.y, curr_pos.z, curr_t)
                if bin_id in self.grid:
                    candidates.update(self.grid[bin_id])
        return candidates

# --- The Hybrid Engine Orchestrator ---

def check_conflicts_hybrid(
    primary_mission: PrimaryMission, 
    simulated_flights: List[SimulatedFlight], 
    config: Dict[str, Any]
) -> Dict[str, Any]:
    """
    Checks for conflicts using the two-phase hybrid engine.
    """
    # --- Extract parameters from config ---
    safety_buffer = config['safety_buffer_m']
    grid_config = config['grid_bin_size']
    grid_bin_size = (grid_config['x_m'], grid_config['y_m'], grid_config['z_m'], grid_config['t_s'])

    # --- Calculate Primary Mission ETAs ---
    total_dist = sum(np.linalg.norm(np.array([p2.x,p2.y,p2.z]) - np.array([p1.x,p1.y,p1.z])) for p1, p2 in zip(primary_mission.waypoints, primary_mission.waypoints[1:]))
    mission_duration = primary_mission.end_time - primary_mission.start_time
    primary_speed = total_dist / mission_duration if mission_duration > 0 and total_dist > 0 else float('inf')
    primary_etas = [primary_mission.start_time]
    if total_dist > 0:
        for p1, p2 in zip(primary_mission.waypoints, primary_mission.waypoints[1:]):
            segment_dist = np.linalg.norm(np.array([p2.x,p2.y,p2.z]) - np.array([p1.x,p1.y,p1.z]))
            time_for_segment = segment_dist / primary_speed if primary_speed != float('inf') else 0
            primary_etas.append(primary_etas[-1] + time_for_segment)
    else:
        for _ in primary_mission.waypoints[1:]: primary_etas.append(primary_mission.start_time)

    # --- Broad Phase ---
    grid = Grid4D(bin_size=grid_bin_size)
    for flight in simulated_flights:
        grid.add_flight(flight)
    
    candidate_ids = grid.get_candidate_ids(primary_mission, primary_etas)
    candidate_flights = [f for f in simulated_flights if f.flight_id in candidate_ids]
    
    print(f"Broad-phase filter complete. Found {len(candidate_flights)} potential threats out of {len(simulated_flights)} total.")

    # --- Narrow Phase ---
    for sim_flight in candidate_flights:
        for i in range(len(primary_mission.waypoints) - 1):
            for j in range(len(sim_flight.waypoints) - 1):
                pri_p1, pri_p2 = primary_mission.waypoints[i], primary_mission.waypoints[i+1]
                sim_p1, sim_p2 = sim_flight.waypoints[j], sim_flight.waypoints[j+1]
                
                distance, conflict_point = get_closest_points_and_distance_3d(pri_p1, pri_p2, sim_p1, sim_p2)
                
                if distance < safety_buffer:
                    pri_start_t, pri_end_t = primary_etas[i], primary_etas[i+1]
                    sim_start_t, sim_end_t = sim_flight.timestamps[j], sim_flight.timestamps[j+1]
                    
                    overlap_start = max(pri_start_t, sim_start_t)
                    overlap_end = min(pri_end_t, sim_end_t)

                    if overlap_start < overlap_end:
                        return { "conflict": True, "flight_id": sim_flight.flight_id, "location": {"x": conflict_point[0], "y": conflict_point[1], "z": conflict_point[2]}, "time": overlap_start }

    return {"conflict": False}
