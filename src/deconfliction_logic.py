"""
deconfliction_logic.py

[V5 - Final, Bug Fix in Orchestrator]
This module implements the deconfliction engine with a final critical bug fix.

Key Changes:
- Removed a flawed premature geometric check from the main `check_conflicts_hybrid`
  function. This check was incorrectly filtering out valid conflict scenarios
  before the detailed spatio-temporal analysis could be performed.
"""

import math
import numpy as np
from typing import List, Dict, Any, Tuple, Set, Optional

# Import our custom data models and utilities
from .data_models import PrimaryMission, SimulatedFlight, Waypoint
from .utils import calculate_etas

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
    Calculates the shortest distance between two 3D line segments and the midpoint of that shortest distance.
    This function is purely geometric and does not consider time.
    """
    P1, P2 = np.array([p1.x, p1.y, p1.z]), np.array([p2.x, p2.y, p2.z])
    Q1, Q2 = np.array([q1.x, q1.y, q1.z]), np.array([q2.x, q2.y, q2.z])

    u, v, w = P2 - P1, Q2 - Q1, P1 - Q1
    a, b, c = np.dot(u, u), np.dot(u, v), np.dot(v, v)
    d, e = np.dot(u, w), np.dot(v, w)
    
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

    s_c = (b * e - c * d) / denom
    t_c = (a * e - b * d) / denom

    s = np.clip(s_c, 0.0, 1.0)
    t = np.clip(t_c, 0.0, 1.0)

    if s != s_c:
        t_recalc = (b * s + e) / c if c > 1e-7 else 0.0
        t = np.clip(t_recalc, 0.0, 1.0)
    if t != t_c:
        s_recalc = (b*t - d) / a if a > 1e-7 else 0.0
        s = np.clip(s_recalc, 0.0, 1.0)

    closest_p = P1 + s * u
    closest_q = Q1 + t * v
    distance = np.linalg.norm(closest_p - closest_q)
    midpoint = (closest_p + closest_q) / 2
    return distance, midpoint


def check_spatio_temporal_conflict(
    pri_p1: Waypoint, pri_p2: Waypoint, pri_t1: float, pri_t2: float,
    sim_p1: Waypoint, sim_p2: Waypoint, sim_t1: float, sim_t2: float,
    safety_buffer: float
) -> Optional[Dict[str, Any]]:
    """
    Checks for spatio-temporal conflict between two moving drones on their respective segments.
    """
    overlap_start = max(pri_t1, sim_t1)
    overlap_end = min(pri_t2, sim_t2)
    
    if overlap_start >= overlap_end:
        return None

    pos_pri_start = get_position_on_segment_at_time(pri_p1, pri_p2, pri_t1, pri_t2, overlap_start)
    pri_pos_at_overlap_start = np.array([pos_pri_start.x, pos_pri_start.y, pos_pri_start.z])

    pos_sim_start = get_position_on_segment_at_time(sim_p1, sim_p2, sim_t1, sim_t2, overlap_start)
    sim_pos_at_overlap_start = np.array([pos_sim_start.x, pos_sim_start.y, pos_sim_start.z])

    pri_vel = (np.array([pri_p2.x, pri_p2.y, pri_p2.z]) - np.array([pri_p1.x, pri_p1.y, pri_p1.z])) / (pri_t2 - pri_t1) if pri_t2 > pri_t1 else np.zeros(3)
    sim_vel = (np.array([sim_p2.x, sim_p2.y, sim_p2.z]) - np.array([sim_p1.x, sim_p1.y, sim_p1.z])) / (sim_t2 - sim_t1) if sim_t2 > sim_t1 else np.zeros(3)

    relative_pos = sim_pos_at_overlap_start - pri_pos_at_overlap_start
    relative_vel = sim_vel - pri_vel

    dot_rel_vel = np.dot(relative_vel, relative_vel)
    if dot_rel_vel < 1e-9:
        tca_from_overlap_start = 0.0
    else:
        tca_from_overlap_start = -np.dot(relative_pos, relative_vel) / dot_rel_vel

    min_dist_sq = float('inf')
    conflict_time = -1

    dist_sq_start = np.dot(relative_pos, relative_pos)
    if dist_sq_start < min_dist_sq:
        min_dist_sq = dist_sq_start
        conflict_time = overlap_start

    pos_at_end = relative_pos + relative_vel * (overlap_end - overlap_start)
    dist_sq_end = np.dot(pos_at_end, pos_at_end)
    if dist_sq_end < min_dist_sq:
        min_dist_sq = dist_sq_end
        conflict_time = overlap_end

    if 0 < tca_from_overlap_start < (overlap_end - overlap_start):
        pos_at_tca = relative_pos + relative_vel * tca_from_overlap_start
        dist_sq_tca = np.dot(pos_at_tca, pos_at_tca)
        if dist_sq_tca < min_dist_sq:
            min_dist_sq = dist_sq_tca
            conflict_time = overlap_start + tca_from_overlap_start

    if min_dist_sq < safety_buffer ** 2:
        pos_pri_conflict = get_position_on_segment_at_time(pri_p1, pri_p2, pri_t1, pri_t2, conflict_time)
        pri_pos_at_conflict = np.array([pos_pri_conflict.x, pos_pri_conflict.y, pos_pri_conflict.z])

        pos_sim_conflict = get_position_on_segment_at_time(sim_p1, sim_p2, sim_t1, sim_t2, conflict_time)
        sim_pos_at_conflict = np.array([pos_sim_conflict.x, pos_sim_conflict.y, pos_sim_conflict.z])

        conflict_point = (pri_pos_at_conflict + sim_pos_at_conflict) / 2
        return {
            "location": {"x": conflict_point[0], "y": conflict_point[1], "z": conflict_point[2]},
            "time": conflict_time
        }
        
    return None

# --- Phase 1: The Broad Phase (4D Grid Filter) ---

class Grid4D:
    """A class to manage the 4D spatio-temporal grid for broad-phase filtering."""
    def __init__(self, bin_size: Tuple[float, float, float, float]):
        self.x_size, self.y_size, self.z_size, self.t_size = bin_size
        self.grid: Dict[Tuple[int, int, int, int], Set[str]] = {}

    def _get_bin_id(self, x: float, y: float, z: float, t: float) -> Tuple[int, int, int, int]:
        return (math.floor(x / self.x_size), math.floor(y / self.y_size), math.floor(z / self.z_size), math.floor(t / self.t_size))

    def add_flight(self, flight: SimulatedFlight):
        safety_margin = max(self.x_size, self.y_size, self.z_size)
        for i in range(len(flight.waypoints) - 1):
            p1, p2 = flight.waypoints[i], flight.waypoints[i+1]
            t1, t2 = flight.timestamps[i], flight.timestamps[i+1]
            if t1 >= t2: continue

            min_x, max_x = min(p1.x, p2.x) - safety_margin, max(p1.x, p2.x) + safety_margin
            min_y, max_y = min(p1.y, p2.y) - safety_margin, max(p1.y, p2.y) + safety_margin
            min_z, max_z = min(p1.z, p2.z) - safety_margin, max(p1.z, p2.z) + safety_margin
            
            for x_bin in range(math.floor(min_x / self.x_size), math.ceil(max_x / self.x_size)):
                for y_bin in range(math.floor(min_y / self.y_size), math.ceil(max_y / self.y_size)):
                    for z_bin in range(math.floor(min_z / self.z_size), math.ceil(max_z / self.z_size)):
                        for t_bin in range(math.floor(t1 / self.t_size), math.ceil(t2 / self.t_size)):
                            bin_id = (x_bin, y_bin, z_bin, t_bin)
                            if bin_id not in self.grid:
                                self.grid[bin_id] = set()
                            self.grid[bin_id].add(flight.flight_id)

    def get_candidate_ids(self, mission: PrimaryMission, etas: List[float]) -> Set[str]:
        candidates = set()
        for i in range(len(mission.waypoints) - 1):
            p1, p2 = mission.waypoints[i], mission.waypoints[i+1]
            t1, t2 = etas[i], etas[i+1]
            if t1 >= t2: continue

            num_steps = max(2, int(math.ceil(abs(t2 - t1) / (self.t_size / 4.0))))
            sample_times = np.linspace(t1, t2, num_steps)
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
    primary_drone_speed_mps: float,
    config: Dict[str, Any]
) -> Dict[str, Any]:
    safety_buffer = config['safety_buffer_m']
    grid_config = config['grid_bin_size']
    grid_bin_size = (grid_config['x_m'], grid_config['y_m'], grid_config['z_m'], grid_config['t_s'])

    primary_etas = calculate_etas(primary_mission.waypoints, primary_mission.start_time, primary_drone_speed_mps)

    if not primary_etas or primary_etas[-1] > primary_mission.end_time:
        return {
            "status": "MISSION_TIME_VIOLATION",
            "conflict": False,
            "message": f"Mission cannot be completed within the time window. Required: {primary_etas[-1]:.2f}s, Allowed: {primary_mission.end_time}s."
        }

    grid = Grid4D(bin_size=grid_bin_size)
    for flight in simulated_flights:
        grid.add_flight(flight)
    
    candidate_ids = grid.get_candidate_ids(primary_mission, primary_etas)
    candidate_flights = [f for f in simulated_flights if f.flight_id in candidate_ids]
    
    print(f"Broad-phase filter complete. Found {len(candidate_flights)} potential threats out of {len(simulated_flights)} total.")

    for sim_flight in candidate_flights:
        for i in range(len(primary_mission.waypoints) - 1):
            for j in range(len(sim_flight.waypoints) - 1):
                pri_p1, pri_p2 = primary_mission.waypoints[i], primary_mission.waypoints[i+1]
                pri_t1, pri_t2 = primary_etas[i], primary_etas[i+1]
                
                sim_p1, sim_p2 = sim_flight.waypoints[j], sim_flight.waypoints[j+1]
                sim_t1, sim_t2 = sim_flight.timestamps[j], sim_flight.timestamps[j+1]

                # First, a quick check for temporal overlap. If no time overlap, no conflict possible.
                if max(pri_t1, sim_t1) >= min(pri_t2, sim_t2):
                    continue

                # *** BUG FIX ***
                # The incorrect geometric pre-check has been removed.
                # We now proceed directly to the full spatio-temporal check.
                
                conflict_details = check_spatio_temporal_conflict(
                    pri_p1, pri_p2, pri_t1, pri_t2,
                    sim_p1, sim_p2, sim_t1, sim_t2,
                    safety_buffer
                )

                if conflict_details:
                    return {
                        "status": "CONFLICT",
                        "conflict": True,
                        "flight_id": sim_flight.flight_id,
                        **conflict_details
                    }

    return {"status": "CLEAR", "conflict": False, "message": "Mission is clear of conflicts."}
