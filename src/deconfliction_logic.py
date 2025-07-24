"""
deconfliction_logic.py

[V4 - Refactored for Clarity]
This module implements the deconfliction engine. This version removes unused
code to improve clarity and maintainability.
"""

import math
import numpy as np
from typing import List, Dict, Any, Tuple, Set, Optional

# Import our custom data models and utilities
from .data_models import Waypoint, PrimaryMission, SimulatedFlight
from .utils import calculate_etas

# --- Helper for Path Interpolation ---
def get_position_on_segment_at_time(p1, p2, t1, t2, current_t):
    """Calculates the 3D position on a line segment at a given time."""
    if t2 == t1: return p1
    clamped_t = max(t1, min(t2, current_t))
    progress = (clamped_t - t1) / (t2 - t1)
    return Waypoint(p1.x + progress * (p2.x - p1.x), p1.y + progress * (p2.y - p1.y), p1.z + progress * (p2.z - p1.z))

# --- Phase 2: The Narrow Phase ---

def check_spatio_temporal_conflict(pri_p1, pri_p2, pri_t1, pri_t2, sim_p1, sim_p2, sim_t1, sim_t2, safety_buffer):
    """
    Performs a precise check for conflict between two moving drones on their
    respective trajectory segments, based on time of closest approach (TCA).
    """
    overlap_start, overlap_end = max(pri_t1, sim_t1), min(pri_t2, sim_t2)
    if overlap_start >= overlap_end: return None

    # Get initial positions and velocities
    pri_pos_start_wp = get_position_on_segment_at_time(pri_p1, pri_p2, pri_t1, pri_t2, overlap_start)
    sim_pos_start_wp = get_position_on_segment_at_time(sim_p1, sim_p2, sim_t1, sim_t2, overlap_start)
    pri_pos_start = np.array([pri_pos_start_wp.x, pri_pos_start_wp.y, pri_pos_start_wp.z])
    sim_pos_start = np.array([sim_pos_start_wp.x, sim_pos_start_wp.y, sim_pos_start_wp.z])
    pri_vel = (np.array([pri_p2.x, pri_p2.y, pri_p2.z]) - np.array([pri_p1.x, pri_p1.y, pri_p1.z])) / (pri_t2 - pri_t1) if pri_t2 > pri_t1 else np.zeros(3)
    sim_vel = (np.array([sim_p2.x, sim_p2.y, sim_p2.z]) - np.array([sim_p1.x, sim_p1.y, sim_p1.z])) / (sim_t2 - sim_t1) if sim_t2 > sim_t1 else np.zeros(3)

    # Calculate relative position and velocity
    relative_pos, relative_vel = sim_pos_start - pri_pos_start, sim_vel - pri_vel
    dot_rel_vel = np.dot(relative_vel, relative_vel)
    tca = -np.dot(relative_pos, relative_vel) / dot_rel_vel if dot_rel_vel > 1e-9 else 0.0
    
    # Check distances at the start, end, and TCA to find the minimum
    min_dist_sq, conflict_time = np.dot(relative_pos, relative_pos), overlap_start
    
    pos_at_end = relative_pos + relative_vel * (overlap_end - overlap_start)
    dist_sq_end = np.dot(pos_at_end, pos_at_end)
    if dist_sq_end < min_dist_sq:
        min_dist_sq, conflict_time = dist_sq_end, overlap_end
        
    if 0 < tca < (overlap_end - overlap_start):
        pos_at_tca = relative_pos + relative_vel * tca
        dist_sq_tca = np.dot(pos_at_tca, pos_at_tca)
        if dist_sq_tca < min_dist_sq:
            min_dist_sq, conflict_time = dist_sq_tca, overlap_start + tca
            
    # If the minimum distance is within the buffer, it's a conflict
    if min_dist_sq < safety_buffer ** 2:
        pri_pos_conflict_wp = get_position_on_segment_at_time(pri_p1, pri_p2, pri_t1, pri_t2, conflict_time)
        sim_pos_conflict_wp = get_position_on_segment_at_time(sim_p1, sim_p2, sim_t1, sim_t2, conflict_time)
        pri_pos_conflict = np.array([pri_pos_conflict_wp.x, pri_pos_conflict_wp.y, pri_pos_conflict_wp.z])
        sim_pos_conflict = np.array([sim_pos_conflict_wp.x, sim_pos_conflict_wp.y, sim_pos_conflict_wp.z])
        conflict_point = (pri_pos_conflict + sim_pos_conflict) / 2
        return {"location": {"x": conflict_point[0], "y": conflict_point[1], "z": conflict_point[2]}, "time": conflict_time}
        
    return None

# --- Phase 1: The Broad Phase (4D Grid Filter) ---
class Grid4D:
    def __init__(self, bin_size: Tuple[float, float, float, float]):
        self.x_size, self.y_size, self.z_size, self.t_size = bin_size
        self.grid: Dict[Tuple[int, int, int, int], Set[str]] = {}

    def _get_bin_id(self, x, y, z, t):
        return (math.floor(x / self.x_size), math.floor(y / self.y_size), math.floor(z / self.z_size), math.floor(t / self.t_size))

    def add_flight(self, flight: SimulatedFlight):
        for i in range(len(flight.waypoints) - 1):
            p1, p2 = flight.waypoints[i], flight.waypoints[i+1]
            t1, t2 = flight.timestamps[i], flight.timestamps[i+1]
            if t1 >= t2: continue

            # Robust time-based sampling to ensure all occupied grid cells are registered
            time_duration = t2 - t1
            num_samples = int(math.ceil(time_duration / self.t_size) * 4)
            if num_samples < 2:
                num_samples = 2

            for j in range(num_samples):
                progress = j / (num_samples - 1) if num_samples > 1 else 0
                curr_t = t1 + progress * time_duration
                
                curr_x = p1.x + progress * (p2.x - p1.x)
                curr_y = p1.y + progress * (p2.y - p1.y)
                curr_z = p1.z + progress * (p2.z - p1.z)
                
                bin_id = self._get_bin_id(curr_x, curr_y, curr_z, curr_t)
                if bin_id not in self.grid: self.grid[bin_id] = set()
                self.grid[bin_id].add(flight.flight_id)

    def get_candidate_ids(self, mission_segments):
        candidates = set()
        for p1, p2, t1, t2 in mission_segments:
            if t1 >= t2: continue

            # Sample the primary mission's path to find grid cells it occupies
            num_steps = max(2, int(math.ceil(abs(t2 - t1) / (self.t_size / 4.0))))
            sample_times = np.linspace(t1, t2, num_steps)
            
            for curr_t in sample_times:
                curr_pos = get_position_on_segment_at_time(p1, p2, t1, t2, curr_t)
                bin_id = self._get_bin_id(curr_pos.x, curr_pos.y, curr_pos.z, curr_t)
                if bin_id in self.grid:
                    candidates.update(self.grid[bin_id])
        return candidates

# --- The Hybrid Engine Orchestrator ---
def check_conflicts_hybrid(primary_mission, simulated_flights, primary_drone_speed_mps, config):
    safety_buffer = config['safety_buffer_m']
    grid_config = config['grid_bin_size']
    grid_bin_size = (grid_config['x_m'], grid_config['y_m'], grid_config['z_m'], grid_config['t_s'])

    primary_etas = calculate_etas(primary_mission.waypoints, primary_mission.start_time, primary_drone_speed_mps)
    if not primary_etas or primary_etas[-1] > primary_mission.end_time:
        return {"status": "MISSION_TIME_VIOLATION", "message": f"Mission cannot be completed within the time window. Required: {primary_etas[-1]:.2f}s, Allowed: {primary_mission.end_time}s."}
    
    grid = Grid4D(bin_size=grid_bin_size)
    for flight in simulated_flights:
        grid.add_flight(flight)
    
    primary_segments = []
    is_stationary = all(wp == primary_mission.waypoints[0] for wp in primary_mission.waypoints)
    if is_stationary:
        p1 = primary_mission.waypoints[0]
        primary_segments.append((p1, p1, primary_mission.start_time, primary_mission.end_time))
    else:
        for i in range(len(primary_mission.waypoints) - 1):
            segment = (primary_mission.waypoints[i], primary_mission.waypoints[i+1], primary_etas[i], primary_etas[i+1])
            primary_segments.append(segment)

    candidate_ids = grid.get_candidate_ids(primary_segments)
    candidate_flights = [f for f in simulated_flights if f.flight_id in candidate_ids]
    print(f"Broad-phase filter complete. Found {len(candidate_flights)} potential threats out of {len(simulated_flights)} total.")
    
    for sim_flight in candidate_flights:
        for pri_p1, pri_p2, pri_t1, pri_t2 in primary_segments:
            for j in range(len(sim_flight.waypoints) - 1):
                sim_p1, sim_p2 = sim_flight.waypoints[j], sim_flight.waypoints[j+1]
                sim_t1, sim_t2 = sim_flight.timestamps[j], sim_flight.timestamps[j+1]

                if max(pri_t1, sim_t1) >= min(pri_t2, sim_t2):
                    continue

                conflict_details = check_spatio_temporal_conflict(
                    pri_p1, pri_p2, pri_t1, pri_t2, 
                    sim_p1, sim_p2, sim_t1, sim_t2, 
                    safety_buffer
                )
                
                if conflict_details:
                    return {"status": "CONFLICT", "flight_id": sim_flight.flight_id, **conflict_details}
    
    return {"status": "CLEAR", "message": "Mission is clear of conflicts."}