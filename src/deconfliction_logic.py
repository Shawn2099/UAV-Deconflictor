"""
deconfliction_logic.py

[V5 - Hybrid Engine] This module implements a highly scalable, two-phase
deconfliction engine. It uses a coarse 4D grid to first filter for potential
threats and then runs a precise analytical check on the filtered candidates.
"""

import math
import numpy as np
from typing import List, Dict, Any, Tuple, Set

# Import our custom data models
from .data_models import PrimaryMission, SimulatedFlight, Waypoint

# --- Phase 2: The Narrow Phase (Precise Analytical Geometry) ---
# We reuse our perfected analytical engine from the previous version.

def get_closest_points_and_distance_3d(p1: Waypoint, p2: Waypoint, q1: Waypoint, q2: Waypoint) -> Tuple[float, np.ndarray]:
    """Calculates the shortest distance between two 3D line segments and the midpoint of the two closest points."""
    p1_np, p2_np = np.array([p1.x, p1.y, p1.z]), np.array([p2.x, p2.y, p2.z])
    q1_np, q2_np = np.array([q1.x, q1.y, q1.z]), np.array([q2.x, q2.y, q2.z])

    d1, d2, r = p2_np - p1_np, q2_np - q1_np, p1_np - q1_np
    a, c = np.dot(d1, d1), np.dot(d2, d2)
    b, e, f = np.dot(d1, d2), np.dot(d1, r), np.dot(d2, r)

    if a == 0 and c == 0: return np.linalg.norm(p1_np - q1_np), (p1_np + q1_np) / 2
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
    
    if det != 0: t = np.clip(t_num / det, 0, 1)
    else: t = np.clip(f / c if c > 0 else 0, 0, 1)

    closest_p, closest_q = p1_np + s * d1, q1_np + t * d2
    distance = np.linalg.norm(closest_p - closest_q)
    midpoint_of_closest = (closest_p + closest_q) / 2
    return distance, midpoint_of_closest

# --- Phase 1: The Broad Phase (4D Grid Filter) ---

class Grid4D:
    """A class to manage the 4D spatio-temporal grid for broad-phase filtering."""
    def __init__(self, bin_size: Tuple[float, float, float, float]):
        self.x_size, self.y_size, self.z_size, self.t_size = bin_size
        self.grid: Dict[Tuple[int, int, int, int], Set[str]] = {}

    def _get_bin_id(self, x: float, y: float, z: float, t: float) -> Tuple[int, int, int, int]:
        """Calculates the 4D bin ID for a given point."""
        return (
            math.floor(x / self.x_size),
            math.floor(y / self.y_size),
            math.floor(z / self.z_size),
            math.floor(t / self.t_size),
        )

    def add_flight(self, flight: SimulatedFlight):
        """Samples a flight's trajectory and adds its ID to the corresponding grid bins."""
        for i in range(len(flight.waypoints) - 1):
            p1, p2 = flight.waypoints[i], flight.waypoints[i+1]
            t1, t2 = flight.timestamps[i], flight.timestamps[i+1]
            
            # Simple sampling to determine which bins are occupied
            num_samples = int(math.ceil(max(abs(t2-t1), 1)))
            for j in range(num_samples + 1):
                progress = j / num_samples if num_samples > 0 else 0
                curr_t = t1 + progress * (t2 - t1)
                curr_x = p1.x + progress * (p2.x - p1.x)
                curr_y = p1.y + progress * (p2.y - p1.y)
                curr_z = p1.z + progress * (p2.z - p1.z)
                
                bin_id = self._get_bin_id(curr_x, curr_y, curr_z, curr_t)
                if bin_id not in self.grid:
                    self.grid[bin_id] = set()
                self.grid[bin_id].add(flight.flight_id)

    def get_candidate_ids(self, mission: PrimaryMission, etas: List[float]) -> Set[str]:
        """Gets a set of flight IDs that are potential threats to the primary mission."""
        candidates = set()
        for i in range(len(mission.waypoints) - 1):
            p1, p2 = mission.waypoints[i], mission.waypoints[i+1]
            t1, t2 = etas[i], etas[i+1]
            
            num_samples = int(math.ceil(max(abs(t2-t1), 1)))
            for j in range(num_samples + 1):
                progress = j / num_samples if num_samples > 0 else 0
                curr_t = t1 + progress * (t2 - t1)
                curr_x = p1.x + progress * (p2.x - p1.x)
                curr_y = p1.y + progress * (p2.y - p1.y)
                curr_z = p1.z + progress * (p2.z - p1.z)
                
                bin_id = self._get_bin_id(curr_x, curr_y, curr_z, curr_t)
                if bin_id in self.grid:
                    candidates.update(self.grid[bin_id])
        return candidates

# --- The Hybrid Engine Orchestrator ---

def check_conflicts_hybrid(
    primary_mission: PrimaryMission, 
    simulated_flights: List[SimulatedFlight], 
    safety_buffer: float
) -> Dict[str, Any]:
    """
    Checks for conflicts using the two-phase hybrid engine.
    """
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
    # 1. Initialize the grid
    grid = Grid4D(bin_size=(100.0, 100.0, 100.0, 10.0)) # 100m spatial bins, 10s temporal bins
    # 2. Populate the grid with simulated flights
    for flight in simulated_flights:
        grid.add_flight(flight)
    # 3. Get a short list of candidate flights
    candidate_ids = grid.get_candidate_ids(primary_mission, primary_etas)
    candidate_flights = [f for f in simulated_flights if f.flight_id in candidate_ids]
    
    print(f"Broad-phase filter complete. Found {len(candidate_flights)} potential threats out of {len(simulated_flights)} total.")

    # --- Narrow Phase ---
    # 4. Run the precise analytical check on ONLY the candidate flights
    for sim_flight in candidate_flights:
        for i in range(len(primary_mission.waypoints) - 1):
            for j in range(len(sim_flight.waypoints) - 1):
                pri_p1, pri_p2 = primary_mission.waypoints[i], primary_mission.waypoints[i+1]
                sim_p1, sim_p2 = sim_flight.waypoints[j], sim_flight.waypoints[j+1]
                distance, conflict_point = get_closest_points_and_distance_3d(pri_p1, pri_p2, sim_p1, sim_p2)
                if distance < safety_buffer:
                    pri_start_t, pri_end_t = primary_etas[i], primary_etas[i+1]
                    sim_start_t, sim_end_t = sim_flight.timestamps[j], sim_flight.timestamps[j+1]
                    if max(pri_start_t, sim_start_t) < min(pri_end_t, sim_end_t):
                        return { "conflict": True, "flight_id": sim_flight.flight_id, "location": {"x": conflict_point[0], "y": conflict_point[1], "z": conflict_point[2]}, "time": max(pri_start_t, sim_start_t) }

    return {"conflict": False}
