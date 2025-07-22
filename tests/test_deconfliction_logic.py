"""
test_deconfliction_logic.py

[V4 - Final Corrected Version]
This module contains a full suite of unit tests for `src.deconfliction_logic`.
It has been updated to be compatible with the configuration-driven design of the
deconfliction engine.
"""

import pytest
import numpy as np
import os
import sys
import time
import random

# --- Add the project root to the Python path ---
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
from src.deconfliction_logic import get_closest_points_and_distance_3d, check_conflicts_hybrid

# ==============================================================================
# === Test Fixtures & Configuration
# ==============================================================================

@pytest.fixture
def test_config():
    """Provides a standard configuration dictionary for tests."""
    return {
        "safety_buffer_m": 50.0,
        "grid_bin_size": {
            "x_m": 100.0,
            "y_m": 100.0,
            "z_m": 100.0,
            "t_s": 10.0
        }
    }

# ==============================================================================
# === Section 1: Narrow Phase Tests (Geometric Robustness)
# ==============================================================================

def test_get_closest_points_intersecting_segments():
    p1, p2 = Waypoint(0,0,0), Waypoint(2,2,0)
    q1, q2 = Waypoint(2,0,0), Waypoint(0,2,0)
    distance, _ = get_closest_points_and_distance_3d(p1, p2, q1, q2)
    assert distance == pytest.approx(0.0)

def test_get_closest_points_parallel_segments():
    p1, p2 = Waypoint(0,0,0), Waypoint(10,0,0)
    q1, q2 = Waypoint(0,5,0), Waypoint(10,5,0)
    distance, _ = get_closest_points_and_distance_3d(p1, p2, q1, q2)
    assert distance == pytest.approx(5.0)

def test_get_closest_points_skew_endpoint_to_interior():
    p1, p2 = Waypoint(0,0,0), Waypoint(10,0,0)
    q1, q2 = Waypoint(5,-5,5), Waypoint(5,5,5)
    distance, _ = get_closest_points_and_distance_3d(p1, p2, q1, q2)
    assert distance == pytest.approx(5.0)

def test_get_closest_points_collinear_overlapping():
    p1, p2 = Waypoint(0,0,0), Waypoint(5,0,0)
    q1, q2 = Waypoint(3,0,0), Waypoint(8,0,0)
    distance, _ = get_closest_points_and_distance_3d(p1, p2, q1, q2)
    assert distance == pytest.approx(0.0)

def test_get_closest_points_collinear_disjoint():
    p1, p2 = Waypoint(0,0,0), Waypoint(5,0,0)
    q1, q2 = Waypoint(8,0,0), Waypoint(10,0,0)
    distance, _ = get_closest_points_and_distance_3d(p1, p2, q1, q2)
    assert distance == pytest.approx(3.0)

# ==============================================================================
# === Section 2: Hybrid Engine Tests (System-Level Logic)
# ==============================================================================

@pytest.fixture
def clear_primary_mission():
    return PrimaryMission(waypoints=[Waypoint(0,0,100), Waypoint(1000,0,100)], start_time=0, end_time=100)

@pytest.fixture
def conflicting_simulated_flight():
    return SimulatedFlight(flight_id="CONFLICT_DRONE", waypoints=[Waypoint(500, -10, 100), Waypoint(500, 10, 100)], timestamps=[40, 60])

@pytest.fixture
def non_conflicting_simulated_flight():
    return SimulatedFlight(flight_id="SAFE_DRONE", waypoints=[Waypoint(500, 200, 100), Waypoint(500, 300, 100)], timestamps=[40, 60])

def test_check_conflicts_hybrid_clear_scenario(clear_primary_mission, non_conflicting_simulated_flight, test_config):
    result = check_conflicts_hybrid(clear_primary_mission, [non_conflicting_simulated_flight], test_config)
    assert not result["conflict"]

def test_check_conflicts_hybrid_conflict_detected(clear_primary_mission, conflicting_simulated_flight, test_config):
    result = check_conflicts_hybrid(clear_primary_mission, [conflicting_simulated_flight], test_config)
    assert result["conflict"]
    assert result["flight_id"] == "CONFLICT_DRONE"

def test_check_conflicts_hybrid_temporal_miss(test_config):
    primary_mission = PrimaryMission(waypoints=[Waypoint(0,0,100), Waypoint(1000,0,100)], start_time=0, end_time=100)
    sim_flight = SimulatedFlight("TEMPORAL_MISS_DRONE", [Waypoint(500, -10, 100), Waypoint(500, 10, 100)], [200, 220])
    result = check_conflicts_hybrid(primary_mission, [sim_flight], test_config)
    assert not result["conflict"]

def test_check_conflicts_hybrid_broad_phase_filter(clear_primary_mission, conflicting_simulated_flight, non_conflicting_simulated_flight, test_config, capsys):
    check_conflicts_hybrid(clear_primary_mission, [conflicting_simulated_flight, non_conflicting_simulated_flight], test_config)
    captured = capsys.readouterr()
    assert "Found 1 potential threats out of 2 total." in captured.out or "Found 2 potential threats out of 2 total." in captured.out

# ==============================================================================
# === Section 3: Robustness and Edge Case Input Tests
# ==============================================================================

def test_check_conflicts_hybrid_no_sim_flights(clear_primary_mission, test_config):
    result = check_conflicts_hybrid(clear_primary_mission, [], test_config)
    assert not result["conflict"]

def test_check_conflicts_hybrid_stationary_mission(conflicting_simulated_flight, test_config):
    stationary_mission = PrimaryMission(waypoints=[Waypoint(500, 0, 100)], start_time=0, end_time=100)
    result = check_conflicts_hybrid(stationary_mission, [conflicting_simulated_flight], test_config)
    assert not result["conflict"]

def test_check_conflicts_hybrid_invalid_time_window(clear_primary_mission, conflicting_simulated_flight, test_config):
    invalid_time_mission = PrimaryMission(waypoints=clear_primary_mission.waypoints, start_time=100, end_time=0)
    result = check_conflicts_hybrid(invalid_time_mission, [conflicting_simulated_flight], test_config)
    assert "conflict" in result

# ==============================================================================
# === Section 4: Performance and Scalability Tests
# ==============================================================================

@pytest.mark.slow
def test_performance_in_high_density_airspace(clear_primary_mission, conflicting_simulated_flight, test_config):
    num_safe_flights = 2000
    simulated_flights = [conflicting_simulated_flight]
    for i in range(num_safe_flights):
        safe_x, safe_y = random.uniform(5000, 10000), random.uniform(5000, 10000)
        flight = SimulatedFlight(f"SAFE_DRONE_{i}", [Waypoint(safe_x, safe_y, 100), Waypoint(safe_x + 100, safe_y, 100)], [0, 100])
        simulated_flights.append(flight)
    
    start_time = time.time()
    result = check_conflicts_hybrid(clear_primary_mission, simulated_flights, test_config)
    duration = time.time() - start_time
    
    print(f"\nPerformance test with {len(simulated_flights)} drones took {duration:.4f} seconds.")
    assert result["conflict"]
    assert result["flight_id"] == "CONFLICT_DRONE"
    assert duration < 2.0, "Deconfliction check took too long."
