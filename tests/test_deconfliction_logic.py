"""
test_deconfliction_logic.py

[V2 - Updated for Refactored Engine]
This module contains an updated suite of unit tests for the refactored `src.deconfliction_logic`.
It validates the new, physically realistic flight model and conflict detection logic.
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
        "drone_performance_profiles": {
            "DJI_Mavic_4_Pro": {"speed_mps": 20.0}
        },
        "deconfliction_parameters": {
            "safety_buffer_m": 50.0,
            "grid_bin_size": {
                "x_m": 100.0,
                "y_m": 100.0,
                "z_m": 100.0,
                "t_s": 10.0
            }
        }
    }

# ==============================================================================
# === Section 1: Geometric Tests (Unchanged)
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

# ==============================================================================
# === Section 2: NEW Tests for Refactored Hybrid Engine
# ==============================================================================

def test_mission_time_violation(test_config):
    """
    Tests that the system correctly identifies a mission that cannot be completed
    within the specified time window.
    """
    # This mission is 1000m long. At 20 m/s, it takes 50 seconds.
    # The time window is only 40 seconds.
    primary_mission = PrimaryMission(waypoints=[Waypoint(0,0,100), Waypoint(1000,0,100)], start_time=0, end_time=40)
    drone_speed = test_config["drone_performance_profiles"]["DJI_Mavic_4_Pro"]["speed_mps"]
    
    result = check_conflicts_hybrid(primary_mission, [], drone_speed, test_config["deconfliction_parameters"])
    
    assert result["status"] == "MISSION_TIME_VIOLATION"
    assert not result["conflict"]
    assert "Required: 50.00s" in result["message"]

def test_realistic_conflict_scenario(test_config):
    """
    Tests a true spatio-temporal conflict based on the drone's actual speed.
    """
    # Mission: 1000m path, start at t=0. At 20 m/s, it takes 50s.
    # It will reach the midpoint (x=500) at t=25s.
    primary_mission = PrimaryMission(waypoints=[Waypoint(0,0,100), Waypoint(1000,0,100)], start_time=0, end_time=100)
    drone_speed = test_config["drone_performance_profiles"]["DJI_Mavic_4_Pro"]["speed_mps"]

    # This drone flies right through the primary mission's path at the same time.
    # It crosses x=500 at t=25s.
    conflicting_flight = SimulatedFlight(
        flight_id="CONFLICT_DRONE",
        waypoints=[Waypoint(500, -200, 100), Waypoint(500, 200, 100)],
        timestamps=[15, 35] # Speed is 400m / 20s = 20 m/s
    )

    result = check_conflicts_hybrid(primary_mission, [conflicting_flight], drone_speed, test_config["deconfliction_parameters"])

    assert result["status"] == "CONFLICT"
    assert result["conflict"]
    assert result["flight_id"] == "CONFLICT_DRONE"
    assert result["time"] == pytest.approx(25.0)
    assert result["location"]["x"] == pytest.approx(500.0)

def test_realistic_clear_scenario_temporal_miss(test_config):
    """
    Tests a "near miss" where paths are geometrically close but drones pass at different times.
    """
    # Mission: 1000m path, start at t=0. At 20 m/s, it takes 50s.
    # It will reach the midpoint (x=500) at t=25s.
    primary_mission = PrimaryMission(waypoints=[Waypoint(0,0,100), Waypoint(1000,0,100)], start_time=0, end_time=100)
    drone_speed = test_config["drone_performance_profiles"]["DJI_Mavic_4_Pro"]["speed_mps"]

    # This drone flies through the same path, but much later.
    safe_flight = SimulatedFlight(
        flight_id="SAFE_DRONE",
        waypoints=[Waypoint(500, -200, 100), Waypoint(500, 200, 100)],
        timestamps=[80, 100] # Crosses at t=90s
    )

    result = check_conflicts_hybrid(primary_mission, [safe_flight], drone_speed, test_config["deconfliction_parameters"])

    assert result["status"] == "CLEAR"
    assert not result["conflict"]

def test_broad_phase_filter_with_new_logic(test_config, capsys):
    """
    Ensures the broad-phase filter still works, catching potential threats.
    """
    primary_mission = PrimaryMission(waypoints=[Waypoint(0,0,100), Waypoint(1000,0,100)], start_time=0, end_time=100)
    drone_speed = test_config["drone_performance_profiles"]["DJI_Mavic_4_Pro"]["speed_mps"]
    
    conflicting_flight = SimulatedFlight("C1", [Waypoint(500,0,100), Waypoint(500,10,100)], [24,26])
    non_conflicting_flight = SimulatedFlight("NC1", [Waypoint(5000,5000,100), Waypoint(5010,5000,100)], [0,1])
    
    check_conflicts_hybrid(primary_mission, [conflicting_flight, non_conflicting_flight], drone_speed, test_config["deconfliction_parameters"])
    captured = capsys.readouterr()
    
    assert "Found 1 potential threats out of 2 total." in captured.out

@pytest.mark.slow
def test_performance_with_new_logic(test_config):
    """
    Validates that the performance of the refactored engine remains acceptable.
    """
    primary_mission = PrimaryMission(waypoints=[Waypoint(0,0,100), Waypoint(1000,0,100)], start_time=0, end_time=100)
    drone_speed = test_config["drone_performance_profiles"]["DJI_Mavic_4_Pro"]["speed_mps"]

    num_safe_flights = 2000
    simulated_flights = [
        SimulatedFlight("CONFLICT", [Waypoint(500,0,100), Waypoint(510,0,100)], [24,26])
    ]
    for i in range(num_safe_flights):
        safe_x, safe_y = random.uniform(5000, 10000), random.uniform(5000, 10000)
        flight = SimulatedFlight(f"SAFE_{i}", [Waypoint(safe_x, safe_y, 100), Waypoint(safe_x + 10, safe_y, 100)], [0, 10])
        simulated_flights.append(flight)
    
    start_tm = time.time()
    result = check_conflicts_hybrid(primary_mission, simulated_flights, drone_speed, test_config["deconfliction_parameters"])
    duration = time.time() - start_tm
    
    print(f"\nPerformance test with {len(simulated_flights)} drones took {duration:.4f} seconds.")
    assert result["status"] == "CONFLICT"
    assert duration < 2.0, "Deconfliction check took too long."
