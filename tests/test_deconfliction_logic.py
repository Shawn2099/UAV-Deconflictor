"""
test_deconfliction_logic.py

Unit tests for the core deconfliction engine.
[V3 - Corrected] Updated tests to match the refactored, cleaner return
signatures and corrected a geometric assertion.
"""

import pytest
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
from src.deconfliction_logic import check_conflicts_hybrid, get_closest_points_and_distance_3d
import numpy as np

# A common config fixture for tests
@pytest.fixture
def test_config():
    return {
        "safety_buffer_m": 50.0,
        "grid_bin_size": {
            "x_m": 100.0,
            "y_m": 100.0,
            "z_m": 100.0,
            "t_s": 10.0
        }
    }

# --- Tests for check_conflicts_hybrid ---

def test_clear_mission(test_config):
    """Tests a simple mission with no other traffic."""
    primary_mission = PrimaryMission(waypoints=[Waypoint(0,0,100), Waypoint(1000,0,100)], start_time=0, end_time=100)
    result = check_conflicts_hybrid(primary_mission, [], 20.0, test_config)
    assert result["status"] == "CLEAR"

def test_mission_time_violation(test_config):
    """
    Tests that the system correctly identifies a mission that cannot be completed
    within the specified time window.
    """
    # This mission is 1000m long. At 20 m/s, it takes 50 seconds.
    # The time window is only 40 seconds.
    primary_mission = PrimaryMission(waypoints=[Waypoint(0,0,100), Waypoint(1000,0,100)], start_time=0, end_time=40)
    drone_speed = 20.0

    result = check_conflicts_hybrid(primary_mission, [], drone_speed, test_config)

    assert result["status"] == "MISSION_TIME_VIOLATION"

def test_realistic_conflict_scenario(test_config):
    """
    Tests a true spatio-temporal conflict based on the drone's actual speed.
    """
    # Mission: 1000m path, start at t=0. At 20 m/s, it takes 50s.
    # It will reach the midpoint (x=500) at t=25s.
    primary_mission = PrimaryMission(waypoints=[Waypoint(0,0,100), Waypoint(1000,0,100)], start_time=0, end_time=100)
    drone_speed = 20.0

    # This drone flies right through the primary mission's path at the same time.
    # It crosses x=500 at t=25s.
    conflicting_flight = SimulatedFlight(
        flight_id="CONFLICT_DRONE",
        waypoints=[Waypoint(500, -200, 100), Waypoint(500, 200, 100)],
        timestamps=[15, 35] # Speed is 400m / 20s = 20 m/s
    )

    result = check_conflicts_hybrid(primary_mission, [conflicting_flight], drone_speed, test_config)

    assert result["status"] == "CONFLICT"
    assert result["flight_id"] == "CONFLICT_DRONE"
    assert "location" in result
    assert "time" in result

def test_realistic_clear_scenario_temporal_miss(test_config):
    """
    Tests a "near miss" where paths are geometrically close but drones pass at different times.
    """
    # Mission: 1000m path, start at t=0. At 20 m/s, it takes 50s.
    # It will reach the midpoint (x=500) at t=25s.
    primary_mission = PrimaryMission(waypoints=[Waypoint(0,0,100), Waypoint(1000,0,100)], start_time=0, end_time=100)
    drone_speed = 20.0

    # This drone flies through the same path, but much later.
    safe_flight = SimulatedFlight(
        flight_id="SAFE_DRONE",
        waypoints=[Waypoint(500, -200, 100), Waypoint(500, 200, 100)],
        timestamps=[80, 100] # Crosses at t=90s
    )

    result = check_conflicts_hybrid(primary_mission, [safe_flight], drone_speed, test_config)

    assert result["status"] == "CLEAR"

# --- Tests for get_closest_points_and_distance_3d ---

def test_closest_distance_parallel_segments():
    """Tests two parallel line segments."""
    p1, p2 = Waypoint(0, 0, 0), Waypoint(10, 0, 0)
    q1, q2 = Waypoint(0, 5, 0), Waypoint(10, 5, 0)
    dist, _ = get_closest_points_and_distance_3d(p1, p2, q1, q2)
    assert np.isclose(dist, 5.0)

def test_closest_distance_intersecting_segments():
    """Tests two segments that intersect."""
    p1, p2 = Waypoint(0, 0, 0), Waypoint(10, 10, 0)
    q1, q2 = Waypoint(0, 10, 0), Waypoint(10, 0, 0)
    dist, _ = get_closest_points_and_distance_3d(p1, p2, q1, q2)
    assert np.isclose(dist, 0.0)

def test_closest_distance_skew_segments():
    """Tests two skew segments that don't intersect."""
    p1, p2 = Waypoint(0, 0, 0), Waypoint(10, 0, 0)
    q1, q2 = Waypoint(5, 5, 5), Waypoint(5, 5, 15)
    dist, midpoint = get_closest_points_and_distance_3d(p1, p2, q1, q2)
    # The expected distance is sqrt(5^2 + 5^2) = sqrt(50)
    assert np.isclose(dist, np.sqrt(50.0))
    # The midpoint is halfway between (5,0,0) and (5,5,5)
    assert np.allclose(midpoint, [5.0, 2.5, 2.5])

