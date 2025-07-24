"""
Unit tests for the core deconfliction engine.
[V4 - Corrected] Updated tests to match the refactored, cleaner return
signatures and removed tests for a non-existent function.
"""

import pytest
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
from src.deconfliction_logic import check_conflicts_hybrid

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
