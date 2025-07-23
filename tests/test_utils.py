"""
test_utils.py

Unit tests for the helper functions in the utils module.
"""

import pytest  # Import pytest to use its features
import math
from src.data_models import Waypoint
from src.utils import calculate_etas

# --- Test Cases for calculate_etas ---

def test_calculate_etas_normal_case():
    """Tests a standard mission with multiple waypoints."""
    waypoints = [Waypoint(0, 0, 0), Waypoint(30, 40, 0)]  # 50m distance
    etas = calculate_etas(waypoints, start_time=10, drone_speed_mps=10)
    assert len(etas) == 2
    assert math.isclose(etas[0], 10.0)
    assert math.isclose(etas[1], 15.0) # 10s (start) + 50m / 10m/s

def test_calculate_etas_empty_mission():
    """Tests an empty list of waypoints."""
    assert calculate_etas([], start_time=100, drone_speed_mps=20) == []

def test_calculate_etas_single_waypoint():
    """Tests a mission with only one waypoint."""
    etas = calculate_etas([Waypoint(1, 2, 3)], start_time=50, drone_speed_mps=20)
    assert etas == [50.0]

def test_calculate_etas_zero_speed():
    """
    Tests that a ValueError is raised for a drone with zero speed.
    This test has been UPDATED to reflect the new, correct behavior.
    """
    waypoints = [Waypoint(0, 0, 0), Waypoint(100, 0, 0)]
    with pytest.raises(ValueError, match="Drone speed must be a positive number."):
        calculate_etas(waypoints, start_time=5, drone_speed_mps=0.0)

def test_calculate_etas_negative_speed():
    """
    Tests that a ValueError is raised for a drone with negative speed.
    This is a new test to ensure robustness.
    """
    waypoints = [Waypoint(0, 0, 0), Waypoint(100, 0, 0)]
    with pytest.raises(ValueError, match="Drone speed must be a positive number."):
        calculate_etas(waypoints, start_time=5, drone_speed_mps=-10.0)

def test_calculate_etas_multi_segment():
    """Tests a more complex path with multiple segments."""
    waypoints = [
        Waypoint(0, 0, 0),
        Waypoint(10, 0, 0),  # 10m
        Waypoint(10, 10, 0)   # 10m
    ]
    etas = calculate_etas(waypoints, start_time=0, drone_speed_mps=5)
    assert len(etas) == 3
    assert math.isclose(etas[0], 0.0)
    assert math.isclose(etas[1], 2.0)  # 0s + 10m / 5m/s
    assert math.isclose(etas[2], 4.0)  # 2s + 10m / 5m/s

