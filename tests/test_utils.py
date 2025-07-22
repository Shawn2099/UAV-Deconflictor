"""
test_utils.py

Unit tests for the helper functions in `src.utils`.
This test suite specifically validates the `calculate_etas` function to ensure
its correctness under various conditions, including edge cases.
"""

import pytest
import os
import sys

# Add the project root to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.data_models import Waypoint
from src.utils import calculate_etas

def test_calculate_etas_normal_scenario():
    """Tests a standard, multi-segment path."""
    waypoints = [Waypoint(0,0,0), Waypoint(100,0,0), Waypoint(100,100,0)]
    # Total distance: 100m + 100m = 200m
    # Speed: 10 m/s
    # Expected times: 0s, 10s, 20s
    etas = calculate_etas(waypoints, start_time=0, drone_speed_mps=10.0)
    assert len(etas) == 3
    assert etas == pytest.approx([0.0, 10.0, 20.0])

def test_calculate_etas_single_waypoint():
    """Tests a mission with only one waypoint (hovering)."""
    waypoints = [Waypoint(50,50,50)]
    etas = calculate_etas(waypoints, start_time=10, drone_speed_mps=20.0)
    assert etas == [10.0]

def test_calculate_etas_zero_speed():
    """Tests a drone with zero speed. All ETAs should be the start time."""
    waypoints = [Waypoint(0,0,0), Waypoint(100,0,0)]
    etas = calculate_etas(waypoints, start_time=5, drone_speed_mps=0.0)
    assert etas == [5.0, 5.0]

def test_calculate_etas_empty_waypoints():
    """Tests with an empty list of waypoints. Should return an empty list."""
    waypoints = []
    etas = calculate_etas(waypoints, start_time=0, drone_speed_mps=10.0)
    assert etas == []

def test_calculate_etas_no_movement():
    """Tests a path with identical waypoints. Time should not advance."""
    waypoints = [Waypoint(10,10,10), Waypoint(10,10,10), Waypoint(10,10,10)]
    etas = calculate_etas(waypoints, start_time=0, drone_speed_mps=10.0)
    assert etas == pytest.approx([0.0, 0.0, 0.0])
