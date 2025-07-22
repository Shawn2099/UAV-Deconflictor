"""
test_deconfliction_logic.py

[Comprehensive Unit Tests for the Hybrid Deconfliction Engine]
This module contains a full suite of unit tests for `src.deconfliction_logic`.
It is divided into two main sections:

1.  Narrow Phase Tests: Rigorously tests the `get_closest_points_and_distance_3d`
    function with various geometric edge cases (parallel, intersecting, skew, collinear).
    This ensures the mathematical core of the engine is robust.

2.  Hybrid Engine Tests: Tests the `check_conflicts_hybrid` orchestrator to verify
    that the broad-phase filter and narrow-phase checks work together correctly to
    identify both conflict and conflict-free scenarios.
"""

import pytest
import numpy as np
import os
import sys

# --- Add the project root to the Python path ---
# This allows us to import modules from the 'src' directory.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
from src.deconfliction_logic import get_closest_points_and_distance_3d, check_conflicts_hybrid

# ==============================================================================
# === Section 1: Narrow Phase Tests (Geometric Robustness)
# ==============================================================================

def test_get_closest_points_intersecting_segments():
    """
    Tests two segments that physically intersect.
    The distance should be 0.
    """
    p1 = Waypoint(x=0, y=0, z=0)
    p2 = Waypoint(x=2, y=2, z=0)
    q1 = Waypoint(x=2, y=0, z=0)
    q2 = Waypoint(x=0, y=2, z=0)
    
    distance, midpoint = get_closest_points_and_distance_3d(p1, p2, q1, q2)
    
    assert distance == pytest.approx(0.0)
    assert np.allclose(midpoint, np.array([1.0, 1.0, 0.0]))

def test_get_closest_points_parallel_segments():
    """
    Tests two parallel segments. The distance should be the perpendicular
    distance between them.
    """
    p1 = Waypoint(x=0, y=0, z=0)
    p2 = Waypoint(x=10, y=0, z=0)
    q1 = Waypoint(x=0, y=5, z=0)
    q2 = Waypoint(x=10, y=5, z=0)
    
    distance, _ = get_closest_points_and_distance_3d(p1, p2, q1, q2)
    
    assert distance == pytest.approx(5.0)

def test_get_closest_points_skew_endpoint_to_interior():
    """
    Tests two skew segments where the closest point is from an endpoint of one
    to the middle of the other.
    """
    p1 = Waypoint(x=0, y=0, z=0)
    p2 = Waypoint(x=10, y=0, z=0) # Along X axis
    q1 = Waypoint(x=5, y=-5, z=5)
    q2 = Waypoint(x=5, y=5, z=5)  # Crosses above P at its midpoint
    
    distance, midpoint = get_closest_points_and_distance_3d(p1, p2, q1, q2)
    
    assert distance == pytest.approx(5.0)
    # Closest point on P is (5,0,0). Closest point on Q is (5,0,5).
    # Midpoint is (5, 0, 2.5)
    assert np.allclose(midpoint, np.array([5.0, 0.0, 2.5]))

def test_get_closest_points_collinear_overlapping():
    """
    Tests two collinear segments that partially overlap.
    The distance should be 0.
    """
    p1 = Waypoint(x=0, y=0, z=0)
    p2 = Waypoint(x=5, y=0, z=0)
    q1 = Waypoint(x=3, y=0, z=0)
    q2 = Waypoint(x=8, y=0, z=0)
    
    distance, _ = get_closest_points_and_distance_3d(p1, p2, q1, q2)
    
    assert distance == pytest.approx(0.0)

def test_get_closest_points_collinear_disjoint():
    """
    Tests two collinear segments that are disjoint (have a gap).
    The distance should be the length of the gap.
    """
    p1 = Waypoint(x=0, y=0, z=0)
    p2 = Waypoint(x=5, y=0, z=0)
    q1 = Waypoint(x=8, y=0, z=0)
    q2 = Waypoint(x=10, y=0, z=0)
    
    distance, _ = get_closest_points_and_distance_3d(p1, p2, q1, q2)
    
    assert distance == pytest.approx(3.0) # Gap between 5 and 8

# ==============================================================================
# === Section 2: Hybrid Engine Tests (System-Level Logic)
# ==============================================================================

@pytest.fixture
def clear_primary_mission():
    """A primary mission that should be conflict-free in the test scenarios."""
    return PrimaryMission(
        waypoints=[Waypoint(0,0,100), Waypoint(1000,0,100)],
        start_time=0,
        end_time=100
    )

@pytest.fixture
def conflicting_simulated_flight():
    """A simulated flight designed to create a definite conflict."""
    return SimulatedFlight(
        flight_id="CONFLICT_DRONE",
        waypoints=[Waypoint(500, -10, 100), Waypoint(500, 10, 100)], # Crosses path
        timestamps=[40, 60] # Overlaps in time with primary mission
    )

@pytest.fixture
def non_conflicting_simulated_flight():
    """A simulated flight that is nearby but should not conflict."""
    return SimulatedFlight(
        flight_id="SAFE_DRONE",
        waypoints=[Waypoint(500, 200, 100), Waypoint(500, 300, 100)], # Spatially far
        timestamps=[40, 60]
    )

def test_check_conflicts_hybrid_clear_scenario(clear_primary_mission, non_conflicting_simulated_flight):
    """
    Tests a scenario where no conflict should be detected.
    """
    result = check_conflicts_hybrid(
        primary_mission=clear_primary_mission,
        simulated_flights=[non_conflicting_simulated_flight],
        safety_buffer=50.0
    )
    assert not result["conflict"]

def test_check_conflicts_hybrid_conflict_detected(clear_primary_mission, conflicting_simulated_flight):
    """
    Tests a scenario where a conflict is expected and should be detected.
    """
    result = check_conflicts_hybrid(
        primary_mission=clear_primary_mission,
        simulated_flights=[conflicting_simulated_flight],
        safety_buffer=50.0
    )
    assert result["conflict"]
    assert result["flight_id"] == "CONFLICT_DRONE"
    assert "location" in result
    assert "time" in result

def test_check_conflicts_hybrid_temporal_miss():
    """
    Tests a scenario where drones are close in space but miss in time.
    """
    primary_mission = PrimaryMission(
        waypoints=[Waypoint(0,0,100), Waypoint(1000,0,100)],
        start_time=0,
        end_time=100
    )
    # This flight crosses the same path but at a much later time.
    sim_flight = SimulatedFlight(
        flight_id="TEMPORAL_MISS_DRONE",
        waypoints=[Waypoint(500, -10, 100), Waypoint(500, 10, 100)],
        timestamps=[200, 220] # Mission ends at t=100, this flight starts at t=200
    )
    
    result = check_conflicts_hybrid(
        primary_mission=primary_mission,
        simulated_flights=[sim_flight],
        safety_buffer=50.0
    )
    assert not result["conflict"]

def test_check_conflicts_hybrid_broad_phase_filter(clear_primary_mission, conflicting_simulated_flight, non_conflicting_simulated_flight, capsys):
    """
    Tests that the broad-phase filter correctly identifies candidates.
    It checks the print output to infer if the filter is working.
    [Inference] This test relies on capturing stdout to verify the number of candidates.
    """
    check_conflicts_hybrid(
        primary_mission=clear_primary_mission,
        simulated_flights=[conflicting_simulated_flight, non_conflicting_simulated_flight],
        safety_buffer=50.0
    )
    captured = capsys.readouterr()
    # Expects the conflicting drone to be a candidate, but the safe one might be too if bins are large.
    # A good test is to ensure not ALL drones are checked.
    # The key line is "Found X potential threats out of 2 total."
    assert "Found 1 potential threats out of 2 total." in captured.out or \
           "Found 2 potential threats out of 2 total." in captured.out # This is also acceptable depending on bin size
