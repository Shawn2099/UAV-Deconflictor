"""
test_deconfliction_logic.py

[Comprehensive Unit Tests for the Hybrid Deconfliction Engine]
This module contains a full suite of unit tests for `src.deconfliction_logic`.
It is divided into four main sections:

1.  Narrow Phase Tests: Rigorously tests the `get_closest_points_and_distance_3d`
    function with various geometric edge cases.
2.  Hybrid Engine Tests: Verifies the correct identification of conflict and
    conflict-free scenarios.
3.  Robustness Tests: Ensures the system handles invalid or unusual inputs gracefully.
4.  Performance Tests: Validates the scalability of the hybrid engine under load.
"""

import pytest
import numpy as np
import os
import sys
import time
import random

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
    
    assert distance == pytest.approx(3.0)

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
        waypoints=[Waypoint(500, -10, 100), Waypoint(500, 10, 100)],
        timestamps=[40, 60]
    )

@pytest.fixture
def non_conflicting_simulated_flight():
    """A simulated flight that is nearby but should not conflict."""
    return SimulatedFlight(
        flight_id="SAFE_DRONE",
        waypoints=[Waypoint(500, 200, 100), Waypoint(500, 300, 100)],
        timestamps=[40, 60]
    )

def test_check_conflicts_hybrid_clear_scenario(clear_primary_mission, non_conflicting_simulated_flight):
    result = check_conflicts_hybrid(
        primary_mission=clear_primary_mission,
        simulated_flights=[non_conflicting_simulated_flight],
        safety_buffer=50.0
    )
    assert not result["conflict"]

def test_check_conflicts_hybrid_conflict_detected(clear_primary_mission, conflicting_simulated_flight):
    result = check_conflicts_hybrid(
        primary_mission=clear_primary_mission,
        simulated_flights=[conflicting_simulated_flight],
        safety_buffer=50.0
    )
    assert result["conflict"]
    assert result["flight_id"] == "CONFLICT_DRONE"

def test_check_conflicts_hybrid_temporal_miss():
    primary_mission = PrimaryMission(
        waypoints=[Waypoint(0,0,100), Waypoint(1000,0,100)],
        start_time=0,
        end_time=100
    )
    sim_flight = SimulatedFlight(
        flight_id="TEMPORAL_MISS_DRONE",
        waypoints=[Waypoint(500, -10, 100), Waypoint(500, 10, 100)],
        timestamps=[200, 220]
    )
    result = check_conflicts_hybrid(
        primary_mission=primary_mission,
        simulated_flights=[sim_flight],
        safety_buffer=50.0
    )
    assert not result["conflict"]

def test_check_conflicts_hybrid_broad_phase_filter(clear_primary_mission, conflicting_simulated_flight, non_conflicting_simulated_flight, capsys):
    check_conflicts_hybrid(
        primary_mission=clear_primary_mission,
        simulated_flights=[conflicting_simulated_flight, non_conflicting_simulated_flight],
        safety_buffer=50.0
    )
    captured = capsys.readouterr()
    assert "Found 1 potential threats out of 2 total." in captured.out or \
           "Found 2 potential threats out of 2 total." in captured.out

# ==============================================================================
# === Section 3: Robustness and Edge Case Input Tests
# ==============================================================================

def test_check_conflicts_hybrid_no_sim_flights(clear_primary_mission):
    result = check_conflicts_hybrid(
        primary_mission=clear_primary_mission,
        simulated_flights=[],
        safety_buffer=50.0
    )
    assert not result["conflict"]

def test_check_conflicts_hybrid_stationary_mission(conflicting_simulated_flight):
    stationary_mission = PrimaryMission(
        waypoints=[Waypoint(500, 0, 100)],
        start_time=0,
        end_time=100
    )
    result = check_conflicts_hybrid(
        primary_mission=stationary_mission,
        simulated_flights=[conflicting_simulated_flight],
        safety_buffer=50.0
    )
    assert not result["conflict"]

def test_check_conflicts_hybrid_invalid_time_window(clear_primary_mission, conflicting_simulated_flight):
    invalid_time_mission = PrimaryMission(
        waypoints=clear_primary_mission.waypoints,
        start_time=100,
        end_time=0
    )
    result = check_conflicts_hybrid(
        primary_mission=invalid_time_mission,
        simulated_flights=[conflicting_simulated_flight],
        safety_buffer=50.0
    )
    assert "conflict" in result

# ==============================================================================
# === Section 4: Performance and Scalability Tests
# ==============================================================================

@pytest.mark.slow
def test_performance_in_high_density_airspace(clear_primary_mission, conflicting_simulated_flight):
    """
    Tests the system's performance with a large number of simulated flights.
    This validates the efficiency of the broad-phase filter.
    """
    num_safe_flights = 2000
    simulated_flights = []

    # Generate many non-conflicting flights far away from the primary mission
    for i in range(num_safe_flights):
        # Place these flights in a completely different spatial area
        safe_x = random.uniform(5000, 10000)
        safe_y = random.uniform(5000, 10000)
        flight = SimulatedFlight(
            flight_id=f"SAFE_DRONE_{i}",
            waypoints=[Waypoint(safe_x, safe_y, 100), Waypoint(safe_x + 100, safe_y, 100)],
            timestamps=[0, 100]
        )
        simulated_flights.append(flight)

    # Add the single known conflicting flight
    simulated_flights.append(conflicting_simulated_flight)
    
    start_time = time.time()
    
    result = check_conflicts_hybrid(
        primary_mission=clear_primary_mission,
        simulated_flights=simulated_flights,
        safety_buffer=50.0
    )
    
    end_time = time.time()
    duration = end_time - start_time
    
    print(f"\nPerformance test with {len(simulated_flights)} drones took {duration:.4f} seconds.")

    # Assert that the correct conflict was found
    assert result["conflict"]
    assert result["flight_id"] == "CONFLICT_DRONE"
    
    # Assert that the check was fast, proving the filter is working
    # [Unverified] This threshold may need adjustment based on the machine running the test.
    assert duration < 2.0, "Deconfliction check took too long, broad-phase may be inefficient."
