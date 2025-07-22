"""
test_data_models.py

[Unit Tests for Core Data Structures]
This module contains unit tests for the data models defined in `src.data_models`.
It verifies that the dataclass models for Waypoint, PrimaryMission, and SimulatedFlight
are correctly instantiated and that their attributes are properly handled.

These tests ensure the fundamental building blocks of our system are reliable.
"""

import pytest
import os
import sys

# --- Add the project root to the Python path ---
# This allows us to import modules from the 'src' directory.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.data_models import Waypoint, PrimaryMission, SimulatedFlight

# --- Test Fixtures for Reusable Data ---

@pytest.fixture
def sample_waypoint():
    """Provides a sample Waypoint object for testing."""
    return Waypoint(x=10.0, y=20.0, z=30.0)

@pytest.fixture
def sample_waypoints_list():
    """Provides a list of sample Waypoint objects."""
    return [
        Waypoint(x=0, y=0, z=100),
        Waypoint(x=100, y=0, z=100),
        Waypoint(x=100, y=100, z=100)
    ]

# --- Unit Tests for Waypoint ---

def test_waypoint_creation(sample_waypoint):
    """
    Tests the successful creation of a Waypoint object and verifies its attributes.
    """
    assert sample_waypoint.x == 10.0
    assert sample_waypoint.y == 20.0
    assert sample_waypoint.z == 30.0

def test_waypoint_default_z():
    """
    Tests that the z attribute defaults to 0.0 if not provided.
    """
    waypoint = Waypoint(x=5, y=15)
    assert waypoint.z == 0.0

# --- Unit Tests for PrimaryMission ---

def test_primary_mission_creation(sample_waypoints_list):
    """
    Tests the successful creation of a PrimaryMission object.
    """
    mission = PrimaryMission(
        waypoints=sample_waypoints_list,
        start_time=0,
        end_time=100
    )
    assert len(mission.waypoints) == 3
    assert mission.start_time == 0
    assert mission.end_time == 100
    assert mission.waypoints[1].x == 100.0

# --- Unit Tests for SimulatedFlight ---

def test_simulated_flight_creation(sample_waypoints_list):
    """
    Tests the successful creation of a SimulatedFlight object.
    """
    flight = SimulatedFlight(
        flight_id="SIM_DRONE_001",
        waypoints=sample_waypoints_list,
        timestamps=[0, 50, 100]
    )
    assert flight.flight_id == "SIM_DRONE_001"
    assert len(flight.waypoints) == 3
    assert len(flight.timestamps) == 3
    assert flight.timestamps[2] == 100
