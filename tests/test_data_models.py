"""
test_data_models.py

Unit tests for the data model classes.
"""

import pytest
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight

# --- Test Cases for Waypoint ---

def test_waypoint_creation():
    """Tests basic Waypoint creation and attribute access."""
    wp = Waypoint(x=10.5, y=-20.0, z=100.0)
    assert wp.x == 10.5
    assert wp.y == -20.0
    assert wp.z == 100.0

def test_waypoint_default_z():
    """Tests that the z-coordinate defaults to 0.0 if not provided."""
    wp = Waypoint(x=5, y=15)
    assert wp.z == 0.0

# --- Test Cases for PrimaryMission ---

def test_primary_mission_creation():
    """Tests valid PrimaryMission creation."""
    wps = [Waypoint(0,0,0), Waypoint(10,10,10)]
    mission = PrimaryMission(waypoints=wps, start_time=100, end_time=200)
    assert mission.waypoints == wps
    assert mission.start_time == 100
    assert mission.end_time == 200

def test_primary_mission_invalid_waypoints():
    """Tests that creating a mission with < 2 waypoints raises ValueError."""
    with pytest.raises(ValueError, match="PrimaryMission must have at least two waypoints."):
        PrimaryMission(waypoints=[Waypoint(0,0,0)], start_time=100, end_time=200)

def test_primary_mission_invalid_time_window():
    """Tests that creating a mission with start_time >= end_time raises ValueError."""
    wps = [Waypoint(0,0,0), Waypoint(10,10,10)]
    with pytest.raises(ValueError, match="must be before end_time"):
        PrimaryMission(waypoints=wps, start_time=200, end_time=100)
    with pytest.raises(ValueError, match="must be before end_time"):
        PrimaryMission(waypoints=wps, start_time=200, end_time=200)

# --- Test Cases for SimulatedFlight ---

def test_simulated_flight_creation():
    """Tests valid SimulatedFlight creation."""
    wps = [Waypoint(0,0,0), Waypoint(10,10,10)]
    ts = [100, 120]
    flight = SimulatedFlight(flight_id="SIM001", waypoints=wps, timestamps=ts)
    assert flight.flight_id == "SIM001"
    assert flight.waypoints == wps
    assert flight.timestamps == ts

def test_simulated_flight_mismatched_lengths():
    """Tests that creating a flight with mismatched waypoints and timestamps raises ValueError."""
    wps = [Waypoint(0,0,0), Waypoint(10,10,10)]
    ts = [100, 120, 140] # Mismatched length
    with pytest.raises(ValueError, match="must match the number of timestamps"):
        SimulatedFlight(flight_id="SIM002", waypoints=wps, timestamps=ts)

