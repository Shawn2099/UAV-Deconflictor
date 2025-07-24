"""
test_server.py

API Integration tests for the Flask/SocketIO backend server.
This test suite validates the behavior of the HTTP endpoints and
the real-time WebSocket events.
"""

import pytest
from server import app, socketio
import time

# --- Fixture to set up the test clients ---
@pytest.fixture
def client():
    """Create a test client for the Flask app and SocketIO server."""
    # Use the test clients provided by the libraries
    http_client = app.test_client()
    socketio_client = socketio.test_client(app)
    
    # 'yield' provides the clients to the test function
    yield http_client, socketio_client
    
    # Teardown code (runs after the test is complete)
    if socketio_client.is_connected():
        socketio_client.disconnect()

# --- Tests for HTTP API Endpoints ---

def test_get_scenario_data_endpoint(client):
    """Tests that the /get_scenario_data endpoint returns valid data."""
    http_client, _ = client
    response = http_client.get('/get_scenario_data')
    
    assert response.status_code == 200
    data = response.get_json()
    assert isinstance(data, list)
    assert len(data) > 0 # Check that it's not an empty list
    assert "flight_id" in data[0] # Check for expected keys

def test_check_conflicts_clear(client):
    """Tests the /check_conflicts endpoint with a mission that should be clear."""
    http_client, _ = client
    clear_mission = {
        "waypoints": [{"x": 10, "y": 10, "z": 150}, {"x": 30, "y": 10, "z": 150}],
        "start_time": 250,
        "end_time": 300,
        "drone_model": "DJI_Mavic_4_Pro",
        "safety_buffer": 50
    }
    response = http_client.post('/check_conflicts', json=clear_mission)
    
    assert response.status_code == 200
    result = response.get_json()
    assert result["status"] == "CLEAR"

def test_check_conflicts_with_conflict(client):
    """Tests the /check_conflicts endpoint with a mission that should have a conflict."""
    http_client, _ = client
    # This mission is designed to conflict with "Spiral_Ascent_1"
    # which is at (x=40, y=40, z=50) at t=40s.
    conflict_mission = {
        "waypoints": [{"x": 40, "y": 20, "z": 50}, {"x": 40, "y": 60, "z": 50}],
        "start_time": 38,
        "end_time": 42,
        "drone_model": "DJI_Mavic_4_Pro",
        "safety_buffer": 50
    }
    response = http_client.post('/check_conflicts', json=conflict_mission)
    
    assert response.status_code == 200
    result = response.get_json()
    assert result["status"] == "CONFLICT"
    assert result["flight_id"] == "Spiral_Ascent_1"

def test_check_conflicts_invalid_buffer(client):
    """Tests that the server rejects a mission with an invalid safety buffer."""
    http_client, _ = client
    invalid_mission = {
        "waypoints": [{"x": 10, "y": 10, "z": 150}, {"x": 30, "y": 10, "z": 150}],
        "start_time": 250,
        "end_time": 300,
        "drone_model": "DJI_Mavic_4_Pro",
        "safety_buffer": 10 # Buffer is less than the 25m minimum
    }
    response = http_client.post('/check_conflicts', json=invalid_mission)
    
    assert response.status_code == 400 # Bad Request
    result = response.get_json()
    assert "error" in result
    assert "Invalid safety buffer" in result["error"]

# --- Tests for WebSocket Events ---

def test_websocket_connection(client):
    """Tests if a client can successfully connect via WebSockets."""
    _, socketio_client = client
    assert socketio_client.is_connected()

def test_simulation_start_and_update(client):
    """Tests the full simulation start -> update -> stop lifecycle."""
    _, socketio_client = client
    
    # Start the simulation
    socketio_client.emit('start_simulation')
    
    # Give the server a moment to start the loop and emit an event
    time.sleep(0.2)
    
    # Wait for the first 'update_state' event from the server
    received_events = socketio_client.get_received()
    
    # Check that we received an update
    assert len(received_events) > 0
    update_event = received_events[0]
    assert update_event['name'] == 'update_state'
    
    # Check the structure of the update data
    data = update_event['args'][0]
    assert 'time' in data
    assert 'drones' in data
    assert data['time'] > 0
    assert len(data['drones']) > 0
    
    # Stop the simulation
    socketio_client.emit('stop_simulation')
    
    # Give the server a moment to process the stop and emit the confirmation
    time.sleep(0.2)
    
    # Check for the 'simulation_stopped' confirmation event
    stopped_event = socketio_client.get_received()
    assert len(stopped_event) > 0
    assert stopped_event[0]['name'] == 'simulation_stopped'

