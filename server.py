"""
server.py

[V8 - User-Defined Safety Buffer]
This script creates a real-time, stateful web server using Flask and Flask-SocketIO.
It now accepts a user-defined safety buffer from the front-end for more
dynamic deconfliction checks.
"""

import time
import json
from flask import Flask, request, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO, emit
import threading

# Import our existing deconfliction logic and data models
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
from src.deconfliction_logic import check_conflicts_hybrid

# --- App and WebSocket Initialization ---
app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

# --- Global State Management ---
SIMULATION_STATE = { "is_running": False }
SIMULATION_THREAD = None
THREAD_LOCK = threading.Lock()

# --- Load Static Data ---
print("Loading configuration and scenario data...")
try:
    with open('config.json', 'r') as f:
        CONFIG_DATA = json.load(f)
    with open('scenarios.json', 'r') as f:
        SCENARIO_FILE_CONTENT = json.load(f)
        SIMULATED_FLIGHTS_DATA = SCENARIO_FILE_CONTENT.get("scenarios", [{}])[0].get("simulated_flights", [])
        
        SIMULATED_FLIGHTS_OBJECTS = []
        for flight_data in SIMULATED_FLIGHTS_DATA:
            flight_waypoints = [Waypoint(**wp) for wp in flight_data.get("waypoints", [])]
            sim_flight = SimulatedFlight(
                flight_id=flight_data.get("flight_id"),
                waypoints=flight_waypoints,
                timestamps=flight_data.get("timestamps", [])
            )
            SIMULATED_FLIGHTS_OBJECTS.append(sim_flight)
            
    print("Data loaded successfully.")
except Exception as e:
    print(f"FATAL: Could not load initial data. Error: {e}")
    exit(1)

# --- Simulation Logic ---
def get_position_at_time(flight, t):
    """Helper to get a single drone's position at a specific time."""
    if t < flight.timestamps[0]:
        wp = flight.waypoints[0]
        return {"x": wp.x, "y": wp.y, "z": wp.z}
    if t > flight.timestamps[-1]:
        wp = flight.waypoints[-1]
        return {"x": wp.x, "y": wp.y, "z": wp.z}
    
    for i in range(len(flight.timestamps) - 1):
        t1, t2 = flight.timestamps[i], flight.timestamps[i+1]
        if t1 <= t <= t2:
            p1, p2 = flight.waypoints[i], flight.waypoints[i+1]
            if t1 == t2: return {"x": p1.x, "y": p1.y, "z": p1.z}
            progress = (t - t1) / (t2 - t1)
            return {
                "x": p1.x + progress * (p2.x - p1.x),
                "y": p1.y + progress * (p2.y - p1.y),
                "z": p1.z + progress * (p2.z - p1.z),
            }
    wp = flight.waypoints[-1]
    return {"x": wp.x, "y": wp.y, "z": wp.z}


def simulation_loop():
    """The main loop that runs in a background thread."""
    global SIMULATION_STATE
    start_time = time.time()
    
    while SIMULATION_STATE.get("is_running"):
        with THREAD_LOCK:
            current_time = time.time() - start_time
            
            current_drone_states = []
            for flight in SIMULATED_FLIGHTS_OBJECTS:
                pos = get_position_at_time(flight, current_time)
                current_drone_states.append({
                    "flight_id": flight.flight_id,
                    "position": pos
                })
            
            socketio.emit('update_state', {
                "time": current_time,
                "drones": current_drone_states
            })
        
        socketio.sleep(0.1)
    print("Simulation loop stopped.")
    socketio.emit('simulation_stopped')


# --- API Endpoints (HTTP) ---
@app.route('/get_scenario_data', methods=['GET'])
def get_scenario_data_endpoint():
    return jsonify(SIMULATED_FLIGHTS_DATA)

@app.route('/check_conflicts', methods=['POST'])
def check_conflicts_endpoint():
    data = request.get_json()
    try:
        primary_mission = PrimaryMission(
            waypoints=[Waypoint(**wp) for wp in data.get("waypoints", [])],
            start_time=data.get("start_time"),
            end_time=data.get("end_time")
        )
        drone_model = data.get("drone_model", "DJI_Mavic_4_Pro")
        drone_speed = CONFIG_DATA["drone_performance_profiles"][drone_model]["speed_mps"]
        
        # --- NEW: Get safety buffer from user, with validation ---
        safety_buffer = data.get("safety_buffer", 50.0)
        if not isinstance(safety_buffer, (int, float)) or safety_buffer < 25:
            return jsonify({"error": "Invalid safety buffer. Must be a number and at least 25."}), 400

        # Create a temporary config for this specific check
        deconfliction_params = CONFIG_DATA["deconfliction_parameters"].copy()
        deconfliction_params["safety_buffer_m"] = safety_buffer
        
        result = check_conflicts_hybrid(
            primary_mission, SIMULATED_FLIGHTS_OBJECTS, drone_speed, deconfliction_params
        )
        return jsonify(result)
    except (ValueError, KeyError, TypeError) as e:
        return jsonify({"error": f"Invalid mission data provided. {str(e)}"}), 400


# --- WebSocket Event Handlers ---
@socketio.on('connect')
def handle_connect():
    print('Client connected')

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

@socketio.on('start_simulation')
def handle_start_simulation():
    global SIMULATION_THREAD
    with THREAD_LOCK:
        if not SIMULATION_STATE.get("is_running"):
            SIMULATION_STATE["is_running"] = True
            SIMULATION_THREAD = socketio.start_background_task(target=simulation_loop)
            print("Simulation started.")

@socketio.on('stop_simulation')
def handle_stop_simulation():
    with THREAD_LOCK:
        SIMULATION_STATE["is_running"] = False
    print("Stop signal received. Halting simulation.")


if __name__ == '__main__':
    print("Starting UAV Live Simulation Server at http://127.0.0.1:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, use_reloader=False)
