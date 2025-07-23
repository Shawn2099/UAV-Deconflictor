"""
main.py

[V20 - Data-Driven Scenarios]
This script serves as the entry point for the UAV Strategic Deconfliction system.

It has been refactored to be fully data-driven. All scenario definitions are
loaded from an external `scenarios.json` file, separating the test data from
the application logic.
"""

import json
import numpy as np
import math
from typing import List, Dict, Any

# --- Import project modules ---
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
from src.deconfliction_logic import check_conflicts_hybrid
from src.visualization import create_4d_animation
from src.utils import calculate_etas

def load_json_file(file_path: str) -> Dict[str, Any]:
    """Loads and parses a JSON file."""
    try:
        with open(file_path, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"FATAL: File not found at '{file_path}'.")
        exit(1)
    except json.JSONDecodeError:
        print(f"FATAL: Could not decode JSON from '{file_path}'. Please check its format.")
        exit(1)

def run_scenario(scenario_name: str, primary_mission: PrimaryMission, primary_drone_speed: float, simulated_flights: list, config: Dict[str, Any], generate_visualization: bool):
    """
    A helper function to run a deconfliction scenario with optional visualization.
    """
    print(f"\n--- Running Scenario: {scenario_name} ---")
    
    deconfliction_params = config.get("deconfliction_parameters", {})
    
    conflict_result = check_conflicts_hybrid(
        primary_mission, 
        simulated_flights, 
        primary_drone_speed, 
        deconfliction_params
    )
    
    status = conflict_result.get("status")
    if status == "CONFLICT":
        print(f"Result: CONFLICT DETECTED!")
        print(f"  - With Flight ID: {conflict_result.get('flight_id')}")
        loc = conflict_result.get('location', {})
        print(f"  - At Location (x,y,z): ({loc.get('x', 0):.2f}, {loc.get('y', 0):.2f}, {loc.get('z', 0):.2f})")
        print(f"  - At Time: {conflict_result.get('time', 0):.2f}s")
    elif status == "MISSION_TIME_VIOLATION":
        print(f"Result: MISSION INVALID!")
        print(f"  - Reason: {conflict_result.get('message')}")
    elif status == "CLEAR":
        print("Result: Mission is CLEAR. No conflicts detected.")
    else:
        print("Result: Unknown status returned from deconfliction engine.")

    if generate_visualization:
        output_filename = f"{scenario_name.replace(' ', '_').lower()}_final.gif"
        print(f"Generating visualization: '{output_filename}'...")
        
        primary_etas_for_viz = calculate_etas(primary_mission.waypoints, primary_mission.start_time, primary_drone_speed)

        create_4d_animation(primary_mission, simulated_flights, primary_etas_for_viz, conflict_result, config, output_filename)
        print("Visualization complete.")

if __name__ == "__main__":
    # --- Load Configuration and Scenarios ---
    config_data = load_json_file('config.json')
    scenario_data = load_json_file('scenarios.json')

    drone_profiles = config_data.get("drone_performance_profiles")
    if not drone_profiles:
        print("FATAL: 'drone_performance_profiles' not found in config.json.")
        exit(1)

    # --- Ask user about visualization ---
    viz_choice = input("Do you want to generate GIF visualizations for the scenarios? (y/n): ").lower().strip()
    should_visualize = viz_choice == 'y'

    # --- Loop through scenarios from the file ---
    for scenario in scenario_data.get("scenarios", []):
        scenario_name = scenario.get("name", "Unnamed Scenario")
        
        # --- Parse Primary Mission ---
        mission_details = scenario.get("primary_mission", {})
        primary_waypoints = [Waypoint(**wp) for wp in mission_details.get("waypoints", [])]
        primary_mission = PrimaryMission(
            waypoints=primary_waypoints,
            start_time=mission_details.get("start_time", 0),
            end_time=mission_details.get("end_time", 3600)
        )
        
        # Get the correct speed for the primary drone model
        primary_drone_model = mission_details.get("drone_model", "")
        primary_drone_speed = drone_profiles.get(primary_drone_model, {}).get("speed_mps", 20.0)

        # --- Parse Simulated Flights ---
        simulated_flights = []
        for flight_data in scenario.get("simulated_flights", []):
            sim_waypoints = [Waypoint(**wp) for wp in flight_data.get("waypoints", [])]
            sim_flight = SimulatedFlight(
                flight_id=flight_data.get("flight_id", "UNKNOWN"),
                waypoints=sim_waypoints,
                timestamps=flight_data.get("timestamps", [])
            )
            simulated_flights.append(sim_flight)

        # --- Run the scenario ---
        run_scenario(
            scenario_name,
            primary_mission,
            primary_drone_speed,
            simulated_flights,
            config_data,
            should_visualize
        )
