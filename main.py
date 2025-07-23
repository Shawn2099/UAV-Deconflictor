"""
main.py

[V3 - Refactored for Automation and Flexibility]
This script serves as the entry point for the UAV Strategic Deconfliction system.
It now uses argparse to handle command-line arguments for configuration,
scenarios, and visualization, removing all interactive prompts to allow for
full automation.
"""

import json
import argparse
from typing import List, Dict, Any

# --- Import project modules ---
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
from src.deconfliction_logic import check_conflicts_hybrid
from src.visualization import create_4d_animation
from src.utils import calculate_etas

def load_json_file(file_path: str) -> Dict[str, Any]:
    """
    Loads and parses a JSON file, raising exceptions on failure.
    This is more robust than exiting directly, allowing the caller to handle errors.
    """
    try:
        with open(file_path, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        # Re-raise with a more informative message
        raise FileNotFoundError(f"FATAL: The file was not found at '{file_path}'.")
    except json.JSONDecodeError as e:
        # Re-raise with a more informative message
        raise json.JSONDecodeError(f"FATAL: Could not decode JSON from '{file_path}'. Please check its format. Details: {e.msg}", e.doc, e.pos)

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
        # Note: We pass the full config to visualization as it needs the safety_buffer
        primary_etas_for_viz = calculate_etas(primary_mission.waypoints, primary_mission.start_time, primary_drone_speed)
        create_4d_animation(primary_mission, simulated_flights, primary_etas_for_viz, conflict_result, config, output_filename)
        print("Visualization complete.")

def main_runner(args):
    """
    The core logic of the application, now driven by parsed command-line arguments.
    """
    try:
        config_data = load_json_file(args.config)
        scenario_data = load_json_file(args.scenarios)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(e)
        exit(1) # Exit here in the main execution block

    drone_profiles = config_data.get("drone_performance_profiles")
    if not drone_profiles:
        print("FATAL: 'drone_performance_profiles' not found in config.json.")
        exit(1)

    for scenario in scenario_data.get("scenarios", []):
        scenario_name = scenario.get("name", "Unnamed Scenario")
        
        mission_details = scenario.get("primary_mission", {})
        
        try:
            primary_waypoints = [Waypoint(**wp) for wp in mission_details.get("waypoints", [])]
            primary_mission = PrimaryMission(
                waypoints=primary_waypoints,
                start_time=mission_details.get("start_time", 0),
                end_time=mission_details.get("end_time", 3600)
            )
            
            simulated_flights = []
            for flight_data in scenario.get("simulated_flights", []):
                sim_waypoints = [Waypoint(**wp) for wp in flight_data.get("waypoints", [])]
                sim_flight = SimulatedFlight(
                    flight_id=flight_data.get("flight_id", "UNKNOWN"),
                    waypoints=sim_waypoints,
                    timestamps=flight_data.get("timestamps", [])
                )
                simulated_flights.append(sim_flight)

        except ValueError as e:
            # Catch validation errors from our robust data models
            print(f"\n--- SKIPPING Scenario: {scenario_name} ---")
            print(f"Invalid data detected: {e}")
            continue # Move to the next scenario

        primary_drone_model = mission_details.get("drone_model", "")
        primary_drone_speed = drone_profiles.get(primary_drone_model, {}).get("speed_mps", 20.0)

        run_scenario(
            scenario_name,
            primary_mission,
            primary_drone_speed,
            simulated_flights,
            config_data,
            args.visualize
        )

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="UAV Strategic Deconfliction System")
    parser.add_argument(
        '--config', 
        type=str, 
        default='config.json',
        help='Path to the configuration JSON file.'
    )
    parser.add_argument(
        '--scenarios', 
        type=str, 
        default='scenarios.json',
        help='Path to the scenarios JSON file.'
    )
    parser.add_argument(
        '--visualize',
        action='store_true',
        help='Generate GIF visualizations for each scenario.'
    )
    
    parsed_args = parser.parse_args()
    main_runner(parsed_args)
