"""
main.py

[V19 - Final, Corrected Scenario Data]
This script serves as the final entry point for the UAV Strategic Deconfliction system.

The "Showcase Scenario" has been updated with a true, verifiable conflict to
correctly test the now-robust deconfliction engine.
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

def load_config(config_path: str = 'config.json') -> Dict[str, Any]:
    """Loads the simulation configuration from a JSON file."""
    try:
        with open(config_path, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"FATAL: Configuration file not found at '{config_path}'. Please create it.")
        exit(1)
    except json.JSONDecodeError:
        print(f"FATAL: Could not decode JSON from '{config_path}'. Please check its format.")
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
    config_data = load_config()
    deconfliction_config = config_data.get("deconfliction_parameters")
    drone_profiles = config_data.get("drone_performance_profiles")

    if not deconfliction_config or not drone_profiles:
        print("FATAL: 'deconfliction_parameters' or 'drone_performance_profiles' not found in config.json.")
        exit(1)

    viz_choice = input("Do you want to generate GIF visualizations for the scenarios? (y/n): ").lower().strip()
    should_visualize = viz_choice == 'y'

    mavic_speed = drone_profiles.get("DJI_Mavic_4_Pro", {}).get("speed_mps", 25.0)
    air3_speed = drone_profiles.get("DJI_Air_3", {}).get("speed_mps", 21.0)

    # === SCENARIO 1: High-Density Showcase with a TRUE Conflict ===
    primary_waypoints_showcase = [
        Waypoint(x=500, y=500, z=100), Waypoint(x=300, y=700, z=125),
        Waypoint(x=500, y=500, z=150), Waypoint(x=700, y=700, z=175),
        Waypoint(x=500, y=500, z=200), Waypoint(x=700, y=300, z=225),
        Waypoint(x=500, y=500, z=250),
    ]
    primary_start_time_showcase = 0
    primary_mission_showcase = PrimaryMission(
        waypoints=primary_waypoints_showcase,
        start_time=primary_start_time_showcase, 
        end_time=90 
    )

    # *** CORRECTED CONFLICTING FLIGHT DATA ***
    # The primary drone arrives at (500, 500, 150) at approx t=22.7s.
    # This new flight path creates a direct conflict at that point in space and time.
    # It flies from (500, 500, 120) to (500, 500, 180) between t=20s and t=24s.
    # This ensures it passes through z=150 at t=22s, causing a clear conflict.
    true_conflict_flight = SimulatedFlight(
        "CONFLICT-VERTICAL",
        waypoints=[Waypoint(500, 500, 120), Waypoint(500, 500, 180)],
        timestamps=[20, 24] # Speed is 60m / 4s = 15 m/s
    )

    simulated_flights_showcase = [
        true_conflict_flight,
        SimulatedFlight("SURVEY-DRONE", [Waypoint(100, 100, 50), Waypoint(900, 100, 50)], calculate_etas([Waypoint(100, 100, 50), Waypoint(900, 100, 50)], 0, air3_speed)),
        *[SimulatedFlight(f"GRID_DRONE_{i}", [Waypoint(i*40, 0, 50), Waypoint(i*40, 1000, 50)], calculate_etas([Waypoint(i*40, 0, 50), Waypoint(i*40, 1000, 50)], i*5, air3_speed)) for i in range(28)]
    ]
    run_scenario("Showcase Scenario", primary_mission_showcase, mavic_speed, simulated_flights_showcase, config_data, should_visualize)

    # === SCENARIO 2: High-Density Conflict-Free Mission ===
    primary_waypoints_clear = [
        Waypoint(x=100, y=100, z=50), Waypoint(x=200, y=800, z=75),
        Waypoint(x=800, y=200, z=100), Waypoint(x=900, y=900, z=125),
    ]
    primary_start_time_clear = 0
    primary_mission_clear = PrimaryMission(
        waypoints=primary_waypoints_clear,
        start_time=primary_start_time_clear, 
        end_time=100
    )
    simulated_flights_clear = [
        SimulatedFlight("CLEAR-PATH-1", [Waypoint(0, 500, 300), Waypoint(1000, 500, 300)], calculate_etas([Waypoint(0, 500, 300), Waypoint(1000, 500, 300)], 0, air3_speed)),
        SimulatedFlight("CLEAR-PATH-2", [Waypoint(500, 0, 350), Waypoint(500, 1000, 350)], calculate_etas([Waypoint(500, 0, 350), Waypoint(500, 1000, 350)], 100, air3_speed)),
        SimulatedFlight("TEMPORAL-MISS", [Waypoint(0,0,100), Waypoint(1000,1000,100)], calculate_etas([Waypoint(0,0,100), Waypoint(1000,1000,100)], 300, air3_speed)),
        *[SimulatedFlight(f"GRID_DRONE_B_{i}", [Waypoint(i*40, 0, 400), Waypoint(i*40, 1000, 400)], calculate_etas([Waypoint(i*40, 0, 400), Waypoint(i*40, 1000, 400)], i*5, air3_speed)) for i in range(27)]
    ]
    run_scenario("Conflict-Free Scenario", primary_mission_clear, mavic_speed, simulated_flights_clear, config_data, should_visualize)
