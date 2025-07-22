"""
main.py

[V14 - Final, Refactored, and Production-Ready]
This script serves as the final entry point for the UAV Strategic Deconfliction system.

It uses a professional, configurable architecture and has been refactored to move
all core logic (including ETA calculation) into tested modules within the `src`
package. This script is now purely for orchestration: loading configuration,
defining scenarios, and calling the appropriate library functions.
"""

import json
import numpy as np
import math
from typing import List, Dict, Any

# --- Import project modules ---
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
from src.deconfliction_logic import check_conflicts_hybrid
from src.visualization import create_4d_animation
from src.utils import calculate_etas # <-- IMPORTING THE TESTED FUNCTION

# --- Real-world Drone Speed Constants ---
DJI_MAVIC_4_PRO_SPEED_MPS = 25.0
DJI_AIR_3_SPEED_MPS = 21.0

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

def run_scenario(scenario_name: str, primary_mission: PrimaryMission, simulated_flights: list, config: Dict[str, Any]):
    """
    A helper function to run a deconfliction scenario.
    """
    print(f"\n--- Running Scenario: {scenario_name} ---")
    
    conflict_result = check_conflicts_hybrid(primary_mission, simulated_flights, config)
    
    if conflict_result["conflict"]:
        print(f"Result: CONFLICT DETECTED!")
        print(f"  - With Flight ID: {conflict_result['flight_id']}")
        loc = conflict_result['location']
        print(f"  - At Location (x,y,z): ({loc['x']:.2f}, {loc['y']:.2f}, {loc['z']:.2f})")
        print(f"  - At Time: {conflict_result['time']:.2f}s")
    else:
        print("Result: Mission is CLEAR. No conflicts detected.")
        
    output_filename = f"{scenario_name.replace(' ', '_').lower()}_final.gif"
    print(f"Generating visualization: '{output_filename}'...")
    
    total_primary_dist = sum(np.linalg.norm(np.array([p2.x,p2.y,p2.z]) - np.array([p1.x,p1.y,p1.z])) for p1, p2 in zip(primary_mission.waypoints, primary_mission.waypoints[1:]))
    primary_mission_duration = primary_mission.end_time - primary_mission.start_time
    actual_primary_speed = total_primary_dist / primary_mission_duration if primary_mission_duration > 0 else DJI_MAVIC_4_PRO_SPEED_MPS
    primary_etas_for_viz = calculate_etas(primary_mission.waypoints, primary_mission.start_time, actual_primary_speed)

    create_4d_animation(primary_mission, simulated_flights, primary_etas_for_viz, conflict_result, config, output_filename)
    print("Visualization complete.")

if __name__ == "__main__":
    config_data = load_config()
    deconfliction_config = config_data.get("deconfliction_parameters")
    if not deconfliction_config:
        print("FATAL: 'deconfliction_parameters' not found in config.json.")
        exit(1)

    # === SCENARIO 1: High-Density Showcase with a Conflict ===
    primary_waypoints_showcase = [
        Waypoint(x=500, y=500, z=100), Waypoint(x=300, y=700, z=125),
        Waypoint(x=500, y=500, z=150), Waypoint(x=700, y=700, z=175),
        Waypoint(x=500, y=500, z=200), Waypoint(x=700, y=300, z=225),
        Waypoint(x=500, y=500, z=250),
    ]
    primary_start_time_showcase = 0
    primary_etas_showcase = calculate_etas(primary_waypoints_showcase, primary_start_time_showcase, DJI_MAVIC_4_PRO_SPEED_MPS)
    primary_mission_showcase = PrimaryMission(
        waypoints=primary_waypoints_showcase,
        start_time=primary_start_time_showcase, 
        end_time=int(primary_etas_showcase[-1]) + 1
    )
    simulated_flights_showcase = [
        SimulatedFlight("CONFLICT-VERTICAL", [Waypoint(500, 500, 50), Waypoint(500, 500, 400)], calculate_etas([Waypoint(500, 500, 50), Waypoint(500, 500, 400)], 30, DJI_AIR_3_SPEED_MPS)),
        SimulatedFlight("SURVEY-DRONE", [Waypoint(100, 100, 50), Waypoint(900, 100, 50)], calculate_etas([Waypoint(100, 100, 50), Waypoint(900, 100, 50)], 0, DJI_AIR_3_SPEED_MPS)),
        *[SimulatedFlight(f"GRID_DRONE_{i}", [Waypoint(i*40, 0, 50), Waypoint(i*40, 1000, 50)], calculate_etas([Waypoint(i*40, 0, 50), Waypoint(i*40, 1000, 50)], i*5, DJI_AIR_3_SPEED_MPS)) for i in range(28)]
    ]
    run_scenario("Showcase Scenario", primary_mission_showcase, simulated_flights_showcase, deconfliction_config)

    # === SCENARIO 2: High-Density Conflict-Free Mission ===
    primary_waypoints_clear = [
        Waypoint(x=100, y=100, z=50), Waypoint(x=200, y=800, z=75),
        Waypoint(x=800, y=200, z=100), Waypoint(x=900, y=900, z=125),
    ]
    primary_start_time_clear = 0
    primary_etas_clear = calculate_etas(primary_waypoints_clear, primary_start_time_clear, DJI_MAVIC_4_PRO_SPEED_MPS)
    primary_mission_clear = PrimaryMission(
        waypoints=primary_waypoints_clear,
        start_time=primary_start_time_clear, 
        end_time=int(primary_etas_clear[-1]) + 1
    )
    simulated_flights_clear = [
        SimulatedFlight("CLEAR-PATH-1", [Waypoint(0, 500, 300), Waypoint(1000, 500, 300)], calculate_etas([Waypoint(0, 500, 300), Waypoint(1000, 500, 300)], 0, DJI_AIR_3_SPEED_MPS)),
        SimulatedFlight("CLEAR-PATH-2", [Waypoint(500, 0, 350), Waypoint(500, 1000, 350)], calculate_etas([Waypoint(500, 0, 350), Waypoint(500, 1000, 350)], 100, DJI_AIR_3_SPEED_MPS)),
        SimulatedFlight("TEMPORAL-MISS", [Waypoint(0,0,100), Waypoint(1000,1000,100)], calculate_etas([Waypoint(0,0,100), Waypoint(1000,1000,100)], 300, DJI_AIR_3_SPEED_MPS)),
        *[SimulatedFlight(f"GRID_DRONE_B_{i}", [Waypoint(i*40, 0, 400), Waypoint(i*40, 1000, 400)], calculate_etas([Waypoint(i*40, 0, 400), Waypoint(i*40, 1000, 400)], i*5, DJI_AIR_3_SPEED_MPS)) for i in range(27)]
    ]
    run_scenario("Conflict-Free Scenario", primary_mission_clear, simulated_flights_clear, deconfliction_config)
