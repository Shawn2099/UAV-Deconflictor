"""
main.py

This is the main entry point for the UAV Strategic Deconfliction application.
It sets up different mission scenarios, runs the deconfliction checks,
and generates visualizations of the outcomes.

[V4.2 - Animation Enabled] - Calls the new 4D animation function and
calculates the necessary timing data for it.
"""

import numpy as np
import math
# Import the necessary functions and classes from our 'src' package
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
from src.deconfliction_logic import check_mission_conflicts 
# Import the new 4D animation function
from src.visualization import create_4d_animation 

def run_3d_scenario(scenario_name: str, primary_mission: PrimaryMission, simulated_flights: list, safety_buffer: float):
    """
    A helper function to run and report on a single 3D deconfliction scenario.
    """
    print(f"\n--- Running Scenario: {scenario_name} ---")
    
    # Perform the conflict check
    conflict_result = check_mission_conflicts(primary_mission, simulated_flights, safety_buffer)
    
    # --- Calculate Primary Mission ETAs for Visualization ---
    # This logic is duplicated from the deconfliction engine, but is necessary
    # to pass the timing data to the animation function.
    total_dist = sum(
        np.linalg.norm(np.array([p2.x, p2.y, p2.z]) - np.array([p1.x, p1.y, p1.z]))
        for p1, p2 in zip(primary_mission.waypoints, primary_mission.waypoints[1:])
    )
    mission_duration = primary_mission.end_time - primary_mission.start_time
    primary_speed = total_dist / mission_duration if mission_duration > 0 and total_dist > 0 else float('inf')
    
    primary_etas = [primary_mission.start_time]
    if total_dist > 0:
        for p1, p2 in zip(primary_mission.waypoints, primary_mission.waypoints[1:]):
            segment_dist = np.linalg.norm(np.array([p2.x, p2.y, p2.z]) - np.array([p1.x, p1.y, p1.z]))
            time_for_segment = segment_dist / primary_speed if primary_speed != float('inf') else 0
            primary_etas.append(primary_etas[-1] + time_for_segment)
    else:
        for _ in primary_mission.waypoints[1:]:
            primary_etas.append(primary_mission.start_time)
    
    # Print the results to the console
    if conflict_result["conflict"]:
        print(f"Conflict Detected!")
        print(f"  - With Flight ID: {conflict_result['flight_id']}")
        loc = conflict_result['location']
        print(f"  - Approx. Location (x,y,z): ({loc['x']:.2f}, {loc['y']:.2f}, {loc['z']:.2f})")
        print(f"  - Approx. Time: {conflict_result['time']:.2f}")
    else:
        print("Mission is Clear. No conflicts detected.")
        
    # Generate and save the 4D animation to a file
    output_filename = f"{scenario_name.replace(' ', '_').lower()}.gif" # Save as .gif
    create_4d_animation(primary_mission, simulated_flights, primary_etas, conflict_result, output_filename)


if __name__ == "__main__":
    # Define a constant safety buffer for all scenarios
    SAFETY_BUFFER = 50.0  # 50 meters

    # --- SCENARIO 5: Complex 3D Airspace ---
    primary_mission_5 = PrimaryMission(
        waypoints=[
            Waypoint(x=200, y=200, z=100),
            Waypoint(x=800, y=200, z=100),
            Waypoint(x=800, y=800, z=100),
            Waypoint(x=200, y=800, z=100),
            Waypoint(x=200, y=200, z=100) # Return to start
        ],
        start_time=0,
        end_time=400 # 400 seconds for the mission
    )

    simulated_flights_5 = [
        SimulatedFlight("SF-C1", [Waypoint(0, 500, 300), Waypoint(1000, 500, 300)], [0, 200]),
        SimulatedFlight("SF-C2", [Waypoint(500, 0, 20), Waypoint(500, 1000, 20)], [0, 200]),
        SimulatedFlight("SF-C3", [Waypoint(0, 0, 100), Waypoint(1000, 1000, 100)], [450, 600]),
        SimulatedFlight("SF-C4", [Waypoint(950, 950, 50), Waypoint(960, 960, 50)], [0, 400]),
        SimulatedFlight("SF-C5", [Waypoint(0,0,80), Waypoint(100,100,80), Waypoint(0,200,80)], [10, 50, 100]),
        SimulatedFlight("SF-C6", [Waypoint(50,50,10), Waypoint(50,50,400)], [10, 100]),
        SimulatedFlight("SF-C7", [Waypoint(200, 150, 160), Waypoint(800, 150, 160)], [0, 100]),
        SimulatedFlight("SF-C8", [Waypoint(850, 200, 100), Waypoint(850, 800, 100)], [250, 350]),
        SimulatedFlight("SF-C9", [Waypoint(100, 900, 150), Waypoint(200, 900, 150)], [0, 100]),
        SimulatedFlight(
            flight_id="SF-CONFLICT",
            waypoints=[Waypoint(x=500, y=200, z=50), Waypoint(x=500, y=200, z=150)],
            timestamps=[40, 80] 
        )
    ]
    
    run_3d_scenario("Complex 3D Airspace", primary_mission_5, simulated_flights_5, SAFETY_BUFFER)
