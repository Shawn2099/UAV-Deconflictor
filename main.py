"""
main.py

This is the main entry point for the UAV Strategic Deconfliction application.
It sets up different mission scenarios, runs the deconfliction checks,
and generates visualizations of the outcomes.

[V6 - Showcase Scenario] - A highly complex, deterministic scenario designed
to be a definitive showcase of the system's capabilities. Features intricate
flight paths and multiple, clearly identifiable conflict points.
"""

import numpy as np
import math
# Import the necessary functions and classes from our 'src' package
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
from src.deconfliction_logic import check_mission_conflicts 
from src.visualization import create_4d_animation 

def run_3d_scenario(scenario_name: str, primary_mission: PrimaryMission, simulated_flights: list, safety_buffer: float):
    """
    A helper function to run and report on a single 3D deconfliction scenario.
    """
    print(f"\n--- Running Scenario: {scenario_name} ---")
    
    # Perform the conflict check
    conflict_result = check_mission_conflicts(primary_mission, simulated_flights, safety_buffer)
    
    # --- Calculate Primary Mission ETAs for Visualization ---
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
    output_filename = f"{scenario_name.replace(' ', '_').lower()}.gif"
    create_4d_animation(primary_mission, simulated_flights, primary_etas, conflict_result, output_filename)


if __name__ == "__main__":
    # Define a constant safety buffer for all scenarios
    SAFETY_BUFFER = 50.0  # 50 meters

    # --- SHOWCASE SCENARIO: The Final, Complex Test ---
    # Primary mission flies a complex "Clover" pattern with altitude changes.
    primary_mission_showcase = PrimaryMission(
        waypoints=[
            Waypoint(x=500, y=500, z=100),
            Waypoint(x=300, y=700, z=125),
            Waypoint(x=500, y=500, z=150), # Return to center, higher
            Waypoint(x=700, y=700, z=175),
            Waypoint(x=500, y=500, z=200), # Return to center, higher
            Waypoint(x=700, y=300, z=225),
            Waypoint(x=500, y=500, z=250), # Return to center, higher
            Waypoint(x=300, y=300, z=275),
            Waypoint(x=500, y=500, z=300), # Final point
        ],
        start_time=0,
        end_time=800 
    )

    simulated_flights_showcase = [
        # CONFLICT DRONE 1: Ascends vertically through the center hub of the clover.
        # This will conflict with the primary mission multiple times, but the first
        # conflict should be detected on the first return to center.
        SimulatedFlight(
            "CONFLICT-VERTICAL", 
            [Waypoint(500, 500, 50), Waypoint(500, 500, 400)], 
            [180, 280] # Timed to conflict with the primary's return to center at t=200
        ),

        # SURVEY DRONE: Flies a "lawnmower" grid pattern at a low, safe altitude.
        SimulatedFlight("SURVEY-DRONE", [
            Waypoint(100, 100, 50), Waypoint(900, 100, 50),
            Waypoint(900, 200, 50), Waypoint(100, 200, 50),
            Waypoint(100, 300, 50), Waypoint(900, 300, 50),
        ], [0, 80, 90, 170, 180, 260]),

        # INSPECTION DRONE: Flies a helical (corkscrew) path upwards around a point of interest.
        SimulatedFlight("INSPECTION-DRONE", [
            Waypoint(100, 800, 100), Waypoint(150, 850, 120),
            Waypoint(100, 900, 140), Waypoint(50, 850, 160),
            Waypoint(100, 800, 180), Waypoint(150, 850, 200),
        ], [300, 320, 340, 360, 380, 400]),

        # PERIMETER DRONE: Patrols the outer edges of the airspace.
        SimulatedFlight("PERIMETER-DRONE", [
            Waypoint(0, 0, 200), Waypoint(1000, 0, 200),
            Waypoint(1000, 1000, 200), Waypoint(0, 1000, 200),
            Waypoint(0, 0, 200),
        ], [0, 200, 400, 600, 800]),

        # CONFLICT DRONE 2: High-speed transit drone that cuts across the final leaf of the clover.
        # This conflict will likely not be reported because the vertical conflict happens first.
        SimulatedFlight(
            "CONFLICT-TRANSIT", 
            [Waypoint(0, 300, 275), Waypoint(1000, 300, 275)], 
            [650, 750] # Timed to conflict with the primary's last leg
        ),
        
        # LOITERING DRONE: Hovers at a waypoint for an extended period.
        SimulatedFlight("LOITER-DRONE", [
            Waypoint(800, 800, 400), Waypoint(800, 800, 400)
        ], [100, 500]), # Stays in one spot from t=100 to t=500
    ]
    
    run_3d_scenario("Showcase Scenario", primary_mission_showcase, simulated_flights_showcase, SAFETY_BUFFER)
