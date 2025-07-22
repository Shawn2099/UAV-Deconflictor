"""
main.py

[V7 - Hybrid Engine] - Uses the new, highly scalable two-phase hybrid
deconfliction engine to run the showcase scenario.
"""

import numpy as np
import math
# Import the necessary functions and classes from our 'src' package
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
# Import the new Hybrid Engine orchestrator
from src.deconfliction_logic import check_conflicts_hybrid 
from src.visualization import create_4d_animation 

def run_hybrid_scenario(scenario_name: str, primary_mission: PrimaryMission, simulated_flights: list, safety_buffer: float):
    """
    A helper function to run a scenario using the Hybrid deconfliction engine.
    """
    print(f"\n--- Running Scenario: {scenario_name} (Hybrid Engine) ---")
    
    # --- Perform the conflict check using the new engine ---
    conflict_result = check_conflicts_hybrid(primary_mission, simulated_flights, safety_buffer)
    
    # --- Calculate Primary Mission ETAs for Visualization ---
    # This logic is duplicated but necessary for the animation function.
    # In a real app, this would be calculated once and passed around.
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
    
    # --- Print results and visualize ---
    if conflict_result["conflict"]:
        print(f"Conflict Detected!")
        print(f"  - With Flight ID: {conflict_result['flight_id']}")
        loc = conflict_result['location']
        print(f"  - At Location (x,y,z): ({loc['x']:.2f}, {loc['y']:.2f}, {loc['z']:.2f})")
        print(f"  - At Time: {conflict_result['time']:.2f}")
    else:
        print("Mission is Clear. No conflicts detected.")
        
    output_filename = f"{scenario_name.replace(' ', '_').lower()}_hybrid.gif"
    create_4d_animation(primary_mission, simulated_flights, primary_etas, conflict_result, output_filename)


if __name__ == "__main__":
    SAFETY_BUFFER = 50.0

    # --- SHOWCASE SCENARIO (using the new Hybrid Engine) ---
    primary_mission_showcase = PrimaryMission(
        waypoints=[
            Waypoint(x=500, y=500, z=100), Waypoint(x=300, y=700, z=125),
            Waypoint(x=500, y=500, z=150), Waypoint(x=700, y=700, z=175),
            Waypoint(x=500, y=500, z=200), Waypoint(x=700, y=300, z=225),
            Waypoint(x=500, y=500, z=250), Waypoint(x=300, y=300, z=275),
            Waypoint(x=500, y=500, z=300),
        ],
        start_time=0, end_time=800 
    )

    simulated_flights_showcase = [
        SimulatedFlight(
            "CONFLICT-VERTICAL", 
            [Waypoint(500, 500, 50), Waypoint(500, 500, 400)], 
            [180, 280]
        ),
        SimulatedFlight("SURVEY-DRONE", [
            Waypoint(100, 100, 50), Waypoint(900, 100, 50),
            Waypoint(900, 200, 50), Waypoint(100, 200, 50),
            Waypoint(100, 300, 50), Waypoint(900, 300, 50),
        ], [0, 80, 90, 170, 180, 260]),
        SimulatedFlight("INSPECTION-DRONE", [
            Waypoint(100, 800, 100), Waypoint(150, 850, 120),
            Waypoint(100, 900, 140), Waypoint(50, 850, 160),
            Waypoint(100, 800, 180), Waypoint(150, 850, 200),
        ], [300, 320, 340, 360, 380, 400]),
        SimulatedFlight("PERIMETER-DRONE", [
            Waypoint(0, 0, 200), Waypoint(1000, 0, 200),
            Waypoint(1000, 1000, 200), Waypoint(0, 1000, 200),
            Waypoint(0, 0, 200),
        ], [0, 200, 400, 600, 800]),
        SimulatedFlight(
            "CONFLICT-TRANSIT", 
            [Waypoint(0, 300, 275), Waypoint(1000, 300, 275)], 
            [650, 750]
        ),
        SimulatedFlight("LOITER-DRONE", [
            Waypoint(800, 800, 400), Waypoint(800, 800, 400)
        ], [100, 500]),
    ]
    
    run_hybrid_scenario("Showcase Scenario", primary_mission_showcase, simulated_flights_showcase, SAFETY_BUFFER)
