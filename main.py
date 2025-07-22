"""
main.py

This is the main entry point for the UAV Strategic Deconfliction application.
It sets up different mission scenarios, runs the deconfliction checks,
and generates visualizations of the outcomes.

[V4.1 - Deterministic] - Removes randomness from the complex scenario to ensure
a predictable and reliable test case.
"""

import random
# Import the necessary functions and classes from our 'src' package
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
from src.deconfliction_logic import check_mission_conflicts 
from src.visualization import create_3d_plot 

def run_3d_scenario(scenario_name: str, primary_mission: PrimaryMission, simulated_flights: list, safety_buffer: float):
    """
    A helper function to run and report on a single 3D deconfliction scenario.
    """
    print(f"\n--- Running Scenario: {scenario_name} ---")
    
    # Perform the conflict check using our 3D-updated logic
    conflict_result = check_mission_conflicts(primary_mission, simulated_flights, safety_buffer)
    
    # Print the results to the console
    if conflict_result["conflict"]:
        print(f"Conflict Detected!")
        print(f"  - With Flight ID: {conflict_result['flight_id']}")
        loc = conflict_result['location']
        print(f"  - Approx. Location (x,y,z): ({loc['x']:.2f}, {loc['y']:.2f}, {loc['z']:.2f})")
        print(f"  - Approx. Time: {conflict_result['time']:.2f}")
    else:
        print("Mission is Clear. No conflicts detected.")
        
    # Generate and save the 3D visualization to a file
    output_filename = f"{scenario_name.replace(' ', '_').lower()}.png"
    create_3d_plot(primary_mission, simulated_flights, conflict_result, output_filename)


if __name__ == "__main__":
    # Define a constant safety buffer for all scenarios
    SAFETY_BUFFER = 50.0  # 50 meters

    # --- SCENARIO 5: Complex 3D Airspace ---
    # Primary mission follows a square pattern with 4 waypoints (+ return to start)
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

    # 10 simulated drones with various complex flight patterns
    simulated_flights_5 = [
        # Drone 1: Flies a high-altitude line, no conflict
        SimulatedFlight("SF-C1", [Waypoint(0, 500, 300), Waypoint(1000, 500, 300)], [0, 200]),
        # Drone 2: Flies a low-altitude line, no conflict
        SimulatedFlight("SF-C2", [Waypoint(500, 0, 20), Waypoint(500, 1000, 20)], [0, 200]),
        # Drone 3: Flies a diagonal path, but at a different time, no conflict
        SimulatedFlight("SF-C3", [Waypoint(0, 0, 100), Waypoint(1000, 1000, 100)], [450, 600]),
        # Drone 4: Loiters in a corner, no conflict
        SimulatedFlight("SF-C4", [Waypoint(950, 950, 50), Waypoint(960, 960, 50)], [0, 400]),
        # Drone 5: Flies a zig-zag pattern away from the primary mission
        SimulatedFlight("SF-C5", [Waypoint(0,0,80), Waypoint(100,100,80), Waypoint(0,200,80)], [10, 50, 100]),
        # Drone 6: Flies a vertical path, no conflict
        SimulatedFlight("SF-C6", [Waypoint(50,50,10), Waypoint(50,50,400)], [10, 100]),
        # Drone 7: Flies a path that is spatially close but vertically separated
        SimulatedFlight("SF-C7", [Waypoint(200, 150, 160), Waypoint(800, 150, 160)], [0, 100]),
        # Drone 8: Flies a path that is spatially close but temporally separated
        SimulatedFlight("SF-C8", [Waypoint(850, 200, 100), Waypoint(850, 800, 100)], [250, 350]),
        # Drone 9: Now flies a fixed, non-conflicting path
        SimulatedFlight("SF-C9", [Waypoint(100, 900, 150), Waypoint(200, 900, 150)], [0, 100]),
        
        # DRONE 10: DESIGNED TO CAUSE A CONFLICT
        # This drone flies vertically through the primary mission's path during its flight time
        SimulatedFlight(
            flight_id="SF-CONFLICT",
            waypoints=[
                Waypoint(x=500, y=200, z=50), # Starts below
                Waypoint(x=500, y=200, z=150) # Ends above
            ],
            # Timestamps conflict with the primary mission's first leg (approx time 0-100s)
            timestamps=[40, 80] 
        )
    ]
    
    run_3d_scenario("Complex 3D Airspace", primary_mission_5, simulated_flights_5, SAFETY_BUFFER)
