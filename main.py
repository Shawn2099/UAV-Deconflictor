"""
main.py

This is the main entry point for the UAV Strategic Deconfliction application.
It sets up different mission scenarios, runs the deconfliction checks,
and generates visualizations of the outcomes.

[V3 - 3D Enabled] - Adds a 3D scenario and uses the 3D visualization module.
"""

# Import the necessary functions and classes from our 'src' package
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
# We still use the same logic function, as it's now 3D-aware
from src.deconfliction_logic import check_mission_conflicts 
# We import the new 3D plotting function
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
        # Updated to include Z coordinate
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

    # --- SCENARIO 4A: 3D Mission - Clear (Altitude Separation) ---
    # Drones cross paths but have sufficient vertical separation
    mission_4a = PrimaryMission(
        waypoints=[
            Waypoint(x=0, y=250, z=100), 
            Waypoint(x=500, y=250, z=100)
        ],
        start_time=0,
        end_time=100
    )
    sim_flights_4a = [
        SimulatedFlight(
            flight_id="SF-04A",
            waypoints=[
                Waypoint(x=250, y=0, z=200), # Flying at 200m altitude
                Waypoint(x=250, y=500, z=200)
            ],
            timestamps=[0, 100] 
        )
    ]
    run_3d_scenario("3D Mission Clear", mission_4a, sim_flights_4a, SAFETY_BUFFER)

    # --- SCENARIO 4B: 3D Mission - Conflict (Altitude Conflict) ---
    # Drones cross paths and are too close vertically
    mission_4b = PrimaryMission(
        waypoints=[
            Waypoint(x=0, y=250, z=100), 
            Waypoint(x=500, y=250, z=100)
        ],
        start_time=0,
        end_time=100
    )
    sim_flights_4b = [
        SimulatedFlight(
            flight_id="SF-04B",
            waypoints=[
                Waypoint(x=250, y=0, z=120), # Flying at 120m altitude (within 50m buffer)
                Waypoint(x=250, y=500, z=120)
            ],
            timestamps=[0, 100] 
        )
    ]
    run_3d_scenario("3D Mission Conflict", mission_4b, sim_flights_4b, SAFETY_BUFFER)

