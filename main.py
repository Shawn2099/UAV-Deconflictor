"""
main.py

[V12 - Real-World Drone Speeds & Corrected Scenario Timings] - Uses the new, highly scalable two-phase hybrid
deconfliction engine to run multiple showcase scenarios, now incorporating
real-world speeds for DJI Mavic 4 Pro (primary) and DJI Air 3 (simulated),
with adjusted timings to ensure intended conflict/no-conflict outcomes.
"""

import numpy as np
import math
from typing import List
# Import the necessary functions and classes from our 'src' package
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
# Import the new Hybrid Engine orchestrator
from src.deconfliction_logic import check_conflicts_hybrid 
# Import the enhanced visualization function
from src.visualization import create_4d_animation 

# --- Real-world Drone Speed Constants (Max Horizontal Speed in Sport Mode, windless) ---
# Source: DJI official specifications (approximate values for simulation)
DJI_MAVIC_4_PRO_SPEED_MPS = 25.0 # meters per second
DJI_AIR_3_SPEED_MPS = 21.0       # meters per second

def calculate_etas(mission_waypoints: List[Waypoint], start_time: float, drone_speed_mps: float) -> List[float]:
    """
    Calculates Estimated Times of Arrival (ETAs) for each waypoint based on segment distances
    and a given drone speed.
    """
    if drone_speed_mps <= 0:
        # If speed is zero or negative, assume instantaneous movement or stationary
        return [float(start_time)] * len(mission_waypoints)

    etas = [float(start_time)]
    for p1, p2 in zip(mission_waypoints, mission_waypoints[1:]):
        segment_dist = np.linalg.norm(np.array([p2.x, p2.y, p2.z]) - np.array([p1.x, p1.y, p1.z]))
        time_for_segment = segment_dist / drone_speed_mps
        etas.append(etas[-1] + time_for_segment)
    return etas


def run_hybrid_scenario(scenario_name: str, primary_mission: PrimaryMission, simulated_flights: list, safety_buffer: float):
    """
    A helper function to run a scenario using the Hybrid deconfliction engine.
    """
    print(f"\n--- Running Scenario: {scenario_name} (Hybrid Engine) ---")
    
    # --- Perform the conflict check using the new engine ---
    conflict_result = check_conflicts_hybrid(primary_mission, simulated_flights, safety_buffer)
    
    # --- Print results and visualize ---
    if conflict_result["conflict"]:
        print(f"Conflict Detected!")
        print(f"   - With Flight ID: {conflict_result['flight_id']}")
        loc = conflict_result['location']
        print(f"   - At Location (x,y,z): ({loc['x']:.2f}, {loc['y']:.2f}, {loc['z']:.2f})")
        print(f"   - At Time: {conflict_result['time']:.2f}")
    else:
        print("Mission is Clear. No conflicts detected.")
        
    output_filename = f"{scenario_name.replace(' ', '_').lower()}_hybrid.gif"
    
    # Recalculate primary_etas for visualization using the specific primary drone speed
    # Note: PrimaryMission now has a fixed end_time, so we derive the speed from it for this specific case.
    # For future, it might be better to have primary_mission also define its speed per segment or overall speed.
    total_primary_dist = sum(np.linalg.norm(np.array([p2.x,p2.y,p2.z]) - np.array([p1.x,p1.y,p1.z])) for p1, p2 in zip(primary_mission.waypoints, primary_mission.waypoints[1:]))
    primary_mission_duration = primary_mission.end_time - primary_mission.start_time
    # Avoid division by zero if mission_duration is 0
    actual_primary_speed = total_primary_dist / primary_mission_duration if primary_mission_duration > 0 else DJI_MAVIC_4_PRO_SPEED_MPS
    primary_etas_for_viz = calculate_etas(primary_mission.waypoints, primary_mission.start_time, actual_primary_speed)


    create_4d_animation(primary_mission, simulated_flights, primary_etas_for_viz, conflict_result, output_filename)


if __name__ == "__main__":
    SAFETY_BUFFER = 50.0 # meters

    # --- SCENARIO 1: Showcase Scenario (DJI Mavic 4 Pro vs DJI Air 3s) ---
    print("\n--- Defining Scenario 1: DJI Mavic 4 Pro (Primary) vs DJI Air 3 (Simulated) ---")
    
    # Primary Mission (DJI Mavic 4 Pro)
    primary_waypoints_showcase = [
        Waypoint(x=500, y=500, z=100), Waypoint(x=300, y=700, z=125),
        Waypoint(x=500, y=500, z=150), Waypoint(x=700, y=700, z=175),
        Waypoint(x=500, y=500, z=200), Waypoint(x=700, y=300, z=225),
        Waypoint(x=500, y=500, z=250), # 7 waypoints
    ]
    # Calculate end_time based on Mavic 4 Pro's speed
    primary_start_time_showcase = 0
    primary_etas_showcase = calculate_etas(primary_waypoints_showcase, primary_start_time_showcase, DJI_MAVIC_4_PRO_SPEED_MPS)
    primary_end_time_showcase = primary_etas_showcase[-1] # End time is the ETA of the last waypoint
    
    primary_mission_showcase = PrimaryMission(
        waypoints=primary_waypoints_showcase,
        start_time=primary_start_time_showcase, 
        end_time=int(primary_end_time_showcase) + 1 # Add a small buffer for integer conversion
    )

    # Simulated Flights (DJI Air 3)
    simulated_flights_showcase = [
        SimulatedFlight(
            "CONFLICT-VERTICAL", 
            [Waypoint(500, 500, 50), Waypoint(500, 500, 400)], 
            calculate_etas([Waypoint(500, 500, 50), Waypoint(500, 500, 400)], primary_start_time_showcase + 30, DJI_AIR_3_SPEED_MPS) # Adjusted start time
        ),
        SimulatedFlight("SURVEY-DRONE", [
            Waypoint(100, 100, 50), Waypoint(900, 100, 50),
            Waypoint(900, 200, 50), Waypoint(100, 200, 50), 
        ], calculate_etas([Waypoint(100, 100, 50), Waypoint(900, 100, 50), Waypoint(900, 200, 50), Waypoint(100, 200, 50)], 0, DJI_AIR_3_SPEED_MPS)),
        SimulatedFlight("INSPECTION-DRONE", [
            Waypoint(100, 800, 100), Waypoint(150, 850, 120),
            Waypoint(100, 900, 140), Waypoint(50, 850, 160), 
        ], calculate_etas([Waypoint(100, 800, 100), Waypoint(150, 850, 120), Waypoint(100, 900, 140), Waypoint(50, 850, 160)], 300, DJI_AIR_3_SPEED_MPS)),
        SimulatedFlight("PERIMETER-DRONE", [
            Waypoint(0, 0, 200), Waypoint(1000, 0, 200),
            Waypoint(1000, 1000, 200), Waypoint(0, 1000, 200), 
        ], calculate_etas([Waypoint(0, 0, 200), Waypoint(1000, 0, 200), Waypoint(1000, 1000, 200), Waypoint(0, 1000, 200)], 0, DJI_AIR_3_SPEED_MPS)),
        SimulatedFlight(
            "CONFLICT-TRANSIT", 
            [Waypoint(0, 300, 275), Waypoint(1000, 300, 275)], 
            calculate_etas([Waypoint(0, 300, 275), Waypoint(1000, 300, 275)], primary_start_time_showcase + 20, DJI_AIR_3_SPEED_MPS) # Adjusted start time
        ),
        SimulatedFlight("LOITER-DRONE", [
            Waypoint(800, 800, 400), Waypoint(800, 800, 400) 
        ], calculate_etas([Waypoint(800, 800, 400), Waypoint(800, 800, 400)], 100, DJI_AIR_3_SPEED_MPS)),
        SimulatedFlight("HELIX-DRONE", [
            Waypoint(x=200 + 100*math.cos(math.radians(i*10)), y=200 + 100*math.sin(math.radians(i*10)), z=100 + i*2) for i in range(7) 
        ], calculate_etas([Waypoint(x=200 + 100*math.cos(math.radians(i*10)), y=200 + 100*math.sin(math.radians(i*10)), z=100 + i*2) for i in range(7)], 0, DJI_AIR_3_SPEED_MPS)),
        # Additional drones to reach 30 total in Showcase Scenario
        *[SimulatedFlight(f"DRONE-S{i+1}", [
            Waypoint(x=i*30 + 50, y=100, z=50), Waypoint(x=i*30 + 50, y=900, z=50)
        ], calculate_etas([Waypoint(x=i*30 + 50, y=100, z=50), Waypoint(x=i*30 + 50, y=900, z=50)], 0 + i*10, DJI_AIR_3_SPEED_MPS)) for i in range(23)] 
    ]
    
    run_hybrid_scenario("Showcase Scenario (Mavic 4 Pro vs Air 3)", primary_mission_showcase, simulated_flights_showcase, SAFETY_BUFFER)


    # --- SCENARIO 2: Conflict-Free Mission (DJI Mavic 4 Pro vs DJI Air 3s) ---
    print("\n--- Defining Scenario 2: DJI Mavic 4 Pro (Primary) vs DJI Air 3 (Simulated) ---")
    
    # Primary Mission (DJI Mavic 4 Pro)
    primary_waypoints_clear = [
        Waypoint(x=100, y=100, z=50), Waypoint(x=200, y=800, z=75),
        Waypoint(x=800, y=200, z=100), Waypoint(x=900, y=900, z=125),
        Waypoint(x=500, y=500, z=150), Waypoint(x=100, y=500, z=175),
        Waypoint(x=900, y=100, z=200), # 7 waypoints
    ]
    primary_start_time_clear = 0
    primary_etas_clear = calculate_etas(primary_waypoints_clear, primary_start_time_clear, DJI_MAVIC_4_PRO_SPEED_MPS)
    primary_end_time_clear = primary_etas_clear[-1]
    
    primary_mission_clear = PrimaryMission(
        waypoints=primary_waypoints_clear,
        start_time=primary_start_time_clear, 
        end_time=int(primary_end_time_clear) + 1
    )

    # Simulated Flights (DJI Air 3)
    simulated_flights_clear = [
        SimulatedFlight("CLEAR-PATH-1", [
            Waypoint(x=0, y=500, z=200), Waypoint(x=1000, y=500, z=200) 
        ], calculate_etas([Waypoint(x=0, y=500, z=200), Waypoint(x=1000, y=500, z=200)], 0, DJI_AIR_3_SPEED_MPS)),
        SimulatedFlight("CLEAR-PATH-2", [
            Waypoint(x=500, y=0, z=250), Waypoint(x=500, y=1000, z=250) 
        ], calculate_etas([Waypoint(x=500, y=0, z=250), Waypoint(x=500, y=1000, z=250)], 100, DJI_AIR_3_SPEED_MPS)),
        SimulatedFlight("CLEAR-PATH-3", [
            Waypoint(100, 100, 300), Waypoint(200, 200, 300), 
            Waypoint(300, 100, 300) 
        ], calculate_etas([Waypoint(100, 100, 300), Waypoint(200, 200, 300), Waypoint(300, 100, 300)], 0, DJI_AIR_3_SPEED_MPS)),
        SimulatedFlight("HIGH-ALT-LOITER", [
            Waypoint(x=700, y=700, z=400), Waypoint(x=700, y=700, z=400) 
        ], calculate_etas([Waypoint(x=700, y=700, z=400), Waypoint(x=700, y=700, z=400)], 50, DJI_AIR_3_SPEED_MPS)),
        SimulatedFlight("SPIRAL-UP", [
            Waypoint(x=500 + 50*math.cos(math.radians(i*20)), y=500 + 50*math.sin(math.radians(i*20)), z=i*5) for i in range(7) 
        ], calculate_etas([Waypoint(x=500 + 50*math.cos(math.radians(i*20)), y=500 + 50*math.sin(math.radians(i*20)), z=i*5) for i in range(7)], 0, DJI_AIR_3_SPEED_MPS)),
        SimulatedFlight("CROSS-PATH", [
            Waypoint(x=0, y=0, z=100), Waypoint(x=1000, y=1000, z=100) 
        ], calculate_etas([Waypoint(x=0, y=0, z=100), Waypoint(x=1000, y=1000, z=100)], primary_start_time_clear + 200, DJI_AIR_3_SPEED_MPS)), # Adjusted start time
        # Additional drones to reach 30 total in Conflict-Free Scenario
        *[SimulatedFlight(f"DRONE-C{i+1}", [
            Waypoint(x=900 - i*20, y=100, z=200), Waypoint(x=900 - i*20, y=900, z=200)
        ], calculate_etas([Waypoint(x=900 - i*20, y=100, z=200), Waypoint(x=900 - i*20, y=900, z=200)], 0 + i*10, DJI_AIR_3_SPEED_MPS)) for i in range(24)] 
    ]

    run_hybrid_scenario("Conflict-Free Scenario (Mavic 4 Pro vs Air 3)", primary_mission_clear, simulated_flights_clear, SAFETY_BUFFER)
