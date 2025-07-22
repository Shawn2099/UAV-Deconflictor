"""
visualization.py

[V4.1 - Corrected Import] This module provides functions to visualize the
drone mission scenarios in 3D space. It creates an animated GIF and marks the
conflict zone with a transparent red circle for better visibility.
"""
import matplotlib
# Set the backend to 'Agg' to prevent GUI-related errors.
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Dict, Any, Optional
import numpy as np

# Import our custom data models
# CORRECTED LINE: Added 'Waypoint' to the import statement
from .data_models import PrimaryMission, SimulatedFlight, Waypoint

class DynamicDrone:
    """A helper class to manage the state and position of a drone over time."""
    def __init__(self, waypoints: List[Waypoint], timestamps: List[float], color: str, marker: str, label: str):
        self.waypoints = waypoints
        self.timestamps = timestamps
        self.color = color
        self.marker = marker
        self.label = label
        self.path_line = None # To hold the static full path plot object
        self.point_marker = None # To hold the moving drone marker plot object

    def get_position(self, time: float) -> Optional[np.ndarray]:
        """Calculates the drone's 3D position at a specific time."""
        if not self.timestamps or time < self.timestamps[0] or time > self.timestamps[-1]:
            return None # Drone is not active

        for i in range(len(self.timestamps) - 1):
            t1, t2 = self.timestamps[i], self.timestamps[i+1]
            if t1 <= time <= t2:
                p1 = np.array([self.waypoints[i].x, self.waypoints[i].y, self.waypoints[i].z])
                p2 = np.array([self.waypoints[i+1].x, self.waypoints[i+1].y, self.waypoints[i+1].z])
                
                if t1 == t2: # Handle case where a drone hovers
                    return p1
                
                progress = (time - t1) / (t2 - t1)
                return p1 + progress * (p2 - p1)
        return None

def create_4d_animation(
    primary_mission: PrimaryMission, 
    simulated_flights: List[SimulatedFlight],
    primary_mission_etas: List[float],
    conflict_details: Optional[Dict[str, Any]] = None,
    output_filename: Optional[str] = None,
    duration_seconds: int = 15 # Increased default duration for complex scenarios
):
    """
    Generates and saves a 4D (3D + time) animation of the airspace scenario.
    """
    fig = plt.figure(figsize=(14, 14))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("4D Airspace Visualization", fontsize=16)
    ax.set_xlabel("X Coordinate (meters)")
    ax.set_ylabel("Y Coordinate (meters)")
    ax.set_zlabel("Z Coordinate (Altitude in meters)")

    drones = []
    drones.append(DynamicDrone(primary_mission.waypoints, primary_mission_etas, 'blue', 'o', 'Primary Mission'))
    for flight in simulated_flights:
        drones.append(DynamicDrone(flight.waypoints, flight.timestamps, 'gray', 's', f'Sim Flight: {flight.flight_id}'))

    # Plot static elements
    for drone in drones:
        x = [wp.x for wp in drone.waypoints]
        y = [wp.y for wp in drone.waypoints]
        z = [wp.z for wp in drone.waypoints]
        drone.path_line, = ax.plot(x, y, z, '--', color=drone.color, linewidth=1, alpha=0.5)
        drone.point_marker, = ax.plot([], [], [], drone.marker, color=drone.color, markersize=8, label=drone.label)

    # Highlight the conflict zone with a transparent circle
    if conflict_details and conflict_details.get("conflict"):
        loc = conflict_details["location"]
        ax.plot(
            [loc["x"]], [loc["y"]], [loc["z"]],
            marker='o',
            markersize=35,
            markerfacecolor='none',      # Make the center transparent
            markeredgecolor='red',       # Keep the edge red
            markeredgewidth=3,           # Make the edge thick and visible
            label="Conflict Zone"
        )

    time_text = ax.text2D(0.05, 0.95, '', transform=ax.transAxes, fontsize=14)
    
    # Filter out duplicate labels for a cleaner legend
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys())
    
    # Determine the total simulation time based on the last event of any drone
    all_end_times = [d.timestamps[-1] for d in drones if d.timestamps]
    max_time = max(all_end_times) if all_end_times else 0

    # Animation update function
    def update(frame):
        total_frames = duration_seconds * 15 # 15 fps
        current_time = (frame / total_frames) * max_time if max_time > 0 else 0
        
        for drone in drones:
            pos = drone.get_position(current_time)
            if pos is not None:
                drone.point_marker.set_data_3d([pos[0]], [pos[1]], [pos[2]])
                drone.point_marker.set_visible(True)
            else:
                drone.point_marker.set_visible(False)
        
        time_text.set_text(f'Time: {current_time:.2f}s')
        return [d.point_marker for d in drones] + [time_text]

    total_frames = duration_seconds * 15
    anim = FuncAnimation(fig, update, frames=total_frames, blit=True)

    if output_filename:
        print(f"Saving animation to '{output_filename}'... (this may take a moment)")
        anim.save(output_filename, writer='pillow', fps=15)
        print("Animation saved.")
    
    plt.close(fig)
