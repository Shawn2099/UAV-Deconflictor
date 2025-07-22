"""
visualization.py

[V7 - Fixed Viewport, Legend Position]
This module provides functions to visualize the drone mission scenarios in 3D space.
It creates an animated GIF with a fixed visualization region, circled conflict zone,
and displays drone names in a legend at the top-right for better clarity and consistency.
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
from .data_models import PrimaryMission, SimulatedFlight, Waypoint

class DynamicDrone:
    """A helper class to manage the state and position of a drone over time."""
    def __init__(self, waypoints: List[Waypoint], timestamps: List[float], color: str, marker: str, label: str):
        self.waypoints = waypoints
        self.timestamps = timestamps
        self.color = color
        self.marker = marker
        self.label = label
        self.path_line = None   # To hold the static full path plot object
        self.point_marker = None # To hold the moving drone marker plot object

    def get_position(self, time: float) -> Optional[np.ndarray]:
        """
        Calculates the drone's 3D position at a specific time, interpolating along segments.
        Clamps the time to the drone's active flight period.
        """
        if not self.timestamps or len(self.timestamps) < 2:
            # Handle single waypoint or no waypoints
            if self.waypoints:
                return np.array([self.waypoints[0].x, self.waypoints[0].y, self.waypoints[0].z])
            return None

        # Clamp time to the drone's active flight duration
        clamped_time = max(self.timestamps[0], min(self.timestamps[-1], time))

        # Find the segment the drone is currently in
        for i in range(len(self.timestamps) - 1):
            t1, t2 = self.timestamps[i], self.timestamps[i+1]
            if t1 <= clamped_time <= t2:
                p1_wp = self.waypoints[i]
                p2_wp = self.waypoints[i+1]

                if t1 == t2: # Handle stationary segment (hovering)
                    return np.array([p1_wp.x, p1_wp.y, p1_wp.z])
                
                progress = (clamped_time - t1) / (t2 - t1)
                x = p1_wp.x + progress * (p2_wp.x - p1_wp.x)
                y = p1_wp.y + progress * (p2_wp.y - p1_wp.y)
                z = p1_wp.z + progress * (p2_wp.z - p1_wp.z)
                return np.array([x, y, z])
        
        # This case should ideally not be reached due to clamping, but as a fallback
        return None


def create_4d_animation(
    primary_mission: PrimaryMission, 
    simulated_flights: List[SimulatedFlight],
    primary_mission_etas: List[float],
    conflict_details: Optional[Dict[str, Any]] = None,
    output_filename: Optional[str] = None,
    duration_seconds: int = 10 # Default to 10 seconds for shorter animations
):
    """
    Generates and saves a 4D (3D + time) animation of the airspace scenario.
    """
    fig = plt.figure(figsize=(8, 8)) # Reduced figure size for performance
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("4D Airspace Visualization", fontsize=16)
    ax.set_xlabel("X Coordinate (meters)")
    ax.set_ylabel("Y Coordinate (meters)")
    ax.set_zlabel("Z Coordinate (Altitude in meters)")

    drones = []
    drones.append(DynamicDrone(primary_mission.waypoints, primary_mission_etas, 'blue', 'o', 'Primary Mission (Mavic 4 Pro)'))
    for flight in simulated_flights:
        drones.append(DynamicDrone(flight.waypoints, flight.timestamps, 'gray', 's', f'Sim Flight: {flight.flight_id} (Air 3)'))

    # Set fixed axis limits for consistent visualization region, similar to the example image
    ax.set_xlim([0, 1000])
    ax.set_ylim([0, 1000])
    ax.set_zlim([0, 500]) # Adjusted max Z to accommodate typical drone altitudes

    # Plot static elements and initialize dynamic elements
    for drone in drones:
        x_path = [wp.x for wp in drone.waypoints]
        y_path = [wp.y for wp in drone.waypoints]
        z_path = [wp.z for wp in drone.waypoints]
        drone.path_line, = ax.plot(x_path, y_path, z_path, '--', color=drone.color, linewidth=1, alpha=0.5)
        drone.point_marker, = ax.plot([], [], [], drone.marker, color=drone.color, markersize=8, label=drone.label) # Label for legend

    # Highlight the conflict zone with a transparent red circle
    conflict_marker = None
    if conflict_details and conflict_details.get("conflict"):
        loc = conflict_details["location"]
        conflict_marker, = ax.plot(
            [loc["x"]], [loc["y"]], [loc["z"]],
            marker='o', # Circle marker
            markersize=35, # Size of the circle
            markerfacecolor='red', # Filled red
            markeredgecolor='red',
            markeredgewidth=3,
            alpha=0.3, # Transparency for conflict zone
            label="Conflict Zone"
        )
        conflict_marker.set_visible(False) # Hide initially, show only near conflict time

    time_text = ax.text2D(0.05, 0.95, '', transform=ax.transAxes, fontsize=14)
    
    # Filter out duplicate labels for a cleaner legend and place it at the top-right
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc='upper right', bbox_to_anchor=(1.0, 1.0)) # Position legend
    
    # Determine the total simulation time based on the last event of any drone
    all_end_times = [d.timestamps[-1] for d in drones if d.timestamps]
    max_time = max(all_end_times) if all_end_times else 0

    # Animation update function
    def update(frame):
        # Calculate current time based on total animation duration and max_time
        current_time = (frame / (duration_seconds * 5)) * max_time if max_time > 0 else 0 # Use 5 fps for calculation

        for drone in drones:
            pos = drone.get_position(current_time)
            if pos is not None:
                drone.point_marker.set_data_3d([pos[0]], [pos[1]], [pos[2]])
                drone.point_marker.set_visible(True)
            else:
                drone.point_marker.set_visible(False)
        
        time_text.set_text(f'Time: {current_time:.2f}s')

        # Control visibility of conflict marker
        if conflict_marker and conflict_details and conflict_details.get("conflict"):
            conflict_time = conflict_details["time"]
            # Show for a duration of 2 seconds around the conflict time
            show_duration = 2.0 
            if conflict_time - show_duration/2 <= current_time <= conflict_time + show_duration/2:
                conflict_marker.set_visible(True)
            else:
                conflict_marker.set_visible(False)
        
        # Return all artists that might have changed
        return [drone.point_marker for drone in drones] + \
               [time_text] + \
               ([conflict_marker] if conflict_marker else [])

    total_frames = duration_seconds * 5 # Using 5 fps for total frames
    anim = FuncAnimation(fig, update, frames=total_frames, interval=1000/5, blit=False) # blit=False for 3D reliability

    if output_filename:
        print(f"Generating 4D animation to '{output_filename}'... This may take a moment.")
        anim.save(output_filename, writer='pillow', fps=5) # Save at 5 fps
        print("Animation saved.")
    
    plt.close(fig)
