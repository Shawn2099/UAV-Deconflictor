"""
data_models.py

This module defines the core data structures used throughout the deconfliction application.
Using dataclasses provides a concise and clear way to model the data for waypoints,
primary missions, and simulated drone flights.

[V2 - Added Validation]
This version adds __post_init__ validation to the data classes to ensure their
logical integrity upon creation. This prevents invalid data from propagating
through the system.
"""

from dataclasses import dataclass
from typing import List

@dataclass
class Waypoint:
    """
    Represents a single point in 3D space.

    Attributes:
        x (float): The x-coordinate.
        y (float): The y-coordinate.
        z (float): The z-coordinate (altitude).
    """
    x: float
    y: float
    z: float = 0.0

@dataclass
class PrimaryMission:
    """
    Defines the primary drone's mission parameters.

    Attributes:
        waypoints (List[Waypoint]): A list of Waypoint objects defining the
                                     intended route. Must contain at least 2.
        start_time (int): The earliest possible start time for the mission.
        end_time (int): The latest possible completion time for the mission.
                        Must be greater than start_time.
    """
    waypoints: List[Waypoint]
    start_time: int
    end_time: int

    def __post_init__(self):
        """Validates the mission's integrity after initialization."""
        if len(self.waypoints) < 2:
            raise ValueError("PrimaryMission must have at least two waypoints.")
        if self.start_time >= self.end_time:
            raise ValueError(f"PrimaryMission start_time ({self.start_time}) "
                             f"must be before end_time ({self.end_time}).")

@dataclass
class SimulatedFlight:
    """
    Defines a simulated drone's fixed spatio-temporal trajectory.

    Attributes:
        flight_id (str): A unique identifier for the simulated flight.
        waypoints (List[Waypoint]): A list of Waypoint objects defining the
                                     drone's fixed path.
        timestamps (List[int]): A list of integers representing the exact time
                                the drone arrives at each corresponding waypoint.
                                The length must match the waypoints list.
    """
    flight_id: str
    waypoints: List[Waypoint]
    timestamps: List[int]

    def __post_init__(self):
        """Validates the flight's integrity after initialization."""
        if len(self.waypoints) != len(self.timestamps):
            raise ValueError(
                f"In flight '{self.flight_id}', the number of waypoints ({len(self.waypoints)}) "
                f"must match the number of timestamps ({len(self.timestamps)})."
            )
