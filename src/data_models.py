"""
data_models.py

This module defines the core data structures used throughout the deconfliction application.
Using dataclasses provides a concise and clear way to model the data for waypoints,
primary missions, and simulated drone flights.
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
        z (float): The z-coordinate (altitude). For the initial 2D MVP,
                   this will be set to 0, but it is included for future
                   3D/4D implementation.
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
                                     intended route.
        start_time (int): The earliest possible start time for the mission,
                          represented as an integer (e.g., seconds from epoch).
        end_time (int): The latest possible completion time for the mission.
    """
    waypoints: List[Waypoint]
    start_time: int
    end_time: int

@dataclass
class SimulatedFlight:
    """
    Defines a simulated drone's fixed spatio-temporal trajectory.

    Attributes:
        flight_id (str): A unique identifier for the simulated flight.
        waypoints (List[Waypoint]): A list of Waypoint objects defining the
                                     drone's fixed path.
        timestamps (List[int]): A list of integers representing the exact time
                                the drone arrives at each corresponding waypoint
                                in its path.
    """
    flight_id: str
    waypoints: List[Waypoint]
    timestamps: List[int]

