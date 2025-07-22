# FlytBase Drone Deconfliction Service

This project is a simulation of a central deconfliction service for drone missions, developed as part of the "FlytBase Robotics Assignment 2025". It checks a drone's planned mission for potential time and space conflicts with other active or planned drone flights.

## 🚀 Project Objective

The core objective is to implement a service that can:
1.  Receive a drone's mission plan, which consists of a series of 4D waypoints (`latitude`, `longitude`, `altitude`, `time`).
2.  Check this plan against a database of other drone missions.
3.  Identify and flag any potential conflicts where two drones are too close to each other at the same point in time.

## 📁 File Structure

The project is organized into the following key files:
- **`main.py`**: The main script to run the simulation.
- **`src/data_models.py`**: Defines Pydantic or dataclass models for `Mission`, `Drone`, and `Waypoint`.
- **`src/deconfliction_logic.py`**: Contains the core functions for calculating distances and checking for conflicts.
- **`src/visualization.py`**: Includes functions to plot the drone flight paths for visual analysis.

## 🛠️ Installation

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/](https://github.com/)[Your-GitHub-Handle]/flytbase_challenge.git
    cd flytbase_challenge
    ```
2.  **Create and activate a virtual environment:**
    ```bash
    python -m venv venv
    source venv/bin/activate  # On Windows, use `venv\Scripts\activate`
    ```
3.  **Install the required packages:**
    ```bash
    pip install -r requirements.txt
    ```
    *(Note: You will need to create a `requirements.txt` file)*

## ▶️ How to Run

To run the deconfliction simulation, execute the `main.py` script from the project's root directory:

```bash
python main.py