# Multicopter Vehicle Simulation with PyMAVLink

This project provides a list of Python scripts and tools to control a multicopter vehicle simulation using the PyMAVLink library.

The scripts are designed for educational purposes with fill-in code sections for students to complete for the course "Drones: Application et développement" delivered at ESTACA engineering school.

## Prerequisites

### Virtual Environment Setup

Before running the scripts, set up a Python virtual environment to manage dependencies. You can create a virtual environment using the following commands:

```bash
python -m venv venv
source venv/bin/activate  # On Windows use `venv\Scripts\activate`
```

Once the virtual environment is activated, install the required packages:

```bash
pip install -r requirements.txt
```

## Project Structure

`base/`: Contains base classes for the vehicle simulation.

- `vehicle.py`: Vehicle class to handle MAVLink messages and vehicle data.
- `simulator.py`: Simulator class to manage the ArduCopter SITL subprocess.
- `custom_logger.py`: Custom logger to display better looking logs.
- (BE) `sim_balise.py`: SimBalise class
- (BE) `vehicle_visual.py`: VehicleVisual class

`SITL/`: Contains the ArduCopter SITL (Software In The Loop) simulation files.

1. `1_vehicle-connection.py`: Simple script to connect to the vehicle simulation and print received messages.
2. `2_vehicle-states.py`: Script to receive several MAVLink messages and print vehicle states.
3. `3_vehicle-parameters.py`: Script to list the vehicle parameters.
4. `4_arm_disarm.py`: Script to arm and disarm the vehicle.
5. `5_arm_disarm_thread.py`: Script to use a thread loop to handle MAVLink messages to distinguish between MAVLink RxTx and user's application.
6. `6_takeoff_land.py`: Script to perform a takeoff maneuver.
7. `7_goto.py`: Script to perform a takeoff and go to a specified GPS location.
8. `8_mission.py`: Script to upload and execute a mission.
