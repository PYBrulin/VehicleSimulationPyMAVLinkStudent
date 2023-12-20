import logging
import time

import geopy
import geopy.distance
from pymavlink import mavutil

import base.simulator as simulator  # noqa : F401
from base.custom_logger import setup_logger
from base.vehicle import Vehicle

# Setup logger
setup_logger(debug=False)

print("Trying to connect to the vehicle...")
conn = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
# Make sure the connection is valid
conn.wait_heartbeat()
print("Connected to the vehicle.")

# Create a vehicle object
vehicle = Vehicle(conn, debug=False)
vehicle.run_thread()

try:
    # region mode_change
    # print("Setting mode to GUIDED...")
    # Set mode to guided
    # ...
    # endregion mode_change

    # region Arm
    # ...
    # endregion Arm

    # region TAKEOFF
    # ...
    # endregion TAKEOFF

    # region GOTO

    # Get current coordinates
    print(f"Current coordinates: {vehicle.latitude}, { vehicle.longitude}")

    # Set target coordinates
    # Lets' move 20 meters north and -15 meters east using geopy

    # 1. Create a geopy point
    # ...

    # 2. Create a geopy distance
    # ...

    # 3. Get the new point
    # ...

    # 4. Got to the target coordinates
    # ...

    # 5. Wait until the target coordinates are reached
    # ...

    # endregion GOTO

    # region RTL
    # ...
    # endregion RTL

    # region Disarm
    # ...
    # endregion Disarm

except Exception as e:
    # logging.error(e)
    raise e
finally:
    print("Closing connection...")
    vehicle.terminate()
    conn.close()
    simulator.sim.terminate()
