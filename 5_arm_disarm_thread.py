import logging
import time

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
    _start_time = time.time()
    # region mode_change
    # print("Setting mode to GUIDED...")
    # Set mode to guided
    # ...
    # endregion mode_change

    # region Arm
    # print("Waiting for the vehicle to arm")
    # ...

    # print("Armed!")
    # endregion Arm

    # time.sleep(2)

    # region Disarm
    # print("Disarming...")
    # ...
    # endregion Disarm


except Exception as e:
    logging.error(e)
finally:
    print("Closing connection...")
    vehicle.terminate()
    conn.close()
    simulator.sim.terminate()
