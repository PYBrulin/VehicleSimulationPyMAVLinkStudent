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
vehicle = Vehicle(conn)

try:
    _start_time = time.time()
    _iter_time = time.time()
    while time.time() - _start_time < 2:
        try:
            vehicle.parse_msg(conn.recv_match())

            if time.time() - _iter_time > 1:  # Every second
                _iter_time = time.time()
                print(f"Armed: {vehicle.armed} / Mode: {vehicle.mode}")

        except Exception:
            pass

    # region mode_change
    # print("Setting mode to GUIDED...")
    # Set mode to guided
    # ...
    # endregion mode_change

    # region Arm
    def arm() -> None:
        # ...
        pass

    # print("Waiting for the vehicle to arm")
    # ...

    # print("Armed!")
    # endregion Arm

    # time.sleep(2)

    # region Disarm
    def disarm() -> None:
        # ...
        pass

    # print("Disarming...")
    # ...
    # endregion Disarm

except Exception as e:
    logging.error(e)
finally:
    print("Closing connection...")
    conn.close()
    simulator.sim.terminate()
