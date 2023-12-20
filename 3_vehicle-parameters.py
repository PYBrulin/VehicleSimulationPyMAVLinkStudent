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

# Request all parameters
conn.param_fetch_all()
# or
# conn.mav.param_request_list_send(
#     conn.target_system, conn.target_component
# )

# Get some information !
_start_time = time.time()
try:
    while time.time() - _start_time < 20:
        try:
            vehicle.parse_msg(conn.recv_match())
        except Exception:
            pass

    for param in sorted(vehicle.parameters.keys()):
        print(f"{param:20s} = {vehicle.parameters[param]}")
    print(f"Got {len(vehicle.parameters)} parameters...")

except Exception as e:
    logging.error(e)
finally:
    print("Closing connection...")
    conn.close()
    simulator.sim.terminate()
