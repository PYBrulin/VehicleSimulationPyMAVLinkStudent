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

# Get some information !
_start_time = time.time()
try:
    while time.time() - _start_time < 5:
        try:
            vehicle.parse_msg(conn.recv_match())
        except Exception:
            pass

        # print(
        #     "Attitude: ("
        #     + f"Roll: {round(vehicle.attitude.get('roll', 0),5)} "
        #     + f"/ Pitch: {round(vehicle.attitude.get('pitch', 0),5)} "
        #     + f"/ Yaw: {round(vehicle.attitude.get('yaw', 0),5)} )"
        # )
        # time.sleep(0.1)  # 10Hz

    print("\nVehicle state:")
    print("\tAttitude: {}".format(vehicle.attitude))
    print("\tBattery: {}".format(vehicle.battery))
    print("\tGlobal position: {}".format(vehicle.global_position))
    print("\tGPS raw: {}".format(vehicle.gps_raw_int))
    print("\tHeartbeat: {}".format(vehicle.heartbeat))
    print("\tHome position: {}".format(vehicle.home_position))
    print("\tLocal position: {}".format(vehicle.local_position))
    print()
    print("\tArmed: {}".format(vehicle.armed))
    print("\tMode: {}".format(vehicle.mode))


except Exception as e:
    logging.error(e)
finally:
    print("Closing connection...")
    conn.close()
    simulator.sim.terminate()
