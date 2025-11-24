import time

from pymavlink import mavutil, mavwp

import base.simulator as simulator  # noqa : F401
from base.custom_logger import setup_logger
from base.vehicle import Vehicle

# Setup logger
setup_logger(debug=True)

print("Trying to connect to the vehicle...")
conn = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
# Make sure the connection is valid
conn.wait_heartbeat()
print("Connected to the vehicle.")

wp = mavwp.MAVWPLoader()

# Create a vehicle object
vehicle = Vehicle(conn, debug=False)
vehicle.run_thread()

try:
    # Wait for Home coordinates
    while not vehicle.home_position:
        print("Waiting for home coordinates...")
        time.sleep(1)

    print(f"Home coordinates: {vehicle.home_position}")

    offset_lat_10m = 0.0000899322  # pre-computed offset value
    offset_lon_10m = 0.0000899322  # pre-computed offset value

    waypoint_list = [
        (
            vehicle.latitude_home,
            vehicle.longitude_home,
            0,
        ),
        (
            vehicle.latitude_home,
            vehicle.longitude_home,
            10,
        ),
        (
            vehicle.latitude_home + offset_lat_10m,
            vehicle.longitude_home,
            10,
        ),
        (
            vehicle.latitude_home + offset_lat_10m,
            vehicle.longitude_home + offset_lon_10m,
            10,
        ),
        (
            vehicle.latitude_home - offset_lat_10m,
            vehicle.longitude_home + offset_lon_10m,
            10,
        ),
        (
            vehicle.latitude_home - offset_lat_10m,
            vehicle.longitude_home - offset_lon_10m,
            10,
        ),
        (
            vehicle.latitude_home + offset_lat_10m,
            vehicle.longitude_home - offset_lon_10m,
            10,
        ),
        (
            vehicle.latitude_home + offset_lat_10m,
            vehicle.longitude_home,
            10,
        ),
    ]

    for i, waypoint in enumerate(waypoint_list):
        p = mavutil.mavlink.MAVLink_mission_item_message(
            conn.target_system,
            conn.target_component,
            i,  # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # command
            0,  # current
            1,  # autocontinue
            0,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            waypoint[0],  # x
            waypoint[1],  # y
            waypoint[2],  # z
        )
        wp.add(p)
    # Finally add a RTL command
    p = mavutil.mavlink.MAVLink_mission_item_message(
        conn.target_system,
        conn.target_component,
        len(waypoint_list) + 1,  # seq
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,  # command
        0,  # current
        1,  # autocontinue
        0,  # param1
        0,  # param2
        0,  # param3
        0,  # param4
        0,  # x
        0,  # y
        0,  # z
    )
    wp.add(p)

    # send waypoint to airframe
    conn.waypoint_clear_all_send()  # clear waypoints on the autopilot
    conn.waypoint_count_send(wp.count())  # send waypoint count to airframe

    for i in range(wp.count()):  # send waypoints to autopilot
        print(f"Sending waypoint {i}/{wp.count()-1}")
        # msg = conn.recv_match(type=["MISSION_REQUEST"], blocking=True)
        while True:  # Wait for the correct sequence request
            if vehicle.data.get("MISSION_REQUEST", {}):
                seq = vehicle.data["MISSION_REQUEST"]["seq"]
                if seq == i:
                    break

        conn.mav.send(wp.wp(i))  # send waypoint

    # ARM

    # Set mode to guided
    vehicle.mode = "GUIDED"
    while vehicle.mode != "GUIDED":
        print(f"Armed: {vehicle.armed} / Mode: {vehicle.mode}")
        time.sleep(1)

    # Arm
    # wait until arming confirmed (can manually check with conn.motors_armed())
    print("Waiting for the vehicle to arm")
    # conn.motors_armed_wait()
    # print("Armed!")

    while not vehicle.armed:
        # Ensure mode is guided
        if vehicle.mode != "GUIDED":
            vehicle.mode = "GUIDED"
        else:
            # Send arm command
            vehicle.arm()

        time.sleep(2)

    print("Armed!")

    # TAKEOFF

    target_altitude = 5
    while vehicle.altitude_relative < 0.85:
        vehicle.takeoff(target_altitude)
        time.sleep(1)

    while vehicle.altitude_relative < target_altitude * 0.85:
        print(f"Current altitude: {vehicle.altitude_relative} m")
        time.sleep(1)

    print("Reached target altitude!")

    # RUN MISSION

    # Set mode to auto
    while vehicle.mode != "AUTO":
        vehicle.mode = "AUTO"
        print(f"Armed: {vehicle.armed} / Mode: {vehicle.mode}")
        time.sleep(1)

    # Run mission until complete
    while vehicle.armed:
        print(
            f"Mission sequence running: {vehicle.waypoint_seq}/{vehicle.waypoint_total}"
        )
        time.sleep(1)

    print("Disarmed!")

except Exception as e:
    # logging.error(e)
    raise e
finally:
    print("Closing connection...")
    vehicle.terminate()
    conn.close()
    simulator.sim.terminate()
