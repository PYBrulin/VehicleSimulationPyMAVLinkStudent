import logging
from threading import Thread
from typing import Union

import pymavlink
from pymavlink import mavutil


class Vehicle:
    """An object to store the vehicle's information and methods"""

    def __init__(
        self,
        vehicle_conn: pymavlink.mavutil.mavlink_connection,
        debug: bool = False,
    ) -> None:
        """Initialize the vehicle object"""
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(logging.INFO if not debug else logging.DEBUG)

        self.vehicle_conn = vehicle_conn
        self.mavlink_thread_in = None
        self._alive = False
        self.data = {}
        self.parameters = {}

    @property
    def attitude(self):
        """Return the vehicle's attitude"""
        return self.data.get("ATTITUDE", {})

    @property
    def heading(self):
        """Return the vehicle's heading"""
        return self.attitude.get("yaw", 0)

    @property
    def battery(self):
        """Return the vehicle's battery"""
        return self.data.get("BATTERY_STATUS", {})

    @property
    def global_position(self):
        """Return the vehicle's global position"""
        return self.data.get("GLOBAL_POSITION_INT", {})

    @property
    def altitude_relative(self):
        """Return the vehicle's altitude in meters"""
        return self.global_position.get("relative_alt", 0) / 1000

    @property
    def latitude(self):
        """Return the vehicle's latitude"""
        return self.global_position.get("lat", 0) / 1e7

    @property
    def longitude(self):
        """Return the vehicle's longitude"""
        return self.global_position.get("lon", 0) / 1e7

    @property
    def altitude(self):
        """Return the vehicle's altitude"""
        return self.global_position.get("alt", 0) / 1000

    @property
    def gps_raw_int(self):
        """Return the vehicle's raw GPS"""
        return self.data.get("GPS_RAW_INT", {})

    @property
    def heartbeat(self):
        """Return the vehicle's heartbeat"""
        return self.data.get("HEARTBEAT", {})

    @property
    def home_position(self):
        """Return the vehicle's home position"""
        return self.data.get("HOME_POSITION", {})

    @property
    def latitude_home(self):
        """Return the vehicle's home latitude"""
        return self.home_position.get("latitude", 0) / 1e7

    @property
    def longitude_home(self):
        """Return the vehicle's home longitude"""
        return self.home_position.get("longitude", 0) / 1e7

    @property
    def altitude_home(self):
        """Return the vehicle's home altitude"""
        return self.home_position.get("altitude", 0) / 1000

    @property
    def local_position(self):
        """Return the vehicle's local position"""
        return self.data.get("LOCAL_POSITION_NED", {})

    @property
    def armed(self):
        """Return the vehicle's armed state"""
        return (
            self.heartbeat.get("base_mode", 0)
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        ) != 0

    @property
    def mode(self):
        """Return the vehicle's mode"""
        custom_mode = self.heartbeat.get("custom_mode", 0)

        # find value matching custom_mode in mode_mapping
        for k, v in self.vehicle_conn.mode_mapping().items():
            if v == custom_mode:
                return k

        return "Unknown"

    @mode.setter
    def mode(self, mode: Union[int, str]):
        """Set the vehicle's mode"""
        if isinstance(mode, str):
            # Check if mode is available
            if mode not in self.vehicle_conn.mode_mapping():
                print("Unknown mode : {}".format(mode))
                print("Try:", list(self.vehicle_conn.mode_mapping().keys()))
            mode_id = self.vehicle_conn.mode_mapping()[mode]
        else:
            mode_id = mode
        self.vehicle_conn.set_mode(mode_id)

    @property
    def waypoint_seq(self):
        """Return the vehicle's current waypoint"""
        return self.data.get("MISSION_CURRENT", {}).get("seq", 0)

    @property
    def waypoint_total(self):
        """Return the vehicle's current waypoint"""
        return self.data.get("MISSION_CURRENT", {}).get("total", 0)

    def parse_msg(self, msg: pymavlink.mavutil.mavlink.MAVLink) -> None:
        """Parse a message from the vehicle"""
        # msg = self.vehicle_conn.recv_match(blocking=False, timeout=1)
        if not msg:
            return

        if msg.get_srcSystem() != 1 or msg.get_srcComponent() != 1:
            # Ignore messages not from the autopilot
            return

        if msg.get_type() == "BAD_DATA":
            # Ignore bad data
            self.logger.warning("Received bad data")

        elif msg.get_type() == "STATUSTEXT":
            # Get severity
            severity = msg.to_dict().get("severity")
            # MAV_SEVERITY_EMERGENCY = 0
            # MAV_SEVERITY_ALERT = 1
            # MAV_SEVERITY_CRITICAL = 2
            # MAV_SEVERITY_ERROR = 3
            # MAV_SEVERITY_WARNING = 4
            # MAV_SEVERITY_NOTICE = 5
            # MAV_SEVERITY_INFO = 6
            # MAV_SEVERITY_DEBUG = 7
            if severity == mavutil.mavlink.MAV_SEVERITY_DEBUG:
                self.logger.debug(msg.to_dict().get("text"))
            elif severity == mavutil.mavlink.MAV_SEVERITY_INFO:
                self.logger.info(msg.to_dict().get("text"))
            elif severity in [
                mavutil.mavlink.MAV_SEVERITY_NOTICE,
                mavutil.mavlink.MAV_SEVERITY_WARNING,
            ]:
                self.logger.warning(msg.to_dict().get("text"))
            elif severity == mavutil.mavlink.MAV_SEVERITY_ERROR:
                self.logger.error(msg.to_dict().get("text"))
            elif severity in [
                mavutil.mavlink.MAV_SEVERITY_CRITICAL,
                mavutil.mavlink.MAV_SEVERITY_ALERT,
                mavutil.mavlink.MAV_SEVERITY_EMERGENCY,
            ]:
                self.logger.critical(msg.to_dict().get("text"))

        elif msg.get_type() == "PARAM_VALUE":
            # Store parameter value
            param_id = msg.to_dict().get("param_id")
            param_value = msg.to_dict().get("param_value")
            self.parameters[param_id] = param_value

        else:
            self.logger.debug(f"Received {msg.get_type()} message")

            # Replace the data in the dictionary
            self.data[msg.get_type()] = msg.to_dict()

    # region Thread
    def thread_in(self) -> None:
        """Run the thread to parse messages from the vehicle"""
        self.logger.info("Starting thread in")
        try:
            while self._alive:
                msg = self.vehicle_conn.recv_match(blocking=False, timeout=1)
                self.parse_msg(msg)
        except Exception as e:
            self.logger.exception(e)

    def run_thread(self) -> None:
        """DroneKit-like thread to parse messages from the  self.vehicle_conn. RIP DroneKit"""
        self.logger.info("Starting thread")

        # Sleep
        self.vehicle_conn.select(0.05)
        self._alive = True
        t = Thread(target=self.thread_in)
        t.daemon = True
        self.mavlink_thread_in = t
        self.mavlink_thread_in.start()

    def terminate(self) -> None:
        """Terminate the vehicle connection"""
        self.logger.info("Terminating vehicle connection")
        self._alive = False
        if self.mavlink_thread_in is not None:
            self.mavlink_thread_in.join()
            self.mavlink_thread_in = None

    # endregion

    # region MAVLink commands

    def arm(self) -> None:
        """Arm the vehicle"""
        # create the ARM command using command_long_encode()
        # fmt: off
        pass  # ...
        # fmt: on

    def disarm(self) -> None:
        """Disarm the vehicle"""
        # create the DISARM command using command_long_encode()
        # fmt: off
        pass  # ...
        # fmt: on

    def takeoff(self, altitude: float) -> None:
        """Takeoff to a specified altitude"""
        # create the TAKEOFF command
        # fmt: off
        pass  # ...
        # fmt: on

    def goto_position_target_local_int(self, north, east, down) -> None:
        """Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified location in the
        North, East, Down frame."""
        # fmt: off
        self.vehicle_conn.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.vehicle_conn.target_system,        # target system
            self.vehicle_conn.target_component,     # target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,    # frame
            0b0000111111111000,     # type_mask (only positions enabled)
            north, east, down,      # x, y, z positions (or North, East, Down) in the MAV_FRAME_BODY_NED frame in meters
            0, 0, 0,    # x, y, z velocity in m/s  (not used)
            0, 0, 0,    # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0,       # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        )
        # fmt: on

    def goto_position_target_global_int(self, lat: int, lon: int, alt: int) -> None:
        """Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified location."""
        # fmt: off
        self.vehicle_conn.mav.set_position_target_global_int_send(
            0,          # time_boot_ms (not used)
            self.vehicle_conn.target_system,        # target system
            self.vehicle_conn.target_component,     # target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b0000111111111000,     # type_mask (only positions enabled)
            int(lat*1e7),   # lat_int - X Position in WGS84 frame in 1e7 * meters
            int(lon*1e7),   # lon_int - Y Position in WGS84 frame in 1e7 * meters
            int(alt),       # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative,
                            # above terrain if GLOBAL_TERRAIN_ALT_INT
            0,          # X velocity in NED frame in m/s
            0,          # Y velocity in NED frame in m/s
            0,          # Z velocity in NED frame in m/s
            0, 0, 0,    # afx, afy, afz acceleration
            0, 0,       # yaw, yaw_rate
        )
        # fmt: on

    def condition_yaw(self, heading, relative: int = 0) -> None:
        """Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees)."""
        # create the CONDITION_YAW command using command_long_encode()
        # fmt: off
        pass  # ...
        # fmt: on

    # endregion
