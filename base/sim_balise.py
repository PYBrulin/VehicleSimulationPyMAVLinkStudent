import logging
import math
import random

import geopy.distance

from .vehicle import Vehicle


class Victim:
    def __init__(
        self,
        latitude: float,
        longitude: float,
        altitude: float,
    ) -> None:
        self._latitude = latitude
        self._longitude = longitude
        self._altitude = altitude

    def __str__(self) -> str:
        return f"Victim at {self.latitude}, {self.longitude}, {self.altitude}"

    def __repr__(self) -> str:
        return self.__str__()

    @property
    def latitude(self) -> float:
        return self._latitude

    @property
    def longitude(self) -> float:
        return self._longitude

    @property
    def altitude(self) -> float:
        return self._altitude


class SimBalise:
    """Class that simulates the behaviour of an avalanche beacon.

    Usage:
        import SimBalise

        print("Trying to connect to the vehicle...")
        conn = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
        # Make sure the connection is valid
        conn.wait_heartbeat()
        print("Connected to the vehicle.")

        # Create a vehicle object
        vehicle = Vehicle(conn, debug=False)
        vehicle.run_thread()

        s = SimBalise.SimBalise(vehicle=vehicle)
        while True:
            time.sleep(1)
            print("[SimBalise] Victim Detected : ", s.victim_detected())
            print("[SimBalise] Victim Bearing : ", s.victim_orientation())

        vehicle.close()
    """

    logger = logging.getLogger(__name__)

    DETECTION_RADIUS_M = 250
    ORIENTATION_RADIUS_M = 90

    def __init__(self, vehicle: Vehicle, map_scale: float = 1.0) -> None:
        self.vehicle = vehicle  # Access to vehicle object to get position

        offset_lat_600m = 0.005395932 * map_scale
        offset_lon_800m = 0.007194576 * map_scale

        self.top_right = (-35.3664632, 149.1622996)
        # self.btm_left = (-35.3718524, 149.1551757)
        self.btm_left = (self.top_right[0] - offset_lat_600m, self.top_right[1] - offset_lon_800m)

        self.ROI = [self.top_right, self.btm_left]

        self.victim = Victim(*self.random_location())
        self.has_been_detected = False
        self.has_orientation = False

    def victim_detected(self) -> bool:
        """Returns wether the victim balise has been detected within the detection radius

        Returns:
            bool: Victim is within the detection radius
        """
        _detected = (
            geopy.distance.GeodesicDistance(
                (self.victim.latitude, self.victim.longitude),
                (self.vehicle.latitude, self.vehicle.longitude),
            ).meters
            < self.DETECTION_RADIUS_M
        )
        if _detected and not self.has_been_detected:
            self.logger.info("[SimBalise] Victim Detected!")
            self.has_been_detected = True
        if not _detected and self.has_been_detected:
            self.logger.warning("[SimBalise] Victim Lost!")
            self.has_been_detected = False
        return _detected

    def victim_orientation(self) -> float | None:
        """Returns the victim bearing from the vehicle position,
        only if the victim is within the orientation radius

        Returns:
            Optional[float]: relative bearing between vehicle position and victim position,
                           or None if the victim has not been found
        """
        if (
            geopy.distance.GeodesicDistance(
                (self.victim.latitude, self.victim.longitude),
                (self.vehicle.latitude, self.vehicle.longitude),
            ).meters
            < self.ORIENTATION_RADIUS_M
        ):
            if not self.has_orientation:
                self.logger.info("[SimBalise] Victim Orientation Found!")
                self.has_orientation = True
            # Calculate relative bearing
            bearing = self.get_bearing(
                self.vehicle.latitude,
                self.vehicle.longitude,
                self.victim.latitude,
                self.victim.longitude,
            ) - math.degrees(self.vehicle.heading)
            # Normalize to [-180, 180]
            while bearing > 180:
                bearing -= 360
            while bearing < -180:
                bearing += 360
            return bearing
        elif self.has_orientation:
            self.logger.warning("[SimBalise] Victim Orientation Lost!")
            self.has_orientation = False
        return None

    def random_location(self) -> tuple[float, float, float]:
        """Returns a random location within the Region-Of-Interest

        Returns:
            dronekit.LocationGlobalRelative: Random Location in ROI
        """
        return (
            random.uniform(self.ROI[0][0], self.ROI[1][0]),  # latitude
            random.uniform(self.ROI[0][1], self.ROI[1][1]),  # longitude
            1,  # altitude in meters (1 meter above ground)
        )

    def get_bearing(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate bearing between two set of coordinates

        Args:
            lat1 (float): Latitude of first set of coordinates in degrees
            lon1 (float): Longitude of first set of coordinates in degrees
            lat2 (float): Latitude of second set of coordinates in degrees
            lon2 (float): Longitude of second set of coordinates in degrees

        Returns:
            float: Bearing in degrees (0-360, where 0/360 is North, 90 is East)
        """
        # Convert to radians
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dLon_rad = math.radians(lon2 - lon1)

        y = math.sin(dLon_rad) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dLon_rad)
        brng = math.degrees(math.atan2(y, x))
        # Normalize to 0-360
        brng = (brng + 360) % 360
        return brng
