import math
import random
from typing import Optional, Tuple

import geopy.distance

from .vehicle import Vehicle
import logging


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

    def __init__(self, vehicle: Vehicle) -> None:
        self.vehicle = vehicle  # Access to vehicle object to get position
        self.ROI = [(-35.3664632, 149.1551757), (-35.3718524, 149.1622996)]
        self.victim = Victim(*self.random_location())
        self.has_been_detected = False
        self.has_orientation = False

    def victim_detected(self) -> bool:
        """Returns wether the victim balise has been detected within a 250 meters radius

        Returns:
            bool: Victim is within a 250 meters radius
        """
        _detected = (
            geopy.distance.GeodesicDistance(
                (
                    self.victim.latitude,
                    self.victim.longitude,
                ),
                (
                    self.vehicle.latitude,
                    self.vehicle.longitude,
                ),
            ).meters
            < 250  # meters
        )
        if _detected and not self.has_been_detected:
            self.logger.info("[SimBalise] Victim Detected!")
            self.has_been_detected = True
        if not _detected and self.has_been_detected:
            self.logger.warning("[SimBalise] Victim Lost!")
            self.has_been_detected = False
        return _detected

    def victim_orientation(self) -> Optional[int]:
        """Returns the victim bearing from the vehicle position,
        only if the victim is within an 90 meters radius

        Returns:
            Optional[int]: bearing between vehicle position and victim position,
                           or None if the victim has not been found
        """
        if (
            geopy.distance.GeodesicDistance(
                (
                    self.victim.latitude,
                    self.victim.longitude,
                ),
                (
                    self.vehicle.latitude,
                    self.vehicle.longitude,
                ),
            ).meters
            < 90  # meters
        ):
            if not self.has_orientation:
                self.logger.info("[SimBalise] Victim Orientation Found!")
                self.has_orientation = True
            return self.vehicle.heading - self.get_bearing(
                self.vehicle.latitude,
                self.vehicle.longitude,
                self.victim.latitude,
                self.victim.longitude,
            )
        elif self.has_orientation:
            self.logger.warning("[SimBalise] Victim Orientation Lost!")
            self.has_orientation = False
        return None

    def random_location(self) -> Tuple[float, float, float]:
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
            lat1 (float): Latitude of first set of coordinates
            lon1 (float): Longitude of first set of coordinates
            lat2 (float): Latitude of second set of coordinates
            lon2 (float): Longitude of second set of coordinates

        Returns:
            float: Bearing in degrees
        """
        dLon = lon2 - lon1
        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(
            lat2
        ) * math.cos(dLon)
        brng = math.degrees(math.atan2(y, x))
        if brng < 0:
            brng += 360
        return brng
