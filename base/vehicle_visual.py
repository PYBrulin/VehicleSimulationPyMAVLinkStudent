from math import cos, radians, sin

import matplotlib.pyplot as plt

from .sim_balise import SimBalise
from .vehicle import Vehicle

radius_of_earth = 6378100.0  # in meters
circumference_of_earth = 40075000.0  # in meters


class VehicleVisual:
    def __init__(
        self,
        vehicle: Vehicle,
        sim_balise: SimBalise = None,
        size: int = 1,
        show_animation: bool = True,
    ) -> None:
        self.vehicle = vehicle
        self.sim_balise = sim_balise
        self.size = size

        self.x_data = []
        self.y_data = []
        self.x = 0
        self.y = 0

        self.show_animation = show_animation
        if self.show_animation:
            plt.ion()
            fig = plt.figure()
            # for stopping simulation with the esc key.
            fig.canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None],
            )
            self.ax = fig.add_subplot(111)  # , projection="3d")

    def latlon_approximation(self, lat, lon) -> tuple[float, float]:
        # Basic approximation
        north = radius_of_earth * sin(radians(lat - self.vehicle.latitude_home))
        east = radius_of_earth * sin(radians(lon - self.vehicle.longitude_home))
        return (north, east)

    def update(self, x, y) -> None:
        if self.x != x or self.y != y:
            self.x = x
            self.y = y

            if self.x != 0 and self.y != 0:
                self.x_data.append(self.x)
                self.y_data.append(self.y)

    def plot(self) -> None:  # pragma: no cover
        plt.cla()

        # Basic approximation
        if self.vehicle.latitude_home != 0 and self.vehicle.longitude != 0:
            north, east = self.latlon_approximation(self.vehicle.latitude, self.vehicle.longitude)
            self.update(north, east)

        self.ax.plot(self.y_data, self.x_data, "b:")

        if self.sim_balise is not None:
            north, east = self.latlon_approximation(
                self.sim_balise.victim.latitude,
                self.sim_balise.victim.longitude,
            )
            self.ax.plot(east, north, "rx")

            p1 = self.sim_balise.ROI[0][0], self.sim_balise.ROI[0][1]
            p2 = self.sim_balise.ROI[1][0], self.sim_balise.ROI[0][1]
            p3 = self.sim_balise.ROI[1][0], self.sim_balise.ROI[1][1]
            p4 = self.sim_balise.ROI[0][0], self.sim_balise.ROI[1][1]
            n, e = [], []
            for p in [p1, p2, p3, p4, p1]:
                _n, _e = self.latlon_approximation(p[0], p[1])
                n.append(_n)
                e.append(_e)
            self.ax.plot(e, n, "g--")

            # Draw circle around victim
            circle2 = plt.Circle(
                (east, north),
                250,
                color="g",
                fill=False,
            )
            circle1 = plt.Circle(
                (east, north),
                90,
                color="r",
                fill=False,
            )
            self.ax.add_artist(circle1)
            self.ax.add_artist(circle2)

        self.ax.arrow(
            self.y_data[-1] if self.x_data else self.x,
            self.x_data[-1] if self.x_data else self.y,
            self.size * sin(self.vehicle.heading),
            self.size * cos(self.vehicle.heading),
            head_width=4 * self.size,
            head_length=10 * self.size,
        )
        plt.axis("equal")
        plt.xlabel("East")
        plt.ylabel("North")

        # plt.xlim(-2, 2)
        # plt.ylim(-2, 2)
        # self.ax.set_zlim(0, 10)
        # plt.grid()

        plt.pause(0.1)
