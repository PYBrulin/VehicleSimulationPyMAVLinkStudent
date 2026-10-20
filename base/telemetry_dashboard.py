import asyncio
import functools
import logging
import os
import queue
import threading
import webbrowser
from http.server import HTTPServer, SimpleHTTPRequestHandler

import websockets

os.environ.setdefault("MAVLINK20", "1")
from pymavlink.dialects.v20 import ardupilotmega as mavlink2  # noqa: E402

from .sim_balise import SimBalise  # noqa: E402
from .vehicle import Vehicle  # noqa: E402

# ArduPilot WebTools TelemetryDashboard auto-connects here by default
DEFAULT_WS_HOST = "127.0.0.1"
DEFAULT_WS_PORT = 56781
DEFAULT_HTTP_PORT = 8000

WEBTOOLS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "external", "WebTools"))
DASHBOARD_URL_PATH = "TelemetryDashboard/index.html"

# See external/WebTools/TelemetryDashboard/SandBoxWidgets/Map_Balise.json
BALISE_SYSTEM_ID = 2
MAV_CMD_USER_1 = 31010  # param5/6/7 = lat/lon/alt
MAV_CMD_USER_2 = 31011  # param1-4 = ROI corners lat/lon, param5/6 = detection/orientation radius (m)


class TelemetryDashboard:
    """Serve the ArduPilot WebTools TelemetryDashboard and stream live telemetry to it.

    Messages are received via Vehicle.add_message_listener() instead of polling
    the pymavlink connection directly, since Vehicle already owns the one reader
    a connection can have.

    Usage:
        vehicle = Vehicle(conn)
        vehicle.run_thread()

        dashboard = TelemetryDashboard(vehicle=vehicle, sim_balise=SimBalise(vehicle=vehicle))
        dashboard.start()
        ...
        dashboard.stop()
    """

    def __init__(
        self,
        vehicle: Vehicle,
        sim_balise: SimBalise = None,
        ws_host: str = DEFAULT_WS_HOST,
        ws_port: int = DEFAULT_WS_PORT,
        http_port: int = DEFAULT_HTTP_PORT,
        balise_period: float = 1.0,
    ) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.vehicle = vehicle
        self.sim_balise = sim_balise
        self.ws_host = ws_host
        self.ws_port = ws_port
        self.http_port = http_port
        self.balise_period = balise_period

        self.clients: set = set()
        self._balise_mav = mavlink2.MAVLink(None, srcSystem=BALISE_SYSTEM_ID, srcComponent=1)
        self._outbox: queue.SimpleQueue = queue.SimpleQueue()

        self._http_server: HTTPServer = None
        self._http_thread: threading.Thread = None

        self._ws_loop: asyncio.AbstractEventLoop = None
        self._ws_thread: threading.Thread = None
        self._alive = False

    # region Balise MAVLink messages

    def _next_balise_message(self, command: int, params: list[float]):
        msg = mavlink2.MAVLink_command_long_message(0, 0, command, 0, *params)
        buf = msg.pack(self._balise_mav)
        self._balise_mav.seq = (self._balise_mav.seq + 1) % 256
        return buf

    def _balise_position_message(self):
        victim = self.sim_balise.victim
        return self._next_balise_message(
            MAV_CMD_USER_1,
            [0, 0, 0, 0, victim.latitude, victim.longitude, victim.altitude],
        )

    def _balise_roi_message(self):
        (lat1, lon1), (lat2, lon2) = self.sim_balise.ROI
        return self._next_balise_message(
            MAV_CMD_USER_2,
            [
                lat1,
                lon1,
                lat2,
                lon2,
                self.sim_balise.DETECTION_RADIUS_M,
                self.sim_balise.ORIENTATION_RADIUS_M,
                0,
            ],
        )

    # endregion

    # region WebSocket bridge

    async def _broadcast(self, data: bytes) -> None:
        if not self.clients:
            return
        await asyncio.gather(
            *(client.send(data) for client in self.clients),
            return_exceptions=True,
        )

    async def _handle_client(self, websocket) -> None:
        self.logger.info(f"Dashboard client connected from {websocket.remote_address}")
        self.clients.add(websocket)
        try:
            async for _ in websocket:
                pass  # Dashboard doesn't send data we need to act on
        finally:
            self.clients.discard(websocket)
            self.logger.info(f"Dashboard client disconnected from {websocket.remote_address}")

    def _on_vehicle_message(self, msg) -> None:
        if msg.get_type() != "BAD_DATA":
            self._outbox.put(msg.get_msgbuf())

    async def _relay_outbox(self) -> None:
        while self._alive:
            try:
                await self._broadcast(self._outbox.get_nowait())
            except queue.Empty:
                await asyncio.sleep(0.01)

    async def _relay_balise(self) -> None:
        if self.sim_balise is None:
            return
        while self._alive:
            await self._broadcast(self._balise_position_message())
            await self._broadcast(self._balise_roi_message())
            await asyncio.sleep(self.balise_period)

    async def _serve_ws(self) -> None:
        async with websockets.serve(self._handle_client, self.ws_host, self.ws_port):
            self.logger.info(f"MAVLink WebSocket bridge listening on ws://{self.ws_host}:{self.ws_port}")
            self.vehicle.add_message_listener(self._on_vehicle_message)
            await asyncio.gather(
                self._relay_outbox(),
                self._relay_balise(),
            )

    def _run_ws(self) -> None:
        self._ws_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._ws_loop)
        try:
            self._ws_loop.run_until_complete(self._serve_ws())
        except Exception as e:
            self.logger.exception(e)

    # endregion

    # region HTTP server

    def _run_http(self) -> None:
        class _QuietHandler(SimpleHTTPRequestHandler):
            def log_message(self, format, *args):  # noqa: A002
                pass

        handler = functools.partial(_QuietHandler, directory=WEBTOOLS_DIR)
        HTTPServer.allow_reuse_address = True
        self._http_server = HTTPServer((self.ws_host, self.http_port), handler)
        self.logger.info(f"Dashboard served at {self.url}")
        self._http_server.serve_forever()

    # endregion

    @property
    def url(self) -> str:
        """URL the dashboard is served at"""
        return f"http://{self.ws_host}:{self.http_port}/{DASHBOARD_URL_PATH}"

    def _request_vfr_hud_stream(self, rate_hz: float = 4) -> None:
        """Ask the autopilot to stream VFR_HUD (groundspeed/airspeed) messages"""
        conn = self.vehicle.vehicle_conn
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavlink2.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavlink2.MAVLINK_MSG_ID_VFR_HUD,
            int(1e6 / rate_hz),
            0, 0, 0, 0, 0,
        )

    def start(self, open_browser: bool = True) -> None:
        """Start the HTTP and WebSocket servers in background threads.

        Requires the vehicle's connection to already have a confirmed heartbeat
        (e.g. via conn.wait_heartbeat() before building the Vehicle), since the
        vehicle's own reader thread -- once running -- would otherwise race with
        a blocking wait_heartbeat() call made here.
        """
        self._alive = True

        self._request_vfr_hud_stream()

        self._http_thread = threading.Thread(target=self._run_http, daemon=True)
        self._http_thread.start()

        self._ws_thread = threading.Thread(target=self._run_ws, daemon=True)
        self._ws_thread.start()

        if open_browser:
            webbrowser.open(self.url)

    def stop(self) -> None:
        """Stop the HTTP and WebSocket servers"""
        self._alive = False

        if self._http_server is not None:
            self._http_server.shutdown()
        if self._http_thread is not None:
            self._http_thread.join(timeout=5)

        if self._ws_loop is not None:
            self._ws_loop.call_soon_threadsafe(self._ws_loop.stop)
        if self._ws_thread is not None:
            self._ws_thread.join(timeout=5)
