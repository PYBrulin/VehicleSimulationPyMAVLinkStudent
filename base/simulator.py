import logging
import os
import platform
import stat
import subprocess
import urllib.request

# Configure logging if not already configured
logger = logging.getLogger(__name__)
if not logging.getLogger().handlers:
    logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(asctime)s %(filename)s:%(lineno)d - %(message)s')


class Simulator:
    """A class to start and terminate a SITL instance as a subprocess"""

    # SITL download URLs
    SITL_MASTER_URL = "https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/"
    SITL_BETA_URL = "https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/Beta/"

    # Cygwin DLL dependencies for Windows
    CYGWIN_DLLS = [
        "cygatomic-1.dll",
        "cyggcc_s-1.dll",
        "cyggcc_s-seh-1.dll",
        "cyggomp-1.dll",
        "cygiconv-2.dll",
        "cygintl-8.dll",
        "cygquadmath-0.dll",
        "cygssp-0.dll",
        "cygstdc++-6.dll",
        "cygwin1.dll",
    ]

    def __init__(self) -> None:
        self.sitl_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "SITL"))
        self.param_file = os.path.abspath(os.path.join(self.sitl_dir, "default_params", "default.parm"))
        self.process = None

        # Detect platform and set executable name
        self.platform_info = self._detect_platform()
        self.executable = os.path.join(self.sitl_dir, self._get_executable_name())

        # Ensure SITL directory exists
        os.makedirs(self.sitl_dir, exist_ok=True)

    def _detect_platform(self) -> dict:
        """Detect the operating system and architecture"""
        system = platform.system()
        machine = platform.machine().lower()

        return {
            "system": system,
            "machine": machine,
            "is_windows": system == "Windows",
            "is_linux": system == "Linux",
            "is_x86_64": machine in ["x86_64", "amd64"],
            "is_arm": machine in ["armv7l", "aarch64", "arm64"],
        }

    def _get_executable_name(self) -> str:
        """Get the appropriate executable name for the platform"""
        # Check for existing executables first
        if self.platform_info["is_windows"]:
            # Check for apm.exe (existing) or ArduCopter.elf (Cygwin executable)
            if os.path.exists(os.path.join(self.sitl_dir, "apm.exe")):
                return "apm.exe"
            return "ArduCopter.elf"
        else:
            # For Linux, check for existing files or use arducopter
            if os.path.exists(os.path.join(self.sitl_dir, "arducopter")):
                return "arducopter"
            if os.path.exists(os.path.join(self.sitl_dir, "ArduCopter")):
                return "ArduCopter"
            if os.path.exists(os.path.join(self.sitl_dir, "ArduCopter.elf")):
                return "ArduCopter.elf"
            return "arducopter"

    def _download_file(self, url: str, dest_path: str) -> bool:
        """Download a file from URL to destination path"""
        try:
            logger.info(f"Downloading {url}...")
            urllib.request.urlretrieve(url, dest_path)
            logger.info(f"Downloaded to {dest_path}")
            return True
        except Exception as e:
            logger.error(f"Failed to download {url}: {e}")
            return False

    def _set_executable_permission(self, file_path: str) -> None:
        """Set executable permissions on Linux/Unix systems"""
        if not self.platform_info["is_windows"]:
            try:
                st = os.stat(file_path)
                os.chmod(file_path, st.st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
                logger.info(f"Set executable permission on {file_path}")
            except Exception as e:
                logger.error(f"Failed to set executable permission: {e}")

    def _download_windows_sitl(self) -> bool:
        """Download SITL executable and dependencies for Windows"""
        # Download main executable (Cygwin .elf file)
        exe_url = f"{self.SITL_MASTER_URL}ArduCopter.elf"
        if not self._download_file(exe_url, self.executable):
            return False

        # Download Cygwin DLL dependencies
        for dll in self.CYGWIN_DLLS:
            dll_url = f"{self.SITL_MASTER_URL}{dll}"
            dll_path = os.path.join(self.sitl_dir, dll)
            # Only download if not already present
            if not os.path.exists(dll_path):
                self._download_file(dll_url, dll_path)

        return True

    def _download_linux_sitl(self) -> bool:
        """Download SITL executable for Linux"""
        if self.platform_info["is_x86_64"]:
            # For x86_64 Linux, download native binary
            exe_url = "https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/arducopter"
        elif self.platform_info["is_arm"]:
            # For ARM Linux, download ARM binary
            exe_url = "https://firmware.ardupilot.org/Copter/stable/SITL_arm_linux_gnueabihf/arducopter"
        else:
            logger.error(f"Unsupported Linux architecture: {self.platform_info['machine']}")
            return False

        if not self._download_file(exe_url, self.executable):
            return False

        # Set executable permissions
        self._set_executable_permission(self.executable)
        return True

    def check_and_download_sitl(self) -> bool:
        """Check if SITL executable exists, download if necessary"""
        if os.path.exists(self.executable):
            logger.info(f"SITL executable found at {self.executable}")
            return True

        logger.info(f"SITL executable not found at {self.executable}")
        logger.info(f"Attempting to download SITL for {self.platform_info['system']} ({self.platform_info['machine']})...")

        if self.platform_info["is_windows"]:
            success = self._download_windows_sitl()
        elif self.platform_info["is_linux"]:
            success = self._download_linux_sitl()
        else:
            logger.error(f"Unsupported platform: {self.platform_info['system']}")
            return False

        if success:
            logger.info("SITL download completed successfully")
        else:
            logger.error("SITL download failed")

        return success

    def start(self) -> None:
        """Start SITL instance in external console"""
        # Check if process is already running
        if self.process and self.process.poll() is None:
            logger.info("SITL process is already running")
            return

        # Ensure SITL is downloaded
        if not self.check_and_download_sitl():
            raise RuntimeError("Failed to download SITL executable")

        # Start SITL instance in external console
        # Starts with arguments '-Mx' by default (model quadcopter)
        if self.platform_info["is_windows"]:
            self.process = subprocess.Popen(
                " ".join([f'"{self.executable}"', "-Mx", f'--defaults "{self.param_file}"']),
                creationflags=subprocess.CREATE_NEW_CONSOLE,
            )
        else:
            # For Linux, just run the process directly
            self.process = subprocess.Popen([self.executable, "-Mx", "--defaults", self.param_file])
            logger.info(f"Started SITL process (PID: {self.process.pid})")

    def terminate(self) -> None:
        """Terminate the SITL process"""
        if self.process:
            if self.process.poll() is None:
                logger.info("Terminating SITL process")
                try:
                    self.process.terminate()
                    self.process.wait(timeout=5)
                    logger.info("SITL process terminated successfully")
                except subprocess.TimeoutExpired:
                    logger.warning("SITL process did not terminate gracefully, killing it")
                    self.process.kill()
                    self.process.wait()
            else:
                logger.debug("Process already terminated")
            self.process = None


# Run this class whenever this file is imported
sim = Simulator()

# Start SITL
try:
    logger.info("Starting SITL simulator...")
    sim.start()
    logger.info("SITL simulator started successfully")
except Exception as e:
    logger.error(f"Failed to start SITL: {e}")
    raise e
