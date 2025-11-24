import os
import subprocess


class Simulator:
    """A class to start and terminate a SITL instance as a subprocess"""

    def __init__(self) -> None:
        self.executable = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "SITL",
                "apm.exe",
            )
        )
        self.param_file = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "SITL",
                "default_params",
                "default.parm",
            )
        )
        self.process = None

    def start(self) -> None:
        # Start SITL instance in external console
        # Starts with arguments '-Mx' by default (model quadcopter)
        self.process = subprocess.Popen(
            " ".join(
                [
                    f'"{self.executable}"',
                    "-Mx",
                    f'--defaults "{self.param_file}"',
                ]
            ),
            creationflags=subprocess.CREATE_NEW_CONSOLE,
        )

    def terminate(self) -> None:
        self.process.terminate()


# Run this class whenever this file is imported
sim = Simulator()

# Start SITL
try:
    sim.start()
except Exception as e:
    raise e
