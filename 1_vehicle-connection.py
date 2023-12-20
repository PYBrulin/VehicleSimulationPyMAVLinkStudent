import time

from pymavlink import mavutil

import base.simulator as simulator  # noqa : F401

print("Trying to connect to the conn...")
conn = mavutil.mavlink_connection("tcp:127.0.0.1:5760")

# Make sure the connection is valid
conn.wait_heartbeat()
print("Connected to the conn.")

# Get some information !
_start_time = time.time()
try:
    while time.time() - _start_time < 30:
        try:
            print(conn.recv_match().to_dict())
        except Exception:
            pass
        time.sleep(0.1)  # 10Hz

except Exception:
    pass
finally:
    print("Closing connection...")
    conn.close()
    simulator.sim.terminate()
