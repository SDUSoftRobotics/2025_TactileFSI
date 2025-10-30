import socket
import time
import rtde_control
import rtde_receive

# -------------------
# Robot / frame setup
# -------------------
ROBOT_IP = "177.22.22.2"
rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

speed   = 0.1
accel   = 0.1
t_hold  = 2.0              # dwell at poke depth

# Feature plane pose in base: [px, py, pz, rx, ry, rz]  (replace with your calibrated plane if needed)
plane = [-38.52 * 0.001, -417.55 * 0.001, 0.02707 - 0.0016 + 0.15, 1.743, -2.614, 0.000]

# Behavior params
Y_OFFSET        = 0.20     # m along plane-Y (constant)
GAIN_X          = 3.0      # 50 mm estimate -> 150 mm robot move (3×)
X_START_OFFSET  = 0.050    # m extra inside along plane-X at start AND added to every target
Z_MAX_DOWN      = 0.12  # m (10 mm) safety clamp, matches MATLAB clamp

# -------------------
# Helpers
# -------------------
def pose_plane_to_base(x, y, z):
    """Build a plane-relative pose [x,y,z,0,0,0] and transform to base."""
    return rtde_c.poseTrans(plane, [x, y, z, 0.0, 0.0, 0.0])

def go_to_initial():
    """Before TCP starts: X=+50 mm, Y=+0.20 m, Z=0 in the plane frame."""
    x0 = X_START_OFFSET
    y0 = Y_OFFSET
    z0 = 0.0
    print(f"[INIT] MoveL to plane: X={x0:.3f} m, Y={y0:.3f} m, Z={z0:.3f} m")
    rtde_c.moveL(pose_plane_to_base(x0, y0, z0), speed, accel)
    time.sleep(0.5)

def move_poke(x_est_m: float, z_cmd_m: float):
    """
    x_est_m: X estimate from MATLAB (meters).
    z_cmd_m: Z command from MATLAB (meters), positive downward in plane-Z.
    Behavior:
      planeX = X_START_OFFSET + GAIN_X * x_est_m
      planeY = Y_OFFSET
      planeZ = clamp(z_cmd_m, 0 .. Z_MAX_DOWN)
      Sequence: (X, Y, Z=0) -> (X, Y, Z) [hold] -> (X, Y, Z=0)
    """
    x_plane = X_START_OFFSET + GAIN_X * x_est_m
    y_plane = Y_OFFSET
    z_plane = max(0.0, min(z_cmd_m, Z_MAX_DOWN))  # safety clamp

    print(f"[MOVE] estX={x_est_m*1000:.1f} mm, cmdZ={z_cmd_m*1000:.1f} mm  "
          f"-> planeX={x_plane:.3f} m, planeY={y_plane:.3f} m, planeZ={z_plane:.3f} m")

    # Approach at Z=0
    rtde_c.moveL(pose_plane_to_base(x_plane, y_plane, 0.0), speed, accel)
    # Poke down
    rtde_c.moveL(pose_plane_to_base(x_plane, y_plane, z_plane), speed, accel)
    time.sleep(t_hold)
    # Return to Z=0 at same X,Y
    rtde_c.moveL(pose_plane_to_base(x_plane, y_plane, 0.0), speed, accel)

# -------------------
# TCP server: expects "X,Z" in meters or 's' to stop
# -------------------
def receive_xz():
    HOST = "127.0.0.1"
    PORT = 65432

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        s.settimeout(20.0)
        print(f"[SERVER] Listening on {HOST}:{PORT}… waiting for MATLAB (20s timeout).")

        conn = None
        while conn is None:
            try:
                conn, addr = s.accept()
                print(f"[SERVER] Connected by {addr}")
                conn.settimeout(20.0)
            except socket.timeout:
                print("[SERVER] No connection in 20s. Still waiting…")
                continue

        try:
            buffer = ""
            while True:
                try:
                    chunk = conn.recv(1024)
                    if not chunk:
                        continue
                    buffer += chunk.decode("utf-8")

                    # Process line by line or token by token (commas/whitespace)
                    # We look for a token that contains a comma first.
                    # If not, fall back to splitting by whitespace.
                    # This keeps things robust if MATLAB sends without newline.
                    items = buffer.replace("\n", " ").replace("\r", " ").split()
                    if not items:
                        continue

                    token = items[0]
                    # Trim the consumed token from buffer
                    cut = buffer.find(token) + len(token)
                    buffer = buffer[cut:]

                    token = token.strip()
                    if not token:
                        continue

                    if token.lower() == "s":
                        print("[SERVER] Stop signal received. Shutting down.")
                        break

                    # Expect "X,Z" (meters)
                    if "," in token:
                        try:
                            x_str, z_str = token.split(",", 1)
                            x_m = float(x_str)
                            z_m = float(z_str)
                        except Exception as e:
                            print(f"[WARN] Bad token {token!r}: {e}")
                            continue
                    else:
                        # If a single value arrives, treat Z as 0 for safety
                        try:
                            x_m = float(token)
                            z_m = 0.0
                            print(f"[WARN] Got single value {x_m:.4f}; assuming Z=0.0")
                        except ValueError:
                            print(f"[WARN] Non-numeric token ignored: {token!r}")
                            continue

                    move_poke(x_m, z_m)

                except socket.timeout:
                    print("[SERVER] No data in 20s. Waiting…")
                    continue
        finally:
            conn.close()
            print("[SERVER] Connection closed.")

# -------------------
# Main
# -------------------
if __name__ == "__main__":
    # 1) Move to initial pose first (X=+50 mm, Y=+0.20 m, Z=0)
    go_to_initial()
    # 2) Start TCP server loop (expects "X,Z" in meters)
    receive_xz()
