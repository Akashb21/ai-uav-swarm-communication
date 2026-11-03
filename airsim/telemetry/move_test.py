import airsim

import math

import os

import socket

import json

import time

import traceback
from swarm_visualization_patch import SwarmVisualizer

# ============================================================

# CITYENVIRON / AIRSIM - FINAL 6 UAV + NS-3 TELEMETRY CONTROLLER

# ============================================================

# Mapping:

#   DRONE 0 = UAV0

#   DRONE 1 = UAV1

#   DRONE 2 = UAV2

#   DRONE 3 = UAV3

#   DRONE 4 = UAV4

#   DRONE 5 = UAV5

#

# This controller fixes the current failure:

#   AirSim is actually spawning all six UAVs at (0,0).

#   The previous V11 intentionally stopped when separation was 0 m.

#

# Repair:

#   1. Detect collapsed start.

#   2. Create a verified 3x2 formation using simSetVehiclePose.

#   3. If AirSim does not retain it, fall back to sequential

#      moveToPositionAsync after takeoff.

#   4. Verify separation BEFORE mission.

#   5. Fly forward with LiDAR-guided local heading selection.

#   6. Send JSON telemetry continuously to NS-3 UDP 41451.

#

# NS-3 payload:

# {

#   "uav_id": 0,

#   "vehicle_name": "Drone1",

#   "airsim_name": "UAV0",

#   "x": ...,

#   "y": ...,

#   "z": ...,

#   "vx": ...,

#   "vy": ...,

#   "vz": ...,

#   "timestamp": ...,

#   "airsim_timestamp": ...

# }

VISUALIZER = None

# ============================================================

UAVS = [f"UAV{i}" for i in range(6)]

LIDAR_NAME = "LidarSensor1"

# ---------------- Mission ----------------

FLIGHT_Z = -9.0

MISSION_DISTANCE = 20.0

TARGET_TOLERANCE = 1.5

NORMAL_SPEED = 1.60

AVOID_SPEED = 1.25

REJOIN_SPEED = 1.25

RECOVERY_SPEED = 0.85

CONTROL_PERIOD = 0.20

COMMAND_DURATION = 0.80

MAX_RUNTIME = 120.0

# ---------------- Startup formation ----------------

COLLAPSED_START_THRESHOLD = 2.5

MIN_UAV_SEPARATION = 3.5

# Relative to the actual AirSim common start:

# UAV0 (-7,-6)

# UAV1 (-7,+6)

# UAV2 ( 0,-6)

# UAV3 ( 0,+6)

# UAV4 (+7,-6)

# UAV5 (+7,+6)

#

# These positions are based on the user's successful earlier

# CityEnviron preflight rather than guessed old map coordinates.

FORMATION = [

    (-7.0, -6.0),

    (-7.0,  6.0),

    ( 0.0, -6.0),

    ( 0.0,  6.0),

    ( 7.0, -6.0),

    ( 7.0,  6.0),

]

FORMATION_MOVE_SPEED = 2.2

FORMATION_TIMEOUT = 12.0

FORMATION_TOLERANCE = 1.0

# ---------------- LiDAR ----------------

MAX_SCAN_DISTANCE = 30.0

MIN_POINT_DISTANCE = 0.50

VERTICAL_BAND = 7.5

OBSTACLE_TRIGGER = 7.0

EMERGENCY_DISTANCE = 3.0

CLEAR_DISTANCE = 9.0

HEADINGS_DEG = [-75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75]

SECTOR_HALF_WIDTH_DEG = 10.0

# ---------------- Avoidance ----------------

MAX_AVOID_TIME = 30.0

REQUIRED_CLEAR_SCANS = 4

REQUIRED_PASS_PROGRESS = 5.0

MAX_AVOIDANCES = 8

BACKUP_TIME = 1.20

MAX_RECOVERY_COUNT = 5

# ---------------- Swarm separation ----------------

SEPARATION_RADIUS = 5.0

SEPARATION_GAIN = 0.35

# ---------------- Visualization ----------------
# IMPORTANT: this controller currently draws a geometric proximity graph.
# It does not receive NS-3 neighbor/connectivity data from the C++ simulation.
# Set UAV_TOPOLOGY_RANGE in the environment to test another threshold.
TOPOLOGY_RANGE = float(
    os.getenv("UAV_TOPOLOGY_RANGE", "150.0")
)
VISUAL_PERIOD = float(os.getenv("UAV_VISUAL_PERIOD", "0.10"))
LABEL_HEIGHT = float(os.getenv("UAV_LABEL_HEIGHT", "2.5"))
LABEL_SCALE = float(os.getenv("UAV_LABEL_SCALE", "1.5"))
LABEL_DURATION = 0.16


# ---------------- NS-3 telemetry ----------------

NS3_ENABLED = True

NS3_HOST = os.getenv("NS3_BRIDGE_HOST", "172.29.32.189")

NS3_PORT = int(os.getenv("NS3_BRIDGE_PORT", "41451"))

NS3_SEND_PERIOD = 0.10

CONTROL = None

VIS = None

NS3_SOCKET = None

LAST_NS3_SEND = {uav: 0.0 for uav in UAVS}

LAST_POSITIONS = {}

STATE = {}

for uav in UAVS:

    STATE[uav] = {

        "mode": "INIT",

        "done": False,

        "failed": False,

        "failure": "",

        "start_x": 0.0,

        "start_y": 0.0,

        "target_x": 0.0,

        "target_y": 0.0,

        "route_dx": 1.0,

        "route_dy": 0.0,

        "position": None,

        "sector": [MAX_SCAN_DISTANCE]  *  len(HEADINGS_DEG),

        "front": MAX_SCAN_DISTANCE,

        "left": MAX_SCAN_DISTANCE,

        "right": MAX_SCAN_DISTANCE,

        "scan_valid": False,

        "points": 0,

        "lidar_frame": "unknown",

        "last_scan_time": 0.0,

        "heading_deg": 0.0,

        "avoid_count": 0,

        "avoid_started": 0.0,

        "avoid_start_progress": 0.0,

        "clear_scans": 0,

        "recovery_count": 0,

        "recovery_started": 0.0,

        "last_progress": 0.0,

        "last_progress_time": 0.0,

        "last_log": 0.0,

        "formation_x": 0.0,

        "formation_y": 0.0,

    }

# ============================================================

# Math

# ============================================================

def norm2(x, y):

    d = math.hypot(x, y)

    if d < 1e-9:

        return 1.0, 0.0

    return x / d, y / d

def clamp(v, lo, hi):

    return max(lo, min(hi, v))

def quat_to_yaw(q):

    siny = 2.0  *  (q.w_val  *  q.z_val + q.x_val  *  q.y_val)

    cosy = 1.0 - 2.0  *  (q.y_val  *  q.y_val + q.z_val  *  q.z_val)

    return math.atan2(siny, cosy)

def rotate_xy(x, y, yaw):

    c = math.cos(yaw)

    s = math.sin(yaw)

    return x  *  c - y  *  s, x  *  s + y  *  c

def route_progress(uav, p):

    s = STATE[uav]

    return (

        (p.x_val - s["start_x"])  *  s["route_dx"]

        + (p.y_val - s["start_y"])  *  s["route_dy"]

    )

def lateral_offset(uav, p):

    s = STATE[uav]

    lx = -s["route_dy"]

    ly = s["route_dx"]

    return (

        (p.x_val - s["start_x"])  *  lx

        + (p.y_val - s["start_y"])  *  ly

    )

def heading_vector(uav, relative_deg):

    s = STATE[uav]

    a = math.radians(relative_deg)

    c = math.cos(a)

    sn = math.sin(a)

    lx = -s["route_dy"]

    ly = s["route_dx"]

    vx = s["route_dx"]  *  c + lx  *  sn

    vy = s["route_dy"]  *  c + ly  *  sn

    return norm2(vx, vy)

# ============================================================

# Connections

# ============================================================

def connect():

    global CONTROL, VIS, NS3_SOCKET

    CONTROL = airsim.MultirotorClient()

    CONTROL.confirmConnection()

    print("[CONTROL] AirSim connected.")

    try:

        VIS = airsim.MultirotorClient()

        VIS.confirmConnection()

        print("[VIS] AirSim visualization client connected.")

    except Exception as exc:

        VIS = None

        print(f"[VIS] disabled: {exc}")

    if NS3_ENABLED:

        try:

            NS3_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            NS3_SOCKET.setblocking(False)

            print(f"[NS3] Telemetry target = {NS3_HOST}:{NS3_PORT}")

        except Exception as exc:

            NS3_SOCKET = None

            print(f"[NS3] telemetry disabled: {exc}")

def hover(uav):

    try:

        CONTROL.hoverAsync(vehicle_name=uav)

    except Exception:

        pass

def fail(uav, reason):

    s = STATE[uav]

    if s["done"] or s["failed"]:

        return

    s["failed"] = True

    s["mode"] = "FAILED"

    s["failure"] = str(reason)

    hover(uav)

    print(f"[{uav}] FAILED -> {reason}")

# ============================================================

# NS-3 telemetry

# ============================================================

def send_telemetry(uav, airsim_state, force=False):

    if not NS3_ENABLED or NS3_SOCKET is None:

        return

    # Use a monotonic clock for rate limiting; keep wall-clock time

    # separately for the timestamp sent to NS-3. Each UAV has its own timer.

    now_mono = time.monotonic()

    now_wall = time.time()

    if uav not in LAST_NS3_SEND:

        LAST_NS3_SEND[uav] = 0.0

    if not force and (now_mono - LAST_NS3_SEND[uav]) < NS3_SEND_PERIOD:

        return

    k = airsim_state.kinematics_estimated

    p = k.position

    v = k.linear_velocity

    idx = UAVS.index(uav)

    payload = {

        "uav_id": idx,

        "vehicle_name": f"Drone{idx + 1}",

        "airsim_name": uav,

        "x": float(p.x_val),

        "y": float(p.y_val),

        "z": float(p.z_val),

        "vx": float(v.x_val),

        "vy": float(v.y_val),

        "vz": float(v.z_val),

        "timestamp": float(now_wall),

        "airsim_timestamp": float(airsim_state.timestamp),

    }

    raw = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")

    try:

        NS3_SOCKET.sendto(raw, (NS3_HOST, NS3_PORT))

        LAST_NS3_SEND[uav] = now_mono

    except (BlockingIOError, OSError) as exc:

        if getattr(exc, "errno", None) not in (10035, 11):

            print(f"[NS3] send warning: {exc}")

def send_initial_telemetry_burst():

    if not NS3_ENABLED or NS3_SOCKET is None:

        return

    for _ in range(6):

        for uav in UAVS:

            if STATE[uav]["failed"]:

                continue

            try:

                st = CONTROL.getMultirotorState(vehicle_name=uav)

                send_telemetry(uav, st, force=True)

            except Exception:

                pass

        time.sleep(0.05)

    print("[NS3] Initial telemetry burst sent for Drone1..Drone6.")

# ============================================================

# Position / formation

# ============================================================

def get_position(uav):

    st = CONTROL.getMultirotorState(vehicle_name=uav)

    return st.kinematics_estimated.position

def all_positions():

    result = {}

    for uav in UAVS:

        try:

            result[uav] = get_position(uav)

        except Exception as exc:

            print(f"[{uav}] position read error: {exc}")

    return result

def min_pair_distance(positions):

    vals = list(positions.values())

    if len(vals) < 2:

        return float("inf")

    best = float("inf")

    for i in range(len(vals)):

        for j in range(i + 1, len(vals)):

            dx = vals[i].x_val - vals[j].x_val

            dy = vals[i].y_val - vals[j].y_val

            best = min(best, math.hypot(dx, dy))

    return best

def formation_required(positions):

    return (

        len(positions) == len(UAVS)

        and min_pair_distance(positions) < COLLAPSED_START_THRESHOLD

    )

def formation_targets(reference):

    targets = []

    for uav, (dx, dy) in zip(UAVS, FORMATION):

        targets.append(

            (

                uav,

                reference.x_val + dx,

                reference.y_val + dy,

                FLIGHT_Z,

            )

        )

    return targets

def apply_pose_formation(reference):

    print("[FORMATION] Collapsed start detected.")

    print("[FORMATION] Applying verified 3x2 initialization formation...")

    for uav, x, y, z in formation_targets(reference):

        try:

            pose = airsim.Pose(

                airsim.Vector3r(

                    float(x),

                    float(y),

                    float(z),

                ),

                airsim.to_quaternion(0.0, 0.0, 0.0),

            )

            CONTROL.simSetVehiclePose(

                pose,

                True,

                vehicle_name=uav,

            )

            print(

                f"[FORMATION] pose {uav} -> "

                f"({x:.1f},{y:.1f},{z:.1f})"

            )

        except Exception as exc:

            print(f"[FORMATION] pose-set warning {uav}: {exc}")

    time.sleep(0.5)

    verify = all_positions()

    sep = min_pair_distance(verify)

    print(f"[FORMATION] post-pose minimum separation = {sep:.2f} m")

    return sep >= MIN_UAV_SEPARATION

def apply_move_formation_after_takeoff(reference):

    print("[FORMATION] Pose method was not retained. Falling back to physical position commands.")

    for uav, x, y, z in formation_targets(reference):

        if STATE[uav]["failed"]:

            continue

        try:

            CONTROL.moveToPositionAsync(

                float(x),

                float(y),

                float(z),

                FORMATION_MOVE_SPEED,

                timeout_sec=FORMATION_TIMEOUT,

                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,

                yaw_mode=airsim.YawMode(

                    is_rate=False,

                    yaw_or_rate=0.0

                ),

                vehicle_name=uav,

            ).join()

            p = get_position(uav)

            err = math.hypot(p.x_val - x, p.y_val - y)

            print(

                f"[FORMATION] move {uav} -> "

                f"({p.x_val:.2f},{p.y_val:.2f},{p.z_val:.2f}) "

                f"err={err:.2f}m"

            )

            if err > FORMATION_TOLERANCE:

                print(f"[FORMATION] {uav} move verification failed.")

                return False

        except Exception as exc:

            print(f"[FORMATION] {uav} physical move failed: {exc}")

            return False

    verify = all_positions()

    sep = min_pair_distance(verify)

    print(f"[FORMATION] post-move minimum separation = {sep:.2f} m")

    return sep >= MIN_UAV_SEPARATION

def initialize_formation():

    positions = all_positions()

    sep = min_pair_distance(positions)

    print(f"[FORMATION] current minimum separation = {sep:.2f} m")

    if not formation_required(positions):

        print("[FORMATION] Existing positions are already separated.")

        return True

    # Use the actual current common start as the reference.

    reference = positions[UAVS[0]]

    # First try pose initialization while grounded/disarmed.

    if apply_pose_formation(reference):

        return True

    print("[FORMATION] Pose verification failed; physical fallback will run after takeoff.")

    return False

# ============================================================

# Arm / takeoff

# ============================================================

def arm_all():

    print("\n[SYSTEM] Enabling API control and arming...")

    for uav in UAVS:

        try:

            CONTROL.enableApiControl(True, vehicle_name=uav)

            CONTROL.armDisarm(True, vehicle_name=uav)

            print(f"[{uav}] API control + armed.")

        except Exception as exc:

            fail(uav, f"arm/API error: {exc}")

def takeoff_all():

    print("\n[SYSTEM] Taking off sequentially...")

    for uav in UAVS:

        if STATE[uav]["failed"]:

            continue

        try:

            CONTROL.takeoffAsync(vehicle_name=uav).join()

            CONTROL.moveToZAsync(

                FLIGHT_Z,

                1.5,

                timeout_sec=8.0,

                vehicle_name=uav,

            ).join()

            print(f"[{uav}] takeoff complete.")

        except Exception as exc:

            fail(uav, f"takeoff error: {exc}")

def verify_or_repair_formation_after_takeoff():

    positions = all_positions()

    sep = min_pair_distance(positions)

    print(f"[FORMATION] airborne minimum separation = {sep:.2f} m")

    if sep >= MIN_UAV_SEPARATION:

        return True

    reference = positions[UAVS[0]]

    # Physical movement fallback first.

    if apply_move_formation_after_takeoff(reference):

        return True

    # Final deterministic simulation fallback.

    print("[FORMATION] physical fallback failed; reapplying exact simulation poses.")

    for uav, x, y, z in formation_targets(reference):

        try:

            pose = airsim.Pose(

                airsim.Vector3r(float(x), float(y), float(z)),

                airsim.to_quaternion(0.0, 0.0, 0.0),

            )

            CONTROL.simSetVehiclePose(

                pose,

                True,

                vehicle_name=uav,

            )

        except Exception as exc:

            print(f"[FORMATION] final pose fallback {uav}: {exc}")

    time.sleep(0.5)

    verify = all_positions()

    sep = min_pair_distance(verify)

    print(f"[FORMATION] final minimum separation = {sep:.2f} m")

    return sep >= MIN_UAV_SEPARATION

# ============================================================

# Mission initialization

# ============================================================

def initialize_routes():

    print("\n========================================")

    print("ACTUAL AIRSIM START / ROUTE INITIALIZATION")

    print("========================================")

    print("Verified six-UAV formation is used.")

    print("Route direction is derived from each actual UAV yaw.")

    for i, uav in enumerate(UAVS):

        if STATE[uav]["failed"]:

            continue

        try:

            st = CONTROL.getMultirotorState(vehicle_name=uav)

            p = st.kinematics_estimated.position

            yaw = quat_to_yaw(st.kinematics_estimated.orientation)

            dx = math.cos(yaw)

            dy = math.sin(yaw)

            dx, dy = norm2(dx, dy)

            s = STATE[uav]

            s["start_x"] = float(p.x_val)

            s["start_y"] = float(p.y_val)

            s["target_x"] = s["start_x"] + MISSION_DISTANCE  *  dx

            s["target_y"] = s["start_y"] + MISSION_DISTANCE  *  dy

            s["route_dx"] = dx

            s["route_dy"] = dy

            s["position"] = p

            s["last_progress"] = 0.0

            s["last_progress_time"] = time.time()

            s["mode"] = "NORMAL"

            LAST_POSITIONS[uav] = p

            send_telemetry(uav, st, force=True)

            print(

                f"DRONE {i} = {uav} "

                f"START=({p.x_val:.2f},{p.y_val:.2f},{p.z_val:.2f}) "

                f"YAW={math.degrees(yaw):.1f} "

                f"TARGET=({s['target_x']:.2f},{s['target_y']:.2f},{FLIGHT_Z:.2f})"

            )

        except Exception as exc:

            fail(uav, f"route initialization error: {exc}")

# ============================================================

# LiDAR frame normalization

# ============================================================

def _numeric_component(obj, *names, default=None):

    """Read a numeric component from AirSim objects or dict-shaped RPC data."""

    for name in names:

        try:

            value = obj.get(name) if isinstance(obj, dict) else getattr(obj, name)

        except Exception:

            continue

        if isinstance(value, dict):

            # Some RPC wrappers nest the actual scalar under value/data.

            for nested in ("value", "data", "val"):

                if nested in value:

                    value = value[nested]

                    break

        try:

            return float(value)

        except (TypeError, ValueError):

            continue

    if default is not None:

        return float(default)

    raise TypeError(f"Could not read numeric component {names} from {type(obj).__name__}: {obj!r}")

def _xyz(obj):

    return (

        _numeric_component(obj, "x_val", "x"),

        _numeric_component(obj, "y_val", "y"),

        _numeric_component(obj, "z_val", "z"),

    )

def _quaternion_yaw(q):

    w = _numeric_component(q, "w_val", "w")

    x = _numeric_component(q, "x_val", "x")

    y = _numeric_component(q, "y_val", "y")

    z = _numeric_component(q, "z_val", "z")

    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def normalized_scan_points(uav, lidar_data, vehicle_position):

    """Return finite XYZ LiDAR points in the sensor-local frame.

    AirSim's standard LidarData.point_cloud is a flat list of XYZ floats

    relative to the LiDAR sensor. Do not subtract lidar pose from vehicle

    position here: depending on AirSim RPC/client versions those pose fields

    can be nested dictionaries, which caused the float-minus-dict crash.

    The controller only needs local obstacle direction and distance.

    """

    if isinstance(lidar_data, dict):

        cloud = lidar_data.get("point_cloud", [])

    else:

        cloud = getattr(lidar_data, "point_cloud", [])

    if cloud is None:

        return [], "empty"

    # Some wrappers return point_cloud as a dict instead of a flat sequence.

    if isinstance(cloud, dict):

        for key in ("point_cloud", "data", "values"):

            if key in cloud:

                cloud = cloud[key]

                break

    try:

        cloud = list(cloud)

    except (TypeError, ValueError):

        return [], "invalid-point-cloud"

    if len(cloud) < 3:

        return [], "empty"

    points = []

    # Standard AirSim representation: [x1, y1, z1, x2, y2, z2, ...].

    if not isinstance(cloud[0], (dict, list, tuple)):

        for i in range(0, len(cloud) - 2, 3):

            try:

                x = float(cloud[i])

                y = float(cloud[i + 1])

                z = float(cloud[i + 2])

            except (TypeError, ValueError, OverflowError):

                continue

            if all(math.isfinite(v) for v in (x, y, z)):

                points.append((x, y, z))

        return points, "SensorLocalFrame"

    # Compatibility for wrappers that return one XYZ object per point.

    for item in cloud:

        try:

            if isinstance(item, dict):

                x = _numeric_component(item, "x_val", "x")

                y = _numeric_component(item, "y_val", "y")

                z = _numeric_component(item, "z_val", "z")

            elif isinstance(item, (list, tuple)) and len(item) >= 3:

                x, y, z = float(item[0]), float(item[1]), float(item[2])

            else:

                x, y, z = _xyz(item)

            if all(math.isfinite(v) for v in (x, y, z)):

                points.append((x, y, z))

        except (TypeError, ValueError, AttributeError, OverflowError):

            continue

    return points, "SensorLocalFrame"

def scan_environment(uav, p):

    data = CONTROL.getLidarData(

        lidar_name=LIDAR_NAME,

        vehicle_name=uav,

    )

    points, frame = normalized_scan_points(

        uav,

        data,

        p,

    )

    s = STATE[uav]

    sectors = [MAX_SCAN_DISTANCE]  *  len(HEADINGS_DEG)

    usable = 0

    rdx = s["route_dx"]

    rdy = s["route_dy"]

    left_x = -rdy

    left_y = rdx

    for px, py, pz in points:

        dist = math.sqrt(px  *  px + py  *  py + pz  *  pz)

        if dist < MIN_POINT_DISTANCE or dist > MAX_SCAN_DISTANCE:

            continue

        if abs(pz) > VERTICAL_BAND:

            continue

        forward = px  *  rdx + py  *  rdy

        lateral = px  *  left_x + py  *  left_y

        angle = math.degrees(

            math.atan2(lateral, forward)

        )

        if not -90.0 <= angle <= 90.0:

            continue

        usable += 1

        nearest = min(

            range(len(HEADINGS_DEG)),

            key=lambda k: abs(angle - HEADINGS_DEG[k])

        )

        if abs(angle - HEADINGS_DEG[nearest]) <= SECTOR_HALF_WIDTH_DEG:

            sectors[nearest] = min(

                sectors[nearest],

                dist

            )

    s["sector"] = sectors

    s["points"] = usable

    s["lidar_frame"] = frame

    if usable == 0:

        s["scan_valid"] = False

        return False

    s["scan_valid"] = True

    s["front"] = sectors[HEADINGS_DEG.index(0)]

    s["left"] = min(sectors[-1], sectors[-2], sectors[-3])

    s["right"] = min(sectors[0], sectors[1], sectors[2])

    s["last_scan_time"] = time.time()

    return True

# ============================================================

# State read

# ============================================================

def read_uav(uav):

    s = STATE[uav]

    try:

        st = CONTROL.getMultirotorState(

            vehicle_name=uav

        )

        p = st.kinematics_estimated.position

        if st.collision.has_collided:

            obj = (

                st.collision.object_name

                or "unknown obstacle"

            )

            fail(

                uav,

                f"collision with {obj}"

            )

            return None

        scan_environment(uav, p)

        s["position"] = p

        LAST_POSITIONS[uav] = p

        send_telemetry(uav, st)

        return p

    except Exception as exc:

        print(f"[{uav}] DEBUG traceback for state/LiDAR error:")

        traceback.print_exc()

        fail(uav, f"AirSim state/LiDAR error: {exc}")

        return None

# ============================================================

# Separation

# ============================================================

def separation_vector(uav, p):

    sx = 0.0

    sy = 0.0

    for other in UAVS:

        if other == uav:

            continue

        op = LAST_POSITIONS.get(other)

        if op is None:

            continue

        dx = p.x_val - op.x_val

        dy = p.y_val - op.y_val

        d = math.hypot(dx, dy)

        if d < 0.01 or d >= SEPARATION_RADIUS:

            continue

        weight = (

            SEPARATION_RADIUS - d

        ) / SEPARATION_RADIUS

        sx += dx / d  *  weight

        sy += dy / d  *  weight

    return sx, sy

# ============================================================

# Motion

# ============================================================

def command_velocity(uav, vx, vy, speed):

    s = STATE[uav]

    if s["done"] or s["failed"]:

        return

    vx, vy = norm2(vx, vy)

    # Add swarm separation, but cap its influence.

    rx, ry = separation_vector(

        uav,

        s["position"]

    )

    vx += SEPARATION_GAIN  *  rx

    vy += SEPARATION_GAIN  *  ry

    vx, vy = norm2(vx, vy)

    vx *= speed

    vy *= speed

    try:

        CONTROL.moveByVelocityZAsync(

            vx,

            vy,

            FLIGHT_Z,

            COMMAND_DURATION,

            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,

            yaw_mode=airsim.YawMode(

                is_rate=False,

                yaw_or_rate=0.0,

            ),

            vehicle_name=uav,

        )

    except Exception as exc:

        fail(

            uav,

            f"velocity command error: {exc}"

        )

# ============================================================

# Heading choice

# ============================================================

def candidate_score(uav, index, avoid_mode):

    s = STATE[uav]

    angle = HEADINGS_DEG[index]

    clear = s["sector"][index]

    if clear < EMERGENCY_DISTANCE:

        return -1e9

    clear_score = min(

        clear,

        MAX_SCAN_DISTANCE

    ) / MAX_SCAN_DISTANCE

    goal_score = max(

        0.0,

        math.cos(

            math.radians(angle)

        )

    )

    current = s["heading_deg"]

    turn_penalty = (

        min(

            abs(angle - current) / 90.0,

            1.0

        )

         *  0.10

    )

    if avoid_mode:

        return (

            0.62  *  clear_score

            + 0.38  *  goal_score

            - turn_penalty

        )

    return (

        0.85  *  goal_score

        + 0.15  *  clear_score

        - turn_penalty

    )

def choose_heading(uav, avoid_mode=False):

    s = STATE[uav]

    best_i = None

    best_score = -1e9

    for i in range(len(HEADINGS_DEG)):

        score = candidate_score(

            uav,

            i,

            avoid_mode

        )

        if score > best_score:

            best_score = score

            best_i = i

    if best_i is None:

        return s["route_dx"], s["route_dy"]

    angle = float(

        HEADINGS_DEG[best_i]

    )

    vx, vy = heading_vector(

        uav,

        angle

    )

    s["heading_deg"] = angle

    return vx, vy

# ============================================================

# Avoidance / recovery

# ============================================================

def enter_avoidance(uav, p):

    s = STATE[uav]

    if s["avoid_count"] >= MAX_AVOIDANCES:

        enter_recovery(

            uav,

            "avoidance budget reached"

        )

        return

    s["avoid_count"] += 1

    s["avoid_started"] = time.time()

    s["avoid_start_progress"] = route_progress(

        uav,

        p

    )

    s["clear_scans"] = 0

    s["mode"] = "AVOID"

    print(

        f"[{uav}] AVOID #{s['avoid_count']} "

        f"F={s['front']:.1f} "

        f"L={s['left']:.1f} "

        f"R={s['right']:.1f}"

    )

def enter_recovery(uav, reason):

    s = STATE[uav]

    if s["recovery_count"] >= MAX_RECOVERY_COUNT:

        fail(

            uav,

            f"recovery exhausted: {reason}"

        )

        return

    s["recovery_count"] += 1

    s["recovery_started"] = time.time()

    s["mode"] = "RECOVERY"

    print(

        f"[{uav}] RECOVERY #{s['recovery_count']} "

        f"-> {reason}"

    )

def recovery_step(uav):

    s = STATE[uav]

    elapsed = (

        time.time()

        - s["recovery_started"]

    )

    if elapsed < BACKUP_TIME:

        command_velocity(

            uav,

            -s["route_dx"],

            -s["route_dy"],

            RECOVERY_SPEED,

        )

        return

    s["mode"] = "AVOID"

    s["avoid_started"] = time.time()

    s["clear_scans"] = 0

    print(

        f"[{uav}] RECOVERY COMPLETE -> AVOID"

    )

# ============================================================

# Controller

# ============================================================

def controller_step(uav):

    s = STATE[uav]

    if s["done"] or s["failed"]:

        return

    p = read_uav(uav)

    if p is None:

        return

    progress = route_progress(uav, p)

    distance_to_target = math.hypot(

        s["target_x"] - p.x_val,

        s["target_y"] - p.y_val,

    )

    # Target check.

    if (

        distance_to_target <= TARGET_TOLERANCE

        or progress >= MISSION_DISTANCE - 0.25

    ):

        s["done"] = True

        s["mode"] = "DONE"

        hover(uav)

        print(

            f"[{uav}] SUCCESS -> TARGET REACHED "

            f"POS=({p.x_val:.2f},{p.y_val:.2f},{p.z_val:.2f})"

        )

        return

    # Invalid LiDAR: bounded recovery rather than stopping the swarm.

    if not s["scan_valid"]:

        if s["mode"] == "NORMAL":

            enter_recovery(

                uav,

                "invalid/empty LiDAR scan"

            )

            return

    # Detect stall in normal mode.

    if progress - s["last_progress"] >= 0.25:

        s["last_progress"] = progress

        s["last_progress_time"] = time.time()

    elif (

        s["mode"] == "NORMAL"

        and time.time() - s["last_progress_time"] > 4.0

    ):

        enter_avoidance(

            uav,

            p

        )

        s["last_progress_time"] = time.time()

    if s["mode"] == "NORMAL":

        if s["front"] <= OBSTACLE_TRIGGER:

            enter_avoidance(

                uav,

                p

            )

            return

        vx, vy = choose_heading(

            uav,

            avoid_mode=False

        )

        command_velocity(

            uav,

            vx,

            vy,

            NORMAL_SPEED,

        )

    elif s["mode"] == "AVOID":

        elapsed = (

            time.time()

            - s["avoid_started"]

        )

        if elapsed > MAX_AVOID_TIME:

            enter_recovery(

                uav,

                "avoidance timeout"

            )

            return

        progress_delta = (

            progress

            - s["avoid_start_progress"]

        )

        if s["front"] >= CLEAR_DISTANCE:

            s["clear_scans"] += 1

        else:

            s["clear_scans"] = 0

        if (

            progress_delta >= REQUIRED_PASS_PROGRESS

            and s["clear_scans"] >= REQUIRED_CLEAR_SCANS

        ):

            s["mode"] = "REJOIN"

            s["clear_scans"] = 0

            print(

                f"[{uav}] OBSTACLE PASSED -> REJOIN"

            )

            return

        if s["front"] <= EMERGENCY_DISTANCE:

            vx, vy = choose_heading(

                uav,

                avoid_mode=True

            )

        else:

            vx, vy = choose_heading(

                uav,

                avoid_mode=True

            )

        command_velocity(

            uav,

            vx,

            vy,

            AVOID_SPEED,

        )

    elif s["mode"] == "REJOIN":

        offset = lateral_offset(

            uav,

            p

        )

        if abs(offset) <= 1.2:

            s["mode"] = "NORMAL"

            print(

                f"[{uav}] ROUTE REJOINED -> NORMAL"

            )

            return

        correction = clamp(

            -offset  *  0.15,

            -0.45,

            0.45

        )

        left_x = -s["route_dy"]

        left_y = s["route_dx"]

        vx = (

            s["route_dx"]

            + left_x  *  correction

        )

        vy = (

            s["route_dy"]

            + left_y  *  correction

        )

        command_velocity(

            uav,

            vx,

            vy,

            REJOIN_SPEED,

        )

    elif s["mode"] == "RECOVERY":

        recovery_step(uav)

    now = time.time()

    if now - s["last_log"] >= 0.8:

        print(

            f"[{uav}] {s['mode']:<9} "

            f"POS=({p.x_val:7.2f},{p.y_val:7.2f},{p.z_val:6.2f}) "

            f"REM={distance_to_target:5.1f} "

            f"PROG={progress:5.1f} "

            f"F={s['front']:4.1f} "

            f"L={s['left']:4.1f} "

            f"R={s['right']:4.1f} "

            f"HDG={s['heading_deg']:5.0f} "

            f"PTS={s['points']:3d} "

            f"{s['lidar_frame']}"

        )

        s["last_log"] = now

# ============================================================

# Visualization

# ============================================================

def update_visualization():
    if VIS is None or VISUALIZER is None:
        return

    result = VISUALIZER.update()

def run_swarm():

    started = time.monotonic()

    next_visual = started

    print("\n========================================")

    print("CITYENVIRON 6-UAV FINAL + NS-3")

    print("========================================")

    for i, uav in enumerate(UAVS):

        print(f"DRONE {i} = {uav}")

    print(f"NS-3 telemetry = {NS3_HOST}:{NS3_PORT}")

    print("========================================")

    while not all(

        STATE[uav]["done"] or STATE[uav]["failed"]

        for uav in UAVS

    ):

        loop_start = time.monotonic()

        for uav in UAVS:

            if (

                not STATE[uav]["done"]

                and not STATE[uav]["failed"]

            ):

                controller_step(uav)

            # Refresh between UAV controller steps rather than

            # waiting until all active UAVs have been processed.

            now = time.monotonic()

            if now >= next_visual:

                update_visualization()

                next_visual = time.monotonic() + VISUAL_PERIOD

            # Enforce the global runtime limit during the loop.

            if now - started >= MAX_RUNTIME:

                print("\n[GLOBAL] runtime limit reached.")

                for pending_uav in UAVS:

                    if (

                        not STATE[pending_uav]["done"]

                        and not STATE[pending_uav]["failed"]

                    ):

                        fail(

                            pending_uav,

                            "global runtime timeout",

                        )

                break

        if time.monotonic() - started >= MAX_RUNTIME:

            break

        elapsed = time.monotonic() - loop_start

        remaining = CONTROL_PERIOD - elapsed

        if remaining > 0:

            time.sleep(min(remaining, 0.01))

    # Final refresh before hovering.

    try:

        update_visualization()

    except Exception as exc:

        print(f"[VIS] Final refresh failed: {exc}")

    for uav in UAVS:

        try:

            hover(uav)

        except Exception as exc:

            print(f"[WARN] Could not hover {uav}: {exc}")

def final_report():

    reached = 0

    print("\n========================================")

    print("FINAL UAV STATUS")

    print("========================================")

    for uav in UAVS:

        s = STATE[uav]

        if s["done"] and not s["failed"]:

            print(

                f"{uav}: SUCCESS - TARGET REACHED"

            )

            reached += 1

        else:

            print(

                f"{uav}: FAILED/STOPPED - "

                f"{s['failure'] or s['mode']}"

            )

    print("----------------------------------------")

    print(

        f"Targets reached: "

        f"{reached}/{len(UAVS)}"

    )

    print(

        f"Failed/stopped: "

        f"{len(UAVS) - reached}/{len(UAVS)}"

    )

    print("========================================")

def shutdown():

    print(

        "\n[SHUTDOWN] Hovering and releasing control..."

    )

    for uav in UAVS:

        try:

            hover(uav)

        except Exception:

            pass

    time.sleep(0.5)

    for uav in UAVS:

        try:

            CONTROL.armDisarm(

                False,

                vehicle_name=uav

            )

        except Exception:

            pass

        try:

            CONTROL.enableApiControl(

                False,

                vehicle_name=uav

            )

        except Exception:

            pass

    if NS3_SOCKET is not None:

        try:

            NS3_SOCKET.close()

        except Exception:

            pass

if __name__ == "__main__":

    try:

        connect()

        VISUALIZER = SwarmVisualizer(
            VIS,
            UAVS,
            airsim.Vector3r,
            topology_range=TOPOLOGY_RANGE,
            refresh_period=VISUAL_PERIOD,
            marker_lifetime=max(0.25, 2 * VISUAL_PERIOD),
            label_height=LABEL_HEIGHT,
            label_scale=LABEL_SCALE,
        )

        # Enable API control before formation initialization.

        arm_all()

        # If all six are collapsed, establish a verified formation.

        formation_ok = initialize_formation()

        # Takeoff only after the formation check.

        takeoff_all()

        # If the simulator reset/collapsed the formation during takeoff,

        # repair it using actual AirSim movement, then verify again.

        if not formation_ok:

            if not verify_or_repair_formation_after_takeoff():

                raise RuntimeError(

                    "Could not establish safe six-UAV formation."

                )

        else:

            # Always verify once airborne.

            if not verify_or_repair_formation_after_takeoff():

                raise RuntimeError(

                    "Airborne formation verification failed."

                )

        initialize_routes()

        send_initial_telemetry_burst()

        print("\n[SYSTEM] Starting swarm movement...")

        run_swarm()

        final_report()

    except KeyboardInterrupt:

        print("\n[USER] Interrupted.")

    except Exception as exc:

        print(

            f"\n[FATAL] {exc}"

        )

    finally:

        shutdown()

        print("\n========================================")

        print("FINAL TEST COMPLETE")

        print("========================================")
