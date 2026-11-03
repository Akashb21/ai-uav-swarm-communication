import airsim
import socket
import json
import time


# ============================================================
# CONFIGURATION
# ============================================================

# AirSim RPC
AIRSIM_IP = "127.0.0.1"
AIRSIM_RPC_PORT = 41451

# NS-3 / WSL
NS3_IP = "172.29.32.189"
NS3_PORT = 41451

# AirSim vehicle names from settings.json
VEHICLES = [
    "UAV0",
    "UAV1",
    "UAV2",
    "UAV3",
    "UAV4",
    "UAV5"
]

UAV_TO_NS3_ID = {
    "UAV0": "Drone1",
    "UAV1": "Drone2",
    "UAV2": "Drone3",
    "UAV3": "Drone4",
    "UAV4": "Drone5",
    "UAV5": "Drone6"
}

SEND_HZ = 10
SEND_INTERVAL = 1.0 / SEND_HZ


# ============================================================
# UDP SOCKET FOR NS-3
# ============================================================

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


# ============================================================
# CONNECT TO AIRSIM
# ============================================================

print("Connecting to AirSim...")

client = airsim.MultirotorClient(
    ip=AIRSIM_IP,
    port=AIRSIM_RPC_PORT
)

client.confirmConnection()

print("Connected to AirSim")
print()
print("Checking UAVs...")

print("Connecting to AirSim...")

client = airsim.MultirotorClient(
    ip=AIRSIM_IP,
    port=AIRSIM_RPC_PORT
)

client.confirmConnection()

print("Connected to AirSim")
print()
print("Checking UAVs...")


# ============================================================
# CHECK ALL UAVs
# ============================================================

available_vehicles = []

for vehicle_name in VEHICLES:

    try:

        state = client.getMultirotorState(
            vehicle_name=vehicle_name
        )

        position = state.kinematics_estimated.position

        print(
            f"[OK] {vehicle_name} "
            f"Position: "
            f"X={position.x_val:.2f}, "
            f"Y={position.y_val:.2f}, "
            f"Z={position.z_val:.2f}"
        )

        available_vehicles.append(vehicle_name)

    except Exception as e:

        print(
            f"[ERROR] {vehicle_name}: {e}"
        )


# ============================================================
# VERIFY SIX UAVs
# ============================================================

if len(available_vehicles) != 6:

    print()
    print(
        f"[ERROR] Expected 6 UAVs, "
        f"but only {len(available_vehicles)} are available."
    )

    print("Available vehicles:")

    for vehicle in available_vehicles:
        print(f"  {vehicle}")

    sock.close()
    raise SystemExit(1)


print()
print("All 6 UAVs are available.")
print()
print(
    f"Sending telemetry to "
    f"{NS3_IP}:{NS3_PORT}"
)
print(
    f"Update rate: {SEND_HZ} Hz"
)
print()
print("Press Ctrl+C to stop.")
print()


# ============================================================
# TELEMETRY LOOP
# ============================================================

try:

    while True:

        loop_start = time.time()

        for vehicle_name in VEHICLES:

            try:

                # --------------------------------------------
                # Get UAV state
                # --------------------------------------------

                state = client.getMultirotorState(
                    vehicle_name=vehicle_name
                )

                kinematics = state.kinematics_estimated

                position = kinematics.position
                velocity = kinematics.linear_velocity


                # --------------------------------------------
                # Create telemetry packet
                # --------------------------------------------

                packet = {
                    "uav_id": UAV_TO_NS3_ID[vehicle_name],
                    "timestamp": time.time(),
                    "position": {
                        "x": position.x_val,
                        "y": position.y_val,
                        "z": position.z_val
                    },
                    "velocity": {
                        "vx": velocity.x_val,
                        "vy": velocity.y_val,
                        "vz": velocity.z_val
                    }
                }

                # Compact JSON so the lightweight NS-3 parser
                # receives predictable formatting.
                message = json.dumps(
                    packet,
                    separators=(",", ":"),
                    allow_nan=False
                )


                # --------------------------------------------
                # Send UDP packet to NS-3
                # --------------------------------------------

                sock.sendto(
                    message.encode("utf-8"),
                    (NS3_IP, NS3_PORT)
                )


            except Exception as e:

                print(
                    f"[ERROR] Failed to read "
                    f"{vehicle_name}: {e}"
                )


        # --------------------------------------------
        # Maintain approximately SEND_HZ
        # --------------------------------------------

        elapsed = time.time() - loop_start

        sleep_time = SEND_INTERVAL - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)


except KeyboardInterrupt:

    print()
    print("Stopping telemetry sender...")


finally:

    sock.close()
    print("Telemetry sender stopped.")