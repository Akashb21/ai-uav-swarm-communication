#!/usr/bin/env python3
"""Send AirSim Drone1..Drone6 kinematics to the NS-3 UDP mobility bridge."""

import argparse
import json
import socket
import sys
import time


VEHICLES = [f"Drone{i}" for i in range(1, 7)]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="WSL IPv4 address reachable from Windows")
    parser.add_argument("--port", type=int, default=41451, help="NS-3 UDP bridge port")
    parser.add_argument("--rate-hz", type=float, default=20.0, help="Telemetry frequency")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.rate_hz <= 0 or not 1 <= args.port <= 65535:
        raise SystemExit("--rate-hz must be positive and --port must be in 1..65535")
    try:
        import airsim
    except ImportError as error:
        raise SystemExit("Install the AirSim Python client in Windows: pip install airsim") from error

    client = airsim.MultirotorClient()
    try:
        client.confirmConnection()
        for vehicle in VEHICLES:
            state = client.getMultirotorState(vehicle_name=vehicle)
            if not state:
                raise RuntimeError(f"no state returned for {vehicle}")
    except Exception as error:
        raise SystemExit(f"AirSim verification failed: {error}") from error

    destination = (args.host, args.port)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / args.rate_hz
    print(f"AirSim telemetry: {', '.join(VEHICLES)} -> {args.host}:{args.port} at {args.rate_hz:g} Hz")

    try:
        while True:
            started = time.monotonic()
            for uav_id, vehicle in enumerate(VEHICLES):
                state = client.getMultirotorState(vehicle_name=vehicle)
                kinematics = state.kinematics_estimated
                position = kinematics.position
                velocity = kinematics.linear_velocity
                message = {
                    "uav_id": uav_id,
                    "x": position.x_val,
                    "y": position.y_val,
                    "z": position.z_val,
                    "vx": velocity.x_val,
                    "vy": velocity.y_val,
                    "vz": velocity.z_val,
                    "timestamp": time.time_ns(),
                }
                udp.sendto(json.dumps(message, separators=(",", ":")).encode("utf-8"), destination)
            delay = interval - (time.monotonic() - started)
            if delay > 0:
                time.sleep(delay)
    except KeyboardInterrupt:
        print("\nTelemetry stopped.")
    except (OSError, RuntimeError) as error:
        print(f"Telemetry stopped: {error}", file=sys.stderr)
        return 1
    finally:
        udp.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
