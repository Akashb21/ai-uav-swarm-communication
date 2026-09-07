#!/usr/bin/env python3
"""Minimal WSL-side UDP test for AirSim telemetry reachability."""

import argparse
import json
import socket


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=41451)
    args = parser.parse_args()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((args.bind, args.port))
        print(f"Listening on {args.bind}:{args.port}; press Ctrl+C to stop.")
        while True:
            payload, sender = sock.recvfrom(4096)
            try:
                state = json.loads(payload)
                print(f"{sender[0]}:{sender[1]} UAV {state['uav_id']}: "
                      f"({state['x']}, {state['y']}, {state['z']})")
            except (json.JSONDecodeError, KeyError) as error:
                print(f"{sender[0]}:{sender[1]} invalid payload: {error}")


if __name__ == "__main__":
    main()
