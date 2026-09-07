#!/bin/bash

set -e

PROJECT_DIR="/home/tech_ece/uav-swarm-project"
NS3_DIR="/home/tech_ece/ns-3-dev"

SOURCE_FILE="$PROJECT_DIR/scratch/uav-swarm/uav-phase2.cc"
BRIDGE_HEADER="$PROJECT_DIR/scratch/uav-swarm/airsim-udp-bridge.h"
BRIDGE_SOURCE="$PROJECT_DIR/scratch/uav-swarm/airsim-udp-bridge.cc"
RESULT_DIR="$PROJECT_DIR/results/phase2"
NS3_RESULT_DIR="$NS3_DIR/results/phase2"

UAVS=6
SIM_TIME=15
USE_AIRSIM=false
AIRSIM_BRIDGE_HOST="${AIRSIM_BRIDGE_HOST:-0.0.0.0}"
AIRSIM_BRIDGE_PORT="${AIRSIM_BRIDGE_PORT:-41451}"
AIRSIM_UPDATE_MS="${AIRSIM_UPDATE_MS:-50}"
AIRSIM_STALE_TIMEOUT="${AIRSIM_STALE_TIMEOUT:-1.0}"
AIRSIM_INIT_TIMEOUT="${AIRSIM_INIT_TIMEOUT:-5.0}"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --use-air-sim)
            USE_AIRSIM=true
            ;;
        --help|-h)
            echo "Usage: $0 [--use-air-sim]"
            echo "AirSim settings: AIRSIM_BRIDGE_HOST, AIRSIM_BRIDGE_PORT, AIRSIM_UPDATE_MS,"
            echo "                 AIRSIM_STALE_TIMEOUT, AIRSIM_INIT_TIMEOUT"
            exit 0
            ;;
        *)
            echo "[ERROR] Unknown option: $1"
            exit 1
            ;;
    esac
    shift
done

mkdir -p "$RESULT_DIR"
mkdir -p "$NS3_RESULT_DIR"

if [ ! -f "$NS3_DIR/ns3" ]; then
    echo "[ERROR] NS-3 executable not found:"
    echo "$NS3_DIR/ns3"
    exit 1
fi

for REQUIRED_FILE in "$SOURCE_FILE" "$BRIDGE_HEADER" "$BRIDGE_SOURCE"; do
    if [ ! -f "$REQUIRED_FILE" ]; then
        echo "[ERROR] Required simulation file not found: $REQUIRED_FILE"
        exit 1
    fi
done

echo "=============================================="
echo "        UAV SWARM - PHASE 1"
echo "   BASIC UAV WIFI CONNECTIVITY TEST"
echo "=============================================="

echo "Project : $PROJECT_DIR"
echo "NS-3    : $NS3_DIR"
echo "UAVs    : $UAVS"
echo "Time    : $SIM_TIME s"
echo "=============================================="

mkdir -p "$NS3_DIR/scratch/uav-swarm"

cp "$SOURCE_FILE" "$BRIDGE_HEADER" "$BRIDGE_SOURCE" \
   "$NS3_DIR/scratch/uav-swarm/"

echo ""
echo "[1/3] Building NS-3..."

cd "$NS3_DIR"

./ns3 build

echo ""
echo "[2/3] Running simulation..."

RUN_ARGUMENTS="--nUavs=$UAVS --simTime=$SIM_TIME"

if [ "$USE_AIRSIM" = true ]; then
    echo "[AirSim] Start AirSim/Blocks with Drone1..Drone6 before continuing."
    echo "[AirSim] NS-3 will bind UDP at $AIRSIM_BRIDGE_HOST:$AIRSIM_BRIDGE_PORT"
    RUN_ARGUMENTS="$RUN_ARGUMENTS --useAirSim=true --airSimBindAddress=$AIRSIM_BRIDGE_HOST --airSimPort=$AIRSIM_BRIDGE_PORT --airSimUpdateMs=$AIRSIM_UPDATE_MS --airSimStaleTimeout=$AIRSIM_STALE_TIMEOUT --airSimInitTimeout=$AIRSIM_INIT_TIMEOUT"
fi

./ns3 run "scratch/uav-swarm/uav-phase2 $RUN_ARGUMENTS"

echo ""
echo "[3/3] Collecting results..."

mkdir -p "$RESULT_DIR"

if [ -d "$NS3_RESULT_DIR" ]; then
    cp "$NS3_RESULT_DIR/"* \
       "$RESULT_DIR/" 2>/dev/null || true
fi

echo ""
echo "=============================================="
echo " Simulation completed"
echo "=============================================="

echo "Results:"
ls -lh "$RESULT_DIR" 2>/dev/null || true
