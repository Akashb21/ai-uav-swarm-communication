# AirSim-driven UAV mobility

The existing NS-3 model remains the network authority: its six nodes retain ad-hoc 802.11g, Friis propagation, IPv4/AODV, UDP traffic, FlowMonitor, NetAnim, summary CSV, and neighbor CSV.  AirSim is only the position/velocity authority when AirSim mode is selected.

## Mapping and coordinates

`Drone1` through `Drone6` map respectively to NS-3 node IDs `0` through `5`. The Windows sender emits one JSON datagram per vehicle with `uav_id`, position, velocity, and a nanosecond Unix timestamp.

AirSim reports metres in its local NED frame: `x=north`, `y=east`, `z=down`. NS-3 coordinates have no inherent geographic frame. The default transform preserves north/east and makes altitude positive up:

```
ns3_x =  1 * airsim_x + offset_x
ns3_y =  1 * airsim_y + offset_y
ns3_z = -1 * airsim_z + offset_z
```

Change scales and offsets with NS-3 command-line options (`--airSimScaleX`, `--airSimOffsetX`, etc.) when a different Unreal/AirSim origin is needed.

## Start AirSim on Windows

Configure an AirSim environment with six multirotors named `Drone1` through `Drone6`, then start Unreal/Blocks and wait for it to be ready. In Windows PowerShell, from this project directory (or a copy of the `airsim` directory):

```powershell
py -m pip install airsim
$wslIp = (wsl.exe hostname -I).Trim().Split(' ')[0]
py .\airsim\airsim_telemetry_sender.py --host $wslIp --port 41451 --rate-hz 20
```

Do not assume `localhost` crosses the Windows/WSL boundary. `wsl.exe hostname -I` provides the current WSL address in the usual WSL2 NAT arrangement; use the address/interface that Windows can route to. If Windows cannot reach it, use mirrored networking or a Windows firewall rule as appropriate for the local WSL configuration.

## Test UDP reachability

First, in WSL, run:

```bash
python3 tools/udp_receiver_test.py --bind 0.0.0.0 --port 41451
```

Then start the Windows sender. It should print changing positions for UAV IDs 0–5. Stop the receiver before starting NS-3, since both bind the same UDP port.

## Run NS-3

Baseline (verified independently of AirSim):

```bash
./run_phase2.sh
```

AirSim mode, after the Windows sender is running:

```bash
AIRSIM_BRIDGE_HOST=0.0.0.0 AIRSIM_BRIDGE_PORT=41451 ./run_phase2.sh --use-air-sim
```

The runner also honours `AIRSIM_UPDATE_MS` (default 50), `AIRSIM_STALE_TIMEOUT` (default 1.0), and `AIRSIM_INIT_TIMEOUT` (default 5.0). AirSim mode uses NS-3's real-time scheduler and a non-blocking UDP receiver polled by scheduled events. It never performs a blocking read inside the event loop.

On startup it requires a state for every UAV before the initialization timeout. A missing complete set stops the run with exit code 2; it never falls back to the synthetic trajectory. When an already-seen UAV becomes stale, NS-3 logs a warning and freezes that node at its last received position.

## Outputs

All outputs land in `results/phase2/`: `summary.csv`, `neighbors.csv`, and `uav-phase2.xml`; AirSim mode additionally writes `airsim-mobility.csv`. `neighbors.csv` is sampled once per second and uses a 150 m analysis neighborhood threshold.
