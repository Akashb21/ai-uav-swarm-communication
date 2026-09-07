import csv
import os

import matplotlib.pyplot as plt


SUMMARY_FILE = "results/phase2/summary.csv"
NEIGHBOR_FILE = "results/phase2/neighbors.csv"


def read_summary():
    data = {}

    with open(SUMMARY_FILE, "r", newline="") as file:
        reader = csv.DictReader(file)

        for row in reader:
            try:
                data[row["metric"]] = float(row["value"])
            except ValueError:
                # Metadata such as test,mobile_aodv_multi_hop is not a numeric metric.
                data[row["metric"]] = row["value"]

    return data


def read_neighbors():
    rows = []

    with open(NEIGHBOR_FILE, "r", newline="") as file:
        reader = csv.DictReader(file)

        for row in reader:
            rows.append(
                {
                    "time": float(row["time"]),
                    "uav": int(row["uav_id"]),
                    "neighbor": int(row["neighbor_id"]),
                    "distance": float(row["distance_m"]),
                }
            )

    return rows


def plot_neighbor_distance(rows):
    pairs = {}

    for row in rows:
        key = (row["uav"], row["neighbor"])

        if key not in pairs:
            pairs[key] = {
                "time": [],
                "distance": [],
            }

        pairs[key]["time"].append(row["time"])
        pairs[key]["distance"].append(row["distance"])

    plt.figure(figsize=(10, 6))

    for pair, values in pairs.items():
        plt.plot(
            values["time"],
            values["distance"],
            label=f"UAV {pair[0]} → UAV {pair[1]}",
        )

    plt.xlabel("Time (s)")
    plt.ylabel("Distance (m)")
    plt.title("UAV Neighbor Distance vs Time")
    plt.grid(True)
    plt.legend(fontsize=8)

    plt.tight_layout()

    output = "results/phase2/neighbor_distance.png"

    plt.savefig(output, dpi=300)

    print(f"Saved: {output}")

    plt.show()


def print_summary(summary):
    print("\n========== PHASE 2 SUMMARY ==========")

    print(
        f"UAV count        : "
        f"{int(summary['uav_count'])}"
    )

    print(
        f"Simulation time  : "
        f"{summary['simulation_time_s']:.1f} s"
    )

    print(
        f"Range            : "
        f"{summary['communication_range_m']:.1f} m"
    )

    print(
        f"TX packets       : "
        f"{int(summary['tx_packets'])}"
    )

    print(
        f"RX packets       : "
        f"{int(summary['rx_packets'])}"
    )

    print(
        f"Lost packets     : "
        f"{int(summary['lost_packets'])}"
    )

    print(
        f"PDR              : "
        f"{summary['packet_delivery_ratio_percent']:.2f} %"
    )

    print(
        f"Packet loss      : "
        f"{summary['packet_loss_percent']:.2f} %"
    )

    print(
        f"Average delay    : "
        f"{summary['average_delay_ms']:.2f} ms"
    )

    print(
        f"Throughput       : "
        f"{summary['throughput_kbps']:.2f} Kbps"
    )

    print("=====================================\n")


def main():
    if not os.path.exists(SUMMARY_FILE):
        raise FileNotFoundError(
            f"Missing file: {SUMMARY_FILE}"
        )

    if not os.path.exists(NEIGHBOR_FILE):
        raise FileNotFoundError(
            f"Missing file: {NEIGHBOR_FILE}"
        )

    summary = read_summary()

    neighbors = read_neighbors()

    print_summary(summary)

    if neighbors:
        plot_neighbor_distance(neighbors)


if __name__ == "__main__":
    main()
