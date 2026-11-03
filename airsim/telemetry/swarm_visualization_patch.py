"""AirSim live UAV proximity-graph visualizer.

This module deliberately uses short-lived, non-persistent debug primitives.
It does not call simFlushPersistentMarkers(), because that operation clears
persistent debug markers globally in the simulator.
"""
from __future__ import annotations

import math
from typing import Callable, Iterable


class SwarmVisualizer:
    def __init__(
        self,
        client,
        vehicle_names: Iterable[str],
        vector_factory: Callable[[float, float, float], object],
        *,
        topology_range: float = 150.0,
        refresh_period: float = 0.10,
        marker_lifetime: float | None = None,
        label_height: float = 2.5,
        label_scale: float = 1.0,
        point_size: float = 7.0,
        line_thickness: float = 2.5,
        log: Callable[[str], None] = print,
    ):
        self.client = client
        self.vehicle_names = tuple(vehicle_names)
        self.vector_factory = vector_factory
        self.topology_range = float(topology_range)
        self.refresh_period = float(refresh_period)
        self.marker_lifetime = (
            float(marker_lifetime)
            if marker_lifetime is not None
            else max(0.25, 2.0 * self.refresh_period)
        )
        self.label_height = float(label_height)
        self.label_scale = float(label_scale)
        self.point_size = float(point_size)
        self.line_thickness = float(line_thickness)
        self.log = log
        self._last_topology = None

        if not self.vehicle_names:
            raise ValueError("vehicle_names cannot be empty")
        if len(set(self.vehicle_names)) != len(self.vehicle_names):
            raise ValueError("vehicle_names must be unique")
        if not math.isfinite(self.topology_range) or self.topology_range <= 0:
            raise ValueError("topology_range must be a finite positive number")
        if not math.isfinite(self.refresh_period) or self.refresh_period <= 0:
            raise ValueError("refresh_period must be a finite positive number")
        if (
            not math.isfinite(self.marker_lifetime)
            or self.marker_lifetime <= self.refresh_period
        ):
            raise ValueError(
                "marker_lifetime must be finite and greater than refresh_period"
            )
        for name, value in (
            ("label_height", self.label_height),
            ("label_scale", self.label_scale),
            ("point_size", self.point_size),
            ("line_thickness", self.line_thickness),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")

    @staticmethod
    def _xyz(position):
        xyz = (
            float(position.x_val),
            float(position.y_val),
            float(position.z_val),
        )
        if not all(math.isfinite(v) for v in xyz):
            raise ValueError("position contains NaN or infinity")
        return xyz

    def _read_positions(self):
        positions = {}
        errors = {}
        for vehicle in self.vehicle_names:
            try:
                state = self.client.getMultirotorState(vehicle_name=vehicle)
                p = state.kinematics_estimated.position
                positions[vehicle] = self._xyz(p)
            except Exception as exc:
                errors[vehicle] = str(exc)
        return positions, errors

    def _build_edges(self, positions):
        edges = []
        names = []
        vehicles = list(positions)
        for i, left in enumerate(vehicles):
            ax, ay, az = positions[left]
            for right in vehicles[i + 1:]:
                bx, by, bz = positions[right]
                distance = math.dist((ax, ay, az), (bx, by, bz))
                if distance <= self.topology_range:
                    edges.append((
                        self.vector_factory(ax, ay, az),
                        self.vector_factory(bx, by, bz),
                    ))
                    names.append((left, right))
        return edges, tuple(names)

    def update(self):
        """Read fresh poses and draw one current frame; return testable stats."""
        if self.client is None:
            return {"positions": 0, "edges": 0, "errors": {"client": "not connected"}}

        positions, errors = self._read_positions()
        if not positions:
            if errors:
                self.log(f"[VIS] No UAV poses available: {errors}")
            return {"positions": 0, "edges": 0, "errors": errors}

        point_objects = []
        labels = []
        label_positions = []
        for index, (vehicle, (x, y, z)) in enumerate(positions.items()):
            point_objects.append(self.vector_factory(x, y, z))
            labels.append(f"U{self.vehicle_names.index(vehicle)}")
            # AirSim uses NED: a smaller/more-negative Z is visually higher.
            label_positions.append(
                self.vector_factory(x, y, z - self.label_height)
            )

        edges, topology = self._build_edges(positions)

        if topology != self._last_topology:
            self.log(
                f"[VIS] Geometric proximity graph "
                f"({self.topology_range:.1f} m): "
                f"{topology if topology else 'no links'}"
            )
            self._last_topology = topology

        # Use finite-lived transient primitives. Each refresh draws new world
        # coordinates; previous frames expire automatically. No global flush.
        draw_errors = {}

        try:
            self.client.simPlotPoints(
                point_objects,
                color_rgba=[0.20, 0.80, 1.0, 1.0],
                size=self.point_size,
                duration=self.marker_lifetime,
                is_persistent=False,
            )
        except Exception as exc:
            draw_errors["points"] = str(exc)

        try:
            self.client.simPlotStrings(
                labels,
                label_positions,
                scale=self.label_scale,
                duration=self.marker_lifetime,
            )
        except Exception as exc:
            draw_errors["labels"] = str(exc)

        if edges:
            # AirSim simPlotLineList expects a flat list: p1,p2,p1,p2,...
                    # Draw each UAV-to-UAV connection independently.
            for edge in edges:
                try:
                    self.client.simPlotLineList(
                        list(edge),
                        color_rgba=[1.0, 0.58, 0.08, 1.0],
                        thickness=self.line_thickness,
                        duration=self.marker_lifetime,
                        is_persistent=False,
                    )
                except Exception as exc:
                    draw_errors["links"] = str(exc)

        if draw_errors:
            self.log(f"[VIS] Draw warnings: {draw_errors}")

        return {
            "positions": len(positions),
            "edges": len(edges),
            "topology": topology,
            "errors": errors,
            "draw_errors": draw_errors,
            "positions_by_vehicle": positions,
        }
