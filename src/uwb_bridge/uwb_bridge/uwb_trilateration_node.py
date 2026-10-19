#!/usr/bin/env python3
# Copyright 2026 AerialVehicleControl.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Weighted least-squares trilateration of a UWB tag from 4 anchor ranges.

Subscribes to geometry_msgs/Pose messages from ``ros_gz_bridge``
(produced by the gz-sim plugin ``uwb_gz_plugin``). Each Pose carries
one anchor ranging in a custom encoding (see UwbSystem.cc):
    pose.position.x = range
    pose.position.y = rss
    pose.position.z = anchor_id
    pose.orientation.x = los code (0=LOS, 1=NLOS-soft, 2=NLOS-hard)
Solves for the (x, y) position using a non-linear least-squares
optimizer, and publishes a ``geometry_msgs/PoseWithCovarianceStamped``
that MAVROS ``vision_pose_estimate`` can feed into ArduPilot's EKF.
"""

from __future__ import annotations

import json
import math
import threading
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
)
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


@dataclass
class Anchor:
    """World-frame anchor location (matches the gz plugin's layout)."""

    id: int
    name: str
    position: Tuple[float, float, float]


@dataclass
class RangeMeasurement:
    """One ranging sample to a single anchor."""

    anchor_id: int
    anchor_name: str
    range_m: float
    rss_dbm: float
    los: int  # 0 = LOS, 1 = NLOS soft, 2 = NLOS hard

    def weight(self, los_std: float, nlos_soft_std: float,
               nlos_hard_std: float) -> float:
        """Inverse-variance weight used by the WLS solver.

        We approximate variance from the link quality flag plus RSS.
        NLOS hard measurements are penalised the most.
        """
        if self.los == 0:
            sigma = los_std
        elif self.los == 1:
            sigma = nlos_soft_std
        else:
            sigma = nlos_hard_std
        # Add a small RSS-based term: lower RSS -> higher variance.
        rss_term = 1.0 + max(0.0, (-70.0 - self.rss_dbm)) * 0.05
        sigma = sigma * rss_term
        return 1.0 / (sigma * sigma)


class TrilaterationNode(Node):
    """Subscribes to UWB JSON, solves for (x, y, z), publishes pose."""

    DEFAULT_ANCHORS = [
        # id  name           x    y    z
        Anchor(0, "uwb_anchor0", (0.0,  0.0, 2.0)),
        Anchor(1, "uwb_anchor1", (10.0, 0.0, 2.0)),
        Anchor(2, "uwb_anchor2", (10.0, 10.0, 2.0)),
        Anchor(3, "uwb_anchor3", (0.0,  10.0, 2.0)),
    ]

    def __init__(self) -> None:
        super().__init__("uwb_trilateration_node")

        # ----- parameters -----
        self.declare_parameter("input_topic", "/uwb/ranging/ros")
        self.declare_parameter("output_pose_topic", "uwb_pose")
        self.declare_parameter("output_covariance_topic", "uwb_pose_cov")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "uwb_tag")
        # Anchor layout
        self.declare_parameter("anchor_count", 4)
        self.declare_parameter("anchor_positions",
            [0.0, 0.0, 2.0,
             10.0, 0.0, 2.0,
             10.0, 10.0, 2.0,
             0.0, 10.0, 2.0])
        # Noise model
        self.declare_parameter("los_std", 0.10)         # [m]
        self.declare_parameter("nlos_soft_std", 0.30)
        self.declare_parameter("nlos_hard_std", 0.60)
        # Trust gating
        self.declare_parameter("max_range", 50.0)
        self.declare_parameter("min_anchors", 3)
        # Initial guess for optimiser
        self.declare_parameter("initial_guess", [5.0, 5.0, 1.0])
        # Constant altitude (z) - we are 2D-only for now
        self.declare_parameter("assume_constant_z", True)
        # Publish PoseStamped for convenience
        self.declare_parameter("publish_pose_stamped", True)

        # ----- members -----
        self._lock = threading.Lock()
        self._anchors: List[Anchor] = self._load_anchors()
        self._latest_pose: Optional[Pose] = None
        self._latest_cov: Optional[np.ndarray] = None
        self._last_solution_time = self.get_clock().now()

        # Buffer of the most-recent ranging per anchor, refreshed each
        # incoming Pose message. We solve on a fixed cadence so that we
        # always have a consistent set of 4 measurements.
        self._latest_per_anchor: dict[int, RangeMeasurement] = {}
        self._last_solve_at = self.get_clock().now()

        # ----- QoS -----
        # The C++ bridge publishes with best_effort + volatile durability so
        # any subscriber can receive.  Match that here.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self._sub = self.create_subscription(
            Pose,
            self.get_parameter("input_topic").get_parameter_value().string_value,
            self._on_ranging_pose,
            qos,
        )

        # Solve at 20 Hz to match the gz plugin's ranging rate.
        self._solve_timer = self.create_timer(
            1.0 / 20.0, self._try_solve
        )

        self._pub_cov = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("output_covariance_topic")
                .get_parameter_value().string_value,
            10,
        )
        self._pub_pose: Optional[Node] = None  # type: ignore
        if self.get_parameter("publish_pose_stamped").get_parameter_value().bool_value:
            self._pub_pose = self.create_publisher(
                PoseStamped,
                self.get_parameter("output_pose_topic")
                    .get_parameter_value().string_value,
                10,
            )

        self.get_logger().info(
            f"uwb_trilateration_node ready: anchors={len(self._anchors)} "
            f"input={self.get_parameter('input_topic').value} "
            f"output={self.get_parameter('output_covariance_topic').value}"
        )

    # ------------------------------------------------------------------ utils
    def _load_anchors(self) -> List[Anchor]:
        flat = self.get_parameter("anchor_positions").get_parameter_value()\
            .double_array_value
        n = int(self.get_parameter("anchor_count").value)
        if len(flat) < 3 * n:
            self.get_logger().warn(
                "anchor_positions shorter than anchor_count*3; "
                "falling back to default 4-corner 10x10m layout."
            )
            return list(self.DEFAULT_ANCHORS)
        anchors: List[Anchor] = []
        for i in range(n):
            anchors.append(
                Anchor(
                    id=i,
                    name=f"uwb_anchor{i}",
                    position=(flat[3 * i], flat[3 * i + 1], flat[3 * i + 2]),
                )
            )
        return anchors

    # ------------------------------------------------------------ callback
    def _on_ranging_pose(self, msg: Pose) -> None:
        """Decode a gz Pose-encoded ranging and buffer it."""
        anchor_id = int(round(msg.position.z))
        if anchor_id < 0 or anchor_id >= len(self._anchors):
            return
        anchor = self._anchors[anchor_id]
        rm = RangeMeasurement(
            anchor_id=anchor_id,
            anchor_name=anchor.name,
            range_m=float(msg.position.x),
            rss_dbm=float(msg.position.y),
            los=int(round(msg.orientation.x)),
        )
        with self._lock:
            self._latest_per_anchor[anchor_id] = rm

    def _try_solve(self) -> None:
        """Solve on a fixed cadence using the latest per-anchor samples."""
        with self._lock:
            if not self._latest_per_anchor:
                return
            ranges = list(self._latest_per_anchor.values())
        pose, cov = self._solve(ranges)
        if pose is None:
            return
        with self._lock:
            self._latest_pose = pose
            self._latest_cov = cov
            self._last_solution_time = self.get_clock().now()
        self._publish(pose, cov)

    # ----------------------------------------------------------- solver
    def _solve(
        self,
        ranges: List[RangeMeasurement],
    ) -> Tuple[Optional[Pose], Optional[np.ndarray]]:
        """Weighted non-linear least squares for (x, y, z)."""
        # Build anchor look-up so a ranging can address its anchor.
        anchor_by_id = {a.id: a for a in self._anchors}

        # Filter out ranges to unknown anchors.
        valid: List[RangeMeasurement] = []
        anchors_used: List[Anchor] = []
        weights: List[float] = []
        los_std = float(self.get_parameter("los_std").value)
        nlos_soft_std = float(self.get_parameter("nlos_soft_std").value)
        nlos_hard_std = float(self.get_parameter("nlos_hard_std").value)
        for r in ranges:
            a = anchor_by_id.get(r.anchor_id)
            if a is None:
                continue
            valid.append(r)
            anchors_used.append(a)
            weights.append(r.weight(los_std, nlos_soft_std, nlos_hard_std))

        if len(valid) < int(self.get_parameter("min_anchors").value):
            return None, None

        # Initial guess
        init = self.get_parameter("initial_guess").get_parameter_value()\
            .double_array_value
        if len(init) < 3:
            init = [5.0, 5.0, 1.0]

        # Decide whether to solve 2D (constant z) or 3D.
        assume_z = bool(self.get_parameter("assume_constant_z").value)
        x0 = np.array(init, dtype=float)

        # 1) Closed-form linearisation (3D) to seed the non-linear step.
        try:
            x0 = self._linear_seed(anchors_used, valid, x0)
        except np.linalg.LinAlgError:
            pass  # keep initial guess

        # 2) Gauss-Newton WLS.
        try:
            sol = self._gauss_newton(
                x0, anchors_used, valid, weights,
                assume_z=assume_z,
            )
        except np.linalg.LinAlgError:
            return None, None

        # 3) Build pose + covariance.
        px, py, pz = sol
        pose = Pose()
        pose.position.x = float(px)
        pose.position.y = float(py)
        pose.position.z = float(pz)
        # Heading unknown from ranges; encode as identity quaternion.
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        cov = self._estimate_covariance(
            sol, anchors_used, valid, weights, assume_z=assume_z
        )

        return pose, cov

    def _linear_seed(
        self,
        anchors: List[Anchor],
        ranges: List[RangeMeasurement],
        x0: np.ndarray,
    ) -> np.ndarray:
        """Linearise around x0 to get a better seed for Gauss-Newton.

        d^2 = (xi-x)^2 + (yi-y)^2 + (zi-z)^2
        => 2(xi-x)dx + 2(yi-y)dy + 2(zi-z)dz = di^2 - d0i^2
        """
        x, y, z = x0
        A_rows = []
        b_vec = []
        for a, r in zip(anchors, ranges):
            ax, ay, az = a.position
            d0 = math.sqrt((ax - x) ** 2 + (ay - y) ** 2 + (az - z) ** 2)
            A_rows.append([2.0 * (ax - x), 2.0 * (ay - y), 2.0 * (az - z)])
            b_vec.append(r.range_m ** 2 - d0 ** 2
                         + 2.0 * (ax * x + ay * y + az * z)
                         - (ax * ax + ay * ay + az * az))
        A = np.array(A_rows, dtype=float)
        b = np.array(b_vec, dtype=float)
        # Least-squares
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        return np.array([sol[0], sol[1], sol[2]], dtype=float)

    def _gauss_newton(
        self,
        x0: np.ndarray,
        anchors: List[Anchor],
        ranges: List[RangeMeasurement],
        weights: List[float],
        assume_z: bool,
        max_iter: int = 10,
        tol: float = 1e-4,
    ) -> np.ndarray:
        """Iterative WLS using Gauss-Newton."""
        x = x0.copy()
        n = len(ranges)
        W = np.diag(weights) if assume_z else np.diag(weights)
        for _ in range(max_iter):
            r = np.zeros(n)
            J = np.zeros((n, 3))
            for i, (a, m) in enumerate(zip(anchors, ranges)):
                ax, ay, az = a.position
                dx = x[0] - ax
                dy = x[1] - ay
                dz = x[2] - az
                d = math.sqrt(dx * dx + dy * dy + dz * dz)
                d = max(d, 1e-6)  # avoid /0
                r[i] = d - m.range_m
                J[i, 0] = dx / d
                J[i, 1] = dy / d
                J[i, 2] = dz / d

            if assume_z:
                # Drop z column and solve in 2D; keep z fixed at initial value.
                J2 = J[:, :2]
                try:
                    dx2 = np.linalg.solve(J2.T @ W @ J2, J2.T @ W @ r)
                except np.linalg.LinAlgError:
                    break
                x_new = x.copy()
                x_new[0] -= dx2[0]
                x_new[1] -= dx2[1]
            else:
                try:
                    dx3 = np.linalg.solve(J.T @ W @ J, J.T @ W @ r)
                except np.linalg.LinAlgError:
                    break
                x_new = x - dx3

            if np.linalg.norm(x_new - x) < tol:
                return x_new
            x = x_new
        return x

    def _estimate_covariance(
        self,
        sol: np.ndarray,
        anchors: List[Anchor],
        ranges: List[RangeMeasurement],
        weights: List[float],
        assume_z: bool,
    ) -> np.ndarray:
        """Approximate 6x6 pose covariance from the WLS Jacobian.

        (J^T W J)^-1 scaled by the residual variance gives the
        parameter covariance; for simplicity we put it in the
        position block of the 6x6 message covariance.
        """
        x, y, z = sol
        n = len(ranges)
        W = np.diag(weights)
        J = np.zeros((n, 3))
        for i, a in enumerate(anchors):
            ax, ay, az = a.position
            dx = x - ax
            dy = y - ay
            dz = z - az
            d = math.sqrt(dx * dx + dy * dy + dz * dz) or 1e-6
            J[i] = [dx / d, dy / d, dz / d]

        if assume_z:
            J = J[:, :2]
        try:
            sigma2_param = np.linalg.inv(J.T @ W @ J)
        except np.linalg.LinAlgError:
            sigma2_param = np.eye(J.shape[1]) * 0.5

        cov6 = np.zeros((6, 6))
        cov6[: J.shape[1], : J.shape[1]] = sigma2_param
        # Roll / pitch / yaw covariance is unknown from ranging.
        cov6[3, 3] = 1.0  # large orientation uncertainty (rad^2)
        cov6[4, 4] = 1.0
        cov6[5, 5] = 1.0
        return cov6

    # ---------------------------------------------------------- publish
    def _publish(self, pose: Pose, cov: np.ndarray) -> None:
        stamp = self.get_clock().now().to_msg()
        frame_id = self.get_parameter("frame_id").value
        child_frame_id = self.get_parameter("child_frame_id").value

        cov_msg = PoseWithCovarianceStamped()
        cov_msg.header.stamp = stamp
        cov_msg.header.frame_id = frame_id
        cov_msg.pose.pose = pose
        cov_msg.pose.covariance = cov.flatten().tolist()
        self._pub_cov.publish(cov_msg)

        if self._pub_pose is not None:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = child_frame_id
            pose_msg.pose = pose
            self._pub_pose.publish(pose_msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = TrilaterationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
