"""Clutch-based relative pose mapping for VR teleoperation.

Holding the clutch engages tracking: the target pose moves by the
controller's delta motion from the moment the clutch was engaged. Releasing
the clutch freezes the target in place, so the operator can reposition their
hand without moving the arm (like lifting a mouse).
"""

from __future__ import annotations

import numpy as np


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Return the conjugate (inverse, for unit quaternions) of wxyz `q`."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product `a * b` for wxyz quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


class ClutchController:
    """Tracks one arm's clutch state and produces a target end-effector pose."""

    def __init__(self, scale: float = 1.0) -> None:
        self.scale = scale
        self.engaged = False
        self._origin_controller_pos: np.ndarray | None = None
        self._origin_controller_quat: np.ndarray | None = None
        self._origin_target_pos: np.ndarray | None = None
        self._origin_target_quat: np.ndarray | None = None
        self.target_pos: np.ndarray | None = None
        self.target_quat: np.ndarray | None = None

    def initialize(self, ee_pos: np.ndarray, ee_quat: np.ndarray) -> None:
        """Seed the target pose (e.g. from the arm's current pose at startup)."""
        self.target_pos = np.array(ee_pos, dtype=float)
        self.target_quat = np.array(ee_quat, dtype=float)

    def update(
        self,
        controller_pos: np.ndarray,
        controller_quat: np.ndarray,
        clutch_pressed: bool,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Advance the clutch state machine by one tick.

        Returns the (possibly updated) `(target_pos, target_quat)`.
        """
        if self.target_pos is None or self.target_quat is None:
            raise RuntimeError("ClutchController.initialize() must be called first")

        if clutch_pressed and not self.engaged:
            self.engaged = True
            self._origin_controller_pos = np.array(controller_pos, dtype=float)
            self._origin_controller_quat = np.array(controller_quat, dtype=float)
            self._origin_target_pos = self.target_pos.copy()
            self._origin_target_quat = self.target_quat.copy()
        elif not clutch_pressed and self.engaged:
            self.engaged = False

        if self.engaged:
            delta_pos = self.scale * (controller_pos - self._origin_controller_pos)
            self.target_pos = self._origin_target_pos + delta_pos

            delta_quat = quat_mul(
                controller_quat, quat_conjugate(self._origin_controller_quat)
            )
            self.target_quat = quat_mul(delta_quat, self._origin_target_quat)
            self.target_quat /= np.linalg.norm(self.target_quat)

        return self.target_pos, self.target_quat
