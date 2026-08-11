"""Differential (Jacobian-based) inverse kinematics for a single MuJoCo site."""

from __future__ import annotations

import mujoco
import numpy as np


class DifferentialIK:
    """Damped-least-squares IK that drives one site toward a target pose.

    Iterates on an internal scratch `MjData` so it never disturbs the live
    simulation state; callers read back the resulting joint angles and write
    them wherever they like (e.g. into `data.ctrl` for position actuators).
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        site_name: str,
        qpos_indices: np.ndarray,
        dof_indices: np.ndarray,
        joint_range: np.ndarray,
        damping: float = 0.05,
        max_iters: int = 8,
        pos_tol: float = 1e-4,
        rot_tol: float = 1e-3,
    ) -> None:
        self.model = model
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if self.site_id < 0:
            raise ValueError(f"Site '{site_name}' not found in model")

        self.qpos_indices = np.asarray(qpos_indices)
        self.dof_indices = np.asarray(dof_indices)
        self.qlo = joint_range[:, 0]
        self.qhi = joint_range[:, 1]
        self.damping = damping
        self.max_iters = max_iters
        self.pos_tol = pos_tol
        self.rot_tol = rot_tol

        self._scratch = mujoco.MjData(model)
        self._jacp = np.zeros((3, model.nv))
        self._jacr = np.zeros((3, model.nv))

    def solve(
        self,
        current_qpos: np.ndarray,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
    ) -> np.ndarray:
        """Return joint angles (same order as `qpos_indices`) that move the
        site toward `target_pos`/`target_quat` (wxyz), starting from
        `current_qpos` (the model's full qpos vector).
        """
        scratch = self._scratch
        scratch.qpos[:] = current_qpos
        site_quat = np.empty(4)
        neg_site_quat = np.empty(4)
        err_quat = np.empty(4)
        rot_err = np.empty(3)

        for _ in range(self.max_iters):
            mujoco.mj_kinematics(self.model, scratch)
            mujoco.mj_comPos(self.model, scratch)

            site_pos = scratch.site_xpos[self.site_id]
            pos_err = target_pos - site_pos

            mujoco.mju_mat2Quat(site_quat, scratch.site_xmat[self.site_id])
            mujoco.mju_negQuat(neg_site_quat, site_quat)
            mujoco.mju_mulQuat(err_quat, target_quat, neg_site_quat)
            mujoco.mju_quat2Vel(rot_err, err_quat, 1.0)

            if np.linalg.norm(pos_err) < self.pos_tol and np.linalg.norm(rot_err) < self.rot_tol:
                break

            mujoco.mj_jacSite(self.model, scratch, self._jacp, self._jacr, self.site_id)
            jac = np.vstack(
                [self._jacp[:, self.dof_indices], self._jacr[:, self.dof_indices]]
            )  # 6 x len(dof_indices)
            err = np.concatenate([pos_err, rot_err])

            jjt = jac @ jac.T + (self.damping**2) * np.eye(6)
            dq = jac.T @ np.linalg.solve(jjt, err)

            q = scratch.qpos[self.qpos_indices] + dq
            scratch.qpos[self.qpos_indices] = np.clip(q, self.qlo, self.qhi)

        return scratch.qpos[self.qpos_indices].copy()
