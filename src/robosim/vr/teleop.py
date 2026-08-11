"""Launch the OpenArm bimanual MuJoCo simulator under VR teleoperation."""

from __future__ import annotations

import argparse
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np
import openarm_mujoco.v2 as openarm_mujoco
from openarm_mujoco.v2 import JointResolver

from robosim.vr.clutch import ClutchController
from robosim.vr.ik import DifferentialIK
from robosim.vr.quest_reader import MockQuestReader, QuestPoseSource, QuestReader

_SEGMENTS = ("left", "right")
_SITE_NAMES = {"left": "left_ee_control_point", "right": "right_ee_control_point"}


def _arm_joint_range(model: mujoco.MjModel, segment: str) -> np.ndarray:
    prefix = "openarm_left_" if segment == "left" else "openarm_right_"
    ranges = []
    for i in range(1, 8):
        name = f"{prefix}joint{i}"
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise ValueError(f"Joint '{name}' not found in model")
        ranges.append(model.jnt_range[jid])
    return np.array(ranges)


def _gripper_closed_value(model: mujoco.MjModel, segment: str) -> float:
    name = f"{segment}_finger1_ctrl"
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if aid < 0:
        raise ValueError(f"Actuator '{name}' not found in model")
    lo, hi = model.actuator_ctrlrange[aid]
    return lo if abs(lo) > abs(hi) else hi


def _site_pose(data: mujoco.MjData, site_id: int) -> tuple[np.ndarray, np.ndarray]:
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, data.site_xmat[site_id])
    return data.site_xpos[site_id].copy(), quat


class ArmTeleop:
    """Per-arm clutch + IK state, wired to one JointResolver segment."""

    def __init__(
        self, model: mujoco.MjModel, data: mujoco.MjData, segment: str, scale: float
    ) -> None:
        self.segment = segment
        self.site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, _SITE_NAMES[segment]
        )
        if self.site_id < 0:
            raise ValueError(f"Site '{_SITE_NAMES[segment]}' not found in model")

        mapper = JointResolver(model)
        self.ik = DifferentialIK(
            model,
            _SITE_NAMES[segment],
            mapper.arm_qpos_indices(segment),
            mapper.arm_dof_indices(segment),
            _arm_joint_range(model, segment),
        )
        self.gripper_closed_value = _gripper_closed_value(model, segment)
        self.clutch = ClutchController(scale=scale)
        ee_pos, ee_quat = _site_pose(data, self.site_id)
        self.clutch.initialize(ee_pos, ee_quat)

    def reinitialize(self, data: mujoco.MjData) -> None:
        """Re-anchor the clutch to the arm's current pose (e.g. after a GUI reset)."""
        ee_pos, ee_quat = _site_pose(data, self.site_id)
        self.clutch.engaged = False
        self.clutch.initialize(ee_pos, ee_quat)

    def step(self, mapper: JointResolver, data: mujoco.MjData, controller) -> None:
        if not controller.valid:
            return  # tracking lost: hold the last commanded pose
        target_pos, target_quat = self.clutch.update(
            controller.pos, controller.quat, controller.grip_pressed
        )
        joint_angles = self.ik.solve(data.qpos, target_pos, target_quat)
        gripper_ctrl = controller.trigger * self.gripper_closed_value
        driver = np.concatenate([joint_angles, [gripper_ctrl]])
        mapper.set_ctrl(data.ctrl, driver, self.segment)


def run(xml_path: str, reader: QuestPoseSource, scale: float) -> None:
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    mapper = JointResolver(model)
    arms = {segment: ArmTeleop(model, data, segment, scale) for segment in _SEGMENTS}

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = model.stat.center
        viewer.cam.distance = model.stat.extent
        viewer.cam.azimuth = model.vis.global_.azimuth
        viewer.cam.elevation = model.vis.global_.elevation

        last_time = data.time
        try:
            while viewer.is_running():
                step_start = time.time()

                controllers = reader.read()
                # The GUI (e.g. its Reset button) mutates the same `data` from
                # its own thread; without this lock that races with our ctrl
                # writes and mj_step below, which can look like erratic or
                # looping motion.
                with viewer.lock():
                    if data.time < last_time:
                        # data.time only rewinds if something outside this loop
                        # (e.g. the GUI's Reset button) reset the simulation;
                        # re-anchor the clutch so arms don't snap back toward a
                        # now-stale pre-reset target.
                        for segment in _SEGMENTS:
                            arms[segment].reinitialize(data)

                    for segment in _SEGMENTS:
                        arms[segment].step(mapper, data, controllers[segment])
                    mujoco.mj_step(model, data)
                    last_time = data.time

                elapsed = time.time() - step_start
                time.sleep(max(0, model.opt.timestep - elapsed))
                viewer.sync()
        finally:
            reader.close()


def main() -> int:
    """Launch the bimanual OpenArm sim under Quest VR teleoperation."""
    parser = argparse.ArgumentParser(
        description="Control the OpenArm bimanual simulation with a Meta Quest headset."
    )
    parser.add_argument(
        "xml",
        nargs="?",
        default=None,
        help="Path to MJCF (.xml) file (default: bundled bimanual OpenArm scene)",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Drive the arms with a synthetic controller trajectory instead of a real headset",
    )
    parser.add_argument(
        "--ip",
        default=None,
        help="Quest IP address for network ADB (default: USB connection)",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Clutch position sensitivity (controller-meters to arm-meters)",
    )
    args = parser.parse_args()

    xml_path = args.xml or openarm_mujoco.openarm_bimanual_xml()

    reader: QuestPoseSource
    if args.mock:
        reader = MockQuestReader()
    else:
        try:
            reader = QuestReader(ip_address=args.ip)
        except ImportError:
            print(
                "Error: the 'oculus_reader' package is required for live Quest "
                "teleop. Install it (see project README) or pass --mock to try "
                "the pipeline without a headset.",
                file=sys.stderr,
            )
            return 2

    run(xml_path, reader, args.scale)
    return 0


if __name__ == "__main__":
    sys.exit(main())
