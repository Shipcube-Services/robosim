"""Meta Quest controller pose/button reading for VR teleoperation.

`QuestReader` wraps the `oculus_reader` ADB/logcat bridge (see
https://github.com/rail-berkeley/oculus_reader): a small sideloaded Android
app reads controller poses via the on-device OVR/OpenXR APIs and logs them,
and `oculus_reader.OculusReader` tails that log over ADB. That import is done
lazily so the rest of this package (and `MockQuestReader`) works without ADB,
the sideloaded app, or a headset attached.

Coordinate convention: the on-device app reports poses in the OVR/OpenXR
convention (X-right, Y-up, Z-toward-the-user). This module remaps that into a
right-handed, Z-up frame (X-right, Y-forward, Z-up) to match a typical MuJoCo
world. This mapping is a best-effort default -- it hasn't been validated
against a physical Quest 3S -- so if arm motion feels mirrored or rotated
relative to hand motion once tested on hardware, adjust `_AXIS_REMAP` below
rather than any other module (the clutch/IK logic is axis-agnostic).
"""

from __future__ import annotations

import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import mujoco
import numpy as np

# `oculus_reader` ships its Android app via Git LFS. `uv`'s git-dependency
# fetcher does not run the LFS smudge filter, so the file installed at
# <site-packages>/oculus_reader/APK/teleop-debug.apk is just an LFS pointer
# (~130 bytes of text), not the real ~7.5MB APK -- regardless of whether
# git-lfs is installed locally. GitHub still serves the real bytes over
# plain HTTPS for public repos, so fetch them directly instead.
_APK_LFS_MEDIA_URL = (
    "https://media.githubusercontent.com/media/rail-berkeley/oculus_reader/"
    "main/oculus_reader/APK/teleop-debug.apk"
)
_APK_ZIP_MAGIC = b"PK\x03\x04"


def _ensure_real_apk() -> None:
    import oculus_reader  # noqa: PLC0415

    if oculus_reader.__file__ is None:
        return
    apk_path = Path(oculus_reader.__file__).parent / "APK" / "teleop-debug.apk"
    if not apk_path.exists():
        return  # let oculus_reader's own error handling take over

    with apk_path.open("rb") as f:
        header = f.read(len(_APK_ZIP_MAGIC))
    if header == _APK_ZIP_MAGIC:
        return  # already the real APK

    urllib.request.urlretrieve(_APK_LFS_MEDIA_URL, apk_path)

_AXIS_REMAP = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, -1.0, 0.0],
    ]
)


@dataclass
class ControllerState:
    """One controller's pose and button state for a single tick."""

    pos: np.ndarray  # (3,) meters, world-axis-remapped
    quat: np.ndarray  # (4,) wxyz, world-axis-remapped
    grip: float  # analog grip trigger, 0..1
    trigger: float  # analog index trigger, 0..1
    grip_pressed: bool  # digital grip trigger (clutch)
    trigger_pressed: bool  # digital index trigger
    valid: bool = True  # False if this controller isn't currently tracked

    @staticmethod
    def invalid() -> "ControllerState":
        return ControllerState(
            pos=np.zeros(3),
            quat=np.array([1.0, 0.0, 0.0, 0.0]),
            grip=0.0,
            trigger=0.0,
            grip_pressed=False,
            trigger_pressed=False,
            valid=False,
        )


def _scalar(value, default: float) -> float:
    """Unwrap a `buttons_parser` analog value.

    `oculus_reader`'s button parser reports single-value analog controls
    (e.g. `leftGrip 0.53`) as a 1-element tuple `(0.53,)`, not a bare float.
    """
    if value is None:
        return default
    if isinstance(value, tuple):
        return float(value[0])
    return float(value)


class QuestPoseSource(Protocol):
    """Interface shared by `QuestReader` and `MockQuestReader`."""

    def read(self) -> dict[str, ControllerState]: ...

    def close(self) -> None: ...


def _remap_pose(matrix_4x4: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    pos_raw = matrix_4x4[:3, 3]
    rot_raw = matrix_4x4[:3, :3]
    pos = _AXIS_REMAP @ pos_raw
    rot = _AXIS_REMAP @ rot_raw @ _AXIS_REMAP.T
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, rot.reshape(-1))
    return pos, quat


class QuestReader:
    """Reads live controller state from a Meta Quest over ADB.

    Requires: `adb` on PATH, the headset in Developer Mode and connected over
    USB (or the same network, via `ip_address`), and the `oculus_reader`
    package installed (it sideloads its own APK on first use).
    """

    def __init__(self, ip_address: str | None = None) -> None:
        from oculus_reader.reader import OculusReader  # noqa: PLC0415

        _ensure_real_apk()
        self._reader = OculusReader(ip_address=ip_address)

    def read(self) -> dict[str, ControllerState]:
        transforms, buttons = self._reader.get_transformations_and_buttons()
        return {
            "left": self._parse_side(transforms, buttons, matrix_key="l", prefix="L"),
            "right": self._parse_side(transforms, buttons, matrix_key="r", prefix="R"),
        }

    @staticmethod
    def _parse_side(
        transforms: dict, buttons: dict, matrix_key: str, prefix: str
    ) -> ControllerState:
        if not transforms or matrix_key not in transforms:
            return ControllerState.invalid()

        pos, quat = _remap_pose(transforms[matrix_key])
        side = "left" if prefix == "L" else "right"
        return ControllerState(
            pos=pos,
            quat=quat,
            grip=_scalar(buttons.get(f"{side}Grip"), 0.0),
            trigger=_scalar(buttons.get(f"{side}Trig"), 0.0),
            grip_pressed=bool(buttons.get(f"{prefix}G", False)),
            trigger_pressed=bool(buttons.get(f"{prefix}Tr", False)),
            valid=True,
        )

    def close(self) -> None:
        self._reader.stop()


class MockQuestReader:
    """Synthetic pose source for developing/testing without a headset.

    Drives both controllers through a small circular motion in front of the
    robot with the clutch permanently engaged, purely so the rest of the
    teleop pipeline (clutch mapping, IK, physics) can be exercised
    end-to-end without hardware.
    """

    def __init__(self, dt: float = 1.0 / 60.0, radius: float = 0.05) -> None:
        self._t = 0.0
        self._dt = dt
        self._radius = radius

    def read(self) -> dict[str, ControllerState]:
        self._t += self._dt
        t = self._t
        r = self._radius
        left = ControllerState(
            pos=np.array([0.15 + r * np.cos(t), 0.05 + r * np.sin(t), 0.05]),
            quat=np.array([1.0, 0.0, 0.0, 0.0]),
            grip=1.0,
            trigger=0.5 * (1 + np.sin(t)),
            grip_pressed=True,
            trigger_pressed=False,
            valid=True,
        )
        right = ControllerState(
            pos=np.array([0.15 - r * np.cos(t), -0.05 - r * np.sin(t), 0.05]),
            quat=np.array([1.0, 0.0, 0.0, 0.0]),
            grip=1.0,
            trigger=0.5 * (1 + np.cos(t)),
            grip_pressed=True,
            trigger_pressed=False,
            valid=True,
        )
        return {"left": left, "right": right}

    def close(self) -> None:
        pass
