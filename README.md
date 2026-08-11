# robosim

MuJoCo simulation tooling for the OpenArm bimanual robot.

## Setup

This repo depends on two vendored upstream projects that are cloned
separately (not part of this git history — see `.gitignore`):

```bash
git clone https://github.com/enactic/openarm_hardware.git
git clone https://github.com/enactic/openarm_mujoco.git
```

Then install dependencies:

```bash
uv sync
```

## VR teleoperation (Meta Quest)

Control the simulated bimanual arms directly from a Meta Quest headset and controllers.

### How it works

A small Android app (sideloaded via ADB, from the [`oculus_reader`](https://github.com/rail-berkeley/oculus_reader)
project) reads controller poses and button state from the Quest's native
controller-tracking APIs and logs them; a Python process on your machine
tails that log over ADB, remaps the poses into the simulator's world frame,
and uses them to drive the arms:

```
Quest controller → on-device app → adb logcat → clutch mapping → differential IK → data.ctrl → mj_step
```

Grip button = clutch (hold to move the arm, like lifting a mouse to
reposition it — release to freeze the arm and reposition your hand freely).
Index trigger = gripper open/close.

> **Quest 3 / 3S caveat:** the `oculus_reader` project's controller-reading
> app was built for Quest 2 and hasn't been officially verified on Quest 3
> hardware (its maintainers point to an unmaintained beta fork for Quest 3
> support). In practice, Quest 3/3S usually run older controller-tracking
> API calls fine, but this hasn't been tested against a real headset here —
> if the app doesn't pick up controller poses on your Quest 3S, try the fork
> at https://github.com/jborbik/oculus_reader.

### One-time hardware setup

1. Install `adb` (e.g. `brew install android-platform-tools` on macOS).
2. On the Quest: Meta Horizon app on your phone → device settings → **Developer Mode** → enable.
3. Connect the headset to your computer via USB-C, put it on, and accept the
   **Allow USB Debugging** prompt.
4. Verify it's visible: `adb devices` should list your headset as `device`
   (not `unauthorized`).

The first run of the teleop script auto-installs the reader APK to the
headset — no manual `adb install` needed.

> **Git LFS note:** the `oculus_reader` dependency ships its APK via Git
> LFS. If `git-lfs` isn't installed/initialized on your machine when `uv
> sync` fetches it, you'll get a tiny placeholder file instead of the real
> APK, and the auto-install will fail. Install with `git lfs install` (see
> https://git-lfs.github.com) before syncing if that happens.

### Running it

```bash
uv run mjpython -m robosim.vr.teleop        # real headset, USB
uv run mjpython -m robosim.vr.teleop --ip 10.0.0.42   # real headset, over Wi-Fi (see oculus_reader README for finding the IP)
uv run mjpython -m robosim.vr.teleop --mock # no headset needed — synthetic test motion, for trying the pipeline out
```

(`mjpython` is required instead of plain `python`/`uv run robosim-vr-teleop`
on macOS — MuJoCo's interactive viewer must run on the main thread there.)

`--scale` adjusts clutch sensitivity (controller-meters moved per arm-meter
moved; default `1.0`).

### Tuning

The controller-to-world axis remap in `src/robosim/vr/quest_reader.py`
(`_AXIS_REMAP`) is a best-effort default based on the documented OVR/OpenXR
axis convention — it hasn't been validated against a physical headset. If
arm motion feels mirrored or rotated relative to your hand motion once you
test on hardware, that's the one constant to adjust; the clutch and IK logic
underneath are axis-agnostic.
