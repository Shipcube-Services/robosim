# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

`robosim` (this repo, `src/robosim/`) is a small Python project that adds
Meta Quest VR teleoperation on top of the **OpenArm** bimanual MuJoCo
simulation. It depends on two upstream projects that are cloned as sibling
directories but are **not part of this repo's git history** (gitignored —
see the README's Setup section for clone URLs):

- `openarm_mujoco/` — MJCF models + a `JointResolver` joint/actuator-index
  utility + a bare simulation launcher (`openarm-mujoco-launch`).
- `openarm_hardware/` — BOM/CAD documentation only; not imported by any code.

## Commands

- Install deps: `uv sync`
- Run the base OpenArm viewer (no teleop): `uv run mjpython -m openarm_mujoco.v2.launch [xml] [--keyframe NAME] [--static] [--walls] [--no-sheet]`
- Run VR teleop: `uv run mjpython -m robosim.vr.teleop [xml] [--mock] [--ip ADDR] [--scale N]`
- **Always use `mjpython`, never plain `python`** (nor `uv run <console-script>` directly) for anything that calls `mujoco.viewer.launch_passive` — on macOS this raises `RuntimeError` otherwise, since the interactive viewer must run on `mjpython`'s main thread.
- There is no test suite or lint config for `robosim`'s own code yet. Verification during development has been ad hoc headless scripts (`uv run python3 -c "..."`) that exercise the pure-Python logic (IK convergence, clutch state machine, `ArmTeleop` directly) against the model without opening a viewer — follow that pattern for new logic rather than assuming a `pytest` setup exists.

## Architecture

### VR teleop pipeline (`src/robosim/vr/`)

Data flow: Quest controller pose/buttons → `quest_reader.py` → `clutch.py` → `ik.py` → `data.ctrl` → `mj_step`, orchestrated by `teleop.py`.

- **`quest_reader.py`** — `ControllerState` dataclass plus two interchangeable pose sources sharing a `read()`/`close()` interface: `QuestReader` (real headset; wraps `oculus_reader.OculusReader`, which tails `adb logcat` for pose/button data logged by a sideloaded Android app) and `MockQuestReader` (synthetic circular motion, no hardware — used by `--mock` and should be used for any headless dev/testing). Poses arrive in OVR/OpenXR axis convention and are remapped to a Z-up world frame via `_AXIS_REMAP`; that constant is a best-effort default, unverified against real ergonomics (see the caveat in `docs/quest-teleop-setup.md`) — it's the one place to adjust if arm motion direction feels wrong, not the clutch/IK math.
- **`clutch.py`** — `ClutchController`, one instance per arm: grip-to-engage relative pose tracking (hold grip = arm follows the hand's motion *relative to where you squeezed*; release = arm freezes in place; re-engaging never snaps to a new position). Pure numpy, no `mujoco` import, independently testable.
- **`ik.py`** — `DifferentialIK`: damped-least-squares Jacobian IK against a named site, iterating on a persistent scratch `MjData` so it never disturbs the live sim. It's tuned for the small incremental deltas the clutch produces each tick — don't assume its default iteration count converges from a cold/zero-qpos start to an arbitrary distant target; that's a materially harder problem it wasn't built for.
- **`teleop.py`** — `ArmTeleop` (per-arm: owns one `DifferentialIK` + one `ClutchController`; joint/actuator indices come from `openarm_mujoco.v2.JointResolver` — reuse that resolver for new per-arm indexing rather than re-deriving joint names) and `run()` (the main loop). Two non-obvious correctness requirements here:
  - All `data` mutation (ctrl writes + `mj_step`) must happen inside `with viewer.lock():`. The GUI thread mutates the same `mjData` (e.g. its Reset button calls `mj_resetData` directly), and without this lock that races with the control loop — this previously manifested as erratic/looping arm motion after clicking Reset.
  - The loop detects external resets by watching for `data.time` going backwards and calls `ArmTeleop.reinitialize()` on each arm when it happens, re-anchoring the clutch to the arm's post-reset pose. Without this, a GUI Reset leaves the clutch still targeting a now-stale pre-reset pose and the arm visibly snaps back toward it.

### Dependency wiring gotchas (`pyproject.toml`)

- `openarm_mujoco` is a `[tool.uv.sources]` local editable path dependency, not a PyPI package — it must exist as a sibling checkout at `./openarm_mujoco`.
- `oculus_reader` is a `[tool.uv.sources]` git dependency. It ships its Android APK via Git LFS, but **`uv`'s git-dependency fetcher does not run the LFS smudge filter**, regardless of whether `git-lfs` is installed locally — the APK it installs is a ~130-byte LFS pointer file, not the real ~7.5MB binary. `quest_reader.py`'s `_ensure_real_apk()` works around this: it detects the pointer (checks for the zip magic bytes) and fetches the real bytes from GitHub's LFS media endpoint directly over HTTPS the first time a real (non-mock) `QuestReader` is constructed. If `oculus_reader` is ever swapped for a different fork, re-check whether this workaround is still needed.
- `lerobot[pusht,evaluation]==0.6.1` is pinned here deliberately and is unrelated to the VR teleop feature — it must stay declared. This venv previously had `lerobot`/`torch`/`gymnasium`/etc. installed ad hoc, outside `pyproject.toml`; running `uv sync` after adding unrelated dependencies silently uninstalled all of it, because `uv sync` reconciles the venv strictly to what's declared. Before adding or removing dependencies, assume anything present in the venv but undeclared in `pyproject.toml` will be removed on the next sync.

### Bimanual model conventions (defined in vendored `openarm_mujoco`, referenced by name from this repo's code)

- Sites `left_ee_control_point` / `right_ee_control_point` are the IK targets.
- Actuators are `position` type (`{side}_joint{1-7}_ctrl`, `{side}_finger1_ctrl`) — `ctrl` is a target joint angle tracked by a per-actuator PD controller (`kp`/`kv`), not a torque.
- Gripper closed-direction sign is mirrored between arms: left closes toward its actuator's positive `ctrlrange` bound, right toward its negative bound. Don't hardcode a sign — derive it from `model.actuator_ctrlrange` (see `_gripper_closed_value` in `teleop.py`).
- The base launcher's `--static` mode (`openarm_mujoco.v2.launch`) calls `mj_forward` instead of `mj_step`, which does not integrate `qpos` — so without also passing `--keyframe`, control changes (e.g. GUI sliders) have no visible effect in that mode. This is upstream behavior, not something to "fix" from this repo.
