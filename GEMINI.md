# Gemini CLI - Workspace Context Instructions

This file serves as a foundational guide and instruction set for AI assistants (like Gemini CLI) interacting with this repository. It provides a complete project overview, build/run commands, structural architecture, and development conventions.

---

## 1. Project Overview

This repository is a unified workspace for **OpenArm**—an open-source robot arm platform—consisting of simulation environments, physical hardware specifications, and a Meta Quest VR teleoperation feature built on top of the simulation.

### Key Components
1. **`robosim` (Root Workspace):**
   - The primary Python project, managed with `uv` (using Python `3.13` as specified by `.python-version`).
   - Implements Meta Quest VR teleoperation for the OpenArm bimanual arm simulation under `src/robosim/vr/` (see section 3, and `docs/quest-teleop-setup.md` for headset connection/run instructions).
   - Also declares `lerobot[pusht,evaluation]` as a dependency — unrelated to VR teleop, but pinned deliberately because `uv sync` otherwise silently removes anything installed in the venv but undeclared in `pyproject.toml` (see `CLAUDE.md` for the full story).
   - Stores simulation outputs and evaluation data (such as metrics and episode videos) under `outputs/eval/` (gitignored, not tracked).

2. **`openarm_mujoco` (Simulation Subproject — vendored, not part of this repo's git history):**
   - Cloned separately from `https://github.com/enactic/openarm_mujoco.git` (see the README's Setup section) and referenced by `robosim` as a local editable path dependency.
   - A packaged Python project containing MuJoCo Description Files (MJCF) for OpenArm v2 (active), Cell, v1, and v0.3 simulations.
   - Provides utilities for programmatically resolving asset paths (`openarm_bimanual_xml()`, `openarm_cell_xml()`, etc.) and resolving joint indices (`JointResolver`) — `robosim`'s VR teleop code reuses `JointResolver` rather than re-deriving joint/actuator names itself.
   - Includes a command-line utility launcher (`openarm-mujoco-launch`) to load models in the passive MuJoCo viewer.

3. **`openarm_hardware` (Hardware Subproject — vendored, not part of this repo's git history):**
   - Cloned separately from `https://github.com/enactic/openarm_hardware.git` (see the README's Setup section).
   - Documentation, bill of materials (BOM), CAD assemblies (STEP/STL files for leaders and followers), and wiring diagrams.
   - Standard assets are hosted on Google Drive (links detailed in `openarm_hardware/README.md`).

---

## 2. Directory Architecture

```
/
├── pyproject.toml              # Root project configuration (robosim): mujoco, numpy, openarm_mujoco
│                                #   (local path dep), oculus_reader (git dep), lerobot
├── uv.lock                     # Lockfile for robosim project dependencies
├── .python-version             # Local Python target version (3.13)
├── CLAUDE.md                   # Claude Code guidance: architecture + operational gotchas
├── README.md                   # Setup instructions + VR teleop overview
├── docs/
│   └── quest-teleop-setup.md   # Step-by-step Quest connection/run guide
├── src/
│   └── robosim/
│       ├── __init__.py         # `robosim` console script entry point
│       └── vr/                 # Meta Quest VR teleoperation
│           ├── quest_reader.py #   Controller pose/button reading (real headset + mock)
│           ├── clutch.py       #   Grip-to-engage relative pose mapping
│           ├── ik.py           #   Damped-least-squares differential IK
│           └── teleop.py       #   Main control loop + `robosim-vr-teleop` entry point
├── outputs/                    # Simulation evaluation logs, metrics, and video outputs (gitignored)
├── openarm_hardware/           # Vendored, gitignored — clone separately (see README)
└── openarm_mujoco/             # Vendored, gitignored — clone separately (see README)
    ├── pyproject.toml          # Package configuration (openarm_mujoco dependencies & scripts)
    ├── setup.py                # Setuptools packaging file (handles bundled asset discovery)
    ├── .pre-commit-config.yaml # Linter hooks (Ruff, shfmt) — apply to this vendored package only
    ├── src/
    │   ├── openarm_mujoco/     # Python package source
    │   │   └── v2/             # Active API and assets
    │   │       ├── __init__.py # Path resolvers for simulation MJCF scenes
    │   │       ├── joint_resolver.py  # qpos/ctrl mapping pre-calculator (reused by robosim's VR teleop)
    │   │       └── launch.py   # Command line viewer simulator launch script
    │   └── openarm_mujoco_v2/  # Deprecated backward compatibility import wrapper
    └── v2/                     # Assets for MuJoCo simulation v2 (meshes, XML cell/pedestal scenes)
```

---

## 3. Building and Running Commands

### Root Workspace (`robosim`)
The root workspace uses `uv` for python environments and package execution:

- **Sync dependencies:**
  ```bash
  uv sync
  ```
- **Execute root package main script:**
  ```bash
  uv run robosim
  ```
- **Run VR teleoperation** (controls the bimanual arms from a Meta Quest headset):
  ```bash
  uv run mjpython -m robosim.vr.teleop            # real headset, USB
  uv run mjpython -m robosim.vr.teleop --ip <addr> # real headset, over Wi-Fi
  uv run mjpython -m robosim.vr.teleop --mock      # synthetic test motion, no headset needed
  ```
  See `docs/quest-teleop-setup.md` for headset connection/pairing steps.

**Important:** on macOS, anything that opens the interactive MuJoCo viewer (`mujoco.viewer.launch_passive`) must be run with `mjpython`, not plain `python` (nor a bare `uv run <console-script>`) — otherwise it raises `RuntimeError: launch_passive requires that the Python script be run under mjpython on macOS`.

### MuJoCo Simulation Package (`openarm_mujoco`, vendored)
Clone it first (see the README's Setup section); `uv sync` wires it in automatically via `[tool.uv.sources]`.

- **Launch default simulation scene (passive viewer):**
  ```bash
  uv run mjpython -m openarm_mujoco.v2.launch
  ```
- **Launch with options:**
  - Turn on/off collision walls:
    ```bash
    uv run mjpython -m openarm_mujoco.v2.launch --walls
    ```
  - Hide white sheet mesh:
    ```bash
    uv run mjpython -m openarm_mujoco.v2.launch --no-sheet
    ```
  - Load and freeze at a specific keyframe (e.g., `home`):
    ```bash
    uv run mjpython -m openarm_mujoco.v2.launch -k home --static
    ```
    Note: `--static` calls `mj_forward` instead of `mj_step`, which does not integrate `qpos` — without also loading a keyframe, GUI control changes (e.g. joint sliders) have no visible effect in this mode.
  - Load a custom XML file:
    ```bash
    uv run mjpython -m openarm_mujoco.v2.launch <path_to_scene.xml>
    ```

---

## 4. Development Conventions

### Code Quality & Formatting
The vendored `openarm_mujoco` package adheres to strict standards using Astral's `ruff` for both linting and formatting. This applies to that vendored package only — `robosim`'s own code under `src/robosim/` currently has no lint config or test suite.

- **Pre-commit Checks (within `openarm_mujoco/`):**
  - `ruff-check` with `--fix`
  - `ruff-format`
  - `shfmt` for shell files

- **Linter Requirements:**
  Ruff is configured in `openarm_mujoco/pyproject.toml` with:
  - `extend-select = ["D", "UP"]` (enforcing docstrings and modern Python syntax updates).
  - Docstring requirements are ignored outside of `src/` files via `"!src/**.py" = ["D"]`.

### Testing and CI/CD
GitHub Actions workflows for the vendored `openarm_mujoco` package are defined in `openarm_mujoco/.github/workflows/` (this is the upstream project's own CI, not part of this repo):
- **Linter Workflow (`lint.yaml`):** Executes pre-commit checks on all files.
- **Test Workflow (`test.yaml`):** Validates the package can be successfully installed and checks that `openarm_cell_xml()` resolves to a valid, existing XML file.
- **Package and Release (`package.yaml`):** Builds Python distributions (`.tar.gz`) on tag events and publishes them to PyPI and GitHub Releases.

`robosim`'s own code (`src/robosim/`) has no automated test suite yet. Verification during VR teleop development has been ad hoc headless scripts (`uv run python3 -c "..."`) that exercise the pure-Python logic (IK convergence, the clutch state machine) directly against the model, without opening a viewer.

When extending or modifying the simulation codebase:
1. Always write clean docstrings for public functions and classes under `openarm_mujoco/src/`.
2. Format vendored-package code with `ruff format`.
3. Verify changes by ensuring the relevant launcher (`openarm-mujoco-launch` or `robosim.vr.teleop`) is capable of loading the scenes without throwing parser errors.
