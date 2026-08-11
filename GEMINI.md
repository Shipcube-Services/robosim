# Gemini CLI - Workspace Context Instructions

This file serves as a foundational guide and instruction set for AI assistants (like Gemini CLI) interacting with this repository. It provides a complete project overview, build/run commands, structural architecture, and development conventions.

---

## 1. Project Overview

This repository is a unified workspace for **OpenArm**—an open-source robot arm platform—consisting of simulation environments and physical hardware specifications.

### Key Components
1. **`robosim` (Root Workspace):**
   - The primary Python orchestrator and workspace for simulation evaluation.
   - Managed with `uv` (using Python `3.13` as specified by `.python-version`).
   - Stores simulation outputs and evaluation data (such as metrics and episode videos) under `outputs/eval/`.

2. **`openarm_mujoco` (Simulation Subproject):**
   - A packaged Python project containing MuJoCo Description Files (MJCF) for OpenArm v2 (active), Cell, v1, and v0.3 simulations.
   - Provides utilities for programmatically resolving asset paths (`openarm_bimanual_xml()`, `openarm_cell_xml()`, etc.) and resolving joint indices (`JointResolver`).
   - Includes a command-line utility launcher (`openarm-mujoco-launch`) to load models in the passive MuJoCo viewer.

3. **`openarm_hardware` (Hardware Subproject):**
   - Documentation, bill of materials (BOM), CAD assemblies (STEP/STL files for leaders and followers), and wiring diagrams.
   - Standard assets are hosted on Google Drive (links detailed in `openarm_hardware/README.md`).

---

## 2. Directory Architecture

```
/
├── pyproject.toml              # Root project configuration (robosim)
├── uv.lock                     # Lockfile for robosim project dependencies
├── .python-version             # Local Python target version (3.13)
├── src/
│   └── robosim/                # Root package source (robosim)
│       └── __init__.py         # Main script entry point (prints greeting)
├── outputs/
│   └── eval/                   # Simulation evaluation logs, metrics (eval_info.json), and video outputs
├── openarm_hardware/           # Hardware specifications, CAD links, and release scripts
│   ├── README.md               # Hardware documentation links & assembly guide pointers
│   └── dev/                    # Release and asset sync scripts
└── openarm_mujoco/             # MuJoCo MJCF description files & simulation loader package
    ├── pyproject.toml          # Package configuration (openarm_mujoco dependencies & scripts)
    ├── setup.py                # Setuptools packaging file (handles bundle version asset discovery)
    ├── .pre-commit-config.yaml # Linter hooks (Ruff, shfmt)
    ├── src/
    │   ├── openarm_mujoco/     # Python package source
    │   │   └── v2/             # Active API and assets
    │   │       ├── __init__.py # Path resolvers for simulation MJCF scenes
    │   │       ├── joint_resolver.py  # qpos/ctrl mapping pre-calculator
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

### MuJoCo Simulation Package (`openarm_mujoco`)
To develop, build, and test the `openarm_mujoco` simulator package:

- **Install locally in editable mode:**
  ```bash
  pip install -e openarm_mujoco
  ```
- **Launch default simulation scene (passive viewer):**
  ```bash
  openarm-mujoco-launch
  ```
- **Launch with options:**
  - Turn on/off collision walls:
    ```bash
    openarm-mujoco-launch --walls
    ```
  - Hide white sheet mesh:
    ```bash
    openarm-mujoco-launch --no-sheet
    ```
  - Load and freeze at a specific keyframe (e.g., `home`):
    ```bash
    openarm-mujoco-launch -k home --static
    ```
  - Load a custom XML file:
    ```bash
    openarm-mujoco-launch <path_to_scene.xml>
    ```

---

## 4. Development Conventions

### Code Quality & Formatting
We adhere to strict standards using Astral's `ruff` for both linting and formatting.

- **Pre-commit Checks:**
  Use pre-commit to check code before making commits. The project uses:
  - `ruff-check` with `--fix`
  - `ruff-format`
  - `shfmt` for shell files

- **Linter Requirements:**
  Ruff is configured in `openarm_mujoco/pyproject.toml` with:
  - `extend-select = ["D", "UP"]` (enforcing docstrings and modern Python syntax updates).
  - Docstring requirements are ignored outside of `src/` files via `"!src/**.py" = ["D"]`.

### Testing and CI/CD
GitHub Actions workflows are defined in `openarm_mujoco/.github/workflows/`:
- **Linter Workflow (`lint.yaml`):** Executes pre-commit checks on all files.
- **Test Workflow (`test.yaml`):** Validates the package can be successfully installed and checks that `openarm_cell_xml()` resolves to a valid, existing XML file.
- **Package and Release (`package.yaml`):** Builds Python distributions (`.tar.gz`) on tag events and publishes them to PyPI and GitHub Releases.

When extending or modifying the simulation codebase:
1. Always write clean docstrings for public functions and classes under `src/`.
2. Format the code with `ruff format`.
3. Verify changes by ensuring that `openarm-mujoco-launch` is capable of loading the scenes without throwing parser errors.
