# UAV-Airvision

**Visual-Inertial Odometry and Evaluation Dashboard for GPS-Denied UAV Navigation**

UAV-Airvision is a research-oriented Python project for studying **relative navigation of unmanned aerial vehicles in GPS-denied environments** using a stereo visual-inertial odometry pipeline based on an MSCKF-style estimator.

The project includes the core VIO pipeline, EuRoC dataset support, trajectory evaluation tools, a graphical evaluation dashboard, live camera preview, live 3D trajectory visualization, and detailed debugging reports.

---

## Project Goal

The main goal of this project is to investigate how a UAV can estimate its relative motion using only onboard sensors, especially:

* stereo camera frames,
* inertial measurements,
* feature tracking,
* stereo matching,
* MSCKF-based state estimation,
* trajectory evaluation against ground truth.

This repository is currently used as a learning, debugging, and research platform rather than a production-ready flight navigation stack.

---

## Current Status

The current active development branch is:

```text
phase9c-live-preview-sync
```

Recent work focused on synchronizing the dashboard live view:

```text
camera preview + VIO console output + live 3D trajectory
```

The live dashboard now uses timestamp-based synchronization instead of relying on a free-running preview timer or pose counters.

The current stable direction is:

```text
run from source
use the Python virtual environment
debug live synchronization
avoid EXE packaging until the VIO pipeline and dashboard are stable
```

---

## Main Features

### Visual-Inertial Odometry Core

The project contains a stereo VIO pipeline with modules for:

* IMU propagation,
* feature tracking,
* feature initialization,
* stereo observation handling,
* MSCKF measurement update,
* trajectory output.

### EuRoC Dataset Support

The tools are designed around the EuRoC MAV dataset format. The dashboard and evaluation scripts use dataset registry files to locate and run experiments on EuRoC sequences.

Example datasets:

```text
MH_01_easy
MH_02_easy
MH_03_medium
MH_04_difficult
MH_05_difficult
```

### Evaluation Tools

The repository includes trajectory evaluation utilities for:

* Absolute Trajectory Error,
* Relative Pose Error,
* SE(3) trajectory alignment,
* warm-up skip analysis,
* batch evaluation over multiple datasets.

### Graphical Dashboard

The dashboard provides:

* dataset selection,
* VIO run controls,
* live camera preview,
* live 3D trajectory visualization,
* ATE/RPE plots,
* global comparison tables,
* evaluation summary export,
* result inspection tools.

### Live Synchronization Debugging

A major recent focus has been making sure that the live camera frame and the live trajectory point refer to the same dataset time.

The current live synchronization strategy is:

```text
VIO emits:
VIO_POSE timestamp=... position=[x y z]

Dashboard uses the same timestamp to:
1. select the nearest cam0 image frame,
2. append the position to the live 3D trajectory.
```

This avoids the previous issue where the trajectory could move forward while the camera preview stayed on an early frame, or where the camera preview could run ahead independently of VIO processing.

---

## Repository Structure

```text
.
├── src/                         # Core VIO and MSCKF source code
│   ├── feature/                 # Feature models, initialization and geometry tools
│   ├── image_processing/        # Frontend image processing and tracking modules
│   ├── modules/                 # Main VIO/MSCKF modules
│   └── streaming/               # Dataset streaming and publishing utilities
│
├── tools/                       # Evaluation and dashboard tools
│   ├── dashboard/               # Dashboard UI components
│   ├── evaluation_dashboard.py  # Main graphical dashboard
│   ├── evaluate_trajectory.py   # Headless trajectory evaluation
│   └── evaluate_euroc_batch.py  # Batch evaluation utilities
│
├── configs/                     # Dataset registry and configuration files
├── docs/                        # Project reports, debug reports and development notes
├── results/                     # Generated evaluation outputs
├── datasets/                    # Local EuRoC datasets, usually not committed
├── main.py                      # Main VIO entry point
└── requirements.txt             # Python dependencies
```

---

## Setup

Create and activate a virtual environment:

```bash
python -m venv .venv
source .venv/Scripts/activate
```

Install dependencies:

```bash
python -m pip install -U pip
python -m pip install -r requirements.txt
python -m pip install PyQt5 pyqtgraph PyOpenGL matplotlib reportlab pillow
```

On Windows Git Bash, run commands from the project root:

```bash
cd "/j/Sattar Run"
source .venv/Scripts/activate
```

---

## Running the Dashboard

Run the dashboard from source:

```bash
python tools/evaluation_dashboard.py \
  --datasets-root ./datasets \
  --results-root ./results \
  --warmup-skip-seconds 20
```

For live synchronization debugging:

```bash
DASHBOARD_SYNC_DEBUG=1 python tools/evaluation_dashboard.py \
  --datasets-root ./datasets \
  --results-root ./results \
  --warmup-skip-seconds 20
```

Expected live sync debug output:

```text
[SYNC] target=... range=[..., ...] frame=.../... pose=...
```

The important behavior is that the camera frame should move forward together with the live trajectory. The frame index and pose count do not need to be exactly equal, but the camera preview should not remain frozen at frame 1 while the trajectory keeps moving.

---

## Running VIO Directly

The main VIO entry point is:

```bash
python main.py --path ./datasets/MH_01_easy --offset 10 --view
```

The dashboard internally launches VIO runs and reads stdout telemetry for live visualization.

---

## Evaluation

Run a trajectory evaluation:

```bash
python tools/evaluate_trajectory.py \
  --dataset MH_05_difficult \
  --results-root ./results \
  --warmup-skip-seconds 20
```

Run batch evaluation:

```bash
python tools/evaluate_euroc_batch.py \
  --datasets-root ./datasets \
  --results-root ./results \
  --warmup-skip-seconds 20
```

---

## Important Development Notes

This project is still under active debugging. The current known areas that require careful review are:

* feature tracking failure handling,
* stereo matching robustness,
* feature initialization quality,
* geometric conditioning of visual updates,
* MSCKF chi-square gating,
* covariance update stability,
* early trajectory drift,
* initialization behavior during the first seconds of motion.

The dashboard live synchronization issue is separate from the core VIO drift problem. Live synchronization is a UI/runtime telemetry problem, while trajectory drift is related to estimator, initialization, frontend quality, and visual-inertial update behavior.

---

## Documentation

Important project documents are stored in `docs/`.

Recommended reading order:

```text
docs/PROJECT_STATE_AND_CLEANUP_PLAN.md
docs/LIVE_SYNC_DEBUG_REPORT.md
docs/PHASE6_ROOT_CAUSE_ANALYSIS_REPORT.md
docs/PHASE9_BUG_AUDIT_AND_WARMUP_EVALUATION_REPORT.md
docs/EVALUATION_RUNBOOK.md
```

The documentation records the debugging process, evaluation results, known limitations, and cleanup decisions.

---

## Packaging Status

Windows EXE packaging was explored but is currently paused.

The reason is that packaging the dashboard alone is not enough. The `Run Selected` button also needs a packaged VIO worker executable. Until the dashboard and VIO pipeline are stable, the recommended execution mode is:

```text
source code + virtual environment
```

Packaging should be revisited later with a clean release plan.

---

## Recommended Workflow

Before making changes:

```bash
git status -sb
python -m py_compile tools/evaluation_dashboard.py src/msckf.py
```

After a successful change:

```bash
git add <changed-files>
git commit -m "clear commit message"
git push origin phase9c-live-preview-sync
```

Avoid running:

```bash
git clean -fdX
```

without checking the dry-run first, because local folders such as `.venv`, `datasets`, and `results` may be ignored and could be removed.

Use this first:

```bash
git clean -ndX
```

---

## Project Scope

This repository is intended for:

* UAV navigation research,
* VIO/MSCKF learning and debugging,
* EuRoC trajectory evaluation,
* dashboard-based experiment inspection,
* academic development and technical documentation.

It is not yet a production flight-control or safety-critical navigation system.

---

## Maintainer

Developed and maintained as part of an academic UAV relative navigation project.

Project focus:

```text
GPS-denied UAV navigation
Visual-Inertial Odometry
MSCKF
EuRoC evaluation
Live trajectory visualization
```
