# UAV-Airvision

UAV-Airvision is a Python visual-inertial odometry prototype for EuRoC MAV stereo camera and IMU data. It loads EuRoC sequences, publishes IMU and stereo frames into worker queues, tracks stereo features, and feeds them into an MSCKF-based estimator.

Phase 1 focuses on making the project launch consistently. The core MSCKF/VIO math has not been changed in this runtime-baseline pass.

## Requirements

- Windows with PowerShell or Command Prompt
- Python 3.10 or newer
- A EuRoC MAV sequence extracted locally

Core Python dependencies are listed in `requirements.txt`:

```text
numpy
opencv-python
scipy
```

Viewer mode is optional and additionally requires `PyQt5` and `pyqtgraph`.

## Windows Setup

From the repository root:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Optional viewer dependencies:

```powershell
python -m pip install PyQt5 pyqtgraph
```

## Dataset Layout

Pass `--path` to one EuRoC sequence directory, not to the parent dataset folder.

Expected structure:

```text
datasets\MH_01_easy\mav0\imu0\data.csv
datasets\MH_01_easy\mav0\state_groundtruth_estimate0\data.csv
datasets\MH_01_easy\mav0\cam0\data\*.png
datasets\MH_01_easy\mav0\cam1\data\*.png
```

The runner validates these paths before processing and reports missing files clearly.

## Run

Supported command from the repository root:

```powershell
python main.py --path .\datasets\MH_01_easy --offset 10
```

Equivalent package command:

```powershell
python -m src.main --path .\datasets\MH_01_easy --offset 10
```

Run with the optional viewer:

```powershell
python main.py --path .\datasets\MH_01_easy --offset 10 --view
```

Batch sweeps can be launched with:

```powershell
.\run.bat
```

## Output

Trajectory text files are created automatically under:

```text
results\txts\
```

Default file name:

```text
output_<dataset>_offset<offset>.txt
```

Repeated runs replace the same output file by default and write a header:

```text
# timestamp p_x p_y p_z q_x q_y q_z q_w
```

Append intentionally with:

```powershell
python main.py --path .\datasets\MH_01_easy --offset 10 --append-output
```

Use a different output directory with:

```powershell
python main.py --path .\datasets\MH_01_easy --offset 10 --output-dir .\outputs\txts
```

## Smoke Check

Check imports:

```powershell
python tools\smoke_check.py
```

Check imports and dataset structure:

```powershell
python tools\smoke_check.py --dataset .\datasets\MH_01_easy
```

Optional viewer import check:

```powershell
python tools\smoke_check.py --check-viewer
```

## Documentation

- Phase 0 audit: `docs\AUDIT_REPORT.md`
- Runtime runbook: `docs\RUNBOOK.md`

## Current Limitations

The Phase 1 runtime baseline does not fix core estimator correctness. Known remaining risks include stereo matching geometry, image-processing outlier rejection, threading/data ownership, and MSCKF gating/covariance math. See `docs\AUDIT_REPORT.md` for the staged fix plan.
