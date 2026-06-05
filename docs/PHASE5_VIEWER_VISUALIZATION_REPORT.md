# Phase 5A Viewer And Visualization Report

## Goal

Phase 5A makes visualization deliverables usable without changing MSCKF/VIO
math. It improves the optional live viewer path and adds offline trajectory
plotting.

## What Was Broken

The existing live viewer had several reliability issues:

- `src/viewer.py` imported PyQt5 and pyqtgraph directly, so viewer dependencies
  had to remain optional through `src/main.py`.
- EuRoC images are grayscale, but the viewer unconditionally called
  `cv2.cvtColor(..., cv2.COLOR_BGR2RGB)`, which is only valid for BGR images.
- Video recording opened `output.mp4` by default, creating generated output and
  adding avoidable runtime overhead.
- Viewer status text contained mojibake/non-ASCII text.
- GUI updates needed to stay in the Qt main thread, with worker threads only
  pushing data through queues.

## What Was Fixed

Changed:

```text
src/viewer.py
```

Fixes:

- grayscale, single-channel, BGR, and BGRA images are converted safely for
  display;
- video recording is disabled by default and only starts if `video_path` is
  explicitly provided;
- worker threads still call queue-based `update_image()`, `update_pose()`, and
  `update_points()` methods rather than touching Qt widgets directly;
- old image frames are dropped when the image queue is full, keeping the GUI
  responsive;
- trajectory history is capped;
- status text is ASCII and clear.

No MSCKF, VIO math, image-processing algorithm, or dataset streaming behavior
was changed.

## How To Run Headless

Viewer dependencies are not needed:

```powershell
python main.py --path .\datasets\MH_01_easy --offset 10
```

Headless smoke check:

```powershell
python tools\smoke_check.py --dataset .\datasets\MH_01_easy
```

## How To Run Live Viewer

Install optional GUI dependencies:

```powershell
python -m pip install PyQt5 pyqtgraph
```

Run:

```powershell
python main.py --path .\datasets\MH_01_easy --offset 10 --view
```

If dependencies are missing, `main.py --view` reports:

```text
Viewer mode requires PyQt5 and pyqtgraph. Install them with `python -m pip install PyQt5 pyqtgraph`, or run without --view.
```

The current local environment does not have PyQt5 installed, so live GUI launch
was not fully exercised in this phase. Headless mode remains verified.

## Offline Trajectory Plots

Created:

```text
tools/visualize_trajectory.py
```

Example:

```powershell
python tools\visualize_trajectory.py --estimate results\phase3a_mh01\output_MH_01_easy_offset10.txt --dataset .\datasets\MH_01_easy --output $env:TEMP\mh01_trajectory.png
```

The tool:

- reads the project estimate trajectory;
- reads EuRoC ground truth from `--dataset` or `--groundtruth`;
- reuses `tools/evaluate_trajectory.py` alignment and timestamp logic;
- plots aligned estimate vs ground truth in XY;
- plots translation error over time;
- optionally plots X/Y/Z components with `--components`;
- imports matplotlib only when `--output` is used;
- does not require PyQt5 or pyqtgraph.

If matplotlib is missing, it reports:

```text
--output requires matplotlib. Install it with: python -m pip install matplotlib
```

## Optional Dependencies

| Feature | Dependency | Required for headless? |
| --- | --- | --- |
| Core run | `numpy`, `opencv-python`, `scipy` | yes |
| Live viewer | `PyQt5`, `pyqtgraph` | no |
| Offline plot output | `matplotlib` | no |

## Limitations

- Live viewer launch could not be fully verified locally because PyQt5 is not
  installed.
- The live viewer currently shows cam0 frames and estimated cam0 trajectory, but
  not ground truth.
- Feature tracks are not drawn yet.
- Video recording is disabled by default.
- No generated PNG/MP4 files are committed.
- Qt/OpenGL availability can still vary by machine and graphics driver.

## Next Steps For Feature Tracks

Phase 5B should add optional feature-track display by exposing display-only
`FeatureMetaData.cam0_point` and previous/current point pairs to the viewer via
bounded queues. The implementation should avoid blocking VIO, avoid default
recording, and keep headless mode unchanged.
