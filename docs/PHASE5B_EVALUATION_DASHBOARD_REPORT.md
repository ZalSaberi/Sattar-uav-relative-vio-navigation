# Phase 5B/5C Evaluation Dashboard Report

## Purpose

Phase 5B introduced the first graphical evaluation dashboard. Phase 5C replaces
that prototype-style layout with a more polished fixed-screen desktop
application for running, evaluating, and comparing EuRoC Machine Hall VIO
results.

This work only changes dashboard tooling and documentation. It does not change
MSCKF/VIO math, estimator behavior, image-processing algorithms, or dataset
streaming behavior.

## Prototype Issues Addressed

- The Phase 5B dashboard used freely resizable splitter panels and could open as
  a small centered window.
- The first-frame cam0 preview path was too fragile and could leave the live
  camera panel blank.
- Result refresh and evaluation happened too directly in the GUI path.
- The global comparison view could show multiple rows per dataset instead of a
  clean one-row Machine Hall summary.
- The visual design looked like a functional prototype rather than a polished
  robotics evaluation application.

## Redesign Changes

The redesigned dashboard keeps the launch command:

```powershell
python tools\evaluation_dashboard.py --datasets-root .\datasets --results-root .\results
```

Optional GUI dependencies:

```powershell
python -m pip install PyQt5 pyqtgraph PyOpenGL matplotlib
```

The main window is titled `UAV-Airvision Evaluation Dashboard` and now uses:

- left sidebar titled `1. RUN & LIVE VIEW`;
- central tabbed visualization area;
- right sidebar titled `2. SINGLE EVALUATION`;
- full-width bottom section titled `3. GLOBAL COMPARISON DASHBOARD`;
- premium dark navy/graphite styling with blue, green, orange, red, and purple
  accents;
- rounded panels, metric cards, chart cards, cleaner table styling, compact log
  preview, and stronger visual hierarchy.

## Phase 5D Data Pipeline

Phase 5D turns the dashboard into a registry-backed experiment dashboard. The
normal workflow is now:

```text
dataset selected
-> registry resolves dataset paths
-> dataset inputs are validated
-> first cam0 frame loads
-> VIO runs through main.py
-> trajectory text file is generated
-> evaluate_trajectory.evaluate_files() recomputes metrics
-> metric cards, plots, table, and global charts update from that result
```

No fake, demo, or hard-coded metric values are used in the default path. Empty
datasets or missing estimates show placeholders rather than synthetic data.

## Dataset Registry

The five EuRoC Machine Hall datasets are configured in:

```text
configs/euroc_datasets.json
```

The dashboard loads this registry through:

```text
tools/dataset_registry.py
```

The registry defines each dataset key, display name, difficulty, relative
dataset path, default offset, cam0 paths, IMU CSV, ground truth CSV, output file
pattern, and default dashboard output directory pattern.

Registry validation can be run without GUI dependencies:

```powershell
python tools\dataset_registry.py --datasets-root .\datasets
```

This removes manual per-dataset dashboard configuration for the normal Machine
Hall workflow.

## Taskbar-Safe Fixed Fullscreen Behavior

On launch, the dashboard queries `QScreen.availableGeometry()` rather than raw
screen geometry. It applies a small safe margin, then after the native title bar
is shown it re-measures the frame/client geometry and locks the client window so
the outer frame stays inside the available desktop area.

This keeps the normal desktop title bar while preventing manual resizing. On
another screen or resolution, the same logic reuses that screen's available
geometry rather than a hard-coded size.

## Layout

The top body uses compact sidebar caps and center-heavy stretch proportions:

- left sidebar: maximum 330 px;
- center visualization area: primary stretch region;
- right sidebar: maximum 290 px.

The vertical split uses:

- top application body: 74%;
- bottom global comparison dashboard: 26%.

The bottom global dashboard gives comparison readability priority. The table
receives about 68% of the bottom width and the chart area receives about 32%.
Charts stay bound to evaluated metrics and do not expand long output paths or
squeeze the one-row-per-dataset table.

The Live View tab contains a balanced 2x2 layout:

- `Camera: cam0 (live)`;
- `Trajectory (XY) - Live`;
- `ATE (Absolute Trajectory Error)`;
- `RTE (Relative Translation Error) - w=1s`.

Dedicated tabs provide larger XY, XZ, YZ, ATE, and RPE/RTE plots with small
matplotlib toolbars.

## Live Dataset Preview

Dataset selection now resolves the selected EuRoC sequence through the registry
and loads cam0 frames from:

```text
mav0/cam0/data.csv
mav0/cam0/data/*.png
```

The dashboard prefers `data.csv` for timestamp-to-file mapping and falls back to
globbed PNG files when needed. The first valid grayscale frame is shown
immediately, centered and aspect-ratio preserved.

During a run, the preview updates from published VIO timestamps when available.
It also uses a lightweight timer-based frame advance so the camera panel remains
responsive even if console timestamp updates are sparse.

Missing datasets, missing cam0 frames, or unreadable frames are shown as clear
messages inside the preview panel.

## Performance Improvements

- VIO runs still use `QProcess`, so the GUI thread is not blocked.
- Single evaluation runs in a `QThread` worker.
- Result refresh runs in a `QThread` worker and evaluates only the latest
  accepted result per dataset for the global comparison view.
- The global table is now one row per dataset.
- Plot updates are event-driven after evaluation or selection, not continuously
  redrawn on every console line.
- Large trajectories are downsampled for plotting.
- Console preview is capped to recent lines.
- Live frame preview updates are throttled by a timer.

## Real Plot Binding

The dashboard uses an internal `EvaluationResult` model created from
`tools/evaluate_trajectory.py`. It stores:

- dataset key;
- estimate path;
- ground truth path;
- computed timestamp;
- alignment mode;
- aligned timestamps;
- aligned estimate positions;
- interpolated ground truth positions;
- ATE error norms;
- RPE/RTE errors when available;
- metric values for ATE/RPE/alignment overlap.

Trajectory XY/XZ/YZ plots, ATE plots, RPE/RTE plots, metric cards, global table
rows, and global bar charts all read from this real computed model.

## Evaluation Workflow

The dashboard reuses `tools/evaluate_trajectory.py` through `evaluate_files()`.
It computes:

- ATE RMSE;
- ATE mean;
- ATE median;
- ATE max;
- translation RPE over 1 second;
- aligned sample count;
- overlap duration.

Metrics are persisted to generated JSON:

```text
results/dashboard_metrics_summary.json
```

This file is ignored through the existing `results/` ignore rule and must not be
committed.

Cached rows include dataset key, estimate path, ground truth path, computed
timestamp, alignment mode, ATE/RPE metrics, sample count, overlap duration, and
status. Startup may show cached values, but refresh/evaluation recomputes from
the current estimate files so stale cache does not hide real data.

The right sidebar includes a compact `DATA SOURCE` block showing dataset,
estimate file, ground truth file, computed time, alignment mode, and whether the
metrics were freshly computed, loaded from cache, missing, or failed.

The dashboard also supports non-GUI data-flow validation:

```powershell
python tools\evaluation_dashboard.py --validate-data-flow --datasets-root .\datasets --results-root .\results
```

This reloads the registry, resolves first frames, finds latest accepted
estimates, recomputes metrics, and compares them with cached rows when present.

The dashboard also supports a nonvisual Qt invariant check:

```powershell
python tools\evaluation_dashboard.py --ui-self-check --datasets-root .\datasets --results-root .\results
```

The self-check verifies that the fixed window frame stays inside the available
desktop area, the size is locked, the read-only table cannot create an editor,
the table surface is dark, tab labels are not clipped, sidebars remain compact,
rejected experiments are hidden by default, provenance is visible, and the
bottom dashboard stays above the Windows taskbar.

## Table and Path Handling

The global comparison table now uses a `QTableView` backed by a
`QStandardItemModel` in `tools/dashboard/table.py`. This replaces the fragile
editable table-widget behavior from the prototype.

The table is strictly read-only:

- a `ReadOnlyDelegate` returns `None` from `createEditor()`;
- editing is disabled with `QAbstractItemView.NoEditTriggers`;
- every `QStandardItem` is selectable/enabled only, not editable;
- row selection is used instead of cell editing;
- single-row selection is enforced;
- the viewport, empty area, item selection, corner button, headers, and
  scrollbars are styled in the dark theme to avoid white editor or blank table
  areas.

Columns are exactly: `Dataset`, `Status`, `Output`, `ATE RMSE`,
`RPE 1s RMSE`, `ATE Mean`, `Aligned`, and `Overlap`. Compact fixed widths are
used for dataset/status/metrics, and `Output` is the only stretch column. Long
paths are shortened with middle elision in the table, estimate selector, and
status card. Full paths remain available through tooltips.

## Rejected Experiment Handling

Rejected Phase 3C/raw-Jacobian-style results are filtered out of the default
global dashboard so they do not distort ATE/RPE bar chart scales. The dashboard
uses the latest accepted result per dataset by default.

The right sidebar includes a `Show rejected experiments` checkbox. When enabled,
rejected/outlier estimates are included, tagged in the estimate selector, marked
red in the table, and bar charts switch to log scale if needed to keep large
outliers readable.

`MH_04_difficult` is warning-colored when evaluated because its current ATE is
substantially weaker than the easier Machine Hall sequences.

## Status Handling

Run status and evaluation status are now separate in the left status card:

- `Run`: `Idle`, `Running`, `Completed`, `Stopped`, or `Run Failed`;
- `Evaluation`: `Preview Loaded`, `Missing Output`, `Evaluating`, `Evaluated`,
  `Outlier`, or `Failed`.

This prevents a failed VIO process from making already-loaded/evaluated metrics
look failed. The table row status reflects the evaluated result when metrics are
available.

## Controls

Left sidebar:

- dataset selector for the five Machine Hall datasets;
- offset field;
- output directory selector;
- results root selector;
- `Run Selected`;
- `Run All 5 Datasets`;
- `Stop`;
- status card with separate run/evaluation state, dataset, frame, time, FPS, and
  shortened output file;
- thin progress bar;
- compact console preview.

Right sidebar:

- result-file selector;
- `Show rejected experiments` toggle;
- browse button;
- `Evaluate`;
- metric cards;
- utility buttons for refresh, plot export, summary export, and opening the
  results folder.

Bottom section:

- one-row-per-dataset comparison table;
- ATE RMSE bar chart;
- RPE 1s RMSE bar chart;
- warning/outlier color styling.

## Validation Status

Validated:

```powershell
python -m py_compile tools\evaluation_dashboard.py
python tools\evaluation_dashboard.py --help
python tools\evaluation_dashboard.py --datasets-root .\datasets --results-root .\results
git diff --check
```

The current `python` interpreter used by the shell does not have PyQt5 installed,
so full launch stops with the expected optional-dependency message:

```text
Install them with: python -m pip install PyQt5 pyqtgraph PyOpenGL matplotlib
```

Because of that missing dependency, visual verification of the fixed fullscreen
window, taskbar clearance, read-only table interaction, and live preview could
not be completed with the requested `python` command in this shell.

A secondary short Qt sanity check was run with the repository `.venv`, which has
the optional GUI packages installed. It opened the dashboard with a temporary
results root outside the repository and confirmed:

- the native frame is inside `QScreen.availableGeometry()`;
- min size equals max size after launch;
- the global table uses `NoEditTriggers`;
- row selection is enabled;
- the first cam0 preview frame loads;
- `Show rejected experiments` is off by default.

## Remaining Limitations

- Full interactive launch still requires optional GUI packages in the active
  Python environment.
- The live preview is dataset-frame based and timestamp-assisted; it does not
  yet display feature-track overlays from the VIO worker.
- RPE is translation-only; full SE(3) RPE and orientation error remain future
  evaluation work.
- Export actions write files only when the user explicitly chooses an output
  path, and generated exports must remain uncommitted.

## Next Recommended Phase

Phase 5D: install/verify the GUI dependency environment, visually inspect the
fixed fullscreen dashboard, then add optional feature-track overlays and
annotated frame/video export.
