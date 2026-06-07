# Phase 5E Manual Dashboard Polish Report

## Current Phase

The project is still in Phase 5: Visualization and Evaluation Dashboard.

Earlier Phase 5 work introduced the one-window evaluation dashboard, dataset registry integration, global comparison table, metrics cards, and live trajectory visualization. After the Codex-generated dashboard polish became unstable, the remaining dashboard work was continued manually through direct patches and controlled testing.

This report records the manual changes made after the accepted GUI 3D baseline.

## Stable Checkpoints

### gui-3d-baseline

Accepted GUI baseline.

- Dashboard launches.
- Live View is integrated.
- 3D trajectory view exists.
- Origin and coordinate axes are visible.
- Bottom dashboard includes the global table, creator info card, and global chart tabs.

### gui-3d-aligned-trajectory

Accepted aligned trajectory checkpoint.

- Final evaluated trajectory is shown in the 3D Live View.
- Red path represents aligned estimate.
- Green path represents interpolated ground truth.
- Both are displayed using a shared origin.

## Manual Fixes After Codex

### Live View crash fixes

Several crashes appeared after the dashboard was split and modified.

`ViewBox.setTitle(...)` was called even though `pyqtgraph.ViewBox` has no `setTitle` method. The fix removed direct `image_box.setTitle(...)` usage and routed title updates through Qt labels.

### QSS and f-string fixes

Dashboard stylesheet blocks failed when CSS braces were written inside Python f-strings without escaping. The fix corrected those Qt stylesheet blocks.

### PlotPanel helper fixes

Some dashboard plotting code called helper methods that were not present in `PlotPanel`.

The restored helpers included:

- `_prepare_axes(...)`
- `_finish()`
- `_style_axes(...)`

This allowed global ATE/RPE charts to render again.

### Bottom dashboard layout stabilization

The bottom dashboard was stabilized into the accepted layout:

- Global comparison table on the left.
- Creator information card in the middle.
- Global ATE/RPE chart tabs on the right.
- Fixed bottom height so it does not squeeze the upper Live View area.
- Read-only table behavior.
- Dark table background without the white artifact.
- Adjusted table widths and chart area.

### 3D Live trajectory integration

The 2D live trajectory was replaced by a 3D OpenGL-based trajectory view using `pyqtgraph.opengl`.

The 3D view now includes:

- Red estimate trajectory.
- Green ground truth/reference trajectory.
- White origin point.
- X/Y/Z coordinate axes.
- 3D grid.
- Final aligned trajectory display after evaluation.

### Aligned red/green final trajectory

The earlier display used raw live estimate and ground truth preview with separate origins. This made the red and green paths appear incorrectly separated.

The fix added a final trajectory mode:

- Red = `estimate_aligned`
- Green = `groundtruth_interpolated`
- Both are normalized with one shared origin.

This makes the visual red/green gap meaningful as trajectory error, not a coordinate-frame artifact.

### Trajectory Inspector

Direct clicking on the 3D trajectory was tested but rejected because selecting a 3D point from a 2D mouse click is ambiguous.

A deterministic inspector was added instead:

- Slider selects exact sample index.
- `Prev`, `Next`, and `Latest` buttons move the selected sample.
- Yellow marker shows the selected sample on the red trajectory.
- Compact text shows sample, time, x, y, z, and error.
- Full dx/dy/dz/error values are available through tooltip on the inspector label/title.
- Visible text was compacted so it no longer resizes the UI.

### Live preview FPS improvement

The camera preview was slow because the timer and status/log updates were too heavy.

Performance fixes included:

- Preview timer changed toward 10 FPS behavior.
- Preview frame advancement became timer-owned instead of fighting stdout timestamp updates.
- Log preview flushing was throttled.
- Pose/status updates were throttled.
- OpenCV thread usage was reduced with `cv2.setNumThreads(1)`.
- Loaded frames are now displayed instead of being discarded when a newer request is pending.

Observed result:

- Live preview reached about 9 FPS during testing.
- This is accepted for now because the target was at least around 10 FPS and the behavior is much smoother than before.

### Final visual polish

Latest manual polish includes:

- 3D trajectory hover tooltip disabled.
- Inspector remains the controlled way to inspect trajectory samples.
- Matplotlib toolbar icons/buttons are tinted white for dark UI readability.
- Global comparison table column widths adjusted.
- `Aligned` column widened.
- Last table section stretches to reduce unused right-side blank area.

## Files Modified In This Manual Phase

Main files touched:

- `tools/evaluation_dashboard.py`
- `tools/dashboard/live_view.py`
- `tools/dashboard/table.py`
- `docs/PHASE5E_MANUAL_DASHBOARD_POLISH_REPORT.md`

No VIO, MSCKF, image-processing, or dataset streaming algorithm files were intentionally modified in this manual dashboard polish phase.

## Current Accepted UI Behavior

The current accepted dashboard behavior is:

- Fullscreen-style dashboard launches.
- Live View is embedded in the dashboard.
- Camera preview plays much more smoothly than before.
- 3D trajectory view is available.
- Red trajectory means estimate/VIO.
- Green trajectory means ground truth/reference.
- In final evaluation mode, red is aligned estimate and green is interpolated ground truth.
- Inspector slider is used for exact sample inspection.
- Direct hover/click picking in the 3D scene is not used for sample selection.
- Global comparison table remains read-only.
- Bottom dashboard includes creator info and global charts.

## Validation Commands

Run these commands after changes:

    python -m py_compile tools/evaluation_dashboard.py tools/dashboard/live_view.py tools/dashboard/table.py tools/dataset_registry.py
    python tools/evaluation_dashboard.py --ui-self-check --datasets-root ./datasets --results-root ./results
    python tools/evaluation_dashboard.py --datasets-root ./datasets --results-root ./results
    git diff --check

## Expected Self-Check

Expected self-check items include:

- frame_inside_available=True
- locked_size=True
- table_no_edit=True
- no_editor_delegate=True
- row_selection=True
- single_selection=True
- dark_results_table=True
- no_white_table_area=True
- title_correct=True
- tabs_not_clipped=True
- sidebars_compact=True
- global_charts_tabbed=True
- rejected_hidden_default=True
- provenance_visible=True
- bottom_not_behind_taskbar=True

## Known Remaining Limitations

- Live camera preview is improved but still depends on disk speed, dataset image size, and Qt/OpenGL performance.
- The inspector selects by sample index, not by true 3D picking.
- The 3D view is a visualization tool; scientific validation still depends on ATE/RPE metrics.
- RPE is still translation-only.
- Orientation error and full SE(3) RPE are not implemented yet.
- Feature-track overlay is still not implemented.
- Post-run output validation should continue before final release claims.

## Recommended Next Phase

Continue Phase 5 with post-run output validation, optional feature-track overlay, optional annotated export, final release documentation, and screenshots.
