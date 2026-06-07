# GUI 3D Baseline Checkpoint

This checkpoint records the accepted dashboard GUI baseline.

## Accepted State

- Evaluation dashboard launches successfully.
- Dashboard self-check passes.
- Live View is integrated inside the dashboard.
- Live trajectory is displayed in 3D.
- 3D origin and coordinate axes are visible.
- Bottom dashboard includes:
  - global comparison table,
  - creator info card,
  - global ATE/RPE chart tabs.
- Dataset registry workflow is available for the EuRoC Machine Hall datasets.

## Verified Checks

The following checks passed:

- frame_inside_available
- locked_size
- table_no_edit
- no_editor_delegate
- row_selection
- single_selection
- dark_results_table
- no_white_table_area
- title_correct
- tabs_not_clipped
- sidebars_compact
- global_charts_tabbed
- rejected_hidden_default
- provenance_visible
- bottom_not_behind_taskbar

## Known Remaining Issues

- Live camera preview still has noticeable lag and needs follow-up optimization.
- Final post-run trajectory visualization should be improved.
- Output correctness and final aligned trajectory display still need a separate correction phase.
- This checkpoint is the accepted GUI baseline, not the final validated release.

## Purpose

Use this checkpoint as the stable GUI baseline before continuing with output correction and performance fixes.
