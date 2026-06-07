# GUI 3D Inspector FPS10 Checkpoint

This checkpoint records the accepted reliable Phase 5E dashboard state.

## Accepted State

- Dashboard launches successfully.
- Live View is embedded in the dashboard.
- Camera preview is significantly smoother and reaches around 9-10 FPS in testing.
- 3D trajectory view is active.
- Red trajectory represents the VIO estimate.
- Green trajectory represents ground truth/reference.
- Final aligned trajectory is displayed after evaluation.
- Origin and coordinate axes are visible.
- Trajectory Inspector is available for exact sample inspection.
- Inspector uses Prev, Next, Latest, and slider controls.
- Inspector text is compact and does not resize the UI.
- Matplotlib toolbar icons/buttons are readable on the dark theme.
- Bottom global comparison table is read-only and visually stable.
- Phase 5E manual dashboard polish report is available.

## Known Remaining Issues

- Live preview still depends on disk speed, image size, and Qt/OpenGL performance.
- Feature-track overlay is not implemented yet.
- RPE is still translation-only.
- Orientation error and full SE(3) RPE are not implemented yet.
- Final release still needs screenshots and output validation.

## Purpose

Use this as the reliable dashboard checkpoint after manual Phase 5E polish.
