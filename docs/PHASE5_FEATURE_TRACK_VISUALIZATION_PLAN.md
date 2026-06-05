# Phase 5B Feature-Track Visualization Plan

## Goal

Add optional feature-track visualization without changing MSCKF/VIO math or
slowing the core estimator path.

## Where Feature Points Exist

Feature points exist at two useful stages:

| Location | Data | Notes |
| --- | --- | --- |
| `src/image_processing/feature_meta_data.py::FeatureMetaData` | `id`, `lifetime`, `cam0_point`, `cam1_point`, `response` | Pixel-space points used by tracking, adding, pruning, and display. Best source for drawing tracks. |
| `src/image_processing/pipeline.py` | `prev_features`, `curr_features` grids | Grid-organized `FeatureMetaData` lists before publication. This is where current and previous pixel locations are available together. |
| `src/image_processing/feature_publisher.py` | `FeatureMeasurement` values `u0`, `v0`, `u1`, `v1` | Undistorted normalized coordinates sent to MSCKF. Useful for filter input, not ideal for drawing on raw images. |
| `src/modules/vio.py` | receives `stereo_msg` and `feature_msg` | Good place to hand optional display payloads to the viewer through queues. |

## Safe Exposure Strategy

Expose display-only feature data after image processing finishes for a frame:

1. Keep MSCKF feature messages unchanged.
2. Add an optional debug/display payload to the image-processing result or a
   separate viewer-only callback path.
3. Include only lightweight data:
   - timestamp;
   - cam0 grayscale image or already displayed frame reference;
   - current `cam0_point` pixel coordinates;
   - feature ids;
   - previous `cam0_point` coordinates when available;
   - lifetime.
4. Do not store this payload in `map_server` or MSCKF state.
5. Do not require it for headless mode.

## Drawing Tracks On Frames

Recommended rendering:

- convert cam0 grayscale to BGR/RGB for annotation;
- draw current points as small circles;
- draw previous-to-current segments for tracked features;
- color by feature lifetime or fixed status categories;
- optionally display feature id only when zoomed or when a debug flag is set.

OpenCV drawing should happen on a copied frame, never on the image buffer used
by the tracker.

## Optional Annotated Frames Or Video

If recording is added:

- make it opt-in with a CLI flag such as `--record-feature-video`;
- write only to an explicit output path;
- keep recording disabled by default;
- cap frame rate or sample every N frames;
- never commit generated MP4/PNG files;
- report the output path clearly.

## Avoiding VIO Slowdown

Use a bounded queue between processing threads and the viewer:

- drop old debug frames when the queue is full;
- keep only the latest display frame;
- avoid blocking image processing or MSCKF update threads;
- avoid per-feature Python string drawing in real time unless explicitly enabled;
- keep all Qt calls in the GUI thread.

## Files Likely To Change In Phase 5B

| File | Planned change |
| --- | --- |
| `src/image_processing/pipeline.py` | Build an optional viewer/debug feature-track payload from `prev_features` and `curr_features`. |
| `src/modules/vio.py` | Pass feature-track payloads to the viewer using queue-safe viewer methods. |
| `src/viewer.py` | Add `update_feature_tracks(...)` and draw annotations on copied cam0 frames. |
| `src/main.py` | Add optional flags for feature-track display and possibly recording. |
| `docs/PHASE5_FEATURE_TRACK_VISUALIZATION_PLAN.md` | Update with implementation results after Phase 5B. |

## Do Not Do Yet

- Do not change MSCKF feature measurement math.
- Do not record video by default.
- Do not add generated annotated frames or videos to git.
- Do not use `cv2.imshow` in worker threads as the main visualization path.
- Do not make PyQt5 or pyqtgraph required for headless runs.
