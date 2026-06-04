# Phase 0 Audit Report

Repository: UAV-Airvision / Sattar Run
Audit date: 2026-06-04
Scope: Full repository inspection, folder by folder and file by file. No code changes were made.

## Executive Summary

This repository is a Python visual-inertial odometry prototype for EuRoC stereo camera plus IMU data. The intended data flow is:

EuRoC dataset files -> `EuRoCDataset` readers -> `DataPublisher` threads -> image and IMU queues -> `VIO` worker threads -> `ImageProcessor` feature measurements and `MSCKF` state updates -> optional `SimpleViewer` and `results/txts` trajectory output.

The project is not currently runnable from the documented commands. The most immediate blockers are dependency/import issues: `src/main.py` imports `viewer.py` at module import time, but `requirements.txt` does not include `PyQt5` or `pyqtgraph`; the documented `python main.py` and root `run.bat` target a non-existent root-level `main.py`; and packaging uses bare imports that only work when `src` is on `sys.path`.

After launch hygiene, the highest-risk implementation areas are thread ownership, stereo calibration/matching, and MSCKF update math. Several issues are likely to produce either hard runtime failures or invalid state estimates even after the app starts.

## Architecture and Data Flow

1. `src/main.py` parses `--path`, `--offset`, and `--view`.
2. `EuRoCDataset` opens EuRoC files under `<dataset>/mav0`:
   - `state_groundtruth_estimate0/data.csv`
   - `imu0/data.csv`
   - `cam0/data/*.png`
   - `cam1/data/*.png`
3. `dataset.set_starttime(offset)` applies one offset to ground truth, IMU, cam0, cam1, and stereo.
4. `DataPublisher(dataset.imu, imu_q, ratio=0.4)` and `DataPublisher(dataset.stereo, img_q, ratio=0.4)` start non-daemon publisher threads.
5. `VIO` starts three daemon worker threads:
   - image thread: reads stereo frames, calls `ImageProcessor.stereo_callback`, pushes feature messages
   - IMU thread: calls `ImageProcessor.imu_callback` and `MSCKF.imu_callback`
   - feature thread: calls `MSCKF.feature_callback`, optionally updates viewer pose
6. `ImageProcessingPipeline`:
   - builds current image holders via `PyramidBuilder`
   - initializes first-frame FAST features
   - tracks left-image features with LK and IMU rotation prediction
   - stereo-matches left features into right image
   - adds and prunes features per grid cell
   - publishes undistorted stereo feature measurements
7. `MSCKF`:
   - buffers IMU until 200 samples are available for gravity/bias initialization
   - propagates IMU state up to each feature timestamp
   - augments camera state
   - adds feature observations
   - removes lost features, initializes positions, updates covariance/state
   - prunes camera states
   - writes output state lines to `results/txts/output_<dataset>_offset<offset>.txt`
8. `SimpleViewer`, if enabled, is intended to display the left image, 3D trajectory, and point cloud, and to write `output.mp4`.

## Main Entry Point and Run Command

Current main entry point:

```powershell
python src/main.py --path .\datasets\MH_01_easy --offset 10
```

Current documented commands are incorrect or incomplete:

- `README.md` says `python main.py ...`, but `main.py` lives in `src/`.
- Root `run.bat` calls `python main.py`, which fails from the repository root.
- `src/run.bat` calls `python main.py` and assumes it is launched from `src`.
- `--view` is currently blocked because `SimpleViewer` is created before any `QApplication` and no Qt event loop is started from `src/main.py`.
- Even without `--view`, `src/main.py` imports `viewer.py` at the top level, so missing GUI dependencies can block headless runs.

Recommended eventual command shape:

```powershell
python -m src.main --path .\datasets\MH_01_easy --offset 10
```

That requires import/package cleanup first.

## Folder and File Responsibility Map

### Repository Root

| Path | Responsibility | Notes |
| --- | --- | --- |
| `.gitignore` | Ignores datasets, results, virtual envs, generated media, logs, caches. | Contains duplicated entries and a malformed-looking `/results/rte_summary.pymachine_hall.zip` line. |
| `README.md` | English overview, setup, run instructions, result description. | Structure and run command are stale; references result images not present in repository; text has mojibake in tree glyphs. |
| `README-ru.md` | Russian overview and run instructions. | Displayed content is mojibaked in current checkout/tool output. |
| `requirements.txt` | Python dependency list. | Missing GUI deps, includes stdlib modules, unpinned packages, duplicate OpenCV families. |
| `run.bat` | Batch sweep over EuRoC datasets and offsets. | Calls missing root `main.py`; echo text is mojibaked. |
| `codex_current_changes.patch` | Stored patch artifact. | Not applied according to `git status`; includes changes already visible in files and no-newline notes. Treat as generated/reference artifact, not runtime code. |

### `src/`

| Path | Responsibility | Notes |
| --- | --- | --- |
| `src/__init__.py` | Package marker. | Empty, but code is not consistently package-importable. |
| `src/main.py` | CLI entry point, publisher startup, VIO wiring. | Bare imports; top-level viewer import; no joins; no Qt event loop. |
| `src/config.py` | EuRoC camera, IMU, filter, tracking config. | Imports `Path` and `yaml` but does not use them; hard-coded EuRoC calibration only. |
| `src/msckf.py` | MSCKF state, propagation, feature update, output writing. | Largest risk area; uses bare imports and several mathematically risky update/gating choices. |
| `src/utils.py` | Quaternion, rotation, skew, isometry utilities. | Duplicates some functionality in `src/feature/utils.py`. |
| `src/viewer.py` | PyQt5/pyqtgraph 2D/3D viewer and MP4 recording. | Requires GUI deps, QApplication lifecycle, and event-loop integration. |
| `src/run.bat` | Batch sweep intended from `src`. | Uses `..\Datasets`; echo text mojibaked; path convention differs from root docs. |

### `src/streaming/`

| Path | Responsibility | Notes |
| --- | --- | --- |
| `__init__.py` | Re-exports publisher symbols. | Wildcard import. |
| `dataset.py` | CSV/image readers, EuRoC stereo dataset abstraction. | Hard-coded EuRoC layout; no file validation; stereo sync assert disabled. |
| `publisher.py` | Timed publisher thread that pushes dataset messages into queues. | Thread lifecycle is only partially managed; demo block uses OpenCV GUI. |

### `src/modules/`

| Path | Responsibility | Notes |
| --- | --- | --- |
| `vio.py` | Coordinates image, IMU, and feature worker threads. | Daemon threads, no stop/join API, shared mutable processors without locks. |

### `src/image_processing/`

| Path | Responsibility | Notes |
| --- | --- | --- |
| `__init__.py` | Public image-processing facade. | Preserves typo alias `stareo_callback`; imports misspelled `feature_measurment`. |
| `pipeline.py` | Orchestrates IMU preintegration, pyramids, tracking, stereo matching, add/prune/publish. | Recreates helper objects every frame; shares IMU buffer across threads. |
| `camera_model.py` | Intrinsic matrix, undistort/distort, rotation-compensated tracking prediction. | Used only for cam0 in stereo matcher. |
| `pyramid_builder.py` | Holds current left/right images, nominal pyramid builder. | Returns raw images; commented pyramid code is inactive. |
| `imu_processor.py` | Buffers IMU for image-tracker rotation compensation. | Shared with image/IMU threads without locks; buffer slicing assumes timestamps are available. |
| `stereo_matcher.py` | LK left-to-right matching, reverse check, epipolar filtering. | Uses cam0 model for cam1 projection; epipolar residual appears incorrect; ignores reverse LK status. |
| `feature_tracker.py` | Tracks existing features between left frames and stereo matches them. | RANSAC is described but not implemented; status mask shape handling is fragile. |
| `feature_initializer.py` | Detects first-frame FAST features and stereo matches them. | No bounds guard for computed grid index. |
| `feature_adder.py` | Detects new FAST features and fills grid cells. | Mask indexing can wrap at image borders; no bounds guard for grid index. |
| `feature_pruner.py` | Trims per-grid features by lifetime. | Depends on attributes assigned after construction. |
| `feature_publisher.py` | Converts current features into undistorted stereo `FeatureMeasurement`s. | Duplicates camera model code; misspelled measurement filename. |
| `feature_meta_data.py` | Mutable per-feature tracking metadata. | Simple data holder. |
| `feature_measurment.py` | Stereo feature measurement object. | Filename typo leaks into imports. |
| `utils.py` | Local skew/select helpers. | Duplicates `skew`; `select` relies on truthiness of numpy status entries. |

### `src/feature/`

| Path | Responsibility | Notes |
| --- | --- | --- |
| `__init__.py` | Multiple-inheritance `Feature` composition. | Updates class-level stereo extrinsics when given. |
| `base_feature.py` | Feature ID, observations, position, initialization state. | Class-level extrinsics are global mutable state. |
| `feature_depth_estimator.py` | Two-view initial depth estimate. | No degeneracy/zero-denominator guard. |
| `feature_motion_checker.py` | Translation-parallax check. | Optimization config defaults disable this check via `translation_threshold = -1.0`. |
| `feature_observation.py` | Reprojection cost and Jacobian for inverse-depth solve. | Huber weighting is custom and should be tested. |
| `feature_position_initializer.py` | Levenberg-style feature position initialization. | Assumes enough valid camera poses; no singular solve guard. |
| `utils.py` | Feature-side isometry, rotation, skew helpers. | Duplicates root `src/utils.py`. |

## Runtime Blockers

| Priority | Blocker | Evidence | Impact |
| --- | --- | --- | --- |
| P0 | Documented command cannot find entry point. | `README.md` and root `run.bat` call `python main.py`; entry is `src/main.py`. | New users cannot launch from root as documented. |
| P0 | GUI deps missing but imported unconditionally. | `src/main.py` imports `SimpleViewer`; `viewer.py` imports `PyQt5` and `pyqtgraph`; `requirements.txt` omits both. | Headless run can fail before argument parsing. |
| P0 | `--view` cannot work as wired. | `SimpleViewer()` is constructed without `QApplication`; `main.py` never calls `show()` or starts `app.exec_()`. | Runtime crash or invisible/non-updating viewer. |
| P0 | Dataset path must exist but is not validated. | Default `./datasets/V2_03_difficult`; datasets ignored by git; readers call `open`, `os.listdir`, and `cv2.imread` directly. | Raw `FileNotFoundError` or `None` images. |
| P1 | Process/thread lifecycle is unmanaged. | Publisher threads are non-daemon, VIO threads are daemon, `main.py` never joins or stops them. | Process can exit while daemon workers still have queued work, especially near end-of-dataset. |

## Dependency Issues

- `requirements.txt` omits `PyQt5` and `pyqtgraph`, both required by top-level imports.
- `requirements.txt` includes `logging` and `typing`, which are standard-library modules for supported Python versions and should not be installed as third-party packages.
- `opencv-python` and `opencv-contrib-python` are both listed. Installing both is usually unnecessary and can create wheel/file ownership confusion. Choose one.
- All dependencies are unpinned, so repeatable VIO behavior and OpenCV LK behavior are not reproducible.
- `matplotlib` and `pandas` are listed but no active source file uses them.
- `pyyaml` is listed and `yaml` is imported in `src/config.py`, but no YAML config loading is implemented.

## Import and Path Issues

- `src/main.py`, `src/msckf.py`, and `src/modules/vio.py` use bare imports such as `from config`, `from utils`, `from feature`, and `from image_processing`. This works when `src` is the script directory, but not cleanly as an installed/package module.
- `python -m src.main` is the desirable package-style command, but would currently require relative import fixes.
- `viewer.py` is imported even when `--view` is not set. This couples CLI/headless operation to GUI packages.
- `feature_measurment.py` and `stareo_callback` are misspelled compatibility names. They work only because imports match the typos, but they are maintenance traps.
- Output path `results/txts` is relative to the current working directory, so launching from root versus `src` writes to different result locations.

## Windows Issues

- The repository path contains a space (`Sattar Run`). Current batch files quote dataset paths but not the Python script path. This is probably okay for `python main.py`, but future path handling should stay consistently quoted.
- Root `run.bat` calls `python main.py` from root, which fails because no root `main.py` exists.
- `src/run.bat` assumes the current directory is `src`; running it from root will not resolve `main.py` correctly.
- Batch echo text is mojibaked. If this file is intended to be Russian, it needs a consistent encoding and/or `chcp 65001`.
- `src/run.bat` uses `..\Datasets` while root docs and root `run.bat` use `datasets`. Windows is case-insensitive by default, but this inconsistency will fail on case-sensitive filesystems and confuse docs.
- README tree glyphs and Russian text appear mojibaked in this checkout/tool output, indicating an encoding/documentation problem.

## Dataset and EuRoC Issues

- The dataset loader assumes exact EuRoC MAV paths and does not check them before constructing readers.
- `EuRoCDataset` always constructs `GroundTruthReader`, even though ground truth is not used by the runtime pipeline. Missing ground-truth files will still block VIO.
- `Stereo.start_time()` returns `self.cam0.starttime`, not `cam0.start_time()`. At construction this is `-inf`, so dataset base start time effectively ignores camera start time.
- `Stereo.__iter__` zips filtered cam0 and cam1 streams with the stereo timestamp assert commented out. If a camera has missing frames or different filtering, the code can silently pair wrong images.
- `ImageReader.read()` returns `cv2.imread(path, -1)` without checking for `None`.
- `ImageReader.preload()` can index `self.timestamps[i]` before checking `i < len(self.ids)`, although preloading is currently disabled.
- Image filenames are sorted by `float(x[:-4])`. This works for EuRoC nanosecond names but should be validated with clear errors.
- Offset semantics are documented as seconds, but `run.bat` sweeps large offsets without validating that enough IMU samples remain for gravity initialization.

## Threading Issues

- `VIO` daemon worker threads have no `stop()` or `join()` API.
- `DataPublisher.stop()` exists but `main.py` discards publisher objects and never calls it.
- `ImageProcessingPipeline.imu_processor.imu_buffer` is written by the IMU thread and read/sliced by the image thread without a lock.
- `MSCKF.imu_msg_buffer` is written by the IMU thread and read/sliced by the feature thread without a lock.
- `SimpleViewer` queues are thread-safe, but the Qt object lifecycle is not integrated with `main.py`.
- Exceptions in any worker thread are not propagated to the main thread. A failed image/filter worker can silently kill one processing path while publishers continue.
- End-of-stream handling depends on `None` sentinels but does not coordinate full drain of the feature queue before process exit.

## Image Processing Issues

- `PyramidBuilder` currently returns raw images. OpenCV can build pyramids internally, but the class name and config imply explicit pyramid reuse that is not happening.
- `cv2.OPTFLOW_USE_INITIAL_FLOW` is always set. Every LK call must pass a valid initial point array with the exact expected shape and dtype.
- FAST detection, grid assignment, and masks assume points are strictly in image bounds. Border masking in `FeatureAdder` can use negative slice bounds and wrap behavior.
- Grid index calculation does not clamp `row`/`col`, so points at or beyond the far edge can index outside `grid_num`.
- RANSAC is advertised in README/config, but `FeatureTracker` sets all inliers to true after stereo matching and does not run RANSAC.
- The image processor recreates `StereoMatcher`, `FeaturePublisher`, and other helpers each frame, increasing overhead and making state ownership less explicit.
- There are duplicated camera-model operations in `camera_model.py` and `feature_publisher.py`.

## Stereo Matching Issues

- `StereoMatcher` is constructed with a `CameraModel` initialized from cam0 only. It uses cam0 intrinsics/distortion to predict and distort cam1 points.
- Left-to-right projection uses rotation but no translation/depth model before LK. That can be acceptable as an initial guess only, but it should be documented and tested.
- Reverse LK status `rev_mask` is computed but not included in the inlier mask.
- Epipolar residual appears to use only the first component of `pt1_h * line` rather than the dot product `pt1_h @ line`.
- Vertical-disparity filtering uses `disp = abs(proj1[:,1] - p1[:,1])` and hard-coded `disp < 20`; this is not tied cleanly to calibration, resolution, or `stereo_threshold`.
- Essential matrix construction depends on transform conventions from `T_imu_cam0`, `T_imu_cam1`, and `T_cn_cnm1`; these conventions are not tested.
- If LK returns `None` outputs for bad inputs, stereo matching does not guard before indexing.

## MSCKF and Filtering Risks

- Chi-square gating table uses `chi2.ppf(0.05, i)` while comments say 0.95 confidence. This likely rejects most valid measurements.
- Gating degrees of freedom are likely wrong (`len(cam_state_ids)-1` and `len(involved_cam_state_ids)`), not the residual dimension after feature nullspace projection.
- `measurement_update()` uses `state_cov = (I - K H) P` and comments out the Joseph-form covariance update. This can destroy covariance consistency/PSD.
- `measurement_jacobian()` computes `H_f`, then replaces it with `-H_x[:4, 3:6]`, which is suspicious and needs derivation/testing.
- Feature initialization has no guard for singular `np.linalg.solve`, degenerate depth (`a @ a == 0`), empty `cam_poses`, or insufficient stereo observations after pruning.
- `OptimizationConfigEuRoC.translation_threshold = -1.0` disables the parallax check, allowing weakly constrained features into initialization.
- Gravity initialization assumes the first 200 IMU samples are stationary enough to infer gravity and gyro bias.
- Feature messages arriving before gravity initialization are dropped, not queued.
- `IMUState.next_id` is global mutable class state and is not reset for repeated runs in the same process.
- Output files are opened in append mode and are not cleared at run start, so repeated runs can mix trajectories.
- Filter logging uses unconditional `print()` in the hot path.

## Viewer and Output Issues

- `SimpleViewer` requires `QApplication` before construction and an event loop afterward. `main.py` provides neither.
- `SimpleViewer` is imported even for non-view runs, causing unnecessary GUI dependency.
- Viewer is never shown from `main.py`.
- `update_points()` is defined but never called from `VIO`, so 3D point display is not fed by the pipeline.
- MP4 recording writes `output.mp4` relative to current working directory with fixed widget dimensions captured at construction.
- The comment says first 30 seconds, but `_record_len` is 50.0.
- Output trajectory files are appended, not replaced, and no metadata/header is written.
- README references result plots and plotting scripts that are not in the tracked repository.

## Testing and Documentation Gaps

- No test directory or test runner is present.
- No smoke test for CLI startup, imports, or missing dataset error messages.
- No unit tests for quaternion utilities, camera distortion/undistortion, stereo epipolar checks, feature initialization, or MSCKF covariance/gating.
- No synthetic dataset fixture or minimal EuRoC fixture.
- No CI config.
- No packaging metadata (`pyproject.toml`, setup config) to define the source layout.
- README structure map does not match actual folder layout.
- README says plotting scripts are included, but none are tracked.
- Encoding issues in README/run scripts make documentation hard to trust.

## Priority Table

| Priority | Item | Category | Why It Matters |
| --- | --- | --- | --- |
| P0 | Fix documented/root run command and entry point. | Runtime | Current root launch path fails immediately. |
| P0 | Decouple viewer import from headless runs and add missing GUI deps or optional extra. | Dependency/runtime | Missing GUI deps can block all runs. |
| P0 | Implement proper `--view` Qt lifecycle. | Viewer/runtime | Current viewer path is not viable. |
| P0 | Add dataset path validation with clear EuRoC checks. | Dataset/runtime | Current failures are late/raw and can produce `None` images. |
| P0 | Fix package imports for one supported run mode. | Import/path | Needed before reliable tests or scripts. |
| P1 | Add thread stop/join/error propagation. | Threading | Prevents silent worker death and incomplete processing. |
| P1 | Lock or serialize shared IMU buffers. | Threading/filtering | Current shared mutation can corrupt tracking/filter inputs. |
| P1 | Correct stereo matcher cam1 calibration and epipolar residual. | Stereo | Bad stereo matches poison all downstream feature positions. |
| P1 | Restore/implement RANSAC or remove claims/config. | Image processing | Outliers are currently under-filtered. |
| P1 | Fix chi-square gating confidence/DOF and covariance update form. | MSCKF | Core filter consistency risk. |
| P1 | Guard feature initialization degeneracies. | MSCKF/features | Prevents crashes and invalid landmarks. |
| P1 | Make output files deterministic per run. | Output | Avoids mixed trajectories. |
| P2 | Clean dependency pins and unused deps. | Dependency | Reproducibility and install hygiene. |
| P2 | Normalize filenames/typos with compatibility aliases. | Maintainability | Reduces confusion around public API names. |
| P2 | Remove duplicate utilities/camera code. | Maintainability | Less drift and easier testing. |
| P2 | Repair README encoding and stale structure/result references. | Documentation | Makes onboarding credible. |
| P2 | Add tests, CI, and minimal fixtures. | Testing | Prevents regressions after P0/P1 fixes. |

## Staged Fix Plan

### Stage 1: Make the Project Launch Predictably

1. Choose one supported invocation style, preferably `python -m src.main`.
2. Convert internal imports to package-relative imports.
3. Move `viewer` import behind `--view` or an optional viewer factory.
4. Update root and `src` batch scripts, or remove the duplicate script.
5. Add early dataset validation with actionable error messages.
6. Fix `requirements.txt`: remove stdlib modules, choose one OpenCV package, add or optionalize GUI deps.

### Stage 2: Stabilize Runtime Ownership

1. Store publisher objects in `main.py`.
2. Add `VIO.stop()` and `VIO.join()`.
3. Make publisher and worker failures visible to the main thread.
4. Protect shared IMU buffers with locks or route all time-ordered processing through one owner thread.
5. Ensure end-of-stream drains image, IMU, and feature queues before exit.

### Stage 3: Repair Viewer and Outputs

1. Create `QApplication` before `SimpleViewer`.
2. Show viewer and run the Qt event loop when `--view` is set.
3. Feed map/feature points or remove the unused point display.
4. Make video path, recording duration, and output path configurable.
5. Open trajectory files in write mode at run start and include a header/metadata.

### Stage 4: Validate Dataset and Image Processing

1. Add a EuRoC layout validator.
2. Re-enable stereo timestamp assertions with clear tolerance and diagnostics.
3. Check every image read for `None`.
4. Clamp grid indices and fix border mask behavior.
5. Decide whether to use explicit pyramids or raw images and align naming/config.
6. Implement actual RANSAC/outlier rejection or remove the claim.

### Stage 5: Correct Stereo Geometry

1. Use cam0 and cam1 intrinsics/distortions separately.
2. Re-derive frame transforms for `R0to1`, `t01`, and `E`.
3. Include reverse LK status in the inlier mask.
4. Fix epipolar residual to use a dot product.
5. Add synthetic stereo geometry tests and EuRoC frame smoke tests.

### Stage 6: De-risk MSCKF

1. Correct chi-square confidence and degrees of freedom.
2. Restore a consistent covariance update, preferably Joseph form or a proven equivalent.
3. Add guards for singular solves, invalid depths, insufficient poses, and non-finite updates.
4. Revisit feature Jacobian/nullspace derivation.
5. Add unit tests for propagation, augmentation, gating, feature initialization, and covariance symmetry/PSD.
6. Add a short deterministic replay fixture to compare output against a known baseline.

### Stage 7: Documentation and Maintenance

1. Repair README encoding and stale folder map.
2. Document exact EuRoC expected layout and recommended command examples.
3. Add troubleshooting notes for Windows, PyQt, and OpenCV GUI use.
4. Add CI once smoke/unit tests exist.
5. Clean generated artifacts and duplicate ignore entries.
