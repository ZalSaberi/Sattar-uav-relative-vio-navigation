from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]

CYRILLIC_RE = re.compile(r"[\u0400-\u04FF]")


REPLACEMENTS = {
    # ------------------------------------------------------------------
    # src/config.py
    # ------------------------------------------------------------------
    "Configuration parameters for optimizing the 3D feature position.":
        "Configuration parameters for optimizing the 3D feature position.",
    "# Gravity":
        "# Gravity",
    "# Stereo image frame rate. Used only to define":
        "# Stereo image frame rate. Used only to define",
    "# the time threshold of each filter iteration.":
        "# the time threshold of each filter iteration.",
    "# Maximum number of stored camera states":
        "# Maximum number of stored camera states",
    "# Position uncertainty threshold used to determine":
        "# Position uncertainty threshold used to determine",
    "# when an online system reset is required. Otherwise,":
        "# when an online system reset is required. Otherwise,",
    "# accumulated uncertainty can make the estimate unstable.":
        "# accumulated uncertainty can make the estimate unstable.",
    "# Note that online reset uses dead reckoning.":
        "# Note that online reset uses dead reckoning.",
    "# Set this threshold to a non-positive value to disable online reset.":
        "# Set this threshold to a non-positive value to disable online reset.",
    "# Threshold for keyframe selection":
        "# Threshold for keyframe selection",
    "# Noise-related parameters. Variance is used, not standard deviation.":
        "# Noise-related parameters. Variance is used, not standard deviation.",
    "# The initial covariance for orientation and position may be set to zero.":
        "# The initial covariance for orientation and position may be set to zero.",
    "# Velocity, biases, and extrinsic parameters must have non-zero uncertainty.":
        "# Velocity, biases, and extrinsic parameters must have non-zero uncertainty.",
    "## Calibration parameters":
        "## Calibration parameters",
    "# T_imu_cam transforms a vector from the IMU frame to the camera frame.":
        "# T_imu_cam transforms a vector from the IMU frame to the camera frame.",
    "# T_cn_cnm1 transforms a vector from camera-0 frame to camera-1 frame.":
        "# T_cn_cnm1 transforms a vector from camera-0 frame to camera-1 frame.",
    "# See https://github.com/ethz-asl/kalibr/wiki/yaml-formats":
        "# See https://github.com/ethz-asl/kalibr/wiki/yaml-formats",

    # ------------------------------------------------------------------
    # Generic docstring labels
    # ------------------------------------------------------------------
    "Args:":
        "Args:",
    "Returns:":
        "Returns:",

    # ------------------------------------------------------------------
    # src/feature/*
    # ------------------------------------------------------------------
    "Checks whether the input camera poses have enough translation for feature triangulation.":
        "Checks whether the input camera poses have enough translation for feature triangulation.",
    "cam_states: input camera poses as a dictionary <CAMStateID, CAMState>":
        "cam_states: input camera poses as a dictionary <CAMStateID, CAMState>",
    "True if the translation between camera poses is sufficient for triangulation (bool)":
        "True if the translation between camera poses is sufficient for triangulation (bool)",
    "Computes the camera observation cost (error).":
        "Computes the camera observation cost (error).",
    "Computes the camera observation Jacobian.":
        "Computes the camera observation Jacobian.",
    "Initializes the feature position from all observations.":
        "Initializes the feature position from all observations.",
    "Rigid 3D transformation.":
        "Rigid 3D transformation.",
    "Converts a quaternion to the corresponding rotation matrix.":
        "Converts a quaternion to the corresponding rotation matrix.",
    "This uses the formula from":
        "This uses the formula from",
    '"Indirect Kalman Filter for 3D Attitude Estimation: A Tutorial for Quaternion Algebra", equation (78).':
        '"Indirect Kalman Filter for 3D Attitude Estimation: A Tutorial for Quaternion Algebra", equation (78).',
    "The input quaternion is [q1, q2, q3, q4 (scalar part)].":
        "The input quaternion is [q1, q2, q3, q4 (scalar part)].",
    "Builds a skew-symmetric matrix from a 3D vector.":
        "Builds a skew-symmetric matrix from a 3D vector.",

    # ------------------------------------------------------------------
    # camera_model.py
    # ------------------------------------------------------------------
    "intrinsics: sequence [fx, fy, cx, cy]":
        "intrinsics: sequence [fx, fy, cx, cy]",
    "distortion_model: string such as 'radtan' or 'equidistant'":
        "distortion_model: string such as 'radtan' or 'equidistant'",
    "distortion_coeffs: array of distortion coefficients":
        "distortion_coeffs: array of distortion coefficients",
    "pts_in: source points to distort.":
        "pts_in: source points to distort.",
    "intrinsics: camera intrinsic parameters.":
        "intrinsics: camera intrinsic parameters.",
    "distortion_model: camera distortion model.":
        "distortion_model: camera distortion model.",
    "distortion_coeffs: distortion coefficients.":
        "distortion_coeffs: distortion coefficients.",
    "pts_out: distorted points. (N, 2)":
        "pts_out: distorted points. (N, 2)",

    # ------------------------------------------------------------------
    # feature_adder.py / feature_initializer.py
    # ------------------------------------------------------------------
    "config:                 configuration object with grid_num and related fields.":
        "config:                 configuration object with grid_num and related fields.",
    "cam0_curr_img_msg:      current image message":
        "cam0_curr_img_msg:      current image message",
    "curr_features:          list of [[FeatureMetaData]] to fill":
        "curr_features:          list of [[FeatureMetaData]] to fill",
    "next_feature_id:        counter used to assign ids to new features":
        "next_feature_id:        counter used to assign ids to new features",
    "grid_row/col:           grid layout":
        "grid_row/col:           grid layout",
    "grid_row, grid_col:     grid layout":
        "grid_row, grid_col:     grid layout",
    "grid_max_feature_num:   upper feature limit":
        "grid_max_feature_num:   upper feature limit",
    "grid_min_feature_num:   lower feature limit":
        "grid_min_feature_num:   lower feature limit",
    "grid_min_feature_num:   minimum number of features per grid cell":
        "grid_min_feature_num:   minimum number of features per grid cell",
    "Size of each grid cell.":
        "Size of each grid cell.",
    "Grid cell size in pixels.":
        "Grid cell size in pixels.",
    "Detects new image features to maintain a uniform feature distribution across the frame.":
        "Detects new image features to maintain a uniform feature distribution across the frame.",
    "Detects and initializes features on the first stereo frame.":
        "Detects and initializes features on the first stereo frame.",

    # ------------------------------------------------------------------
    # feature measurement / metadata / pruning
    # ------------------------------------------------------------------
    "Stereo feature measurement.":
        "Stereo feature measurement.",
    "Stores the information required for convenient feature access.":
        "Stores the information required for convenient feature access.",
    "grid_max_feature_num: maximum number of features per grid cell":
        "grid_max_feature_num: maximum number of features per grid cell",
    "Removes some features from a grid cell when there are too many,":
        "Removes some features from a grid cell when there are too many,",
    "so that the number of features in each cell stays bounded.":
        "so that the number of features in each cell stays bounded.",

    # ------------------------------------------------------------------
    # feature_publisher.py
    # ------------------------------------------------------------------
    "Uses the same distortion and intrinsic parameter layout for both cameras.":
        "Uses the same distortion and intrinsic parameter layout for both cameras.",
    "pts_in: points to undistort.":
        "pts_in: points to undistort.",
    "rectification_matrix: rectification matrix.":
        "rectification_matrix: rectification matrix.",
    "new_intrinsics: new camera intrinsic parameters.":
        "new_intrinsics: new camera intrinsic parameters.",
    "pts_out: undistorted points.":
        "pts_out: undistorted points.",
    "pts_in: points to distort.":
        "pts_in: points to distort.",
    "pts_out: distorted points. (N, 2)":
        "pts_out: distorted points. (N, 2)",
    "Publishes features in the current image, including both tracked and newly added features.":
        "Publishes features in the current image, including both tracked and newly added features.",

    # ------------------------------------------------------------------
    # feature_tracker.py
    # ------------------------------------------------------------------
    "lk_params:            dictionary for cv2.calcOpticalFlowPyrLK":
        "lk_params:            dictionary for cv2.calcOpticalFlowPyrLK",
    "imu_processor:        IMUProcessor with integrate_imu_data and R_cam?_imu fields":
        "imu_processor:        IMUProcessor with integrate_imu_data and R_cam?_imu fields",
    "stereo_matcher:       StereoMatcher with stereo_match method":
        "stereo_matcher:       StereoMatcher with stereo_match method",
    "cam?_distortion_*:    distortion model and coefficients":
        "cam?_distortion_*:    distortion model and coefficients",
    "prev_cam0_pyramid:    previous frame pyramid for the left camera":
        "prev_cam0_pyramid:    previous frame pyramid for the left camera",
    "curr_cam0_pyramid:    current frame pyramid for the left camera":
        "curr_cam0_pyramid:    current frame pyramid for the left camera",
    "prev_features:        list of FeatureMetaData lists for the previous frame":
        "prev_features:        list of FeatureMetaData lists for the previous frame",
    "curr_features:        list of FeatureMetaData lists to fill":
        "curr_features:        list of FeatureMetaData lists to fill",
    "num_features:         dictionary for recording feature counts at each stage":
        "num_features:         dictionary for recording feature counts at each stage",
    "grid_row/col:         image grid parameters":
        "grid_row/col:         image grid parameters",
    "ransac_threshold:     threshold for RANSAC filtering":
        "ransac_threshold:     threshold for RANSAC filtering",
    "Returns the (height, width) of one grid cell.":
        "Returns the (height, width) of one grid cell.",
    "Main tracking step: LK + stereo matching + RANSAC + curr_features update.":
        "Main tracking step: LK + stereo matching + RANSAC + curr_features update.",
    "# 6) Reject out-of-bounds points":
        "# 6) Reject out-of-bounds points",
    "# 7) Collect tracked points":
        "# 7) Collect tracked points",
    "# 8) Stereo matching":
        "# 8) Stereo matching",
    "# 10) Update curr_features":
        "# 10) Update curr_features",
    "Rotation compensation before tracking.":
        "Rotation compensation before tracking.",

    # ------------------------------------------------------------------
    # stereo_matcher.py
    # ------------------------------------------------------------------
    "lk_params:         dictionary for cv2.calcOpticalFlowPyrLK":
        "lk_params:         dictionary for cv2.calcOpticalFlowPyrLK",
    "imu_processor:     IMUProcessor with R_cam0_imu, R_cam1_imu, t_cam0_imu, t_cam1_imu fields and integrate_imu_data method":
        "imu_processor:     IMUProcessor with R_cam0_imu, R_cam1_imu, t_cam0_imu, t_cam1_imu fields and integrate_imu_data method",
    "pyramid_builder:   PyramidBuilder with curr_cam0_pyramid and curr_cam1_pyramid attributes":
        "pyramid_builder:   PyramidBuilder with curr_cam0_pyramid and curr_cam1_pyramid attributes",
    "camera_model:      CameraModel with undistort_points and distort_points methods":
        "camera_model:      CameraModel with undistort_points and distort_points methods",
    "stereo_threshold:  threshold for disparity and epipolar filtering":
        "stereo_threshold:  threshold for disparity and epipolar filtering",
    "Matches points from cam0 to cam1 using optical flow and stereo geometry.":
        "Matches points from cam0 to cam1 using optical flow and stereo geometry.",
    "cam0_points: list or array of (x, y) points in the left image.":
        "cam0_points: list or array of (x, y) points in the left image.",
    "cam1_points: array of matched (x, y) points in the right image.":
        "cam1_points: array of matched (x, y) points in the right image.",
    "inlier_mask: boolean array, True for valid correspondences.":
        "inlier_mask: boolean array, True for valid correspondences.",

    # ------------------------------------------------------------------
    # imu_processor.py
    # ------------------------------------------------------------------
    "T_imu_cam0: 4x4 transform matrix from the IMU frame to the camera-0 frame":
        "T_imu_cam0: 4x4 transform matrix from the IMU frame to the camera-0 frame",
    "T_imu_cam1: 4x4 transform matrix from the IMU frame to the camera-1 frame":
        "T_imu_cam1: 4x4 transform matrix from the IMU frame to the camera-1 frame",

    # ------------------------------------------------------------------
    # pipeline.py
    # ------------------------------------------------------------------
    "Forwards IMU messages to IMUProcessor.":
        "Forwards IMU messages to IMUProcessor.",
    "Processes one stereo message (cam0_msg + cam1_msg).":
        "Processes one stereo message (cam0_msg + cam1_msg).",
    "Returns a FeatureMeasurement message.":
        "Returns a FeatureMeasurement message.",
}


README_RU_ENGLISH = """# UAV-Airvision

This file previously contained the Russian README.

The project documentation is now maintained in English in `README.md`.
Please use the main README for the current setup instructions, dashboard usage,
evaluation workflow, and development notes.

Relevant documentation is also available in:

- `docs/PROJECT_STATE_AND_CLEANUP_PLAN.md`
- `docs/LIVE_SYNC_DEBUG_REPORT.md`
- `docs/PHASE6_ROOT_CAUSE_ANALYSIS_REPORT.md`
"""


TARGET_SUFFIXES = {".py", ".md", ".txt", ".bat", ".ps1"}


def should_skip(path: Path) -> bool:
    skip_parts = {
        ".git",
        ".venv",
        "__pycache__",
        ".pytest_cache",
        ".mypy_cache",
        ".ruff_cache",
        "build",
        "dist",
        "dist_installer",
        "datasets",
        "results",
    }
    return any(part in skip_parts for part in path.parts)


def apply_replacements(path: Path) -> bool:
    original = path.read_text(encoding="utf-8")

    if path.name == "README-ru.md":
        new = README_RU_ENGLISH
    else:
        new = original
        for old, replacement in REPLACEMENTS.items():
            new = new.replace(old, replacement)

    if new != original:
        path.write_text(new, encoding="utf-8")
        return True

    return False


def find_cyrillic_files():
    hits = []

    for path in sorted(ROOT.rglob("*")):
        if should_skip(path):
            continue
        if not path.is_file():
            continue
        if path.suffix.lower() not in TARGET_SUFFIXES:
            continue

        try:
            text = path.read_text(encoding="utf-8")
        except Exception:
            continue

        for line_no, line in enumerate(text.splitlines(), start=1):
            if CYRILLIC_RE.search(line):
                hits.append((path.relative_to(ROOT).as_posix(), line_no, line.rstrip()))

    return hits


def main():
    changed = []

    for path in sorted(ROOT.rglob("*")):
        if should_skip(path):
            continue
        if not path.is_file():
            continue
        if path.suffix.lower() not in TARGET_SUFFIXES:
            continue

        try:
            if apply_replacements(path):
                changed.append(path.relative_to(ROOT).as_posix())
        except UnicodeDecodeError:
            continue

    print("Changed files:")
    for item in changed:
        print(f"  {item}")

    leftovers = find_cyrillic_files()

    print()
    print(f"Remaining Cyrillic lines: {len(leftovers)}")

    report_path = ROOT / "cyrillic_remaining_after_translation.txt"
    with report_path.open("w", encoding="utf-8") as f:
        for rel, line_no, line in leftovers:
            f.write(f"{rel}:{line_no}: {line}\n")

    print(f"Remaining report: {report_path}")

    if leftovers:
        print()
        print("First remaining lines:")
        for rel, line_no, line in leftovers[:80]:
            print(f"{rel}:{line_no}: {line}")


if __name__ == "__main__":
    main()
