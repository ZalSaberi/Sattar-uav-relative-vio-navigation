import csv
import os
import threading
from pathlib import Path


class DiagnosticCSVLogger:
    def __init__(self):
        base = os.getenv("VIO_DIAGNOSTICS_DIR")
        if not base:
            out_dir = os.getenv("OUTPUT_DIR", os.path.join("results", "phase6a_runs"))
            dataset = os.getenv("DATASET_NAME", "unknown")
            offset = os.getenv("TIME_OFFSET", "0")
            base = os.path.join(out_dir, "diagnostics", f"{dataset}_offset{offset}")

        self.base = Path(base)
        self.base.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._headers_written = set()

    def _append(self, filename, fieldnames, row):
        path = self.base / filename
        with self._lock:
            write_header = filename not in self._headers_written and not path.exists()
            with path.open("a", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
                if write_header:
                    writer.writeheader()
                writer.writerow(row)
            self._headers_written.add(filename)

    def log_image_pipeline(self, row):
        fields = [
            "timestamp",
            "frame_kind",
            "before_tracking",
            "after_tracking",
            "after_matching",
            "after_ransac",
            "ransac_candidates",
            "ransac_inliers",
            "ransac_rejected",
            "ransac_used",
            "ransac_failed",
            "published_features",
            "next_feature_id",
            "curr_grid_nonempty",
            "curr_grid_total",
            "stereo_input",
            "stereo_lk_forward_success",
            "stereo_lk_reverse_success",
            "stereo_fb_inliers",
            "stereo_bounds_inliers",
            "stereo_epipolar_inliers",
            "stereo_output_inliers",
        ]
        self._append("image_pipeline.csv", fields, row)

    def log_msckf_gating(self, row):
        fields = [
            "timestamp",
            "context",
            "dof",
            "gamma",
            "threshold",
            "accepted",
            "r_norm",
            "H_rows",
            "H_cols",
        ]
        self._append("msckf_gating.csv", fields, row)

    def log_msckf_update(self, row):
        fields = [
            "timestamp",
            "context",
            "status",
            "H_rows",
            "H_cols",
            "r_len",
            "r_norm",
            "delta_norm",
            "delta_orientation_norm",
            "delta_velocity_norm",
            "delta_position_norm",
            "delta_gyro_bias_norm",
            "delta_acc_bias_norm",
            "num_cam_states",
            "map_features",
        ]
        self._append("msckf_update.csv", fields, row)

    def log_msckf_frame(self, row):
        fields = [
            "timestamp",
            "feature_count",
            "tracking_rate",
            "map_features",
            "cam_states",
            "imu_buffer",
            "position_x",
            "position_y",
            "position_z",
            "velocity_norm",
        ]
        self._append("msckf_frame.csv", fields, row)


_LOGGER = None


def get_diagnostic_logger():
    global _LOGGER
    enabled = os.getenv("VIO_DIAGNOSTICS", "0").lower() in ("1", "true", "yes", "on")
    if not enabled:
        return None
    if _LOGGER is None:
        _LOGGER = DiagnosticCSVLogger()
    return _LOGGER
