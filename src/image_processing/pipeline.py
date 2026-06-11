import cv2
from collections import defaultdict
from ..diagnostics import get_diagnostic_logger

from .imu_processor import IMUProcessor
from .pyramid_builder import PyramidBuilder
from .camera_model import CameraModel
from .stereo_matcher import StereoMatcher
from .feature_initializer import FeatureInitializer
from .feature_tracker import FeatureTracker
from .feature_adder import FeatureAdder
from .feature_pruner import FeaturePruner
from .feature_publisher import FeaturePublisher

class ImageProcessingPipeline:
    def __init__(self, config):
        self.config = config
        self.prev_cam0_msg = None
        
        self.imu_processor = IMUProcessor(
            config.T_imu_cam0, config.T_imu_cam1
        )

        self.detector = cv2.FastFeatureDetector_create(
            config.fast_threshold
        )

        self.cam0_camera_model = CameraModel(
            config.cam0_intrinsics,
            config.cam0_distortion_model,
            config.cam0_distortion_coeffs
        )
        self.cam1_camera_model = CameraModel(
            config.cam1_intrinsics,
            config.cam1_distortion_model,
            config.cam1_distortion_coeffs
        )

        self.next_feature_id = 0
        self.prev_features   = [[] for _ in range(config.grid_num)]
        self.curr_features   = [[] for _ in range(config.grid_num)]
        self.num_features    = defaultdict(int)
        self.diagnostic_logger = get_diagnostic_logger()

        self.first_frame = True

        self.prev_pyr0 = None

    def imu_callback(self, imu_msg):
        """Передаёт IMU-сообщения в IMUProcessor."""
        self.imu_processor.imu_callback(imu_msg)

    def stereo_callback(self, stereo_msg):
        """
        Обрабатывает одно стереосообщение (cam0_msg + cam1_msg).
        Возвращает сообщение FeatureMeasurement.
        """

        self.imu_processor.cam0_prev_img_msg = self.prev_cam0_msg
        self.imu_processor.cam0_curr_img_msg = stereo_msg.cam0_msg

        cam0_msg, cam1_msg = stereo_msg.cam0_msg, stereo_msg.cam1_msg

        pyramid_builder = PyramidBuilder(
            self.config.win_size,
            self.config.pyramid_levels,
            cam0_msg,
            cam1_msg
        )
        pyr0, pyr1 = pyramid_builder.create_image_pyramids()

        stereo_matcher = StereoMatcher(
            self.config.lk_params,
            self.imu_processor,
            pyramid_builder,
            self.cam0_camera_model,
            self.cam1_camera_model,
            self.config.stereo_threshold
        )

        frame_kind = "initialization" if self.first_frame else "tracking"

        if self.first_frame:
            initializer = FeatureInitializer(
                detector          = self.detector,
                stereo_matcher    = stereo_matcher,
                config            = self.config,
                cam0_curr_img_msg = cam0_msg,
                curr_features     = self.curr_features,
                next_feature_id   = self.next_feature_id,
                grid_row          = self.config.grid_row,
                grid_col          = self.config.grid_col,
                grid_min_feature_num = self.config.grid_min_feature_num
            )
            initializer.initialize_first_frame()
            self.next_feature_id = initializer.next_feature_id
            self.first_frame = False

        else:
            tracker = FeatureTracker(
                lk_params           = self.config.lk_params,
                imu_processor       = self.imu_processor,
                stereo_matcher      = stereo_matcher,
                cam0_intrinsics     = self.config.cam0_intrinsics,
                cam0_distortion_model  = self.config.cam0_distortion_model,
                cam0_distortion_coeffs = self.config.cam0_distortion_coeffs,
                cam1_intrinsics     = self.config.cam1_intrinsics,
                cam1_distortion_model  = self.config.cam1_distortion_model,
                cam1_distortion_coeffs = self.config.cam1_distortion_coeffs,
                prev_cam0_pyramid   = self.prev_pyr0,
                curr_cam0_pyramid   = pyr0,
                prev_features       = self.prev_features,
                curr_features       = self.curr_features,
                num_features        = self.num_features,
                grid_row            = self.config.grid_row,
                grid_col            = self.config.grid_col,
                ransac_threshold    = self.config.ransac_threshold
            )
            tracker.track_features()

            adder = FeatureAdder(
                detector            = self.detector,
                stereo_matcher      = stereo_matcher,
                config              = self.config,
                cam0_curr_img_msg   = cam0_msg,
                curr_features       = self.curr_features,
                next_feature_id     = self.next_feature_id,
                grid_row            = self.config.grid_row,
                grid_col            = self.config.grid_col,
                grid_max_feature_num = self.config.grid_max_feature_num,
                grid_min_feature_num = self.config.grid_min_feature_num
            )
            adder.add_new_features()
            self.next_feature_id = adder.next_feature_id

            pruner = FeaturePruner(self.config.grid_max_feature_num)
            pruner.curr_features = self.curr_features
            pruner.config        = self.config
            pruner.prune_features()

        publisher = FeaturePublisher(
            self.config.cam0_intrinsics,
            self.config.cam0_distortion_model,
            self.config.cam0_distortion_coeffs,
            self.config.cam1_intrinsics,
            self.config.cam1_distortion_model,
            self.config.cam1_distortion_coeffs
        )
        publisher.cam0_curr_img_msg = cam0_msg
        publisher.cam1_curr_img_msg = cam1_msg
        publisher.curr_features     = self.curr_features

        feature_msg = publisher.publish()

        if self.diagnostic_logger is not None:
            stereo_stats = getattr(stereo_matcher, "last_stats", {}) or {}
            curr_flat = [f for cell in self.curr_features for f in cell]
            self.diagnostic_logger.log_image_pipeline({
                "timestamp": cam0_msg.timestamp,
                "frame_kind": frame_kind,
                "before_tracking": int(self.num_features.get("before_tracking", 0)),
                "after_tracking": int(self.num_features.get("after_tracking", 0)),
                "after_matching": int(self.num_features.get("after_matching", 0)),
                "after_ransac": int(self.num_features.get("after_ransac", 0)),
                "published_features": int(len(feature_msg.features)),
                "next_feature_id": int(self.next_feature_id),
                "curr_grid_nonempty": int(sum(1 for cell in self.curr_features if len(cell) > 0)),
                "curr_grid_total": int(len(self.curr_features)),
                "stereo_input": int(stereo_stats.get("input", 0)),
                "stereo_lk_forward_success": int(stereo_stats.get("lk_forward_success", 0)),
                "stereo_lk_reverse_success": int(stereo_stats.get("lk_reverse_success", 0)),
                "stereo_fb_inliers": int(stereo_stats.get("fb_inliers", 0)),
                "stereo_bounds_inliers": int(stereo_stats.get("bounds_inliers", 0)),
                "stereo_epipolar_inliers": int(stereo_stats.get("epipolar_inliers", 0)),
                "stereo_output_inliers": int(stereo_stats.get("output_inliers", 0)),
            })

        self.prev_cam0_msg = cam0_msg
        self.prev_features = self.curr_features
        self.curr_features = [[] for _ in range(self.config.grid_num)]
        self.prev_pyr0     = pyr0

        return feature_msg
