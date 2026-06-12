import numpy as np
import cv2
from itertools import chain

from .feature_meta_data import FeatureMetaData
from .utils import grid_index, select

class FeatureTracker:
    def __init__(self,
                 lk_params,
                 imu_processor,
                 stereo_matcher,
                 cam0_intrinsics,
                 cam0_distortion_model,
                 cam0_distortion_coeffs,
                 cam1_intrinsics,
                 cam1_distortion_model,
                 cam1_distortion_coeffs,
                 prev_cam0_pyramid,
                 curr_cam0_pyramid,
                 prev_features,
                 curr_features,
                 num_features,
                 grid_row,
                 grid_col,
                 ransac_threshold):
        """
        lk_params:            dictionary for cv2.calcOpticalFlowPyrLK
        imu_processor:        IMUProcessor with integrate_imu_data and R_cam?_imu fields
        stereo_matcher:       StereoMatcher with stereo_match method
        cam?_intrinsics:      vec4 [fx, fy, cx, cy]
        cam?_distortion_*:    distortion model and coefficients
        prev_cam0_pyramid:    previous frame pyramid for the left camera
        curr_cam0_pyramid:    current frame pyramid for the left camera
        prev_features:        list of FeatureMetaData lists for the previous frame
        curr_features:        list of FeatureMetaData lists to fill
        num_features:         dictionary for recording feature counts at each stage
        grid_row/col:         image grid parameters
        ransac_threshold:     threshold for RANSAC filtering
        """
        self.lk_params            = lk_params
        self.integrate_imu_data   = imu_processor.integrate_imu_data
        self.R_cam0_imu           = imu_processor.R_cam0_imu
        self.R_cam1_imu           = imu_processor.R_cam1_imu
        self.stereo_match         = stereo_matcher.stereo_match

        self.cam0_intrinsics      = cam0_intrinsics
        self.cam0_dist_model      = cam0_distortion_model
        self.cam0_dist_coeffs     = cam0_distortion_coeffs
        self.cam1_intrinsics      = cam1_intrinsics
        self.cam1_dist_model      = cam1_distortion_model
        self.cam1_dist_coeffs     = cam1_distortion_coeffs

        self.prev_cam0_pyramid    = prev_cam0_pyramid
        self.curr_cam0_pyramid    = curr_cam0_pyramid

        self.prev_features        = prev_features
        self.curr_features        = curr_features
        self.num_features         = num_features

        self.grid_row             = grid_row
        self.grid_col             = grid_col
        self.ransac_threshold     = ransac_threshold

    def get_grid_size(self, img):
        """
        Returns the (height, width) of one grid cell.
        """
        h, w = img.shape[:2]
        grid_h = int(np.ceil(h / self.grid_row))
        grid_w = int(np.ceil(w / self.grid_col))
        return grid_h, grid_w

    def track_features(self):
        """
        Main tracking step: LK + stereo matching + RANSAC + curr_features update.
        """
        # 1) Сетка по размеру изображения (из пирамиды берём форму)
        img = self.curr_cam0_pyramid
        grid_h, grid_w = self.get_grid_size(img)

        # 2) Предсказание поворота из IMU
        cam0_R_p_c, cam1_R_p_c = self.integrate_imu_data()

        # 3) Собираем прошлые точки
        prev_ids, prev_lifetime = [], []
        prev_cam0_pts, prev_cam1_pts = [], []
        for f in chain.from_iterable(self.prev_features):
            prev_ids.append(f.id)
            prev_lifetime.append(f.lifetime)
            prev_cam0_pts.append(f.cam0_point)
            prev_cam1_pts.append(f.cam1_point)
        prev_cam0_pts = np.array(prev_cam0_pts, dtype=np.float32)

        # 4) Записываем число до трекинга
        self.num_features['before_tracking'] = len(prev_cam0_pts)
        if len(prev_cam0_pts) == 0:
            return

        # 5) Предсказание положения + LK-tracker
        pred_pts = self.predict_feature_tracking(prev_cam0_pts, cam0_R_p_c, self.cam0_intrinsics)
        curr_pts, track_mask, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_cam0_pyramid,
            self.curr_cam0_pyramid,
            prev_cam0_pts,
            pred_pts,
            **self.lk_params
        )
        if curr_pts is None or track_mask is None:
            self.num_features['after_tracking'] = 0
            return

        curr_pts = np.asarray(curr_pts, dtype=np.float32).reshape(-1, 2)
        track_mask = track_mask.reshape(-1).astype(bool)

        # 6) Reject out-of-bounds points
        for i, p in enumerate(curr_pts):
            if not track_mask[i]:
                continue
            if not np.isfinite(p).all() or p[0] < 0 or p[0] >= img.shape[1] or p[1] < 0 or p[1] >= img.shape[0]:
                track_mask[i] = False

        # 7) Collect tracked points
        prev_tr_ids    = select(prev_ids, track_mask)
        prev_tr_life   = select(prev_lifetime, track_mask)
        prev_tr_cam0   = select(prev_cam0_pts, track_mask)
        prev_tr_cam1   = select(prev_cam1_pts, track_mask)
        curr_tr_cam0   = select(curr_pts, track_mask)
        self.num_features['after_tracking'] = len(curr_tr_cam0)

        # 8) Stereo matching
        curr_cam1_pts, match_mask = self.stereo_match(curr_tr_cam0)
        match_mask = np.asarray(match_mask).reshape(-1).astype(bool)
        pm_ids   = select(prev_tr_ids,    match_mask)
        pm_life  = select(prev_tr_life,   match_mask)
        pm_cam0  = select(prev_tr_cam0,   match_mask)
        pm_cam1  = select(prev_tr_cam1,   match_mask)
        cm_cam0  = select(curr_tr_cam0,   match_mask)
        cm_cam1  = select(curr_cam1_pts,  match_mask)
        self.num_features['after_matching'] = len(cm_cam0)

        cam0_inls = [1] * len(pm_cam0)
        cam1_inls = [1] * len(pm_cam1)

        # 10) Update curr_features
        cnt = 0
        for i in range(len(cam0_inls)):
            if not (cam0_inls[i] and cam1_inls[i]):
                continue
            pt = cm_cam0[i]
            idx = grid_index(pt, img.shape, self.grid_row, self.grid_col, grid_h, grid_w)
            if idx is None:
                continue

            fm = FeatureMetaData()
            fm.id         = pm_ids[i]
            fm.lifetime   = pm_life[i] + 1
            fm.cam0_point = cm_cam0[i]
            fm.cam1_point = cm_cam1[i]

            self.curr_features[idx].append(fm)
            cnt += 1

        self.num_features['after_ransac'] = cnt

    def predict_feature_tracking(self, input_pts, R_p_c, intrinsics):
        """
        Rotation compensation before tracking.
        """
        if len(input_pts) == 0:
            return np.array([], dtype=np.float32)

        K = np.array([
            [intrinsics[0], 0.0, intrinsics[2]],
            [0.0, intrinsics[1], intrinsics[3]],
            [0.0, 0.0, 1.0]
        ])
        H = K @ R_p_c @ np.linalg.inv(K)

        pts = []
        for p in input_pts:
            h = H @ np.array([p[0], p[1], 1.0])
            pts.append([h[0]/h[2], h[1]/h[2]])
        return np.array(pts, dtype=np.float32)
