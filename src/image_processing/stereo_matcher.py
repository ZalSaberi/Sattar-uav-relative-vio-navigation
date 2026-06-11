import numpy as np
import cv2

from .utils import skew

class StereoMatcher:
    def __init__(self,
                 lk_params,
                 imu_processor,
                 pyramid_builder,
                 cam0_camera_model,
                 cam1_camera_model,
                 stereo_threshold):
        """
        lk_params:         dict для cv2.calcOpticalFlowPyrLK
        imu_processor:     IMUProcessor с полями R_cam0_imu, R_cam1_imu, t_cam0_imu, t_cam1_imu и методом integrate_imu_data
        pyramid_builder:   PyramidBuilder с атрибутами curr_cam0_pyramid, curr_cam1_pyramid
        camera_model:      CameraModel с методами undistort_points и distort_points
        stereo_threshold:  порог для disparity и эпиполярного фильтра
        """
        self.lk_params       = lk_params
        self.integrate_imu   = imu_processor.integrate_imu_data
        self.R_cam0_imu      = imu_processor.R_cam0_imu
        self.R_cam1_imu      = imu_processor.R_cam1_imu
        self.t_cam0_imu      = imu_processor.t_cam0_imu
        self.t_cam1_imu      = imu_processor.t_cam1_imu

        self.pyr0            = pyramid_builder.curr_cam0_pyramid
        self.pyr1            = pyramid_builder.curr_cam1_pyramid

        self.cam0_model      = cam0_camera_model
        self.cam1_model      = cam1_camera_model
        self.stereo_threshold= stereo_threshold
        self.last_stats      = {}

    @staticmethod
    def _point_line_distance(point, line):
        denom = np.linalg.norm(line[:2])
        if denom <= 1e-12:
            return float('inf')
        point_h = np.array([point[0], point[1], 1.0])
        return abs(point_h @ line) / denom

    def stereo_match(self, cam0_points):
        """
        Сопоставляет точки из cam0 с cam1 с помощью оптического потока и стерео-геометрии.

        Аргументы:
            cam0_points: список или массив точек (x, y) на левом изображении.

        Возвращает:
            cam1_points: массив подобранных точек (x, y) на правом изображении.
            inlier_mask: булевый массив, True для валидных соответствий.
        """
        if len(cam0_points) == 0:
            self.last_stats = {
                "input": 0,
                "lk_forward_success": 0,
                "lk_reverse_success": 0,
                "fb_inliers": 0,
                "bounds_inliers": 0,
                "epipolar_inliers": 0,
                "output_inliers": 0,
            }
            return np.array([]), np.array([], dtype=bool)

        pts0 = np.asarray(cam0_points, dtype=np.float32).reshape(-1, 2)
        self.last_stats = {
            "input": int(len(pts0)),
            "lk_forward_success": 0,
            "lk_reverse_success": 0,
            "fb_inliers": 0,
            "bounds_inliers": 0,
            "epipolar_inliers": 0,
            "output_inliers": 0,
        }

        R0to1 = self.R_cam1_imu.T @ self.R_cam0_imu
        und0 = self.cam0_model.undistort_points(
            pts0, self.cam0_model.intrinsics,
            self.cam0_model.distortion_model,
            self.cam0_model.distortion_coeffs,
            rectification_matrix=R0to1
        )

        proj1 = self.cam1_model.distort_points(
            und0, self.cam1_model.intrinsics,
            self.cam1_model.distortion_model,
            self.cam1_model.distortion_coeffs)
        proj1 = np.asarray(proj1, dtype=np.float32).reshape(-1, 2)

        p1, track_mask, _ = cv2.calcOpticalFlowPyrLK(
            self.pyr0, self.pyr1,
            pts0.reshape(-1, 1, 2), proj1.reshape(-1, 1, 2),
            **self.lk_params
        )
        if p1 is None or track_mask is None:
            return proj1, np.zeros(len(pts0), dtype=bool)

        p1 = np.asarray(p1, dtype=np.float32).reshape(-1, 2)
        track_mask = track_mask.reshape(-1).astype(bool)
        self.last_stats["lk_forward_success"] = int(np.sum(track_mask))

        p0r, rev_mask, _ = cv2.calcOpticalFlowPyrLK(
            self.pyr1, self.pyr0,
            p1.reshape(-1, 1, 2), pts0.reshape(-1, 1, 2).copy(),
            **self.lk_params
        )
        if p0r is None or rev_mask is None:
            p0r = np.full_like(pts0, np.nan)
            rev_mask = np.zeros(len(pts0), dtype=bool)
        else:
            p0r = np.asarray(p0r, dtype=np.float32).reshape(-1, 2)
            rev_mask = rev_mask.reshape(-1).astype(bool)

        self.last_stats["lk_reverse_success"] = int(np.sum(rev_mask))

        err = np.linalg.norm(pts0 - p0r, axis=1)
        disp = np.abs(proj1[:,1] - p1[:,1])

        inlier = (track_mask &
                  rev_mask &
                  np.isfinite(err) &
                  (err < 3) &
                  (disp < 20))
        self.last_stats["fb_inliers"] = int(np.sum(inlier))

        h, w = self.pyr1.shape[:2]
        for i, pt in enumerate(p1):
            if not inlier[i]:
                continue
            x,y = pt
            if not np.isfinite(pt).all() or x<0 or x>=w or y<0 or y>=h:
                inlier[i] = False

        self.last_stats["bounds_inliers"] = int(np.sum(inlier))

        t01 = self.R_cam1_imu.T @ (self.t_cam0_imu - self.t_cam1_imu)
        E = skew(t01) @ R0to1

        undist0 = self.cam0_model.undistort_points(
            pts0, self.cam0_model.intrinsics,
            self.cam0_model.distortion_model,
            self.cam0_model.distortion_coeffs
        )
        undist1 = self.cam1_model.undistort_points(
            p1, self.cam1_model.intrinsics,
            self.cam1_model.distortion_model,
            self.cam1_model.distortion_coeffs
        )
        norm_unit = 4.0 / (
            self.cam0_model.intrinsics[0] + self.cam0_model.intrinsics[1] +
            self.cam1_model.intrinsics[0] + self.cam1_model.intrinsics[1])
        for i, (u0, u1) in enumerate(zip(undist0, undist1)):
            if not inlier[i]:
                continue
            pt0_h = np.array([u0[0], u0[1], 1.0])
            line = E @ pt0_h
            err_epi = self._point_line_distance(u1, line)
            if err_epi > self.stereo_threshold * norm_unit:
                inlier[i] = False

        self.last_stats["epipolar_inliers"] = int(np.sum(inlier))
        self.last_stats["output_inliers"] = int(np.sum(inlier))

        return p1, inlier
