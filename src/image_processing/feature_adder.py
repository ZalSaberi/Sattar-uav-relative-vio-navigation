import numpy as np
import cv2

from itertools import chain
from .feature_meta_data import FeatureMetaData
from .utils import clip_patch_bounds, grid_index

class FeatureAdder:
    def __init__(self,
                 detector,
                 stereo_matcher,
                 config,
                 cam0_curr_img_msg,
                 curr_features,
                 next_feature_id,
                 grid_row,
                 grid_col,
                 grid_max_feature_num,
                 grid_min_feature_num):
        """
        detector:               FastFeatureDetector
        stereo_matcher:         StereoMatcher
        config:                 configuration object with grid_num and related fields.
        cam0_curr_img_msg:      current image message
        curr_features:          list of [[FeatureMetaData]] to fill
        next_feature_id:        counter used to assign ids to new features
        grid_row/col:           grid layout
        grid_max_feature_num:   upper feature limit
        grid_min_feature_num:   lower feature limit
        """
        self.detector          = detector
        self.stereo_matcher    = stereo_matcher
        self.stereo_match      = stereo_matcher.stereo_match

        self.config            = config
        self.cam0_curr_img_msg = cam0_curr_img_msg
        self.curr_features     = curr_features
        self.next_feature_id   = next_feature_id

        self.grid_row          = grid_row
        self.grid_col          = grid_col
        self.grid_max_feature_num = grid_max_feature_num
        self.grid_min_feature_num = grid_min_feature_num

    def get_grid_size(self, img):
        """
        Size of each grid cell.
        """
        grid_height = int(np.ceil(img.shape[0] / self.grid_row))
        grid_width  = int(np.ceil(img.shape[1] / self.grid_col))
        return grid_height, grid_width
    
    def add_new_features(self):
        """
        Detects new image features to maintain a uniform feature distribution across the frame.
        """
        curr_img = self.cam0_curr_img_msg.image
        grid_height, grid_width = self.get_grid_size(curr_img)

        mask = np.ones(curr_img.shape[:2], dtype='uint8')
        for feature in chain.from_iterable(self.curr_features):
            bounds = clip_patch_bounds(feature.cam0_point, curr_img.shape, radius=3)
            if bounds is None:
                continue
            y0, y1, x0, x1 = bounds
            mask[y0:y1, x0:x1] = 0

        new_features = self.detector.detect(curr_img, mask=mask)

        new_feature_sieve = [[] for _ in range(self.config.grid_num)]
        for kp in new_features:
            code = grid_index(kp.pt, curr_img.shape, self.grid_row, self.grid_col, grid_height, grid_width)
            if code is None:
                continue
            new_feature_sieve[code].append(kp)

        new_features = []
        for cell in new_feature_sieve:
            if len(cell) > self.grid_max_feature_num:
                cell = sorted(cell, key=lambda x: x.response, reverse=True)[:self.grid_max_feature_num]
            new_features.extend(cell)

        cam0_points = [kp.pt for kp in new_features]
        cam1_points, inlier_markers = self.stereo_match(cam0_points)

        cam0_inliers, cam1_inliers, response_inliers = [], [], []
        for i, ok in enumerate(inlier_markers):
            if not ok:
                continue
            cam0_inliers.append(cam0_points[i])
            cam1_inliers.append(cam1_points[i])
            response_inliers.append(new_features[i].response)

        grid_new_features = [[] for _ in range(self.config.grid_num)]
        for pt0, pt1, resp in zip(cam0_inliers, cam1_inliers, response_inliers):
            code = grid_index(pt0, curr_img.shape, self.grid_row, self.grid_col, grid_height, grid_width)
            if code is None:
                continue

            fm = FeatureMetaData()
            fm.response   = resp
            fm.cam0_point = pt0
            fm.cam1_point = pt1
            grid_new_features[code].append(fm)

        for i, feats in enumerate(grid_new_features):
            top = sorted(feats, key=lambda x: x.response, reverse=True)[:self.grid_min_feature_num]
            for f in top:
                f.id       = self.next_feature_id
                f.lifetime = 1
                self.curr_features[i].append(f)
                self.next_feature_id += 1
