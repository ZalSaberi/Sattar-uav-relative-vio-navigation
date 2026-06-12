import numpy as np
from itertools import chain
from .feature_meta_data import FeatureMetaData
from .utils import grid_index

class FeatureInitializer:
    def __init__(self,
                 detector,
                 stereo_matcher,
                 config,
                 cam0_curr_img_msg,
                 curr_features,
                 next_feature_id,
                 grid_row,
                 grid_col,
                 grid_min_feature_num):
        """
        detector:               FastFeatureDetector
        stereo_matcher:         StereoMatcher
        config:                 configuration object with grid_num and related fields.
        cam0_curr_img_msg:      current image message
        curr_features:          list of [[FeatureMetaData]] to fill
        next_feature_id:        counter used to assign ids to new features
        grid_row, grid_col:     grid layout
        grid_min_feature_num:   minimum number of features per grid cell
        """
        self.detector            = detector
        self.stereo_match        = stereo_matcher.stereo_match
        self.config              = config
        self.cam0_curr_img_msg   = cam0_curr_img_msg
        self.curr_features       = curr_features
        self.next_feature_id     = next_feature_id
        self.grid_row            = grid_row
        self.grid_col            = grid_col
        self.grid_min_feature_num= grid_min_feature_num

    def get_grid_size(self, img):
        """
        Grid cell size in pixels.
        """
        h, w = img.shape[:2]
        grid_h = int(np.ceil(h / self.grid_row))
        grid_w = int(np.ceil(w / self.grid_col))
        return grid_h, grid_w

    def initialize_first_frame(self):
        """
        Detects and initializes features on the first stereo frame.
        """
        img = self.cam0_curr_img_msg.image
        grid_height, grid_width = self.get_grid_size(img)

        kps = self.detector.detect(img)
        cam0_points = [kp.pt for kp in kps]

        cam1_points, inlier_mask = self.stereo_match(cam0_points)
        inlier_mask = np.asarray(inlier_mask).reshape(-1).astype(bool)

        cam0_inliers = []
        cam1_inliers = []
        responses    = []
        for i, ok in enumerate(inlier_mask):
            if not ok:
                continue
            cam0_inliers.append(cam0_points[i])
            cam1_inliers.append(cam1_points[i])
            responses.append(kps[i].response)

        grid_feats = [[] for _ in range(self.config.grid_num)]
        for pt0, pt1, resp in zip(cam0_inliers, cam1_inliers, responses):
            idx = grid_index(pt0, img.shape, self.grid_row, self.grid_col, grid_height, grid_width)
            if idx is None:
                continue

            fm = FeatureMetaData()
            fm.response   = resp
            fm.cam0_point = pt0
            fm.cam1_point = pt1
            grid_feats[idx].append(fm)

        for idx, feats in enumerate(grid_feats):
            top_feats = sorted(feats, key=lambda f: f.response, reverse=True)[:self.grid_min_feature_num]
            for f in top_feats:
                f.id       = self.next_feature_id
                f.lifetime = 1
                self.curr_features[idx].append(f)
                self.next_feature_id += 1
