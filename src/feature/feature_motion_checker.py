import numpy as np
from .base_feature import BaseFeature
from .utils import Isometry3d, to_rotation

class FeatureMotionChecker:
   def check_motion(self, cam_states):
        """
        Checks whether the input camera poses have enough translation for feature triangulation.

        Args:
            cam_states: input camera poses as a dictionary <CAMStateID, CAMState>

        Returns:
            True if the translation between camera poses is sufficient for triangulation (bool)
        """
        if self.optimization_config.translation_threshold < 0:
            return True

        observation_ids = list(self.observations.keys())
        first_id = observation_ids[0]
        last_id = observation_ids[-1]

        first_cam_pose = Isometry3d(
            to_rotation(cam_states[first_id].orientation).T,
            cam_states[first_id].position)

        last_cam_pose = Isometry3d(
            to_rotation(cam_states[last_id].orientation).T,
            cam_states[last_id].position)

        feature_direction = np.array([*self.observations[first_id][:2], 1.0])
        feature_direction = feature_direction / np.linalg.norm(feature_direction)
        feature_direction = first_cam_pose.R @ feature_direction

        translation = last_cam_pose.t - first_cam_pose.t
        parallel = translation @ feature_direction
        orthogonal_translation = translation - parallel * feature_direction

        return (np.linalg.norm(orthogonal_translation) > 
            self.optimization_config.translation_threshold)