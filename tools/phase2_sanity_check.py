from pathlib import Path
import sys
import os
import tempfile

import numpy as np
from scipy.stats import chi2


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.image_processing.stereo_matcher import StereoMatcher
from src.image_processing.utils import clip_patch_bounds, grid_index, select
from src.config import ConfigEuRoC
from src.feature.feature_depth_estimator import FeatureDepthEstimator
from src.feature.utils import Isometry3d
from src.msckf import MSCKF


def assert_close(actual, expected, name):
    if not np.isclose(actual, expected):
        raise AssertionError(f'{name}: expected {expected}, got {actual}')


def assert_equal(actual, expected, name):
    if actual != expected:
        raise AssertionError(f'{name}: expected {expected}, got {actual}')


def assert_true(value, name):
    if not value:
        raise AssertionError(name)


def assert_false(value, name):
    if value:
        raise AssertionError(name)


def main():
    line = np.array([1.0, 2.0, 3.0])
    point = np.array([4.0, 5.0])
    expected_distance = abs(np.array([4.0, 5.0, 1.0]) @ line) / np.linalg.norm(line[:2])
    assert_close(
        StereoMatcher._point_line_distance(point, line),
        expected_distance,
        'epipolar point-line distance')

    assert_equal(
        grid_index((751.9, 479.9), (480, 752), 4, 5, 120, 151),
        19,
        'bottom-right grid index')
    assert_equal(
        grid_index((752.0, 100.0), (480, 752), 4, 5, 120, 151),
        None,
        'out-of-bounds grid index')

    assert_equal(
        clip_patch_bounds((1, 2), (8, 10), radius=3),
        (0, 6, 0, 5),
        'clipped mask bounds')

    assert_equal(
        select(['a', 'b', 'c'], np.array([[1], [0], [1]], dtype=np.uint8)),
        ['a', 'c'],
        'flattened selector handling')

    degenerate_depth = FeatureDepthEstimator().generate_initial_guess(
        Isometry3d(np.identity(3), np.zeros(3)),
        np.array([0.0, 0.0]),
        np.array([0.0, 0.0]))
    assert_equal(degenerate_depth, None, 'degenerate depth guard')

    old_env = {key: os.environ.get(key) for key in
               ('OUTPUT_DIR', 'DATASET_NAME', 'TIME_OFFSET', 'APPEND_OUTPUT')}
    with tempfile.TemporaryDirectory() as tmp:
        os.environ['OUTPUT_DIR'] = tmp
        os.environ['DATASET_NAME'] = 'sanity'
        os.environ['TIME_OFFSET'] = '0'
        os.environ['APPEND_OUTPUT'] = '0'
        msckf = MSCKF(ConfigEuRoC())
    for key, value in old_env.items():
        if value is None:
            os.environ.pop(key, None)
        else:
            os.environ[key] = value

    assert_close(
        msckf.chi_squared_test_table[5],
        chi2.ppf(0.95, 5),
        'chi-square 0.95 confidence')
    H = np.zeros((5, msckf.state_server.state_cov.shape[0]))
    assert_true(msckf.gating_test(H, np.zeros(5)), 'zero residual gate')
    assert_false(msckf.gating_test(H, np.ones(5) * 100.0), 'large residual gate')

    print('Phase 2/3A sanity checks passed')


if __name__ == '__main__':
    raise SystemExit(main())
