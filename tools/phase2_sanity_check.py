from pathlib import Path
import sys

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.image_processing.stereo_matcher import StereoMatcher
from src.image_processing.utils import clip_patch_bounds, grid_index, select


def assert_close(actual, expected, name):
    if not np.isclose(actual, expected):
        raise AssertionError(f'{name}: expected {expected}, got {actual}')


def assert_equal(actual, expected, name):
    if actual != expected:
        raise AssertionError(f'{name}: expected {expected}, got {actual}')


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

    print('Phase 2 sanity checks passed')


if __name__ == '__main__':
    raise SystemExit(main())
