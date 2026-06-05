from pathlib import Path
import os
import sys
import tempfile

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.config import ConfigEuRoC
from src.feature import Feature
from src.feature.utils import Isometry3d as FeatureIsometry3d
from src.msckf import CAMState, MSCKF
from src.utils import (
    quaternion_multiplication,
    small_angle_quaternion,
    to_quaternion,
    to_rotation,
)


ATOL_FEATURE = 1e-6
RTOL_FEATURE = 1e-5
ATOL_MSCKF = 1e-5
RTOL_MSCKF = 1e-4
EPS_INV_DEPTH = 1e-6
EPS_ROT = 1e-7
EPS_POS = 1e-6


class CheckResult:
    def __init__(self, name, passed, message='', abs_err=None, rel_err=None):
        self.name = name
        self.passed = passed
        self.message = message
        self.abs_err = abs_err
        self.rel_err = rel_err

    @property
    def label(self):
        return 'PASS' if self.passed else 'FAIL'


def ensure_finite(name, value):
    arr = np.asarray(value, dtype=float)
    if not np.all(np.isfinite(arr)):
        raise AssertionError(f'{name} contains NaN/Inf: {arr}')


def max_errors(actual, expected):
    actual = np.asarray(actual, dtype=float)
    expected = np.asarray(expected, dtype=float)
    ensure_finite('actual', actual)
    ensure_finite('expected', expected)
    diff = actual - expected
    abs_err = float(np.max(np.abs(diff))) if diff.size else 0.0
    denom = max(float(np.max(np.abs(actual))) if actual.size else 0.0,
                float(np.max(np.abs(expected))) if expected.size else 0.0,
                1e-12)
    return abs_err, abs_err / denom


def is_close(actual, expected, atol, rtol):
    abs_err, rel_err = max_errors(actual, expected)
    return abs_err <= atol or rel_err <= rtol, abs_err, rel_err


def central_difference(func, x, eps):
    x = np.asarray(x, dtype=float)
    f0 = np.asarray(func(x), dtype=float)
    ensure_finite('central difference base value', f0)
    jac = np.zeros((f0.size, x.size))
    for i in range(x.size):
        dx = np.zeros_like(x)
        dx[i] = eps
        fp = np.asarray(func(x + dx), dtype=float)
        fm = np.asarray(func(x - dx), dtype=float)
        ensure_finite(f'central difference + step {i}', fp)
        ensure_finite(f'central difference - step {i}', fm)
        jac[:, i] = (fp - fm) / (2.0 * eps)
    return jac


def rotation_from_small_angle(vec):
    return to_rotation(small_angle_quaternion(np.asarray(vec, dtype=float)))


def feature_observation_residual(T_c0_ci, x, z):
    alpha, beta, rho = x
    h = T_c0_ci.R @ np.array([alpha, beta, 1.0]) + rho * T_c0_ci.t
    ensure_finite('feature observation h', h)
    if h[2] <= 1e-6:
        raise AssertionError(f'feature observation non-positive depth: {h[2]}')
    return h[:2] / h[2] - z


def check_feature_observation():
    cfg = ConfigEuRoC()
    feature = Feature(1, cfg.optimization_config)
    R = rotation_from_small_angle([0.08, -0.04, 0.03])
    T = FeatureIsometry3d(R, np.array([0.25, -0.08, 0.12]))
    x = np.array([0.08, -0.03, 0.22])
    z = np.array([0.10, -0.04])

    J, r, w = feature.jacobian(T, x, z)
    if J is None:
        return [CheckResult('FeatureObservation.jacobian finite', False,
                            'analytic Jacobian returned None for valid synthetic input')]
    ensure_finite('FeatureObservation J', J)
    ensure_finite('FeatureObservation residual', r)
    ensure_finite('FeatureObservation weight', np.array([w]))

    numeric = central_difference(
        lambda x_vec: feature_observation_residual(T, x_vec, z), x, EPS_INV_DEPTH)
    passed, abs_err, rel_err = is_close(J, numeric, ATOL_FEATURE, RTOL_FEATURE)

    cost = feature.cost(T, x, z)
    expected_cost = float(r @ r)
    cost_passed = np.isfinite(cost) and abs(cost - expected_cost) <= 1e-12

    return [
        CheckResult('FeatureObservation.cost == residual^2', cost_passed,
                    f'cost={cost:.12e}, residual^2={expected_cost:.12e}',
                    abs(cost - expected_cost), 0.0),
        CheckResult('FeatureObservation.jacobian finite difference', passed,
                    'central difference d(z_hat-z)/d(alpha,beta,rho)',
                    abs_err, rel_err),
    ]


def project_stereo(cam_q, cam_pos, feature_pos):
    R_w_c0 = to_rotation(cam_q)
    t_c0_w = cam_pos
    R_w_c1 = CAMState.R_cam0_cam1 @ R_w_c0
    t_c1_w = t_c0_w - R_w_c1.T @ CAMState.t_cam0_cam1
    p_c0 = R_w_c0 @ (feature_pos - t_c0_w)
    p_c1 = R_w_c1 @ (feature_pos - t_c1_w)
    ensure_finite('p_c0', p_c0)
    ensure_finite('p_c1', p_c1)
    if p_c0[2] <= 1e-6 or p_c1[2] <= 1e-6:
        raise AssertionError(f'non-positive stereo depth: cam0={p_c0[2]}, cam1={p_c1[2]}')
    return np.array([p_c0[0] / p_c0[2], p_c0[1] / p_c0[2],
                     p_c1[0] / p_c1[2], p_c1[1] / p_c1[2]])


def stereo_residual(cam_q, cam_pos, feature_pos, measurement):
    return measurement - project_stereo(cam_q, cam_pos, feature_pos)


def make_msckf():
    old_env = {key: os.environ.get(key) for key in
               ('OUTPUT_DIR', 'DATASET_NAME', 'TIME_OFFSET', 'APPEND_OUTPUT')}
    tmp = tempfile.TemporaryDirectory()
    os.environ['OUTPUT_DIR'] = tmp.name
    os.environ['DATASET_NAME'] = 'phase3b_synthetic'
    os.environ['TIME_OFFSET'] = '0'
    os.environ['APPEND_OUTPUT'] = '0'
    try:
        msckf = MSCKF(ConfigEuRoC())
    finally:
        for key, value in old_env.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
    msckf._phase3b_tmp = tmp
    return msckf


def make_cam_state(cam_id, rot_vec, pos):
    cam = CAMState(cam_id)
    cam.orientation = small_angle_quaternion(np.asarray(rot_vec, dtype=float))
    cam.position = np.asarray(pos, dtype=float)
    cam.orientation_null = cam.orientation.copy()
    cam.position_null = cam.position.copy()
    cam.timestamp = float(cam_id)
    return cam


def make_msckf_fixture(num_states=1):
    msckf = make_msckf()
    feature_pos = np.array([0.35, -0.12, 4.8])
    rot_vecs = [
        [0.04, -0.03, 0.02],
        [0.03, -0.02, 0.04],
        [0.05, -0.01, 0.03],
    ]
    positions = [
        [0.0, 0.0, 0.0],
        [0.12, -0.03, 0.02],
        [0.24, 0.02, -0.01],
    ]
    feature = Feature(101, msckf.optimization_config)
    feature.position = feature_pos.copy()
    feature.is_initialized = True

    for i in range(num_states):
        cam_id = i + 1
        cam = make_cam_state(cam_id, rot_vecs[i], positions[i])
        msckf.state_server.cam_states[cam_id] = cam
        feature.observations[cam_id] = project_stereo(
            cam.orientation, cam.position, feature_pos)

    msckf.map_server[feature.id] = feature
    cov_size = 21 + 6 * len(msckf.state_server.cam_states)
    msckf.state_server.state_cov = np.identity(cov_size) * 1e-3
    return msckf, feature


def camera_numeric_jacobian(cam, feature_pos, measurement):
    def residual_from_delta(delta):
        dq = small_angle_quaternion(delta[:3])
        q = quaternion_multiplication(dq, cam.orientation)
        pos = cam.position + delta[3:]
        return stereo_residual(q, pos, feature_pos, measurement)
    return central_difference(residual_from_delta, np.zeros(6), EPS_ROT)


def feature_numeric_jacobian(cam, feature_pos, measurement):
    return central_difference(
        lambda delta: stereo_residual(cam.orientation, cam.position,
                                      feature_pos + delta, measurement),
        np.zeros(3), EPS_POS)


def check_msckf_measurement_jacobian():
    msckf, feature = make_msckf_fixture(num_states=1)
    cam_id = next(iter(msckf.state_server.cam_states))
    cam = msckf.state_server.cam_states[cam_id]
    measurement = feature.observations[cam_id]
    H_x, H_f, r = msckf.measurement_jacobian(cam_id, feature.id)
    if H_x is None:
        return [CheckResult('MSCKF.measurement_jacobian finite', False,
                            'analytic Jacobian returned None for valid synthetic input')]
    ensure_finite('MSCKF H_x', H_x)
    ensure_finite('MSCKF H_f', H_f)
    ensure_finite('MSCKF residual', r)

    expected_r = stereo_residual(cam.orientation, cam.position, feature.position, measurement)
    r_passed, r_abs, r_rel = is_close(r, expected_r, 1e-12, 1e-12)

    H_x_num = camera_numeric_jacobian(cam, feature.position, measurement)
    H_f_num = feature_numeric_jacobian(cam, feature.position, measurement)
    hx_passed, hx_abs, hx_rel = is_close(H_x, H_x_num, ATOL_MSCKF, RTOL_MSCKF)
    hf_passed, hf_abs, hf_rel = is_close(H_f, H_f_num, ATOL_MSCKF, RTOL_MSCKF)

    return [
        CheckResult('MSCKF.measurement_jacobian residual', r_passed,
                    'analytic residual equals direct stereo projection residual', r_abs, r_rel),
        CheckResult('MSCKF.measurement_jacobian camera block', hx_passed,
                    'returned H_x compared with raw central finite differences wrt camera pose',
                    hx_abs, hx_rel),
        CheckResult('MSCKF.measurement_jacobian feature block', hf_passed,
                    'returned H_f compared with raw central finite differences wrt feature position',
                    hf_abs, hf_rel),
    ]


def check_nullspace_projection():
    msckf, feature = make_msckf_fixture(num_states=3)
    cam_ids = list(msckf.state_server.cam_states.keys())
    H_proj, r_proj = msckf.feature_jacobian(feature.id, cam_ids)
    ensure_finite('projected H', H_proj)
    ensure_finite('projected residual', r_proj)

    raw_rows = []
    feature_rows = []
    for cam_id in cam_ids:
        H_xi, H_fi, r_i = msckf.measurement_jacobian(cam_id, feature.id)
        if H_xi is None:
            return [CheckResult('MSCKF.feature_jacobian nullspace setup', False,
                                f'measurement_jacobian returned None for cam {cam_id}')]
        raw_rows.append(r_i)
        feature_rows.append(H_fi)
    H_fj = np.vstack(feature_rows)
    U, _, _ = np.linalg.svd(H_fj)
    A = U[:, 3:]
    orth = A.T @ H_fj
    orth_abs = float(np.max(np.abs(orth))) if orth.size else 0.0
    expected_rows = 4 * len(cam_ids) - 3
    shape_passed = H_proj.shape[0] == expected_rows and r_proj.shape[0] == expected_rows
    orth_passed = orth_abs <= 1e-8

    return [
        CheckResult('MSCKF.feature_jacobian projected dimensions', shape_passed,
                    f'H rows={H_proj.shape[0]}, r rows={r_proj.shape[0]}, expected={expected_rows}',
                    0.0 if shape_passed else float('inf'), 0.0),
        CheckResult('MSCKF.feature_jacobian nullspace orthogonality', orth_passed,
                    'A.T @ H_fj should be near zero', orth_abs, 0.0),
    ]


def print_result(result):
    pieces = [f'[{result.label}] {result.name}']
    if result.abs_err is not None:
        pieces.append(f'max_abs={result.abs_err:.3e}')
    if result.rel_err is not None:
        pieces.append(f'max_rel={result.rel_err:.3e}')
    if result.message:
        pieces.append(f'- {result.message}')
    print(' '.join(pieces))


def main():
    checks = []
    checks.extend(check_feature_observation())
    checks.extend(check_msckf_measurement_jacobian())
    checks.extend(check_nullspace_projection())

    print('Phase 3B Jacobian validation')
    print(f'Tolerances: feature atol={ATOL_FEATURE:g} rtol={RTOL_FEATURE:g}; '
          f'MSCKF atol={ATOL_MSCKF:g} rtol={RTOL_MSCKF:g}')
    print('Synthetic deterministic fixtures only; no EuRoC dataset required.')
    print()
    for result in checks:
        print_result(result)

    failed = [result for result in checks if not result.passed]
    print()
    if failed:
        print('Overall: FAIL')
        print('Suspicious Jacobian area: MSCKF.measurement_jacobian returned blocks do not match raw finite differences.'
              if any('MSCKF.measurement_jacobian' in r.name for r in failed) else
              'One or more validation blocks failed.')
        return 1

    print('Overall: PASS')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
