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
from src.msckf import CAMState, IMUState, MSCKF
from src.utils import (
    quaternion_multiplication,
    skew,
    small_angle_quaternion,
    to_rotation,
)


ATOL_FD = 1e-5
RTOL_FD = 1e-4
ATOL_ALG = 1e-10
ATOL_OBS = 1e-8
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
    denom = max(
        float(np.max(np.abs(actual))) if actual.size else 0.0,
        float(np.max(np.abs(expected))) if expected.size else 0.0,
        1e-12,
    )
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


def project_stereo(cam_q, cam_pos, feature_pos):
    R_w_c0 = to_rotation(cam_q)
    t_c0_w = np.asarray(cam_pos, dtype=float)
    feature_pos = np.asarray(feature_pos, dtype=float)
    R_w_c1 = CAMState.R_cam0_cam1 @ R_w_c0
    t_c1_w = t_c0_w - R_w_c1.T @ CAMState.t_cam0_cam1
    p_c0 = R_w_c0 @ (feature_pos - t_c0_w)
    p_c1 = R_w_c1 @ (feature_pos - t_c1_w)
    ensure_finite('p_c0', p_c0)
    ensure_finite('p_c1', p_c1)
    if p_c0[2] <= 1e-6 or p_c1[2] <= 1e-6:
        raise AssertionError(
            f'non-positive stereo depth: cam0={p_c0[2]}, cam1={p_c1[2]}')
    return np.array([
        p_c0[0] / p_c0[2],
        p_c0[1] / p_c0[2],
        p_c1[0] / p_c1[2],
        p_c1[1] / p_c1[2],
    ])


def stereo_residual(cam_q, cam_pos, feature_pos, measurement):
    return np.asarray(measurement, dtype=float) - project_stereo(
        cam_q, cam_pos, feature_pos)


def camera_numeric_jacobian(func, cam):
    def from_delta(delta):
        dq = small_angle_quaternion(delta[:3])
        q = quaternion_multiplication(dq, cam.orientation)
        pos = cam.position + delta[3:]
        return func(q, pos)

    return central_difference(from_delta, np.zeros(6), EPS_ROT)


def feature_numeric_jacobian(func):
    return central_difference(lambda delta: func(delta), np.zeros(3), EPS_POS)


def make_msckf():
    old_env = {key: os.environ.get(key) for key in (
        'OUTPUT_DIR', 'DATASET_NAME', 'TIME_OFFSET', 'APPEND_OUTPUT')}
    tmp = tempfile.TemporaryDirectory()
    os.environ['OUTPUT_DIR'] = tmp.name
    os.environ['DATASET_NAME'] = 'phase3e_synthetic'
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
    msckf._phase3e_tmp = tmp
    return msckf


def make_cam_state(cam_id, rot_vec, pos, null_rot_vec, null_pos):
    cam = CAMState(cam_id)
    cam.orientation = small_angle_quaternion(np.asarray(rot_vec, dtype=float))
    cam.position = np.asarray(pos, dtype=float)
    cam.orientation_null = small_angle_quaternion(
        np.asarray(null_rot_vec, dtype=float))
    cam.position_null = np.asarray(null_pos, dtype=float)
    cam.timestamp = float(cam_id)
    return cam


def make_msckf_fixture(num_states=1, measurement_offsets=True):
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
    null_rot_vecs = [
        [0.055, -0.020, 0.015],
        [0.045, -0.015, 0.035],
        [0.060, -0.005, 0.025],
    ]
    null_positions = [
        [-0.02, 0.01, 0.03],
        [0.10, -0.01, 0.04],
        [0.22, 0.04, 0.01],
    ]
    offsets = [
        [0.004, -0.003, 0.002, -0.001],
        [-0.002, 0.003, 0.001, 0.004],
        [0.003, 0.001, -0.004, 0.002],
    ]

    feature = Feature(101, msckf.optimization_config)
    feature.position = feature_pos.copy()
    feature.is_initialized = True

    for i in range(num_states):
        cam_id = i + 1
        cam = make_cam_state(
            cam_id, rot_vecs[i], positions[i],
            null_rot_vecs[i], null_positions[i])
        msckf.state_server.cam_states[cam_id] = cam
        measurement = project_stereo(cam.orientation, cam.position, feature_pos)
        if measurement_offsets:
            measurement = measurement + np.asarray(offsets[i], dtype=float)
        feature.observations[cam_id] = measurement

    msckf.map_server[feature.id] = feature
    cov_size = 21 + 6 * len(msckf.state_server.cam_states)
    msckf.state_server.state_cov = np.identity(cov_size) * 1e-3
    return msckf, feature


def raw_measurement_model_jacobians(cam, feature_pos):
    R_w_c0 = to_rotation(cam.orientation)
    t_c0_w = cam.position
    R_w_c1 = CAMState.R_cam0_cam1 @ R_w_c0
    t_c1_w = t_c0_w - R_w_c1.T @ CAMState.t_cam0_cam1

    p_w = np.asarray(feature_pos, dtype=float)
    p_c0 = R_w_c0 @ (p_w - t_c0_w)
    p_c1 = R_w_c1 @ (p_w - t_c1_w)
    ensure_finite('raw p_c0', p_c0)
    ensure_finite('raw p_c1', p_c1)
    if p_c0[2] <= 1e-6 or p_c1[2] <= 1e-6:
        raise AssertionError(
            f'non-positive raw depth: cam0={p_c0[2]}, cam1={p_c1[2]}')

    dz_dpc0 = np.zeros((4, 3))
    dz_dpc0[0, 0] = 1.0 / p_c0[2]
    dz_dpc0[1, 1] = 1.0 / p_c0[2]
    dz_dpc0[0, 2] = -p_c0[0] / (p_c0[2] * p_c0[2])
    dz_dpc0[1, 2] = -p_c0[1] / (p_c0[2] * p_c0[2])

    dz_dpc1 = np.zeros((4, 3))
    dz_dpc1[2, 0] = 1.0 / p_c1[2]
    dz_dpc1[3, 1] = 1.0 / p_c1[2]
    dz_dpc1[2, 2] = -p_c1[0] / (p_c1[2] * p_c1[2])
    dz_dpc1[3, 2] = -p_c1[1] / (p_c1[2] * p_c1[2])

    dpc0_dxc = np.zeros((3, 6))
    dpc0_dxc[:, :3] = skew(p_c0)
    dpc0_dxc[:, 3:] = -R_w_c0

    dpc1_dxc = np.zeros((3, 6))
    dpc1_dxc[:, :3] = CAMState.R_cam0_cam1 @ skew(p_c0)
    dpc1_dxc[:, 3:] = -R_w_c1

    dpc0_dpg = R_w_c0
    dpc1_dpg = R_w_c1

    H_x_raw = dz_dpc0 @ dpc0_dxc + dz_dpc1 @ dpc1_dxc
    H_f_raw = dz_dpc0 @ dpc0_dpg + dz_dpc1 @ dpc1_dpg
    ensure_finite('raw H_x', H_x_raw)
    ensure_finite('raw H_f', H_f_raw)
    return H_x_raw, H_f_raw


def fej_expected_jacobians(cam, feature_pos):
    H_x_raw, _ = raw_measurement_model_jacobians(cam, feature_pos)
    u = np.zeros(6)
    u[:3] = to_rotation(cam.orientation_null) @ IMUState.gravity
    u[3:] = skew(feature_pos - cam.position_null) @ IMUState.gravity
    ensure_finite('FEJ observability vector', u)
    denom = float(u @ u)
    if not np.isfinite(denom) or denom <= 1e-12:
        raise AssertionError(f'invalid FEJ observability vector norm: {denom}')
    H_x_fej = H_x_raw - (H_x_raw @ u)[:, None] * u / denom
    H_f_fej = -H_x_fej[:, 3:6]
    ensure_finite('FEJ H_x', H_x_fej)
    ensure_finite('FEJ H_f', H_f_fej)
    return H_x_fej, H_f_fej, u


def source_measurement_jacobian(msckf, cam_id, feature):
    H_x, H_f, residual = msckf.measurement_jacobian(cam_id, feature.id)
    if H_x is None:
        raise AssertionError('measurement_jacobian returned None for valid fixture')
    ensure_finite('source H_x', H_x)
    ensure_finite('source H_f', H_f)
    ensure_finite('source residual', residual)
    return H_x, H_f, residual


def check_local_jacobians():
    msckf, feature = make_msckf_fixture(num_states=1)
    cam_id = next(iter(msckf.state_server.cam_states))
    cam = msckf.state_server.cam_states[cam_id]
    measurement = feature.observations[cam_id]

    H_x_src, H_f_src, residual_src = source_measurement_jacobian(
        msckf, cam_id, feature)

    projection = lambda q, p: project_stereo(q, p, feature.position)
    residual_fn = lambda q, p: stereo_residual(q, p, feature.position, measurement)
    H_x_dh_num = camera_numeric_jacobian(projection, cam)
    H_x_dr_num = camera_numeric_jacobian(residual_fn, cam)
    H_f_dh_num = feature_numeric_jacobian(
        lambda delta: project_stereo(
            cam.orientation, cam.position, feature.position + delta))
    H_f_dr_num = feature_numeric_jacobian(
        lambda delta: stereo_residual(
            cam.orientation, cam.position, feature.position + delta, measurement))

    H_x_raw, H_f_raw = raw_measurement_model_jacobians(cam, feature.position)
    H_x_fej, H_f_fej, u = fej_expected_jacobians(cam, feature.position)

    residual_expected = stereo_residual(
        cam.orientation, cam.position, feature.position, measurement)
    residual_ok, residual_abs, residual_rel = is_close(
        residual_src, residual_expected, 1e-12, 1e-12)

    residual_sign_ok_x, residual_sign_abs_x, residual_sign_rel_x = is_close(
        H_x_dr_num, -H_x_dh_num, ATOL_FD, RTOL_FD)
    residual_sign_ok_f, residual_sign_abs_f, residual_sign_rel_f = is_close(
        H_f_dr_num, -H_f_dh_num, ATOL_FD, RTOL_FD)

    raw_model_ok_x, raw_model_abs_x, raw_model_rel_x = is_close(
        H_x_raw, H_x_dh_num, ATOL_FD, RTOL_FD)
    raw_model_ok_f, raw_model_abs_f, raw_model_rel_f = is_close(
        H_f_raw, H_f_dh_num, ATOL_FD, RTOL_FD)

    source_fej_ok_x, source_fej_abs_x, source_fej_rel_x = is_close(
        H_x_src, H_x_fej, ATOL_ALG, 1e-12)
    source_fej_ok_f, source_fej_abs_f, source_fej_rel_f = is_close(
        H_f_src, H_f_fej, ATOL_ALG, 1e-12)

    source_vs_res_x = max_errors(H_x_src, H_x_dr_num)
    source_vs_res_f = max_errors(H_f_src, H_f_dr_num)
    source_vs_raw_x = max_errors(H_x_src, H_x_dh_num)
    source_vs_raw_f = max_errors(H_f_src, H_f_dh_num)

    source_matches_residual = (
        source_vs_res_x[0] <= ATOL_FD or source_vs_res_x[1] <= RTOL_FD or
        source_vs_res_f[0] <= ATOL_FD or source_vs_res_f[1] <= RTOL_FD)
    source_matches_raw_model = (
        source_vs_raw_x[0] <= ATOL_FD or source_vs_raw_x[1] <= RTOL_FD or
        source_vs_raw_f[0] <= ATOL_FD or source_vs_raw_f[1] <= RTOL_FD)

    obs = H_x_src @ u
    obs_abs = float(np.max(np.abs(obs))) if obs.size else 0.0
    hf_overwrite_abs, hf_overwrite_rel = max_errors(H_f_src, -H_x_src[:, 3:6])

    return [
        CheckResult(
            'residual check: source r equals z - h(x)',
            residual_ok,
            'measurement_jacobian residual matches direct stereo residual',
            residual_abs,
            residual_rel,
        ),
        CheckResult(
            'raw residual derivative check: d(z-h)/dx = -dh/dx camera block',
            residual_sign_ok_x,
            'raw residual derivative is the negative measurement-model derivative',
            residual_sign_abs_x,
            residual_sign_rel_x,
        ),
        CheckResult(
            'raw residual derivative check: d(z-h)/dx = -dh/dx feature block',
            residual_sign_ok_f,
            'raw residual derivative is the negative measurement-model derivative',
            residual_sign_abs_f,
            residual_sign_rel_f,
        ),
        CheckResult(
            'measurement model derivative check: raw camera dh/dx',
            raw_model_ok_x,
            'unprojected analytic measurement model block matches finite differences of h(x)',
            raw_model_abs_x,
            raw_model_rel_x,
        ),
        CheckResult(
            'measurement model derivative check: raw feature dh/dx',
            raw_model_ok_f,
            'unprojected analytic measurement model block matches finite differences of h(x)',
            raw_model_abs_f,
            raw_model_rel_f,
        ),
        CheckResult(
            'current MSCKF returned H check: not raw residual derivative',
            not source_matches_residual,
            'current H should not be judged against d(z-h)/dx alone',
            max(source_vs_res_x[0], source_vs_res_f[0]),
            max(source_vs_res_x[1], source_vs_res_f[1]),
        ),
        CheckResult(
            'current MSCKF returned H check: not unprojected raw dh/dx',
            not source_matches_raw_model,
            'current H includes an observability projection, so raw dh/dx mismatch is expected',
            max(source_vs_raw_x[0], source_vs_raw_f[0]),
            max(source_vs_raw_x[1], source_vs_raw_f[1]),
        ),
        CheckResult(
            'current MSCKF returned H check: FEJ/OC camera block',
            source_fej_ok_x,
            'source H_x matches the current code-derived observability projection formula',
            source_fej_abs_x,
            source_fej_rel_x,
        ),
        CheckResult(
            'current MSCKF returned H check: FEJ/OC feature block',
            source_fej_ok_f,
            'source H_f matches -H_x[:, 3:6] after observability projection',
            source_fej_abs_f,
            source_fej_rel_f,
        ),
        CheckResult(
            'FEJ / observability consistency check: H_x @ u ~= 0',
            obs_abs <= ATOL_OBS,
            'projected camera block removes the gravity/null-state observability direction',
            obs_abs,
            0.0,
        ),
        CheckResult(
            'FEJ / observability consistency check: H_f = -H_x position block',
            hf_overwrite_abs <= ATOL_ALG,
            'translation coupling is preserved by the current H_f overwrite',
            hf_overwrite_abs,
            hf_overwrite_rel,
        ),
    ]


def check_feature_nullspace_and_update_sign():
    msckf, feature = make_msckf_fixture(num_states=3)
    cam_ids = list(msckf.state_server.cam_states.keys())
    H_proj, r_proj = msckf.feature_jacobian(feature.id, cam_ids)
    ensure_finite('projected H', H_proj)
    ensure_finite('projected residual', r_proj)

    H_x_rows = []
    H_f_rows = []
    r_rows = []
    for cam_id in cam_ids:
        H_xi, H_fi, r_i = source_measurement_jacobian(msckf, cam_id, feature)
        idx = list(msckf.state_server.cam_states.keys()).index(cam_id)
        H_x_global = np.zeros((4, 21 + 6 * len(msckf.state_server.cam_states)))
        H_x_global[:, 21 + 6 * idx:21 + 6 * (idx + 1)] = H_xi
        H_x_rows.append(H_x_global)
        H_f_rows.append(H_fi)
        r_rows.append(r_i)

    H_xj = np.vstack(H_x_rows)
    H_fj = np.vstack(H_f_rows)
    r_j = np.hstack(r_rows)
    U, _, _ = np.linalg.svd(H_fj)
    A = U[:, 3:]
    H_expected = A.T @ H_xj
    r_expected = A.T @ r_j
    orth = A.T @ H_fj
    orth_abs = float(np.max(np.abs(orth))) if orth.size else 0.0

    shape_ok = (
        H_proj.shape[0] == 4 * len(cam_ids) - 3 and
        r_proj.shape[0] == 4 * len(cam_ids) - 3 and
        H_proj.shape[1] == 21 + 6 * len(cam_ids)
    )
    H_ok, H_abs, H_rel = is_close(H_proj, H_expected, ATOL_ALG, 1e-12)
    r_ok, r_abs, r_rel = is_close(r_proj, r_expected, ATOL_ALG, 1e-12)

    P = np.identity(H_proj.shape[1]) * 1e-3
    R = np.identity(H_proj.shape[0]) * ConfigEuRoC().observation_noise
    S = H_proj @ P @ H_proj.T + R
    delta = P @ H_proj.T @ np.linalg.solve(S, r_proj)
    residual_after = r_proj - H_proj @ delta
    residual_wrong_sign = r_proj + H_proj @ delta
    norm_before = float(np.linalg.norm(r_proj))
    norm_after = float(np.linalg.norm(residual_after))
    norm_wrong = float(np.linalg.norm(residual_wrong_sign))
    update_sign_ok = norm_after < norm_before and norm_wrong > norm_before

    return [
        CheckResult(
            'nullspace projection check: projected dimensions',
            shape_ok,
            f'H rows={H_proj.shape[0]}, r rows={r_proj.shape[0]}, states={len(cam_ids)}',
            0.0 if shape_ok else float('inf'),
            0.0,
        ),
        CheckResult(
            'nullspace projection check: A.T @ H_f ~= 0',
            orth_abs <= ATOL_OBS,
            'feature-position columns are eliminated by the SVD nullspace basis',
            orth_abs,
            0.0,
        ),
        CheckResult(
            'nullspace projection check: H projection matches explicit A.T @ H_x',
            H_ok,
            'feature_jacobian uses the same projected H as the explicit reconstruction',
            H_abs,
            H_rel,
        ),
        CheckResult(
            'nullspace projection check: residual projection matches explicit A.T @ r',
            r_ok,
            'feature_jacobian uses the same projected residual as the explicit reconstruction',
            r_abs,
            r_rel,
        ),
        CheckResult(
            'filter-compatible Jacobian check: projected update sign',
            update_sign_ok,
            f'||r||={norm_before:.3e}, ||r-H*dx||={norm_after:.3e}, '
            f'||r+H*dx||={norm_wrong:.3e}',
            abs(norm_before - norm_after),
            0.0,
        ),
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
    print('Phase 3E FEJ-aware Jacobian validation')
    print(f'Tolerances: finite-difference atol={ATOL_FD:g} rtol={RTOL_FD:g}; '
          f'algebraic atol={ATOL_ALG:g}; observability atol={ATOL_OBS:g}')
    print('Synthetic deterministic fixtures only; no EuRoC dataset required.')
    print('Current-pose and frozen-null-pose values intentionally differ.')
    print()

    checks = []
    checks.extend(check_local_jacobians())
    checks.extend(check_feature_nullspace_and_update_sign())

    for result in checks:
        print_result(result)

    failed = [result for result in checks if not result.passed]
    print()
    if failed:
        print('Overall: FAIL')
        print('Source fix is not automatically justified; inspect the failed '
              'FEJ-aware block before changing MSCKF equations.')
        return 1

    print('Overall: PASS')
    print('Conclusion: the current MSCKF H is FEJ/OC-style and filter-sign '
          'compatible in this synthetic validation. No source-code Jacobian '
          'fix is justified by this tool alone.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
