from pathlib import Path
import argparse
import csv
import math
import sys

import numpy as np


ESTIMATE_COLUMNS = (
    'timestamp', 'p_x', 'p_y', 'p_z', 'q_x', 'q_y', 'q_z', 'q_w')
GROUNDTRUTH_RELATIVE = Path('mav0') / 'state_groundtruth_estimate0' / 'data.csv'


class EvaluationError(RuntimeError):
    pass


def parse_args():
    parser = argparse.ArgumentParser(
        description='Evaluate a VIO trajectory against EuRoC ground truth.')
    parser.add_argument(
        '--estimate', required=True,
        help='Path to trajectory output: timestamp p_x p_y p_z q_x q_y q_z q_w.')
    gt_group = parser.add_mutually_exclusive_group(required=True)
    gt_group.add_argument(
        '--groundtruth',
        help='Path to EuRoC mav0/state_groundtruth_estimate0/data.csv.')
    gt_group.add_argument(
        '--dataset',
        help='Path to a EuRoC sequence directory containing mav0/state_groundtruth_estimate0/data.csv.')
    parser.add_argument(
        '--align', choices=('se3', 'sim3', 'translation', 'none'), default='se3',
        help='Alignment applied before ATE metrics. Default: se3.')
    parser.add_argument(
        '--max-samples', type=int, default=None,
        help='Optional cap on aligned samples after timestamp filtering.')
    parser.add_argument(
        '--csv',
        help='Optional output CSV for aligned samples and per-sample error.')
    parser.add_argument(
        '--plot',
        help='Optional output PNG with XY trajectory and translation error plots. Requires matplotlib.')
    parser.add_argument(
        '--rpe-delta-seconds', type=float, default=1.0,
        help='Compute translation RPE over this time delta in seconds. Default: 1.0.')
    parser.add_argument(
        '--no-rpe', action='store_true',
        help='Skip translation RPE computation.')
    return parser.parse_args()


def require_file(path, label):
    path = Path(path)
    if not path.exists():
        raise EvaluationError(f'{label} file does not exist: {path}')
    if not path.is_file():
        raise EvaluationError(f'{label} path is not a file: {path}')
    return path


def groundtruth_path_from_args(args):
    if args.groundtruth:
        return require_file(args.groundtruth, 'Ground truth')

    dataset = Path(args.dataset)
    if not dataset.exists():
        raise EvaluationError(f'Dataset path does not exist: {dataset}')
    if not dataset.is_dir():
        raise EvaluationError(f'Dataset path is not a directory: {dataset}')
    return require_file(dataset / GROUNDTRUTH_RELATIVE, 'Ground truth')


def seconds_from_timestamp(value):
    value = float(value)
    if abs(value) > 1e12:
        return value * 1e-9
    return value


def read_estimate(path):
    rows = []
    with path.open('r', newline='') as handle:
        for line_number, line in enumerate(handle, start=1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 8:
                raise EvaluationError(
                    f'Estimate row {line_number} has {len(parts)} columns; expected at least 8.')
            try:
                values = [float(part) for part in parts[:8]]
            except ValueError as exc:
                raise EvaluationError(
                    f'Estimate row {line_number} contains a non-numeric value: {line}') from exc
            if not np.all(np.isfinite(values)):
                raise EvaluationError(
                    f'Estimate row {line_number} contains NaN/Inf: {line}')
            rows.append(values)

    if not rows:
        raise EvaluationError(f'No estimate samples found in {path}')

    data = np.asarray(rows, dtype=float)
    data[:, 0] = np.asarray([seconds_from_timestamp(t) for t in data[:, 0]])
    data = sort_and_unique_by_time(data, 'estimate')
    return data[:, 0], data[:, 1:4], data[:, 4:8]


def read_groundtruth(path):
    rows = []
    with path.open('r', newline='') as handle:
        reader = csv.reader(handle)
        for line_number, row in enumerate(reader, start=1):
            if not row:
                continue
            first = row[0].strip()
            if not first or first.startswith('#'):
                continue
            if len(row) < 8:
                raise EvaluationError(
                    f'Ground truth row {line_number} has {len(row)} columns; expected at least 8.')
            try:
                timestamp = seconds_from_timestamp(first)
                position = [float(row[i]) for i in range(1, 4)]
                quaternion_wxyz = [float(row[i]) for i in range(4, 8)]
            except ValueError as exc:
                raise EvaluationError(
                    f'Ground truth row {line_number} contains a non-numeric value: {row}') from exc
            values = [timestamp, *position, *quaternion_wxyz]
            if not np.all(np.isfinite(values)):
                raise EvaluationError(
                    f'Ground truth row {line_number} contains NaN/Inf: {row}')
            rows.append(values)

    if not rows:
        raise EvaluationError(f'No ground-truth samples found in {path}')

    data = np.asarray(rows, dtype=float)
    data = sort_and_unique_by_time(data, 'ground truth')
    return data[:, 0], data[:, 1:4], data[:, 4:8]


def sort_and_unique_by_time(data, label):
    order = np.argsort(data[:, 0], kind='stable')
    data = data[order]
    if np.any(np.diff(data[:, 0]) < 0):
        raise EvaluationError(f'{label} timestamps could not be sorted')

    keep = np.ones(len(data), dtype=bool)
    keep[1:] = np.diff(data[:, 0]) > 0
    data = data[keep]
    if len(data) < 2:
        raise EvaluationError(f'{label} needs at least two unique timestamp samples')
    return data


def interpolate_groundtruth(gt_t, gt_p, est_t):
    valid = (est_t >= gt_t[0]) & (est_t <= gt_t[-1])
    if not np.any(valid):
        raise EvaluationError(
            'No estimate timestamps fall inside the ground-truth time range: '
            f'estimate {est_t[0]:.6f}..{est_t[-1]:.6f}, '
            f'ground truth {gt_t[0]:.6f}..{gt_t[-1]:.6f}')

    aligned_t = est_t[valid]
    aligned_gt = np.column_stack([
        np.interp(aligned_t, gt_t, gt_p[:, axis]) for axis in range(3)
    ])
    return valid, aligned_t, aligned_gt


def fit_alignment(source, target, mode):
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise EvaluationError('Alignment requires Nx3 source and target positions')

    if mode == 'none':
        return 1.0, np.identity(3), np.zeros(3)

    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean

    if mode == 'translation':
        return 1.0, np.identity(3), target_mean - source_mean

    covariance = (target_centered.T @ source_centered) / len(source)
    u, singular_values, vt = np.linalg.svd(covariance)
    sign = np.ones(3)
    if np.linalg.det(u @ vt) < 0:
        sign[-1] = -1.0
    rotation = u @ np.diag(sign) @ vt

    scale = 1.0
    if mode == 'sim3':
        source_var = np.mean(np.sum(source_centered * source_centered, axis=1))
        if source_var <= 1e-15:
            raise EvaluationError('Cannot compute Sim(3) scale for near-constant estimate trajectory')
        scale = float(np.sum(singular_values * sign) / source_var)

    translation = target_mean - scale * (rotation @ source_mean)
    return scale, rotation, translation


def apply_alignment(points, scale, rotation, translation):
    return scale * (points @ rotation.T) + translation


def compute_metrics(errors):
    norms = np.linalg.norm(errors, axis=1)
    return {
        'rmse_m': math.sqrt(float(np.mean(norms * norms))),
        'mean_m': float(np.mean(norms)),
        'median_m': float(np.median(norms)),
        'min_m': float(np.min(norms)),
        'max_m': float(np.max(norms)),
        'std_m': float(np.std(norms)),
    }, norms


def compute_rpe_translation(times, estimate_aligned, groundtruth, delta_seconds):
    if delta_seconds is None:
        return None
    if delta_seconds <= 0:
        raise EvaluationError('--rpe-delta-seconds must be positive')

    times = np.asarray(times, dtype=float)
    estimate_aligned = np.asarray(estimate_aligned, dtype=float)
    groundtruth = np.asarray(groundtruth, dtype=float)

    errors = []
    for i, timestamp in enumerate(times):
        j = int(np.searchsorted(times, timestamp + delta_seconds, side='left'))
        if j >= len(times):
            continue
        if j == i:
            continue
        estimate_delta = estimate_aligned[j] - estimate_aligned[i]
        gt_delta = groundtruth[j] - groundtruth[i]
        errors.append(estimate_delta - gt_delta)

    if not errors:
        return {
            'delta_seconds': delta_seconds,
            'pairs': 0,
            'metrics': None,
        }

    metrics, norms = compute_metrics(np.asarray(errors, dtype=float))
    return {
        'delta_seconds': delta_seconds,
        'pairs': len(errors),
        'metrics': metrics,
        'errors_m': norms,
    }


def maybe_limit_samples(times, estimate, groundtruth, max_samples):
    if max_samples is None or len(times) <= max_samples:
        return times, estimate, groundtruth
    if max_samples <= 1:
        raise EvaluationError('--max-samples must be greater than 1')
    indices = np.linspace(0, len(times) - 1, max_samples).round().astype(int)
    return times[indices], estimate[indices], groundtruth[indices]


def write_aligned_csv(path, times, estimate_aligned, groundtruth, error_norms):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow([
            'timestamp',
            'estimate_x', 'estimate_y', 'estimate_z',
            'groundtruth_x', 'groundtruth_y', 'groundtruth_z',
            'error_m',
        ])
        for row in zip(times, estimate_aligned, groundtruth, error_norms):
            timestamp, estimate, gt, error = row
            writer.writerow([
                f'{timestamp:.9f}',
                f'{estimate[0]:.9f}', f'{estimate[1]:.9f}', f'{estimate[2]:.9f}',
                f'{gt[0]:.9f}', f'{gt[1]:.9f}', f'{gt[2]:.9f}',
                f'{error:.9f}',
            ])


def write_plot(path, times, estimate_aligned, groundtruth, error_norms):
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise EvaluationError(
            '--plot requires matplotlib. Install it with: python -m pip install matplotlib') from exc

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].plot(groundtruth[:, 0], groundtruth[:, 1], label='ground truth', linewidth=1.5)
    axes[0].plot(estimate_aligned[:, 0], estimate_aligned[:, 1], label='estimate aligned', linewidth=1.2)
    axes[0].set_title('XY trajectory')
    axes[0].set_xlabel('x [m]')
    axes[0].set_ylabel('y [m]')
    axes[0].axis('equal')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(times - times[0], error_norms, linewidth=1.2)
    axes[1].set_title('Translation error')
    axes[1].set_xlabel('time since first aligned sample [s]')
    axes[1].set_ylabel('error [m]')
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def print_summary(result):
    metrics = result['ate']
    rpe = result['rpe']
    print('Trajectory evaluation')
    print(f'  estimate:    {result["estimate_path"]}')
    print(f'  groundtruth: {result["groundtruth_path"]}')
    print(f'  alignment:   {result["alignment"]}')
    print('  samples:     '
          f'estimate={result["estimate_samples"]} '
          f'groundtruth={result["groundtruth_samples"]} '
          f'aligned={result["aligned_samples"]}')
    print('  time range:  '
          f'estimate={result["estimate_start"]:.6f}..{result["estimate_end"]:.6f}')
    print('               '
          f'groundtruth={result["groundtruth_start"]:.6f}..{result["groundtruth_end"]:.6f}')
    print('               '
          f'aligned={result["aligned_start"]:.6f}..{result["aligned_end"]:.6f}')
    print('  transform estimate->groundtruth:')
    print(f'    scale: {result["scale"]:.9f}')
    translation = result['translation']
    print(f'    translation: {translation[0]:.9f} {translation[1]:.9f} {translation[2]:.9f}')
    print(f'    rotation det: {result["rotation_det"]:.9f}')
    print('  ATE translation error [m]:')
    print(f'    rmse:   {metrics["rmse_m"]:.6f}')
    print(f'    mean:   {metrics["mean_m"]:.6f}')
    print(f'    median: {metrics["median_m"]:.6f}')
    print(f'    std:    {metrics["std_m"]:.6f}')
    print(f'    min:    {metrics["min_m"]:.6f}')
    print(f'    max:    {metrics["max_m"]:.6f}')
    if rpe is not None:
        print(f'  RPE translation error over {rpe["delta_seconds"]:.3f}s [m]:')
        if rpe['metrics'] is None:
            print('    not enough aligned pairs')
        else:
            rpe_metrics = rpe['metrics']
            print(f'    pairs:  {rpe["pairs"]}')
            print(f'    rmse:   {rpe_metrics["rmse_m"]:.6f}')
            print(f'    mean:   {rpe_metrics["mean_m"]:.6f}')
            print(f'    median: {rpe_metrics["median_m"]:.6f}')
            print(f'    max:    {rpe_metrics["max_m"]:.6f}')


def evaluate_files(estimate_path, groundtruth_path, align='se3', max_samples=None,
                   rpe_delta_seconds=1.0):
    estimate_path = require_file(estimate_path, 'Estimate')
    groundtruth_path = require_file(groundtruth_path, 'Ground truth')
    estimate_t, estimate_p, _ = read_estimate(estimate_path)
    gt_t, gt_p, _ = read_groundtruth(groundtruth_path)
    valid, aligned_t, aligned_gt = interpolate_groundtruth(gt_t, gt_p, estimate_t)
    aligned_estimate = estimate_p[valid]
    aligned_t, aligned_estimate, aligned_gt = maybe_limit_samples(
        aligned_t, aligned_estimate, aligned_gt, max_samples)

    if len(aligned_t) < 3 and align in ('se3', 'sim3'):
        raise EvaluationError(
            f'Need at least 3 aligned samples for {align} alignment; got {len(aligned_t)}')

    scale, rotation, translation = fit_alignment(aligned_estimate, aligned_gt, align)
    estimate_aligned = apply_alignment(aligned_estimate, scale, rotation, translation)
    metrics, error_norms = compute_metrics(estimate_aligned - aligned_gt)
    rpe = compute_rpe_translation(
        aligned_t, estimate_aligned, aligned_gt, rpe_delta_seconds)

    return {
        'estimate_path': str(estimate_path),
        'groundtruth_path': str(groundtruth_path),
        'alignment': align,
        'estimate_samples': len(estimate_t),
        'groundtruth_samples': len(gt_t),
        'aligned_samples': len(aligned_t),
        'estimate_start': float(estimate_t[0]),
        'estimate_end': float(estimate_t[-1]),
        'groundtruth_start': float(gt_t[0]),
        'groundtruth_end': float(gt_t[-1]),
        'aligned_start': float(aligned_t[0]),
        'aligned_end': float(aligned_t[-1]),
        'scale': scale,
        'rotation_det': float(np.linalg.det(rotation)),
        'translation': translation,
        'ate': metrics,
        'rpe': rpe,
        'aligned_times': aligned_t,
        'estimate_aligned': estimate_aligned,
        'groundtruth_aligned': aligned_gt,
        'ate_errors_m': error_norms,
    }


def evaluate(args):
    estimate_path = require_file(args.estimate, 'Estimate')
    groundtruth_path = groundtruth_path_from_args(args)
    rpe_delta_seconds = None if args.no_rpe else args.rpe_delta_seconds

    result = evaluate_files(
        estimate_path, groundtruth_path, args.align, args.max_samples,
        rpe_delta_seconds)

    print_summary(result)

    if args.csv:
        write_aligned_csv(
            args.csv, result['aligned_times'], result['estimate_aligned'],
            result['groundtruth_aligned'], result['ate_errors_m'])
        print(f'  wrote CSV: {args.csv}')

    if args.plot:
        write_plot(
            args.plot, result['aligned_times'], result['estimate_aligned'],
            result['groundtruth_aligned'], result['ate_errors_m'])
        print(f'  wrote plot: {args.plot}')

    return result


def main():
    args = parse_args()
    try:
        evaluate(args)
    except EvaluationError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
