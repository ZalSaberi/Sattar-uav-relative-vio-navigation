from pathlib import Path
import argparse
import sys

import numpy as np


TOOL_DIR = Path(__file__).resolve().parent
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

from evaluate_trajectory import (  # noqa: E402
    EvaluationError,
    GROUNDTRUTH_RELATIVE,
    evaluate_files,
    require_file,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Create an offline trajectory visualization from a project estimate and EuRoC ground truth.')
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
        '--output',
        help='PNG output path. If omitted, the tool prints metrics only and writes no files.')
    parser.add_argument(
        '--align', choices=('se3', 'translation', 'none', 'sim3'), default='se3',
        help='Alignment mode. Default: se3.')
    parser.add_argument(
        '--max-samples', type=int, default=None,
        help='Optional cap on aligned samples after timestamp filtering.')
    parser.add_argument(
        '--components', action='store_true',
        help='Also plot X/Y/Z positions over time.')
    return parser.parse_args()


def groundtruth_path_from_args(args):
    if args.groundtruth:
        return require_file(args.groundtruth, 'Ground truth')

    dataset = Path(args.dataset)
    if not dataset.exists():
        raise EvaluationError(f'Dataset path does not exist: {dataset}')
    if not dataset.is_dir():
        raise EvaluationError(f'Dataset path is not a directory: {dataset}')
    return require_file(dataset / GROUNDTRUTH_RELATIVE, 'Ground truth')


def write_plot(output_path, result, include_components=False):
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise EvaluationError(
            '--output requires matplotlib. Install it with: python -m pip install matplotlib') from exc

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    times = result['aligned_times']
    time_rel = times - times[0]
    estimate = result['estimate_aligned']
    groundtruth = result['groundtruth_aligned']
    errors = result['ate_errors_m']

    if include_components:
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        ax_xy = axes[0, 0]
        ax_error = axes[0, 1]
        ax_components = axes[1, 0]
        axes[1, 1].axis('off')
    else:
        fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
        ax_xy, ax_error = axes
        ax_components = None

    ax_xy.plot(groundtruth[:, 0], groundtruth[:, 1], label='ground truth', linewidth=1.5)
    ax_xy.plot(estimate[:, 0], estimate[:, 1], label='estimate aligned', linewidth=1.2)
    ax_xy.set_title('XY trajectory')
    ax_xy.set_xlabel('x [m]')
    ax_xy.set_ylabel('y [m]')
    ax_xy.axis('equal')
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend()

    ax_error.plot(time_rel, errors, linewidth=1.2)
    ax_error.set_title('Translation error')
    ax_error.set_xlabel('time [s]')
    ax_error.set_ylabel('error [m]')
    ax_error.grid(True, alpha=0.3)

    if ax_components is not None:
        labels = ('x', 'y', 'z')
        for axis, label in enumerate(labels):
            ax_components.plot(time_rel, groundtruth[:, axis], label=f'gt {label}', linewidth=1.2)
            ax_components.plot(time_rel, estimate[:, axis], '--', label=f'est {label}', linewidth=1.0)
        ax_components.set_title('Position components')
        ax_components.set_xlabel('time [s]')
        ax_components.set_ylabel('position [m]')
        ax_components.grid(True, alpha=0.3)
        ax_components.legend(ncol=2, fontsize='small')

    fig.suptitle(
        f'ATE RMSE {result["ate"]["rmse_m"]:.3f} m, '
        f'aligned samples {result["aligned_samples"]}')
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def main():
    args = parse_args()
    try:
        estimate_path = require_file(args.estimate, 'Estimate')
        groundtruth_path = groundtruth_path_from_args(args)
        result = evaluate_files(
            estimate_path, groundtruth_path, align=args.align,
            max_samples=args.max_samples, rpe_delta_seconds=1.0)
        ate = result['ate']
        print('Trajectory visualization')
        print(f'  estimate: {estimate_path}')
        print(f'  groundtruth: {groundtruth_path}')
        print(f'  alignment: {args.align}')
        print(f'  aligned samples: {result["aligned_samples"]}')
        print(f'  ATE RMSE: {ate["rmse_m"]:.6f} m')
        if args.output:
            write_plot(args.output, result, include_components=args.components)
            print(f'  wrote plot: {args.output}')
        else:
            print('  no --output provided; no files written')
    except EvaluationError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
