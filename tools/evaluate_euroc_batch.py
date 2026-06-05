from pathlib import Path
import argparse
import csv
import sys


TOOL_DIR = Path(__file__).resolve().parent
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

from evaluate_trajectory import (  # noqa: E402
    EvaluationError,
    GROUNDTRUTH_RELATIVE,
    evaluate_files,
)


DEFAULT_DATASETS = (
    'MH_01_easy',
    'MH_02_easy',
    'MH_03_medium',
    'MH_04_difficult',
    'MH_05_difficult',
)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Batch-evaluate EuRoC trajectory estimates.')
    parser.add_argument(
        '--datasets-root', required=True,
        help='Directory containing EuRoC sequence folders.')
    roots = parser.add_mutually_exclusive_group(required=True)
    roots.add_argument(
        '--results-root', dest='estimates_root',
        help='Root directory containing output_<dataset>_offset*.txt files.')
    roots.add_argument(
        '--estimates-root', dest='estimates_root',
        help='Alias for --results-root.')
    parser.add_argument(
        '--datasets', nargs='*', default=list(DEFAULT_DATASETS),
        help='Dataset names to evaluate. Default: all EuRoC Machine Hall sequences.')
    parser.add_argument(
        '--align', choices=('se3', 'translation', 'none', 'sim3'), default='se3',
        help='Alignment mode passed to evaluate_trajectory.py. Default: se3.')
    parser.add_argument(
        '--max-samples', type=int, default=None,
        help='Optional cap on aligned samples after timestamp filtering.')
    parser.add_argument(
        '--rpe-delta-seconds', type=float, default=1.0,
        help='Compute translation RPE over this time delta in seconds. Default: 1.0.')
    parser.add_argument(
        '--no-rpe', action='store_true',
        help='Skip translation RPE computation.')
    parser.add_argument(
        '--save-csv',
        help='Optional path for a small batch summary CSV. No files are written unless this is set.')
    return parser.parse_args()


def require_dir(path, label):
    path = Path(path)
    if not path.exists():
        raise EvaluationError(f'{label} directory does not exist: {path}')
    if not path.is_dir():
        raise EvaluationError(f'{label} path is not a directory: {path}')
    return path


def find_estimates(estimates_root, dataset_name):
    pattern = f'output_{dataset_name}_offset*.txt'
    return sorted(path for path in estimates_root.rglob(pattern) if path.is_file())


def run_command_for_missing(dataset_name):
    short = dataset_name.lower().replace('_easy', '').replace('_medium', '').replace('_difficult', '')
    short = short.replace('mh_', 'mh')
    return (
        f'python main.py --path .\\datasets\\{dataset_name} --offset 10 '
        f'--output-dir .\\results\\phase4b_{short}'
    )


def relative_path(path, root):
    try:
        return str(path.relative_to(root))
    except ValueError:
        return str(path)


def evaluate_batch(args):
    datasets_root = require_dir(args.datasets_root, 'Datasets root')
    estimates_root = require_dir(args.estimates_root, 'Estimates root')
    rpe_delta_seconds = None if args.no_rpe else args.rpe_delta_seconds

    results = []
    warnings = []
    missing = []

    for dataset_name in args.datasets:
        dataset_path = datasets_root / dataset_name
        if not dataset_path.exists():
            warnings.append(f'{dataset_name}: dataset folder missing: {dataset_path}')
            missing.append((dataset_name, 'dataset missing', run_command_for_missing(dataset_name)))
            continue

        groundtruth_path = dataset_path / GROUNDTRUTH_RELATIVE
        if not groundtruth_path.exists():
            warnings.append(f'{dataset_name}: ground truth missing: {groundtruth_path}')
            missing.append((dataset_name, 'ground truth missing', run_command_for_missing(dataset_name)))
            continue

        estimates = find_estimates(estimates_root, dataset_name)
        if not estimates:
            warnings.append(
                f'{dataset_name}: no estimates matching output_{dataset_name}_offset*.txt under {estimates_root}')
            missing.append((dataset_name, 'estimate missing', run_command_for_missing(dataset_name)))
            continue

        for estimate_path in estimates:
            try:
                result = evaluate_files(
                    estimate_path,
                    groundtruth_path,
                    align=args.align,
                    max_samples=args.max_samples,
                    rpe_delta_seconds=rpe_delta_seconds,
                )
            except EvaluationError as exc:
                warnings.append(f'{dataset_name}: {estimate_path}: {exc}')
                continue
            result['dataset'] = dataset_name
            result['estimate_relpath'] = relative_path(estimate_path, estimates_root)
            results.append(result)

    return results, warnings, missing


def format_float(value):
    if value is None:
        return '-'
    return f'{value:.6f}'


def rpe_rmse(result):
    rpe = result.get('rpe')
    if not rpe or rpe['metrics'] is None:
        return None
    return rpe['metrics']['rmse_m']


def print_table(results, warnings, missing):
    if warnings:
        print('Warnings:')
        for warning in warnings:
            print(f'  - {warning}')
        print()

    if not results:
        print('No trajectory pairs were evaluated.')
    else:
        headers = [
            'dataset', 'estimate', 'aligned', 'ATE rmse', 'ATE mean',
            'ATE median', 'ATE max', 'RPE rmse',
        ]
        rows = []
        for result in results:
            ate = result['ate']
            rows.append([
                result['dataset'],
                result['estimate_relpath'],
                str(result['aligned_samples']),
                format_float(ate['rmse_m']),
                format_float(ate['mean_m']),
                format_float(ate['median_m']),
                format_float(ate['max_m']),
                format_float(rpe_rmse(result)),
            ])

        widths = [
            max(len(headers[i]), *(len(row[i]) for row in rows))
            for i in range(len(headers))
        ]
        print('Evaluation summary')
        print('  ' + '  '.join(headers[i].ljust(widths[i]) for i in range(len(headers))))
        print('  ' + '  '.join('-' * widths[i] for i in range(len(headers))))
        for row in rows:
            print('  ' + '  '.join(row[i].ljust(widths[i]) for i in range(len(row))))

    if missing:
        print()
        print('Missing datasets/outputs:')
        for dataset_name, reason, command in missing:
            print(f'  - {dataset_name}: {reason}')
            print(f'    generate with: {command}')


def write_summary_csv(path, results):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow([
            'dataset',
            'estimate',
            'alignment',
            'aligned_samples',
            'ate_rmse_m',
            'ate_mean_m',
            'ate_median_m',
            'ate_max_m',
            'rpe_delta_seconds',
            'rpe_pairs',
            'rpe_rmse_m',
        ])
        for result in results:
            ate = result['ate']
            rpe = result['rpe']
            writer.writerow([
                result['dataset'],
                result['estimate_relpath'],
                result['alignment'],
                result['aligned_samples'],
                f'{ate["rmse_m"]:.9f}',
                f'{ate["mean_m"]:.9f}',
                f'{ate["median_m"]:.9f}',
                f'{ate["max_m"]:.9f}',
                '' if rpe is None else f'{rpe["delta_seconds"]:.9f}',
                '' if rpe is None else rpe['pairs'],
                '' if rpe is None or rpe['metrics'] is None else f'{rpe["metrics"]["rmse_m"]:.9f}',
            ])


def main():
    args = parse_args()
    try:
        results, warnings, missing = evaluate_batch(args)
    except EvaluationError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2

    print_table(results, warnings, missing)

    if args.save_csv:
        write_summary_csv(args.save_csv, results)
        print()
        print(f'Wrote summary CSV: {args.save_csv}')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
