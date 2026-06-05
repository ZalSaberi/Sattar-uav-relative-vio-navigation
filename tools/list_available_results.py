from pathlib import Path
import argparse


GROUNDTRUTH_RELATIVE = Path('mav0') / 'state_groundtruth_estimate0' / 'data.csv'
DEFAULT_DATASETS = (
    'MH_01_easy',
    'MH_02_easy',
    'MH_03_medium',
    'MH_04_difficult',
    'MH_05_difficult',
)


def parse_args():
    parser = argparse.ArgumentParser(
        description='List available EuRoC datasets and matching trajectory output files.')
    parser.add_argument(
        '--datasets-root', default='./datasets',
        help='Directory containing EuRoC sequence folders. Default: ./datasets.')
    parser.add_argument(
        '--results-root', default='./results',
        help='Directory containing output_<dataset>_offset*.txt files. Default: ./results.')
    parser.add_argument(
        '--datasets', nargs='*', default=list(DEFAULT_DATASETS),
        help='Dataset names to inspect. Default: EuRoC Machine Hall sequences.')
    return parser.parse_args()


def find_estimates(results_root, dataset_name):
    pattern = f'output_{dataset_name}_offset*.txt'
    return sorted(path for path in results_root.rglob(pattern) if path.is_file())


def relative(path, root):
    try:
        return str(path.relative_to(root))
    except ValueError:
        return str(path)


def main():
    args = parse_args()
    datasets_root = Path(args.datasets_root)
    results_root = Path(args.results_root)

    print('Available EuRoC trajectory results')
    print(f'  datasets root: {datasets_root}')
    print(f'  results root:  {results_root}')
    print()

    rows = []
    for dataset_name in args.datasets:
        dataset_path = datasets_root / dataset_name
        gt_path = dataset_path / GROUNDTRUTH_RELATIVE
        estimates = find_estimates(results_root, dataset_name) if results_root.exists() else []
        rows.append([
            dataset_name,
            'yes' if dataset_path.is_dir() else 'no',
            'yes' if gt_path.is_file() else 'no',
            str(len(estimates)),
            ', '.join(relative(path, results_root) for path in estimates) if estimates else '-',
        ])

    headers = ['dataset', 'dataset dir', 'ground truth', 'estimates', 'files']
    widths = [
        max(len(headers[i]), *(len(row[i]) for row in rows))
        for i in range(len(headers))
    ]
    print('  ' + '  '.join(headers[i].ljust(widths[i]) for i in range(len(headers))))
    print('  ' + '  '.join('-' * widths[i] for i in range(len(headers))))
    for row in rows:
        print('  ' + '  '.join(row[i].ljust(widths[i]) for i in range(len(row))))

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
