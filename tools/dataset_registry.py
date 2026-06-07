from dataclasses import dataclass
from pathlib import Path
import argparse
import csv
import json
from datetime import datetime


REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_PATH = REPO_ROOT / 'configs' / 'euroc_datasets.json'


class DatasetRegistryError(ValueError):
    pass


@dataclass(frozen=True)
class DatasetSpec:
    key: str
    display_name: str
    difficulty: str
    relative_path: str
    default_offset: int
    cam0_data_csv: str
    cam0_data_dir: str
    imu_csv: str
    groundtruth_csv: str
    expected_output_pattern: str
    default_output_subdir_pattern: str


@dataclass(frozen=True)
class ResolvedDataset:
    spec: DatasetSpec
    dataset_path: Path

    @property
    def key(self):
        return self.spec.key

    @property
    def display_name(self):
        return self.spec.display_name

    @property
    def default_offset(self):
        return self.spec.default_offset

    @property
    def cam0_csv_path(self):
        return self.dataset_path / self.spec.cam0_data_csv

    @property
    def cam0_data_dir_path(self):
        return self.dataset_path / self.spec.cam0_data_dir

    @property
    def imu_csv_path(self):
        return self.dataset_path / self.spec.imu_csv

    @property
    def groundtruth_path(self):
        return self.dataset_path / self.spec.groundtruth_csv

    def validate(self):
        checks = {
            'dataset': self.dataset_path.is_dir(),
            'cam0_csv': self.cam0_csv_path.is_file(),
            'cam0_data_dir': self.cam0_data_dir_path.is_dir(),
            'imu_csv': self.imu_csv_path.is_file(),
            'groundtruth': self.groundtruth_path.is_file(),
            'first_frame': self.first_frame_path() is not None,
        }
        return checks

    def is_runnable(self):
        checks = self.validate()
        return all(checks[key] for key in (
            'dataset', 'cam0_csv', 'cam0_data_dir', 'imu_csv',
            'groundtruth', 'first_frame'))

    def first_frame_path(self):
        if self.cam0_csv_path.is_file():
            with self.cam0_csv_path.open('r', newline='') as handle:
                reader = csv.reader(handle)
                for row in reader:
                    if not row:
                        continue
                    timestamp = row[0].strip()
                    if not timestamp or timestamp.startswith('#'):
                        continue
                    filename = row[1].strip() if len(row) > 1 and row[1].strip() else f'{timestamp}.png'
                    frame_path = self.cam0_data_dir_path / filename
                    if frame_path.is_file():
                        return frame_path

        if self.cam0_data_dir_path.is_dir():
            for frame_path in sorted(self.cam0_data_dir_path.glob('*.png')):
                if frame_path.is_file():
                    return frame_path
        return None

    def cam0_frame_index(self):
        times = []
        paths = []
        if self.cam0_csv_path.is_file():
            with self.cam0_csv_path.open('r', newline='') as handle:
                reader = csv.reader(handle)
                for row in reader:
                    if not row:
                        continue
                    timestamp_text = row[0].strip()
                    if not timestamp_text or timestamp_text.startswith('#'):
                        continue
                    filename = row[1].strip() if len(row) > 1 and row[1].strip() else f'{timestamp_text}.png'
                    frame_path = self.cam0_data_dir_path / filename
                    if not frame_path.is_file():
                        continue
                    try:
                        timestamp = seconds_from_timestamp(timestamp_text)
                    except ValueError:
                        continue
                    times.append(timestamp)
                    paths.append(frame_path)
            if paths:
                return times, paths

        if self.cam0_data_dir_path.is_dir():
            for frame_path in sorted(self.cam0_data_dir_path.glob('*.png')):
                try:
                    timestamp = seconds_from_timestamp(frame_path.stem)
                except ValueError:
                    continue
                times.append(timestamp)
                paths.append(frame_path)
        return times, paths

    def default_output_dir(self, results_root, offset=None, timestamp=None):
        offset = self.default_offset if offset is None else int(offset)
        timestamp = timestamp or datetime.now().strftime('%Y%m%d_%H%M%S')
        subdir = self.spec.default_output_subdir_pattern.format(
            timestamp=timestamp,
            dataset=self.key,
            offset=offset,
        )
        return Path(results_root) / subdir

    def expected_output_path(self, output_dir, offset=None):
        offset = self.default_offset if offset is None else int(offset)
        filename = self.spec.expected_output_pattern.format(
            dataset=self.key,
            offset=offset,
        )
        return Path(output_dir) / filename


class DatasetRegistry:
    def __init__(self, specs):
        self.specs = specs

    @property
    def keys(self):
        return tuple(self.specs.keys())

    def resolve(self, key, datasets_root):
        try:
            spec = self.specs[key]
        except KeyError as exc:
            raise DatasetRegistryError(f'Unknown dataset key: {key}') from exc
        return ResolvedDataset(spec, Path(datasets_root) / spec.relative_path)

    def resolve_all(self, datasets_root):
        return [self.resolve(key, datasets_root) for key in self.keys]


def seconds_from_timestamp(value):
    value = float(value)
    if abs(value) > 1e12:
        return value * 1e-9
    return value


def load_dataset_registry(config_path=None):
    config_path = Path(config_path) if config_path else DEFAULT_CONFIG_PATH
    if not config_path.is_file():
        raise DatasetRegistryError(f'Dataset registry config not found: {config_path}')
    try:
        data = json.loads(config_path.read_text())
    except json.JSONDecodeError as exc:
        raise DatasetRegistryError(f'Invalid dataset registry JSON: {config_path}') from exc

    datasets = data.get('datasets')
    if not isinstance(datasets, dict):
        raise DatasetRegistryError('Dataset registry must contain a "datasets" object')

    specs = {}
    required = {
        'key', 'display_name', 'difficulty', 'relative_path',
        'default_offset', 'cam0_data_csv', 'cam0_data_dir',
        'imu_csv', 'groundtruth_csv', 'expected_output_pattern',
        'default_output_subdir_pattern',
    }
    for key, item in datasets.items():
        if not isinstance(item, dict):
            raise DatasetRegistryError(f'Dataset entry must be an object: {key}')
        missing = sorted(required - set(item.keys()))
        if missing:
            raise DatasetRegistryError(f'Dataset {key} missing fields: {", ".join(missing)}')
        if item['key'] != key:
            raise DatasetRegistryError(f'Dataset key mismatch: map key {key}, item key {item["key"]}')
        specs[key] = DatasetSpec(**{field: item[field] for field in required})

    return DatasetRegistry(specs)


def status_text(value):
    return 'ok' if value else 'missing'


def print_registry_check(datasets_root, config_path=None):
    registry = load_dataset_registry(config_path)
    print('EuRoC dataset registry check')
    print(f'  config:        {Path(config_path) if config_path else DEFAULT_CONFIG_PATH}')
    print(f'  datasets root: {Path(datasets_root)}')
    print()

    rows = []
    for resolved in registry.resolve_all(datasets_root):
        checks = resolved.validate()
        first_frame = resolved.first_frame_path()
        rows.append([
            resolved.key,
            str(resolved.dataset_path),
            status_text(checks['cam0_csv'] and checks['cam0_data_dir']),
            status_text(checks['groundtruth']),
            first_frame.name if first_frame else 'missing',
            str(resolved.default_offset),
        ])

    headers = [
        'dataset', 'dataset path', 'cam0', 'ground truth',
        'first frame', 'default offset',
    ]
    widths = [
        max(len(headers[index]), *(len(row[index]) for row in rows))
        for index in range(len(headers))
    ]
    print('  ' + '  '.join(headers[index].ljust(widths[index]) for index in range(len(headers))))
    print('  ' + '  '.join('-' * widths[index] for index in range(len(headers))))
    for row in rows:
        print('  ' + '  '.join(row[index].ljust(widths[index]) for index in range(len(row))))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Validate configured EuRoC datasets.')
    parser.add_argument(
        '--datasets-root',
        default='./datasets',
        help='Root containing EuRoC dataset folders. Default: ./datasets.')
    parser.add_argument(
        '--config',
        default=str(DEFAULT_CONFIG_PATH),
        help='Dataset registry JSON path.')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        print_registry_check(args.datasets_root, args.config)
    except DatasetRegistryError as exc:
        print(f'error: {exc}')
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
