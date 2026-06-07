from pathlib import Path
import argparse
import csv
import json
import os
import re
import sys
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime

import numpy as np


TOOL_DIR = Path(__file__).resolve().parent
REPO_ROOT = TOOL_DIR.parent
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

from evaluate_trajectory import (  # noqa: E402
    EvaluationError,
    evaluate_files,
)
from dataset_registry import (  # noqa: E402
    DatasetRegistryError,
    load_dataset_registry,
)


DATASET_REGISTRY = load_dataset_registry()
DATASETS = DATASET_REGISTRY.keys
INSTALL_COMMAND = 'python -m pip install PyQt5 pyqtgraph PyOpenGL matplotlib'
METRICS_FILENAME = 'dashboard_metrics_summary.json'
MAX_PLOT_POINTS = 2400
MAX_LOG_LINES = 120
TASKBAR_SAFE_MARGIN = 8
REJECTED_PATH_MARKERS = ('phase3c', 'raw_jacobian')
MIN_ACCEPTED_ALIGNED_SAMPLES = 100
MIN_ACCEPTED_OVERLAP_SECONDS = 1.0
MAX_ACCEPTED_ATE_RMSE = 10.0

COLORS = {
    'background': '#0A0F1A',
    'panel': '#111827',
    'panel_deep': '#0B1220',
    'panel_soft': '#101A2A',
    'border': '#243244',
    'accent': '#2F6BFF',
    'cyan': '#38BDF8',
    'green': '#24C768',
    'red': '#D94141',
    'orange': '#F2A93B',
    'purple': '#C084FC',
    'text': '#F4F7FB',
    'muted': '#A8B3C5',
}

QtCore = None
QtGui = None
QtWidgets = None
QDesktopServices = None
QUrl = None
Figure = None
FigureCanvas = None
NavigationToolbar = None


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Launch the UAV-Airvision Evaluation Dashboard.')
    parser.add_argument(
        '--datasets-root', default='./datasets',
        help='Directory containing EuRoC sequence folders. Default: ./datasets.')
    parser.add_argument(
        '--results-root', default='./results',
        help='Directory containing output_<dataset>_offset*.txt files. Default: ./results.')
    parser.add_argument(
        '--default-offset', type=int, default=10,
        help='Default run offset shown in the dashboard. Default: 10.')
    parser.add_argument(
        '--validate-data-flow',
        action='store_true',
        help='Validate registry, latest accepted estimates, and real evaluation data without launching the GUI.')
    parser.add_argument(
        '--ui-self-check',
        action='store_true',
        help='Instantiate the Qt dashboard and print nonvisual UI invariants.')
    return parser.parse_args(argv)


def load_gui_dependencies():
    global QtCore, QtGui, QtWidgets, QDesktopServices, QUrl
    global Figure, FigureCanvas, NavigationToolbar
    try:
        from PyQt5 import QtCore as _QtCore
        from PyQt5 import QtGui as _QtGui
        from PyQt5 import QtWidgets as _QtWidgets
        from PyQt5.QtCore import QUrl as _QUrl
        from PyQt5.QtGui import QDesktopServices as _QDesktopServices
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as _FigureCanvas
        from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as _NavigationToolbar
        from matplotlib.figure import Figure as _Figure
        import pyqtgraph  # noqa: F401
    except ImportError as exc:
        raise RuntimeError(
            'Dashboard GUI dependencies are missing.\n'
            f'Install them with: {INSTALL_COMMAND}\n'
            'Headless CLI tools remain available without these packages.'
        ) from exc

    QtCore = _QtCore
    QtGui = _QtGui
    QtWidgets = _QtWidgets
    QDesktopServices = _QDesktopServices
    QUrl = _QUrl
    Figure = _Figure
    FigureCanvas = _FigureCanvas
    NavigationToolbar = _NavigationToolbar


def dataset_path(datasets_root, dataset_name):
    return DATASET_REGISTRY.resolve(dataset_name, datasets_root).dataset_path


def groundtruth_path(datasets_root, dataset_name):
    return DATASET_REGISTRY.resolve(dataset_name, datasets_root).groundtruth_path


def estimate_pattern(dataset_name):
    return f'output_{dataset_name}_offset*.txt'


def is_rejected_estimate_path(path):
    text = str(path).lower()
    return any(marker in text for marker in REJECTED_PATH_MARKERS)


def find_estimates(results_root, dataset_name, include_rejected=True):
    root = Path(results_root)
    if not root.exists():
        return []
    paths = [path for path in root.rglob(estimate_pattern(dataset_name)) if path.is_file()]
    if not include_rejected:
        paths = [path for path in paths if not is_rejected_estimate_path(path)]
    return sorted(paths, key=lambda path: path.stat().st_mtime, reverse=True)


def relative_path(path, root):
    path = Path(path)
    root = Path(root)
    try:
        return str(path.relative_to(root))
    except ValueError:
        return str(path)


def elide_middle(text, max_chars=42):
    text = str(text)
    if len(text) <= max_chars:
        return text
    keep = max(8, (max_chars - 3) // 2)
    tail = max(8, max_chars - 3 - keep)
    return f'{text[:keep]}...{text[-tail:]}'


def display_path(path, root=None, max_chars=42):
    if not path:
        return '-'
    text = relative_path(path, root) if root is not None else str(path)
    return elide_middle(text, max_chars=max_chars)


def timestamp_to_seconds(value):
    value = float(value)
    if abs(value) > 1e12:
        return value * 1e-9
    return value


def output_file_for_run(output_dir, dataset_name, offset):
    resolved = DATASET_REGISTRY.resolve(dataset_name, '.')
    return resolved.expected_output_path(output_dir, offset)


def unique_run_dir(output_root, dataset_name, offset):
    resolved = DATASET_REGISTRY.resolve(dataset_name, '.')
    return resolved.default_output_dir(output_root, offset)


def classify_result(dataset_name, estimate_path, ate_rmse, aligned_samples=None,
                    overlap_duration=None):
    estimate_text = str(estimate_path).lower()
    if is_rejected_estimate_path(estimate_text):
        return 'rejected', COLORS['red'], False, 'path marked as rejected experiment'
    if ate_rmse > MAX_ACCEPTED_ATE_RMSE:
        return 'outlier', COLORS['red'], False, f'ATE RMSE > {MAX_ACCEPTED_ATE_RMSE:g} m'
    if aligned_samples is not None and aligned_samples < MIN_ACCEPTED_ALIGNED_SAMPLES:
        return 'partial', COLORS['orange'], False, f'aligned samples < {MIN_ACCEPTED_ALIGNED_SAMPLES}'
    if overlap_duration is not None and overlap_duration < MIN_ACCEPTED_OVERLAP_SECONDS:
        return 'partial', COLORS['orange'], False, f'overlap < {MIN_ACCEPTED_OVERLAP_SECONDS:g}s'
    if dataset_name == 'MH_04_difficult' and ate_rmse > 0.25:
        return 'accepted', COLORS['orange'], True, 'accepted; difficult sequence warning'
    if ate_rmse < 0.25:
        return 'accepted', COLORS['green'], True, 'accepted'
    if ate_rmse < 1.0:
        return 'accepted', COLORS['orange'], True, 'accepted; warning threshold'
    return 'accepted', COLORS['orange'], True, 'accepted; high but not rejected'


def empty_row(dataset_name, status='missing', color=None):
    return {
        'dataset_key': dataset_name,
        'dataset': dataset_name,
        'status': status,
        'estimate': '',
        'estimate_relpath': '-',
        'groundtruth_path': '',
        'computed_at': '-',
        'alignment_mode': 'se3',
        'data_source_status': 'Missing',
        'accepted': False,
        'acceptance_reason': status,
        'color': color or COLORS['orange'],
        'aligned_samples': 0,
        'overlap_duration_s': 0.0,
        'ate_rmse_m': None,
        'ate_mean_m': None,
        'ate_median_m': None,
        'ate_max_m': None,
        'rpe_1s_rmse_m': None,
        'evaluated_at': '-',
    }


@dataclass
class EvaluationResult:
    dataset_key: str
    estimate_path: str
    groundtruth_path: str
    computed_at: str
    alignment_mode: str
    timestamps: np.ndarray
    estimate_aligned: np.ndarray
    groundtruth_interpolated: np.ndarray
    error_norms: np.ndarray
    rpe_errors: object
    metrics: dict
    row: dict
    data_source_status: str


def serialize_result(dataset_name, estimate_path, groundtruth_path, results_root, result,
                     data_source_status='Freshly computed'):
    ate = result['ate']
    rpe = result.get('rpe')
    rpe_rmse = None
    if rpe and rpe.get('metrics'):
        rpe_rmse = rpe['metrics']['rmse_m']
    aligned_samples = int(result['aligned_samples'])
    overlap = max(0.0, float(result['aligned_end'] - result['aligned_start']))
    status, color, accepted, reason = classify_result(
        dataset_name, estimate_path, ate['rmse_m'],
        aligned_samples=aligned_samples,
        overlap_duration=overlap)
    computed_at = datetime.now().isoformat(timespec='seconds')
    return {
        'dataset_key': dataset_name,
        'dataset': dataset_name,
        'status': status,
        'estimate': str(estimate_path),
        'estimate_relpath': relative_path(estimate_path, results_root),
        'groundtruth_path': str(groundtruth_path),
        'computed_at': computed_at,
        'alignment_mode': result['alignment'],
        'data_source_status': data_source_status,
        'accepted': accepted,
        'acceptance_reason': reason,
        'color': color,
        'aligned_samples': aligned_samples,
        'overlap_duration_s': overlap,
        'ate_rmse_m': ate['rmse_m'],
        'ate_mean_m': ate['mean_m'],
        'ate_median_m': ate['median_m'],
        'ate_max_m': ate['max_m'],
        'rpe_1s_rmse_m': rpe_rmse,
        'evaluated_at': computed_at,
    }


def evaluate_estimate(dataset_name, estimate_path, datasets_root, results_root):
    estimate_path = Path(estimate_path)
    resolved = DATASET_REGISTRY.resolve(dataset_name, datasets_root)
    gt_path = resolved.groundtruth_path
    if not gt_path.exists():
        raise EvaluationError(f'Ground truth missing: {gt_path}')
    result = evaluate_files(estimate_path, gt_path, align='se3', rpe_delta_seconds=1.0)
    row = serialize_result(dataset_name, estimate_path, gt_path, results_root, result)
    metrics = {
        'ATE RMSE': row['ate_rmse_m'],
        'ATE mean': row['ate_mean_m'],
        'ATE median': row['ate_median_m'],
        'ATE max': row['ate_max_m'],
        'RPE 1s RMSE': row['rpe_1s_rmse_m'],
        'aligned samples': row['aligned_samples'],
        'overlap duration': row['overlap_duration_s'],
    }
    rpe = result.get('rpe')
    model = EvaluationResult(
        dataset_key=dataset_name,
        estimate_path=str(estimate_path),
        groundtruth_path=str(gt_path),
        computed_at=row['computed_at'],
        alignment_mode=result['alignment'],
        timestamps=result['aligned_times'],
        estimate_aligned=result['estimate_aligned'],
        groundtruth_interpolated=result['groundtruth_aligned'],
        error_norms=result['ate_errors_m'],
        rpe_errors=rpe.get('errors_m') if rpe and 'errors_m' in rpe else None,
        metrics=metrics,
        row=row,
        data_source_status=row['data_source_status'],
    )
    return row, model


def sample_points(*arrays, max_points=MAX_PLOT_POINTS):
    if not arrays:
        return arrays
    count = len(arrays[0])
    if count <= max_points:
        return arrays
    indices = np.linspace(0, count - 1, max_points).round().astype(int)
    return tuple(np.asarray(array)[indices] for array in arrays)


def metric_text(value, suffix='m'):
    if value is None:
        return '-'
    if suffix == 'count':
        return str(int(value))
    if suffix == 's':
        return f'{value:.1f} s'
    return f'{value:.3f} m'


def load_cached_rows(results_root):
    cache_path = Path(results_root) / METRICS_FILENAME
    if not cache_path.is_file():
        return {}
    try:
        rows = json.loads(cache_path.read_text())
    except json.JSONDecodeError:
        return {}
    if not isinstance(rows, list):
        return {}
    return {
        row.get('dataset'): row
        for row in rows
        if isinstance(row, dict) and row.get('dataset')
    }


def compare_metric_rows(recomputed, cached, tolerance=1e-6):
    mismatches = []
    for key in (
        'ate_rmse_m', 'ate_mean_m', 'ate_median_m', 'ate_max_m',
        'rpe_1s_rmse_m', 'aligned_samples', 'overlap_duration_s',
    ):
        expected = recomputed.get(key)
        actual = cached.get(key)
        if expected is None or actual is None:
            if expected != actual:
                mismatches.append((key, expected, actual))
            continue
        if abs(float(expected) - float(actual)) > tolerance:
            mismatches.append((key, expected, actual))
    return mismatches


def validate_data_flow(args):
    print('Dashboard data-flow validation')
    print(f'  datasets root: {Path(args.datasets_root)}')
    print(f'  results root:  {Path(args.results_root)}')
    print(f'  registry:      {DATASET_REGISTRY.keys}')
    print()

    cached = load_cached_rows(args.results_root)
    evaluated = 0
    mismatches = 0
    missing = 0

    for resolved in DATASET_REGISTRY.resolve_all(args.datasets_root):
        checks = resolved.validate()
        first_frame = resolved.first_frame_path()
        estimates = find_estimates(args.results_root, resolved.key, include_rejected=False)
        print(f'{resolved.key}')
        print(f'  dataset path: {resolved.dataset_path}')
        print(f'  cam0 csv:     {"ok" if checks["cam0_csv"] else "missing"}')
        print(f'  cam0 dir:     {"ok" if checks["cam0_data_dir"] else "missing"}')
        print(f'  ground truth: {"ok" if checks["groundtruth"] else "missing"}')
        print(f'  first frame:  {first_frame if first_frame else "missing"}')
        print(f'  estimates:    {len(estimates)} accepted candidate(s)')

        if not estimates or not checks['groundtruth']:
            missing += 1
            print('  selected:     none')
            print('  status:       missing')
            print('  reason:       no accepted estimate or ground truth missing')
            print()
            continue

        selected_row = None
        selected_model = None
        fallback_row = None
        fallback_model = None
        for estimate_path in estimates:
            row, model = evaluate_estimate(
                resolved.key, estimate_path, args.datasets_root, args.results_root)
            if row.get('accepted'):
                selected_row = row
                selected_model = model
                break
            if fallback_row is None:
                fallback_row = row
                fallback_model = model

        row = selected_row or fallback_row
        model = selected_model or fallback_model
        if row is None:
            missing += 1
            print('  selected:     none')
            print('  status:       failed')
            print('  reason:       no estimate could be evaluated')
            print()
            continue

        evaluated += 1
        estimate_path = Path(row['estimate'])
        rpe_text = metric_text(row['rpe_1s_rmse_m'])
        overlap_text = metric_text(row['overlap_duration_s'], 's')
        print(f'  selected:     {estimate_path}')
        print(f'  status:       {row["status"]}')
        print(f'  accepted:     {"yes" if row.get("accepted") else "no"}')
        print(f'  reason:       {row.get("acceptance_reason", "-")}')
        print(
            '  evaluation:   '
            f'ATE RMSE={row["ate_rmse_m"]:.6f} m, '
            f'RPE 1s={rpe_text}, '
            f'aligned={row["aligned_samples"]}, '
            f'overlap={overlap_text}')

        cached_row = cached.get(resolved.key)
        if cached_row and Path(cached_row.get('estimate', '')) == Path(row['estimate']):
            differences = compare_metric_rows(row, cached_row)
            if differences:
                mismatches += len(differences)
                for key, expected, actual in differences:
                    print(f'  cache mismatch: {key} recomputed={expected} cached={actual}')
            else:
                print('  cache:        matches recomputed metrics')
        elif cached_row:
            print('  cache:        present for a different estimate file')
        else:
            print('  cache:        not present')
        print()

    print('Summary')
    print(f'  evaluated datasets: {evaluated}')
    print(f'  skipped datasets:   {missing}')
    print(f'  cache mismatches:   {mismatches}')
    print('  metric source:      evaluate_trajectory.evaluate_files')
    print('  demo metrics:       none in normal path')
    return 0 if evaluated > 0 and mismatches == 0 else 1


def create_dashboard_classes():
    from dashboard.table import (
        DarkResultsTable,
        GlobalResultsModel,
        configure_results_table,
        table_self_check,
    )
    from dashboard.live_view import LiveRunView

    class EvaluationWorker(QtCore.QObject):
        finished = QtCore.pyqtSignal(str, str, object, object)
        failed = QtCore.pyqtSignal(str, str, str)

        def __init__(self, dataset_name, estimate_path, datasets_root, results_root):
            super().__init__()
            self.dataset_name = dataset_name
            self.estimate_path = str(estimate_path)
            self.datasets_root = str(datasets_root)
            self.results_root = str(results_root)

        @QtCore.pyqtSlot()
        def run(self):
            try:
                row, result = evaluate_estimate(
                    self.dataset_name, self.estimate_path,
                    self.datasets_root, self.results_root)
            except Exception as exc:  # Keep the GUI boundary explicit.
                self.failed.emit(self.dataset_name, self.estimate_path, str(exc))
                return
            self.finished.emit(self.dataset_name, self.estimate_path, row, result)

    class RefreshWorker(QtCore.QObject):
        finished = QtCore.pyqtSignal(object)
        message = QtCore.pyqtSignal(str)

        def __init__(self, datasets_root, results_root, include_rejected=False):
            super().__init__()
            self.datasets_root = Path(datasets_root)
            self.results_root = Path(results_root)
            self.include_rejected = include_rejected

        @QtCore.pyqtSlot()
        def run(self):
            rows = []
            for dataset_name in DATASETS:
                estimates = find_estimates(
                    self.results_root, dataset_name,
                    include_rejected=self.include_rejected)
                if not estimates:
                    rows.append(empty_row(dataset_name, 'missing', COLORS['orange']))
                    continue
                accepted_row = None
                fallback_row = None
                for estimate_path in estimates:
                    self.message.emit(f'Evaluating latest result for {dataset_name}: {estimate_path.name}')
                    try:
                        row, _ = evaluate_estimate(
                            dataset_name, estimate_path, self.datasets_root, self.results_root)
                    except Exception as exc:
                        fallback_row = empty_row(dataset_name, 'failed', COLORS['red'])
                        fallback_row['estimate'] = str(estimate_path)
                        fallback_row['estimate_relpath'] = relative_path(estimate_path, self.results_root)
                        self.message.emit(f'Skipped {dataset_name}: {exc}')
                        continue
                    if self.include_rejected or row.get('accepted'):
                        accepted_row = row
                        break
                    if fallback_row is None:
                        fallback_row = row
                    self.message.emit(
                        f'Skipped non-accepted result for {dataset_name}: '
                        f'{estimate_path.name} ({row["status"]}: {row["acceptance_reason"]})')
                if accepted_row is not None:
                    rows.append(accepted_row)
                elif fallback_row is not None:
                    rows.append(fallback_row)
                else:
                    rows.append(empty_row(dataset_name, 'missing', COLORS['orange']))
            self.finished.emit(rows)

    class PanelFrame(QtWidgets.QFrame):
        def __init__(self, title=None):
            super().__init__()
            self.setObjectName('PanelFrame')
            self.body = QtWidgets.QVBoxLayout(self)
            self.body.setContentsMargins(14, 12, 14, 12)
            self.body.setSpacing(7)
            if title:
                label = QtWidgets.QLabel(title)
                label.setObjectName('SectionTitle')
                self.body.addWidget(label)

    class MetricCard(QtWidgets.QFrame):
        def __init__(self, title):
            super().__init__()
            self.setObjectName('MetricCard')
            layout = QtWidgets.QVBoxLayout(self)
            layout.setContentsMargins(7, 3, 7, 3)
            layout.setSpacing(1)
            label = QtWidgets.QLabel(title)
            label.setObjectName('MetricTitle')
            self.value = QtWidgets.QLabel('-')
            self.value.setObjectName('MetricValue')
            layout.addWidget(label)
            layout.addWidget(self.value)

        def set_value(self, value, color=COLORS['text']):
            self.value.setText(value)
            self.value.setStyleSheet(f'color: {color};')

    class StatusBlock(QtWidgets.QFrame):
        def __init__(self):
            super().__init__()
            self.setObjectName('StatusCard')
            self.values = {}
            layout = QtWidgets.QGridLayout(self)
            layout.setContentsMargins(7, 5, 7, 5)
            layout.setHorizontalSpacing(8)
            layout.setVerticalSpacing(2)
            for row, key in enumerate(['Run', 'Evaluation', 'Dataset', 'Frame', 'Time', 'FPS', 'Output File']):
                label = QtWidgets.QLabel(key)
                label.setObjectName('StatusName')
                value = QtWidgets.QLabel('-')
                value.setObjectName('StatusValue')
                value.setWordWrap(True)
                self.values[key] = value
                layout.addWidget(label, row, 0)
                layout.addWidget(value, row, 1)

        def set_value(self, key, value, color=None, tooltip=None):
            widget = self.values[key]
            widget.setText(str(value))
            widget.setToolTip(str(tooltip or value))
            if color:
                widget.setStyleSheet(f'color: {color};')
            else:
                widget.setStyleSheet('')

    class ProvenanceBlock(QtWidgets.QFrame):
        def __init__(self):
            super().__init__()
            self.setObjectName('StatusCard')
            self.values = {}
            layout = QtWidgets.QGridLayout(self)
            layout.setContentsMargins(7, 5, 7, 5)
            layout.setHorizontalSpacing(8)
            layout.setVerticalSpacing(2)
            for row, key in enumerate([
                'Data Source', 'Dataset', 'Estimate', 'Ground Truth',
                'Computed At', 'Alignment', 'Accepted', 'Reason',
            ]):
                label = QtWidgets.QLabel(key)
                label.setObjectName('StatusName')
                value = QtWidgets.QLabel('-')
                value.setObjectName('StatusValue')
                value.setWordWrap(True)
                self.values[key] = value
                layout.addWidget(label, row, 0)
                layout.addWidget(value, row, 1)

        def set_value(self, key, value, color=None, tooltip=None):
            widget = self.values[key]
            widget.setText(str(value))
            widget.setToolTip(str(tooltip or value))
            if color:
                widget.setStyleSheet(f'color: {color};')
            else:
                widget.setStyleSheet('')

        def show_row(self, row, results_root):
            color = row.get('color', COLORS['muted'])
            self.set_value('Data Source', row.get('data_source_status', 'Missing'), color)
            self.set_value('Dataset', row.get('dataset', '-'), color)
            estimate = row.get('estimate') or '-'
            groundtruth = row.get('groundtruth_path') or '-'
            self.set_value(
                'Estimate',
                display_path(estimate, results_root, max_chars=22),
                tooltip=estimate)
            self.set_value(
                'Ground Truth',
                display_path(groundtruth, None, max_chars=22),
                tooltip=groundtruth)
            self.set_value('Computed At', row.get('computed_at') or row.get('evaluated_at') or '-')
            self.set_value('Alignment', row.get('alignment_mode', 'se3'))
            self.set_value('Accepted', 'yes' if row.get('accepted') else 'no', color)
            self.set_value('Reason', row.get('acceptance_reason', '-'), color)

    class CameraPreview(QtWidgets.QFrame):
        def __init__(self):
            super().__init__()
            self.setObjectName('ChartCard')
            self.pixmap = None
            layout = QtWidgets.QVBoxLayout(self)
            layout.setContentsMargins(12, 10, 12, 10)
            layout.setSpacing(8)
            title = QtWidgets.QLabel('Camera: cam0 (live)')
            title.setObjectName('PanelTitle')
            self.image_label = QtWidgets.QLabel('No frame preview available')
            self.image_label.setObjectName('CameraImage')
            self.image_label.setAlignment(QtCore.Qt.AlignCenter)
            self.image_label.setMinimumSize(260, 180)
            layout.addWidget(title)
            layout.addWidget(self.image_label, 1)

        def set_message(self, text):
            self.pixmap = None
            self.image_label.setPixmap(QtGui.QPixmap())
            self.image_label.setText(text)

        def set_frame(self, path):
            pixmap = QtGui.QPixmap(str(path))
            if pixmap.isNull():
                self.set_message(f'Could not load frame:\n{Path(path).name}')
                return
            self.pixmap = pixmap
            self.image_label.setText('')
            self._rescale()

        def resizeEvent(self, event):
            super().resizeEvent(event)
            self._rescale()

        def _rescale(self):
            if self.pixmap is None or self.pixmap.isNull():
                return
            size = self.image_label.size()
            scaled = self.pixmap.scaled(
                size, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
            self.image_label.setPixmap(scaled)

    class PlotPanel(QtWidgets.QFrame):
        def __init__(self, title, toolbar=False):
            super().__init__()
            self.setObjectName('ChartCard')
            self.last_key = None
            layout = QtWidgets.QVBoxLayout(self)
            layout.setContentsMargins(12, 10, 12, 10)
            layout.setSpacing(6)
            self.title_label = QtWidgets.QLabel(title)
            self.title_label.setObjectName('PanelTitle')
            self.figure = Figure(figsize=(5, 3), facecolor=COLORS['panel'])
            self.figure.subplots_adjust(left=0.18, right=0.96, top=0.84, bottom=0.18)
            self.canvas = FigureCanvas(self.figure)
            layout.addWidget(self.title_label)
            if toolbar:
                toolbar_widget = NavigationToolbar(self.canvas, self)
                toolbar_widget.setIconSize(QtCore.QSize(16, 16))
                toolbar_widget.setObjectName('ChartToolbar')
                layout.addWidget(toolbar_widget)
            layout.addWidget(self.canvas, 1)
            self.draw_empty('No evaluated trajectory yet')

        def draw_empty(self, message):
            self.figure.clear()
            ax = self.figure.add_subplot(111)
            self._style_axes(ax)
            self.figure.subplots_adjust(left=0.04, right=0.96, top=0.90, bottom=0.08)
            ax.text(
                0.5, 0.5, message, ha='center', va='center',
                color=COLORS['muted'], transform=ax.transAxes)
            ax.set_xticks([])
            ax.set_yticks([])
            self.canvas.draw_idle()

        def draw_trajectory(self, estimate, groundtruth, axes, title, estimate_color=COLORS['cyan']):
            estimate, groundtruth = sample_points(estimate, groundtruth)
            self.figure.clear()
            ax = self.figure.add_subplot(111)
            self._style_axes(ax)
            self.figure.subplots_adjust(left=0.16, right=0.96, top=0.86, bottom=0.20)
            a, b = axes
            ax.plot(
                groundtruth[:, a], groundtruth[:, b], color=COLORS['green'],
                linewidth=1.35, linestyle='--', label='ground truth')
            ax.plot(
                estimate[:, a], estimate[:, b], color=estimate_color,
                linewidth=1.25, label='estimate')
            ax.set_title(title, color=COLORS['text'])
            ax.set_xlabel(f'{"xyz"[a]} [m]')
            ax.set_ylabel(f'{"xyz"[b]} [m]')
            ax.axis('equal')
            ax.legend(facecolor=COLORS['panel_deep'], edgecolor=COLORS['border'])
            self.canvas.draw_idle()

        def draw_error(self, x, errors, title, color, ylabel='error [m]'):
            x, errors = sample_points(x, errors)
            self.figure.clear()
            ax = self.figure.add_subplot(111)
            self._style_axes(ax)
            self.figure.subplots_adjust(left=0.16, right=0.96, top=0.86, bottom=0.20)
            mean = float(np.mean(errors)) if len(errors) else 0.0
            ax.plot(x, errors, color=color, linewidth=1.2)
            ax.axhline(mean, color=COLORS['cyan'], linestyle='--', linewidth=1.0, label=f'mean {mean:.3f} m')
            ax.set_title(title, color=COLORS['text'])
            ax.set_xlabel('time [s]' if len(x) and np.nanmax(x) > 10 else 'sample')
            ax.set_ylabel(ylabel)
            ax.legend(facecolor=COLORS['panel_deep'], edgecolor=COLORS['border'])
            self.canvas.draw_idle()

        def _prepare_axes(self, left=0.16, right=0.96, top=0.84, bottom=0.20):
            self.figure.clear()
            self.figure.set_facecolor(COLORS['panel'])
            ax = self.figure.add_subplot(111)
            self.figure.subplots_adjust(left=left, right=right, top=top, bottom=bottom)
            self._style_axes(ax)
            return ax

        def _finish(self):
            self.canvas.draw_idle()

        def draw_bar(self, rows, metric_key, title):
            ax = self._prepare_axes(left=0.15, right=0.98, top=0.88, bottom=0.25)

            values = []
            labels = []
            colors = []

            for row in rows:
                value = row.get(metric_key)
                if value is None:
                    continue

                label = (
                    row['dataset']
                    .replace('MH_', 'MH')
                    .replace('_easy', '')
                    .replace('_medium', '')
                    .replace('_difficult', '')
                )

                labels.append(label)
                values.append(float(value))
                colors.append(row.get('color', COLORS['cyan']))

            if not values:
                ax.text(
                    0.5,
                    0.5,
                    'No evaluated results available',
                    ha='center',
                    va='center',
                    color=COLORS['muted'],
                    fontsize=8,
                    transform=ax.transAxes,
                )
                ax.set_xticks([])
                ax.set_yticks([])
                self._finish()
                return

            y = np.arange(len(values))
            max_value = max(values)
            x_limit = max(max_value * 1.10, 0.05)

            ax.barh(y, values, color=colors, height=0.52)
            ax.set_xlim(0, x_limit)
            ax.set_yticks(y)
            ax.set_yticklabels(labels, fontsize=7)
            ax.invert_yaxis()

            # The panel already has a title. Do not duplicate a huge title inside the plot.
            axis_label = 'ATE RMSE (m)' if metric_key == 'ate_rmse_m' else 'RPE 1s RMSE (m)'
            ax.set_xlabel(axis_label, fontsize=7, labelpad=4)

            for index, value in enumerate(values):
                x_pos = value + x_limit * 0.012
                ha = 'left'
                if x_pos > x_limit * 0.96:
                    x_pos = max(value - x_limit * 0.020, 0)
                    ha = 'right'

                ax.text(
                    x_pos,
                    index,
                    f'{value:.3f}',
                    va='center',
                    ha=ha,
                    color=COLORS['text'],
                    fontsize=7,
                )

            self._finish()

        def _style_axes(self, ax):
            ax.set_facecolor(COLORS['panel_deep'])
            ax.figure.set_facecolor(COLORS['panel'])
            ax.tick_params(colors=COLORS['muted'], labelsize=8)
            ax.xaxis.label.set_color(COLORS['muted'])
            ax.yaxis.label.set_color(COLORS['muted'])
            for spine in ax.spines.values():
                spine.set_color(COLORS['border'])
            ax.grid(True, color='#36506F', alpha=0.24, linewidth=0.7)

    class DashboardWindow(QtWidgets.QMainWindow):
        def __init__(self, datasets_root, results_root, default_offset,
                     auto_refresh=True, auto_evaluate=True):
            super().__init__()
            self.datasets_root = Path(datasets_root)
            self.results_root = Path(results_root)
            self.output_root = self.results_root
            self.metrics_path = self.results_root / METRICS_FILENAME
            self.default_offset = int(default_offset)
            self.rows_by_dataset = {name: empty_row(name) for name in DATASETS}
            self.result_cache = {}
            self.estimate_cache = {}
            self.current_dataset = DATASETS[0]
            self.current_output_file = None
            self.previous_rows_before_run = {}
            self.process = None
            self.run_queue = []
            self.continue_queue_after_eval = False
            self.stop_requested = False
            self.eval_thread = None
            self.eval_worker = None
            self.refresh_thread = None
            self.refresh_worker = None
            self.log_lines = deque(maxlen=MAX_LOG_LINES)
            self.camera_frame_times = []
            self.camera_frame_paths = []
            self.camera_frame_index = -1
            self.run_started_at = None
            self.preview_frames_shown = 0
            self._last_stdout_timestamp = None
            self.updating_table = False
            self.suppress_auto_evaluate = False
            self.geometry_locked = False

            self.setWindowTitle('UAV-Airvision Evaluation Dashboard')
            self._build_ui()
            self._apply_theme()
            self._harden_summary_table_surface()
            self._lock_to_available_geometry()
            self._load_metrics()
            self._set_running_buttons(False)
            self._set_run_status('Idle', COLORS['muted'])
            self._set_eval_status('Missing Output', COLORS['orange'])

            self.preview_timer = QtCore.QTimer(self)
            self.preview_timer.setInterval(330)
            self.preview_timer.timeout.connect(self._advance_preview)

            self._dataset_changed(self.current_dataset, auto_evaluate=False)
            if auto_refresh:
                QtCore.QTimer.singleShot(0, self.refresh_results)
            elif auto_evaluate:
                QtCore.QTimer.singleShot(
                    0,
                    lambda: self._dataset_changed(
                        self.current_dataset,
                        auto_evaluate=True))

        def _lock_to_available_geometry(self):
            screen = QtWidgets.QApplication.screenAt(QtGui.QCursor.pos())
            if screen is None:
                screen = QtWidgets.QApplication.primaryScreen()
            geometry = QtCore.QRect(screen.availableGeometry())
            geometry.adjust(TASKBAR_SAFE_MARGIN, TASKBAR_SAFE_MARGIN,
                            -TASKBAR_SAFE_MARGIN, -TASKBAR_SAFE_MARGIN)
            self.setMinimumSize(640, 480)
            self.setMaximumSize(16777215, 16777215)
            self.setGeometry(geometry)

        def showEvent(self, event):
            super().showEvent(event)
            if not self.geometry_locked:
                QtCore.QTimer.singleShot(0, self._apply_taskbar_safe_lock)

        def _apply_taskbar_safe_lock(self):
            screen = QtWidgets.QApplication.screenAt(self.frameGeometry().center())
            if screen is None:
                screen = QtWidgets.QApplication.primaryScreen()
            available = QtCore.QRect(screen.availableGeometry())
            available.adjust(TASKBAR_SAFE_MARGIN, TASKBAR_SAFE_MARGIN,
                             -TASKBAR_SAFE_MARGIN, -TASKBAR_SAFE_MARGIN)

            frame = self.frameGeometry()
            client = self.geometry()
            left_frame = max(0, client.left() - frame.left())
            top_frame = max(0, client.top() - frame.top())
            right_frame = max(0, frame.right() - client.right())
            bottom_frame = max(0, frame.bottom() - client.bottom())
            client_width = max(320, available.width() - left_frame - right_frame)
            client_height = max(320, available.height() - top_frame - bottom_frame)
            locked = QtCore.QRect(
                available.left() + left_frame,
                available.top() + top_frame,
                client_width,
                client_height,
            )
            self.setMinimumSize(1, 1)
            self.setMaximumSize(16777215, 16777215)
            self.setGeometry(locked)
            self.setMinimumSize(locked.size())
            self.setMaximumSize(locked.size())
            self.geometry_locked = True

        def _build_ui(self):
            root = QtWidgets.QWidget()
            root.setObjectName('AppRoot')
            self.setCentralWidget(root)
            outer = QtWidgets.QVBoxLayout(root)
            outer.setContentsMargins(12, 12, 12, 12)
            outer.setSpacing(11)

            top = QtWidgets.QWidget()
            top_layout = QtWidgets.QHBoxLayout(top)
            top_layout.setContentsMargins(0, 0, 0, 0)
            top_layout.setSpacing(11)
            self.left_sidebar = self._build_left_sidebar()
            self.center_area = self._build_center_area()
            self.right_sidebar = self._build_right_sidebar()
            self.left_sidebar.setFixedWidth(285)
            self.right_sidebar.setFixedWidth(245)
            top_layout.addWidget(self.left_sidebar)
            top_layout.addWidget(self.center_area, 1)
            top_layout.addWidget(self.right_sidebar)
            bottom_panel = self._build_bottom_dashboard()
            bottom_panel.setFixedHeight(205)
            bottom_panel.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            outer.addWidget(top, 1)
            outer.addWidget(bottom_panel, 0)

        def _build_left_sidebar(self):
            panel = PanelFrame('1. RUN & LIVE VIEW')
            panel.setObjectName('SidebarPanel')

            panel.body.addWidget(self._field_label('Dataset'))
            self.dataset_combo = QtWidgets.QComboBox()
            self.dataset_combo.addItems(DATASETS)
            self.dataset_combo.currentTextChanged.connect(self._dataset_changed)
            panel.body.addWidget(self.dataset_combo)

            panel.body.addWidget(self._field_label('Offset (frames)'))
            self.offset_spin = QtWidgets.QSpinBox()
            self.offset_spin.setRange(0, 300)
            self.offset_spin.setSingleStep(1)
            self.offset_spin.setValue(self.default_offset)
            panel.body.addWidget(self.offset_spin)

            panel.body.addWidget(self._field_label('Output Directory'))
            output_row = QtWidgets.QHBoxLayout()
            output_row.setSpacing(6)
            self.output_dir_edit = QtWidgets.QLineEdit(str(self.output_root))
            self.output_dir_edit.setToolTip(str(self.output_root))
            output_button = QtWidgets.QPushButton('...')
            output_button.setObjectName('TinyButton')
            output_button.clicked.connect(self._select_output_root)
            output_row.addWidget(self.output_dir_edit, 1)
            output_row.addWidget(output_button)
            panel.body.addLayout(output_row)

            panel.body.addWidget(self._field_label('Results Root'))
            results_row = QtWidgets.QHBoxLayout()
            results_row.setSpacing(6)
            self.results_root_edit = QtWidgets.QLineEdit(str(self.results_root))
            self.results_root_edit.setToolTip(str(self.results_root))
            results_button = QtWidgets.QPushButton('...')
            results_button.setObjectName('TinyButton')
            results_button.clicked.connect(self._select_results_root)
            results_row.addWidget(self.results_root_edit, 1)
            results_row.addWidget(results_button)
            panel.body.addLayout(results_row)

            self.run_selected_button = self._button('Run Selected', 'RunButton', self.run_selected_dataset)
            self.run_all_button = self._button('Run All 5 Datasets', 'BlueButton', self.run_all_datasets)
            self.stop_button = self._button('Stop', 'StopButton', self.stop_current_run)
            panel.body.addWidget(self.run_selected_button)
            panel.body.addWidget(self.run_all_button)
            panel.body.addWidget(self.stop_button)

            self.status_block = StatusBlock()
            panel.body.addWidget(self.status_block)

            self.progress_bar = QtWidgets.QProgressBar()
            self.progress_bar.setObjectName('ThinProgress')
            self.progress_bar.setRange(0, 100)
            self.progress_bar.setValue(0)
            self.progress_bar.setTextVisible(False)
            panel.body.addWidget(self.progress_bar)

            panel.body.addWidget(self._field_label('Console Preview'))
            self.log_preview = QtWidgets.QTextEdit()
            self.log_preview.setObjectName('ConsolePreview')
            self.log_preview.setReadOnly(True)
            self.log_preview.setMinimumHeight(62)
            self.log_preview.setMaximumHeight(92)
            panel.body.addWidget(self.log_preview, 1)
            return panel

        def _build_center_area(self):
            panel = PanelFrame()
            panel.setObjectName('CenterPanel')
            self.tabs = QtWidgets.QTabWidget()
            self.tabs.setObjectName('MainTabs')
            tab_bar = self.tabs.tabBar()
            tab_bar.setElideMode(QtCore.Qt.ElideNone)
            tab_bar.setExpanding(False)
            tab_bar.setUsesScrollButtons(True)
            self.tabs.setMovable(False)
            panel.body.addWidget(self.tabs, 1)

            self.live_view = LiveRunView(COLORS)
            self.tabs.addTab(self.live_view, 'Live View')

            self.xy_plot = PlotPanel('Trajectory (XY)', toolbar=True)
            self.xz_plot = PlotPanel('Trajectory (XZ)', toolbar=True)
            self.yz_plot = PlotPanel('Trajectory (YZ)', toolbar=True)
            self.ate_plot = PlotPanel('ATE Plot', toolbar=True)
            self.rpe_plot = PlotPanel('RPE/RTE Plot', toolbar=True)
            self.tabs.addTab(self.xy_plot, 'Trajectory (XY)')
            self.tabs.addTab(self.xz_plot, 'Trajectory (XZ)')
            self.tabs.addTab(self.yz_plot, 'Trajectory (YZ)')
            self.tabs.addTab(self.ate_plot, 'ATE Plot')
            self.tabs.addTab(self.rpe_plot, 'RPE/RTE Plot')
            return panel

        def _build_right_sidebar(self):
            panel = PanelFrame('2. SINGLE EVALUATION')
            panel.setObjectName('SidebarPanel')

            panel.body.addWidget(self._field_label('Select Estimate (result file)'))
            estimate_row = QtWidgets.QHBoxLayout()
            estimate_row.setSpacing(6)
            self.estimate_combo = QtWidgets.QComboBox()
            self.estimate_combo.setMinimumContentsLength(18)
            browse_button = QtWidgets.QPushButton('...')
            browse_button.setObjectName('TinyButton')
            browse_button.clicked.connect(self._browse_estimate)
            estimate_row.addWidget(self.estimate_combo, 1)
            estimate_row.addWidget(browse_button)
            panel.body.addLayout(estimate_row)

            self.show_rejected_checkbox = QtWidgets.QCheckBox('Show rejected experiments')
            self.show_rejected_checkbox.setObjectName('RejectedToggle')
            self.show_rejected_checkbox.stateChanged.connect(self._show_rejected_changed)
            panel.body.addWidget(self.show_rejected_checkbox)

            panel.body.addWidget(self._button('Evaluate', 'BlueButton', self.evaluate_selected))

            metrics_label = QtWidgets.QLabel('METRICS')
            metrics_label.setObjectName('SubsectionTitle')
            panel.body.addWidget(metrics_label)

            self.metric_cards = {}
            for key, label in [
                ('ate_rmse_m', 'ATE RMSE'),
                ('ate_mean_m', 'ATE Mean'),
                ('ate_median_m', 'ATE Median'),
                ('ate_max_m', 'ATE Max'),
                ('rpe_1s_rmse_m', 'RPE 1s RMSE'),
                ('aligned_samples', 'Aligned Samples'),
                ('overlap_duration_s', 'Overlap Duration'),
            ]:
                card = MetricCard(label)
                self.metric_cards[key] = card
                panel.body.addWidget(card)

            provenance_label = QtWidgets.QLabel('DATA SOURCE')
            provenance_label.setObjectName('SubsectionTitle')
            panel.body.addWidget(provenance_label)
            self.provenance_block = ProvenanceBlock()
            panel.body.addWidget(self.provenance_block)

            utility = QtWidgets.QGridLayout()
            utility.setSpacing(6)
            utility.addWidget(self._button('Refresh Results', 'GhostButton', self.refresh_results), 0, 0)
            utility.addWidget(self._button('Export Current Plot', 'GhostButton', self.export_current_plot), 0, 1)
            utility.addWidget(self._button('Export Summary', 'GhostButton', self.export_summary), 1, 0)
            utility.addWidget(self._button('Open Results Folder', 'GhostButton', self.open_results_folder), 1, 1)
            panel.body.addLayout(utility)
            panel.body.addStretch(1)
            return panel

        def _build_bottom_dashboard(self):
            panel = PanelFrame('3. GLOBAL COMPARISON DASHBOARD')
            panel.setObjectName('BottomPanel')

            body = QtWidgets.QHBoxLayout()
            body.setContentsMargins(0, 0, 0, 0)
            body.setSpacing(10)
            panel.body.addLayout(body, 1)

            # Left: compact comparison table
            self.summary_model = GlobalResultsModel(self)
            self.summary_table = DarkResultsTable()
            self.summary_table.setModel(self.summary_model)
            configure_results_table(self.summary_table)
            self.summary_table.setFixedWidth(720)
            self.summary_table.setSizePolicy(
                QtWidgets.QSizePolicy.Fixed,
                QtWidgets.QSizePolicy.Expanding,
            )
            self.summary_table.selectionModel().selectionChanged.connect(self._table_selection_changed)
            body.addWidget(self.summary_table, 0)

            # Middle: creator info card, no avatar icon
            creator_card = QtWidgets.QFrame()
            creator_card.setObjectName('CreatorCard')
            creator_card.setFixedWidth(300)
            creator_card.setSizePolicy(
                QtWidgets.QSizePolicy.Fixed,
                QtWidgets.QSizePolicy.Expanding,
            )

            creator_layout = QtWidgets.QVBoxLayout(creator_card)
            creator_layout.setContentsMargins(16, 10, 16, 10)
            creator_layout.setSpacing(6)

            creator_title = QtWidgets.QLabel('CREATOR INFO')
            creator_title.setObjectName('CreatorTitle')
            creator_layout.addWidget(creator_title)

            creator_grid = QtWidgets.QGridLayout()
            creator_grid.setContentsMargins(0, 6, 0, 0)
            creator_grid.setHorizontalSpacing(18)
            creator_grid.setVerticalSpacing(8)

            def add_creator_row(row, label_text, value_text):
                label = QtWidgets.QLabel(label_text)
                label.setObjectName('CreatorLabel')
                label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)

                value = QtWidgets.QLabel(value_text)
                value.setObjectName('CreatorValue')
                value.setWordWrap(True)
                value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
                value.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)

                creator_grid.addWidget(label, row, 0)
                creator_grid.addWidget(value, row, 1)

            add_creator_row(0, 'Developer', 'Sara Saberi')
            add_creator_row(1, 'Project', 'UAV-vio-navigation')
            add_creator_row(2, 'GitHub', 'github.com/ZalSaberi')
            add_creator_row(3, 'Email', 'Zal.saberi.s@gmail.com')
            add_creator_row(4, 'Linkedin', 'linkedin.com/in/saberisara')


            creator_grid.setColumnStretch(0, 0)
            creator_grid.setColumnStretch(1, 1)

            creator_layout.addLayout(creator_grid)
            creator_layout.addStretch(1)

            body.addWidget(creator_card, 0)

            # Right: global charts
            self.global_chart_tabs = QtWidgets.QTabWidget()
            self.global_chart_tabs.setObjectName('GlobalChartTabs')
            self.global_chart_tabs.tabBar().setElideMode(QtCore.Qt.ElideNone)
            self.global_chart_tabs.tabBar().setExpanding(False)
            self.global_chart_tabs.tabBar().setUsesScrollButtons(False)
            self.global_chart_tabs.setMinimumWidth(520)
            self.global_chart_tabs.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Expanding,
            )

            self.ate_bar_plot = PlotPanel('ATE RMSE (m) - All Datasets')
            self.rpe_bar_plot = PlotPanel('RPE 1s RMSE (m) - All Datasets')
            self.global_chart_tabs.addTab(self.ate_bar_plot, 'ATE RMSE')
            self.global_chart_tabs.addTab(self.rpe_bar_plot, 'RPE 1s')

            body.addWidget(self.global_chart_tabs, 1)

            return panel

        def _apply_theme(self):
            self.setStyleSheet(f"""
                QWidget#AppRoot {{
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                        stop:0 {COLORS['background']}, stop:0.55 #0B1424, stop:1 #07111F);
                    color: {COLORS['text']};
                    font-family: "Segoe UI", Arial, sans-serif;
                    font-size: 12px;
                }}
                QFrame#SidebarPanel, QFrame#CenterPanel, QFrame#BottomPanel, QFrame#PanelFrame {{
                    background: rgba(17, 24, 39, 228);
                    border: 1px solid rgba(255, 255, 255, 0.08);
                    border-radius: 12px;
                }}
                QFrame#ChartCard, QFrame#MetricCard, QFrame#StatusCard {{
                    background: {COLORS['panel_soft']};
                    border: 1px solid rgba(255, 255, 255, 0.08);
                    border-radius: 11px;
                }}
                QLabel#SectionTitle {{
                    color: {COLORS['text']};
                    font-size: 17px;
                    font-weight: 700;
                    letter-spacing: 0;
                }}
                QLabel#SubsectionTitle {{
                    color: {COLORS['muted']};
                    font-size: 12px;
                    font-weight: 700;
                    padding-top: 4px;
                }}
                QLabel#PanelTitle {{
                    color: {COLORS['text']};
                    font-size: 13px;
                    font-weight: 700;
                }}
                QLabel#FieldLabel, QLabel#StatusName, QLabel#MetricTitle {{
                    color: {COLORS['muted']};
                    font-size: 10px;
                    font-weight: 600;
                }}
                QLabel#StatusValue {{
                    color: {COLORS['text']};
                    font-size: 10px;
                    font-weight: 600;
                }}

                QFrame#CreatorCard {{
                    background: {COLORS['panel_soft']};
                    border: 1px solid rgba(255, 255, 255, 0.08);
                    border-radius: 11px;
                }}
                QLabel#CreatorTitle {{
                    color: {COLORS['text']};
                    font-size: 12px;
                    font-weight: 800;
                    letter-spacing: 0.5px;
                }}
                QLabel#CreatorLabel {{
                    color: {COLORS['muted']};
                    font-size: 9px;
                    font-weight: 700;
                }}
                QLabel#CreatorValue {{
                    color: {COLORS['text']};
                    font-size: 10px;
                    font-weight: 600;
                }}
                QTabWidget#GlobalChartTabs::pane {{
                    border: 1px solid rgba(255, 255, 255, 0.08);
                    border-radius: 10px;
                    background: {COLORS['panel_soft']};
                    top: -1px;
                }}
                QTabWidget#GlobalChartTabs QTabBar::tab {{
                    background: {COLORS['panel_deep']};
                    color: {COLORS['muted']};
                    padding: 5px 14px;
                    margin-right: 3px;
                    min-width: 110px;
                    border-top-left-radius: 7px;
                    border-top-right-radius: 7px;
                    font-size: 9px;
                    font-weight: 700;
                }}
                QTabWidget#GlobalChartTabs QTabBar::tab:selected {{
                    background: {COLORS['accent']};
                    color: white;
                }}QLabel#MetricValue {{
                    color: {COLORS['text']};
                    font-size: 13px;
                    font-weight: 700;
                }}
                QLabel#CameraImage {{
                    background: #020617;
                    border: 1px solid rgba(255, 255, 255, 0.06);
                    border-radius: 9px;
                    color: {COLORS['muted']};
                    font-size: 13px;
                }}
                QPushButton {{
                    border: 0;
                    border-radius: 9px;
                    padding: 6px 9px;
                    font-weight: 700;
                    color: {COLORS['text']};
                }}
                QPushButton#RunButton {{ background: {COLORS['green']}; }}
                QPushButton#RunButton:hover {{ background: #2DD978; }}
                QPushButton#BlueButton {{ background: {COLORS['accent']}; }}
                QPushButton#BlueButton:hover {{ background: #3F7BFF; }}
                QPushButton#StopButton {{ background: {COLORS['red']}; }}
                QPushButton#StopButton:hover {{ background: #EA5353; }}
                QPushButton#TinyButton {{
                    background: #1F2A44;
                    min-width: 28px;
                    max-width: 34px;
                    padding-left: 0;
                    padding-right: 0;
                }}
                QPushButton#GhostButton {{
                    background: #162033;
                    color: {COLORS['muted']};
                    font-size: 10px;
                    padding: 6px;
                }}
                QPushButton#GhostButton:hover {{ background: #1C2B48; color: {COLORS['text']}; }}
                QCheckBox#RejectedToggle {{
                    color: {COLORS['muted']};
                    font-size: 10px;
                    font-weight: 600;
                    spacing: 8px;
                }}
                QCheckBox#RejectedToggle::indicator {{
                    width: 14px;
                    height: 14px;
                    border-radius: 4px;
                    border: 1px solid rgba(255, 255, 255, 0.18);
                    background: #07101F;
                }}
                QCheckBox#RejectedToggle::indicator:checked {{
                    background: {COLORS['accent']};
                    border: 1px solid {COLORS['accent']};
                }}
                QLineEdit, QComboBox, QSpinBox, QTextEdit {{
                    background: #07101F;
                    color: {COLORS['text']};
                    border: 1px solid rgba(255, 255, 255, 0.10);
                    border-radius: 8px;
                    padding: 7px;
                    selection-background-color: {COLORS['accent']};
                }}
                QTextEdit#ConsolePreview {{
                    font-family: Consolas, "Cascadia Mono", monospace;
                    font-size: 10px;
                    color: #C8D3E3;
                }}
                QProgressBar#ThinProgress {{
                    background: #07101F;
                    border: 1px solid rgba(255, 255, 255, 0.08);
                    border-radius: 5px;
                    min-height: 8px;
                    max-height: 8px;
                }}
                QProgressBar#ThinProgress::chunk {{
                    background: {COLORS['accent']};
                    border-radius: 5px;
                }}
                QTabWidget::pane {{
                    border: 0;
                    padding-top: 6px;
                }}
                QTabBar::tab {{
                    background: #101A2A;
                    color: {COLORS['muted']};
                    padding: 6px 12px;
                    margin-right: 3px;
                    min-width: 126px;
                    border-top-left-radius: 8px;
                    border-top-right-radius: 8px;
                    font-weight: 700;
                    font-size: 10px;
                }}
                QTabBar::tab:selected {{
                    background: {COLORS['accent']};
                    color: {COLORS['text']};
                }}
                QTableView#SummaryTable {{
                    background: #07101F;
                    background-color: #07101F;
                    alternate-background-color: #0D1728;
                    color: {COLORS['text']};
                    gridline-color: rgba(255, 255, 255, 0.06);
                    border: 1px solid rgba(255, 255, 255, 0.08);
                    border-radius: 9px;
                    selection-background-color: #20375D;
                    selection-color: {COLORS['text']};
                }}
                QTableView#SummaryTable::item {{
                    padding-left: 4px;
                    padding-right: 4px;
                    border: 0;
                }}
                QTableView#SummaryTable::item:selected {{
                    background: #20375D;
                    color: {COLORS['text']};
                }}
                QTableView#SummaryTable::viewport {{
                    background: #07101F;
                }}
                QTableCornerButton::section {{
                    background: #142033;
                    border: 0;
                }}
                QHeaderView::section {{
                    background: #142033;
                    color: {COLORS['muted']};
                    border: 0;
                    padding: 7px;
                    font-weight: 700;
                }}
                QScrollBar:horizontal, QScrollBar:vertical {{
                    background: #07101F;
                    border: 0;
                    margin: 0;
                }}
                QScrollBar::handle:horizontal, QScrollBar::handle:vertical {{
                    background: #26364F;
                    border-radius: 4px;
                    min-height: 18px;
                    min-width: 18px;
                }}
                QScrollBar::add-line, QScrollBar::sub-line {{
                    width: 0;
                    height: 0;
                }}
                QToolBar#ChartToolbar {{
                    background: #0B1220;
                    border: 1px solid rgba(255, 255, 255, 0.06);
                    border-radius: 7px;
                    spacing: 2px;
                }}
            """)

        def _harden_summary_table_surface(self):
            palette = self.summary_table.palette()
            palette.setColor(QtGui.QPalette.Base, QtGui.QColor('#07101F'))
            palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor('#0D1728'))
            palette.setColor(QtGui.QPalette.Window, QtGui.QColor('#07101F'))
            self.summary_table.setPalette(palette)
            self.summary_table.setAutoFillBackground(True)

            viewport = self.summary_table.viewport()
            viewport_palette = viewport.palette()
            viewport_palette.setColor(QtGui.QPalette.Base, QtGui.QColor('#07101F'))
            viewport_palette.setColor(QtGui.QPalette.Window, QtGui.QColor('#07101F'))
            viewport.setPalette(viewport_palette)
            viewport.setAutoFillBackground(True)
            viewport.setStyleSheet('background-color: #07101F;')

        def _field_label(self, text):
            label = QtWidgets.QLabel(text)
            label.setObjectName('FieldLabel')
            return label

        def _button(self, text, object_name, slot):
            button = QtWidgets.QPushButton(text)
            button.setObjectName(object_name)
            button.clicked.connect(slot)
            return button

        def _set_run_status(self, status, color=None):
            color = color or COLORS['cyan']
            self.status_block.set_value('Run', status, color)

        def _set_eval_status(self, status, color=None):
            color = color or COLORS['cyan']
            self.status_block.set_value('Evaluation', status, color)

        def _set_status(self, status, color=None):
            self._set_eval_status(status, color)

        def _set_status_values(self, dataset=None, frame=None, timestamp=None,
                               fps=None, output=None):
            if dataset is not None:
                self.status_block.set_value('Dataset', dataset)
            if frame is not None:
                self.status_block.set_value('Frame', frame)
            if timestamp is not None:
                self.status_block.set_value('Time', timestamp)
            if fps is not None:
                self.status_block.set_value('FPS', fps)
            if output is not None:
                self.status_block.set_value(
                    'Output File',
                    display_path(output, self.results_root, max_chars=34),
                    tooltip=output)

        def append_log(self, text):
            if not text:
                return
            for line in str(text).splitlines():
                if line.strip():
                    self.log_lines.append(line.rstrip())
            self.log_preview.setPlainText('\n'.join(self.log_lines))
            cursor = self.log_preview.textCursor()
            cursor.movePosition(cursor.End)
            self.log_preview.setTextCursor(cursor)

        def _select_output_root(self):
            directory = QtWidgets.QFileDialog.getExistingDirectory(
                self, 'Select output root', self.output_dir_edit.text())
            if directory:
                self.output_dir_edit.setText(directory)
                self.output_dir_edit.setToolTip(directory)

        def _select_results_root(self):
            directory = QtWidgets.QFileDialog.getExistingDirectory(
                self, 'Select results root', self.results_root_edit.text())
            if not directory:
                return
            self.results_root_edit.setText(directory)
            self.results_root_edit.setToolTip(directory)
            self.results_root = Path(directory)
            self.metrics_path = self.results_root / METRICS_FILENAME
            self._load_metrics()
            self._populate_estimates(self.current_dataset)
            self.refresh_results()

        def _browse_estimate(self):
            start_dir = str(self.results_root)
            path, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, 'Select estimate file', start_dir, 'Trajectory text (*.txt);;All files (*)')
            if not path:
                return
            dataset_name = self.dataset_combo.currentText()
            self._add_estimate_option(Path(path), select=True)
            self.start_evaluation(dataset_name, Path(path))

        def _show_rejected_changed(self):
            self._populate_estimates(self.current_dataset)
            self.refresh_results()

        def _load_metrics(self):
            self.rows_by_dataset = {name: empty_row(name) for name in DATASETS}
            if not self.metrics_path.exists():
                self._update_table()
                self._update_bar_charts()
                return
            try:
                data = json.loads(self.metrics_path.read_text())
            except (OSError, json.JSONDecodeError):
                self._update_table()
                self._update_bar_charts()
                return
            if not isinstance(data, list):
                return
            for row in data:
                dataset_name = row.get('dataset')
                if dataset_name in DATASETS:
                    if row.get('ate_rmse_m') is not None:
                        row['data_source_status'] = 'Loaded from cache'
                    row.setdefault('dataset_key', dataset_name)
                    row.setdefault('groundtruth_path', '')
                    row.setdefault('computed_at', row.get('evaluated_at', '-'))
                    row.setdefault('alignment_mode', 'se3')
                    self.rows_by_dataset[dataset_name] = row
            self._update_table()
            self._update_bar_charts()

        def _save_metrics(self):
            rows = [self.rows_by_dataset[name] for name in DATASETS]
            try:
                self.results_root.mkdir(parents=True, exist_ok=True)
                self.metrics_path.write_text(json.dumps(rows, indent=2))
            except OSError as exc:
                if hasattr(self, 'append_log'):
                    self.append_log(f'Could not update metrics cache: {exc}')

        def _dataset_changed(self, dataset_name, auto_evaluate=True):
            if not dataset_name:
                return
            auto_evaluate = auto_evaluate and not self.suppress_auto_evaluate
            auto_evaluate = auto_evaluate and self.process is None
            self.current_dataset = dataset_name
            try:
                resolved = DATASET_REGISTRY.resolve(dataset_name, self.datasets_root)
                checks = resolved.validate()
            except DatasetRegistryError as exc:
                self._set_run_status('Idle', COLORS['muted'])
                self._set_eval_status('Missing Dataset', COLORS['red'])
                self.append_log(str(exc))
                return
            if self.process is None:
                self.offset_spin.setValue(resolved.default_offset)
            if self.process is None:
                self._set_run_status('Idle', COLORS['muted'])
            if not checks['dataset']:
                self._set_eval_status('Missing Dataset', COLORS['red'])
            elif not checks['groundtruth']:
                self._set_eval_status('Missing Ground Truth', COLORS['red'])
            elif not checks['first_frame']:
                self._set_eval_status('Missing Frame Preview', COLORS['orange'])
            else:
                self._set_eval_status('Preview Loaded', COLORS['cyan'])
            self._set_status_values(dataset=dataset_name, fps='0.0', output='-')
            self._load_camera_preview(dataset_name)
            self._populate_estimates(dataset_name)
            row = self.rows_by_dataset.get(dataset_name, empty_row(dataset_name))
            if row.get('estimate'):
                estimate_candidate = Path(row['estimate'])
                if estimate_candidate.exists():
                    self._add_estimate_option(estimate_candidate, select=True)
            self._show_row(row)
            estimate_path = self.current_estimate_path()
            if auto_evaluate and estimate_path and str(estimate_path) not in self.result_cache:
                self.start_evaluation(dataset_name, estimate_path, automatic=True)
            elif estimate_path and str(estimate_path) in self.result_cache:
                self._update_current_plots(self.result_cache[str(estimate_path)])

        def _load_camera_preview(self, dataset_name):
            try:
                resolved = DATASET_REGISTRY.resolve(dataset_name, self.datasets_root)
                times, paths = resolved.cam0_frame_index()
            except (DatasetRegistryError, OSError) as exc:
                self.camera_frame_times = []
                self.camera_frame_paths = []
                self.camera_frame_index = -1
                self.live_view.set_message(f'No frame preview available\n{exc}')
                self._set_status_values(frame='-', timestamp='-')
                return
            self.camera_frame_times = times
            self.camera_frame_paths = paths
            self.camera_frame_index = -1
            if not paths:
                self.live_view.set_message('No frame preview available')
                self._set_status_values(frame='-', timestamp='-')
                return
            self._show_camera_frame(0)

        def _show_camera_frame(self, index):
            if index < 0 or index >= len(self.camera_frame_paths):
                return
            self.camera_frame_index = index
            self.live_view.update_image_from_path(self.camera_frame_paths[index])
            time_text = '-'
            if self.camera_frame_times:
                first = self.camera_frame_times[0]
                time_text = f'{self.camera_frame_times[index] - first:.3f} s'
            self._set_status_values(
                frame=f'{index + 1} / {len(self.camera_frame_paths)}',
                timestamp=time_text)
            if self.camera_frame_paths:
                progress = int(100 * (index + 1) / len(self.camera_frame_paths))
                self.progress_bar.setValue(max(0, min(100, progress)))

        def _update_camera_preview_for_timestamp(self, timestamp):
            if not self.camera_frame_times:
                return
            index = int(np.searchsorted(self.camera_frame_times, timestamp, side='left'))
            if index >= len(self.camera_frame_times):
                index = len(self.camera_frame_times) - 1
            if index > 0:
                previous = abs(self.camera_frame_times[index - 1] - timestamp)
                current = abs(self.camera_frame_times[index] - timestamp)
                if previous < current:
                    index -= 1
            self._show_camera_frame(index)

        def _advance_preview(self):
            if not self.camera_frame_paths:
                return
            step = max(1, len(self.camera_frame_paths) // 1200)
            next_index = self.camera_frame_index + step
            if next_index >= len(self.camera_frame_paths):
                next_index = len(self.camera_frame_paths) - 1
            self._show_camera_frame(next_index)
            self.preview_frames_shown += 1
            if self.run_started_at is not None:
                elapsed = max(1e-3, time.time() - self.run_started_at)
                self._set_status_values(fps=f'{self.preview_frames_shown / elapsed:.1f}')

        def _populate_estimates(self, dataset_name):
            include_rejected = self.show_rejected_checkbox.isChecked()
            estimates = find_estimates(
                self.results_root, dataset_name,
                include_rejected=include_rejected)
            self.estimate_cache[dataset_name] = estimates
            self.estimate_combo.blockSignals(True)
            self.estimate_combo.clear()
            if not estimates:
                self.estimate_combo.addItem('No result files found', None)
            else:
                for estimate in estimates:
                    text = display_path(estimate, self.results_root, max_chars=46)
                    if is_rejected_estimate_path(estimate):
                        text = f'[rejected] {text}'
                    self.estimate_combo.addItem(text, str(estimate))
                    self.estimate_combo.setItemData(
                        self.estimate_combo.count() - 1,
                        str(estimate),
                        QtCore.Qt.ToolTipRole)
            self.estimate_combo.blockSignals(False)

        def _add_estimate_option(self, estimate_path, select=False):
            estimate_path = Path(estimate_path)
            text = display_path(estimate_path, self.results_root, max_chars=46)
            if is_rejected_estimate_path(estimate_path):
                text = f'[rejected] {text}'
            for index in range(self.estimate_combo.count()):
                if self.estimate_combo.itemData(index) == str(estimate_path):
                    if select:
                        self.estimate_combo.setCurrentIndex(index)
                    return
            self.estimate_combo.addItem(text, str(estimate_path))
            self.estimate_combo.setItemData(
                self.estimate_combo.count() - 1,
                str(estimate_path),
                QtCore.Qt.ToolTipRole)
            if select:
                self.estimate_combo.setCurrentIndex(self.estimate_combo.count() - 1)

        def current_estimate_path(self):
            data = self.estimate_combo.currentData()
            return Path(data) if data else None

        def run_selected_dataset(self):
            self.run_queue = []
            self.stop_requested = False
            self._start_dataset_run(self.dataset_combo.currentText(), continue_queue=False)

        def run_all_datasets(self):
            if self.process is not None:
                self.append_log('A run is already active.')
                return
            self.stop_requested = False
            self.run_queue = list(DATASETS)
            self._run_next_in_queue()

        def _run_next_in_queue(self):
            if self.stop_requested or not self.run_queue:
                self._set_run_status('Stopped' if self.stop_requested else 'Completed',
                                     COLORS['orange'] if self.stop_requested else COLORS['green'])
                self.refresh_results()
                return
            dataset_name = self.run_queue.pop(0)
            self.suppress_auto_evaluate = True
            self.dataset_combo.setCurrentText(dataset_name)
            self.suppress_auto_evaluate = False
            self._dataset_changed(dataset_name, auto_evaluate=False)
            self._start_dataset_run(dataset_name, continue_queue=True)

        def _start_dataset_run(self, dataset_name, continue_queue):
            if self.process is not None:
                self.append_log('A run is already active.')
                return
            offset = int(self.offset_spin.value())
            output_root = Path(self.output_dir_edit.text())
            try:
                resolved = DATASET_REGISTRY.resolve(dataset_name, self.datasets_root)
            except DatasetRegistryError as exc:
                self._set_run_status('Run Failed', COLORS['red'])
                self._set_eval_status('Missing Dataset', COLORS['red'])
                self.append_log(str(exc))
                return
            checks = resolved.validate()
            if not resolved.is_runnable():
                missing = ', '.join(name for name, ok in checks.items() if not ok)
                self._set_run_status('Run Failed', COLORS['red'])
                self._set_eval_status('Missing Dataset', COLORS['red'])
                self.append_log(f'Cannot run {dataset_name}; missing registry inputs: {missing}')
                return

            run_dir = resolved.default_output_dir(output_root, offset)
            dataset_dir = resolved.dataset_path
            expected_output = resolved.expected_output_path(run_dir, offset)
            command = [
                sys.executable,
                'main.py',
                '--path',
                str(dataset_dir),
                '--offset',
                str(offset),
                '--output-dir',
                str(run_dir),
            ]

            self.current_output_file = expected_output
            self.continue_queue_after_eval = continue_queue
            self.previous_rows_before_run[dataset_name] = self.rows_by_dataset.get(
                dataset_name, empty_row(dataset_name))
            self.rows_by_dataset[dataset_name] = empty_row(dataset_name, 'candidate', COLORS['cyan'])
            self.rows_by_dataset[dataset_name]['estimate'] = str(expected_output)
            self.rows_by_dataset[dataset_name]['estimate_relpath'] = relative_path(expected_output, self.results_root)
            self._update_table()
            self._set_run_status('Running', COLORS['cyan'])
            self._set_eval_status('Missing Output', COLORS['orange'])
            self._set_status_values(
                dataset=dataset_name,
                output=relative_path(expected_output, self.results_root),
                fps='0.0')
            self.progress_bar.setValue(0)
            self.run_started_at = time.time()
            self.preview_frames_shown = 0
            self._last_stdout_timestamp = None
            self.live_view.reset_run()
            self._set_running_buttons(True)
            self.append_log('Running: ' + ' '.join(command))

            self.process = QtCore.QProcess(self)


            process_env = QtCore.QProcessEnvironment.systemEnvironment()


            process_env.insert('PYTHONUNBUFFERED', '1')


            process_env.insert('PYTHONIOENCODING', 'utf-8')


            self.process.setProcessEnvironment(process_env)
            self.process.setWorkingDirectory(str(REPO_ROOT))
            self.process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
            self.process.readyReadStandardOutput.connect(self._read_process_output)
            self.process.finished.connect(
                lambda exit_code, exit_status: self._run_finished(
                    dataset_name, expected_output, exit_code, exit_status))
            self.process.start(command[0], command[1:])

        def _parse_stdout_vector(self, line):
            match = re.search(r'\[([^\]]+)\]', line)
            if not match:
                return None
            try:
                values = [float(part) for part in match.group(1).replace(',', ' ').split()]
            except ValueError:
                return None
            if len(values) < 3:
                return None
            return np.asarray(values[:3], dtype=float)

        def _read_process_output(self):
            if self.process is None:
                return
            text = bytes(self.process.readAllStandardOutput()).decode(errors='replace')
            if not text:
                return
            lines = text.splitlines()
            for line in lines[-40:]:
                self.append_log(line)
                stripped = line.strip()

                if stripped.startswith('timestamp:') or 'timestamp:' in stripped:
                    try:
                        timestamp = float(stripped.split('timestamp:', 1)[1].strip().split()[0])
                    except (IndexError, ValueError):
                        continue
                    self._last_stdout_timestamp = timestamp
                    self._update_camera_preview_for_timestamp(timestamp)
                    continue

                if stripped.startswith('position:') or 'position:' in stripped:
                    position = self._parse_stdout_vector(stripped)
                    if position is None:
                        continue
                    timestamp = self._last_stdout_timestamp
                    if timestamp is None:
                        timestamp = time.time()
                    self.live_view.append_pose(timestamp, position)
                    self._set_status_values(
                        fps=f'frames {self.live_view.frame_count} / poses {self.live_view.pose_count}')
                    continue

        def _run_finished(self, dataset_name, expected_output, exit_code, exit_status):
            self.process = None
            self.preview_timer.stop()
            self.live_view.freeze_final_state()
            self._set_running_buttons(False)
            if exit_code != 0:
                self._set_run_status('Run Failed', COLORS['red'])
                existing = self.previous_rows_before_run.pop(
                    dataset_name,
                    self.rows_by_dataset.get(dataset_name, empty_row(dataset_name)))
                if existing.get('ate_rmse_m') is not None:
                    self.rows_by_dataset[dataset_name] = existing
                    self._set_eval_status(existing.get('status', 'evaluated'), existing.get('color', COLORS['green']))
                else:
                    self._set_eval_status('Missing Output', COLORS['orange'])
                    self.rows_by_dataset[dataset_name] = empty_row(dataset_name, 'failed', COLORS['red'])
                    self.rows_by_dataset[dataset_name]['estimate'] = str(expected_output)
                    self.rows_by_dataset[dataset_name]['estimate_relpath'] = relative_path(expected_output, self.results_root)
                self._update_table()
                self.append_log(f'Run failed for {dataset_name}; exit code {exit_code}.')
                if self.continue_queue_after_eval:
                    self._run_next_in_queue()
                return

            self.progress_bar.setValue(100)
            self._set_run_status('Completed', COLORS['green'])
            self._set_eval_status('Evaluating', COLORS['orange'])
            self.append_log(f'Run completed for {dataset_name}.')
            if not expected_output.exists():
                self._set_eval_status('Missing Output', COLORS['orange'])
                current = self.previous_rows_before_run.pop(
                    dataset_name,
                    self.rows_by_dataset.get(dataset_name, empty_row(dataset_name)))
                if current.get('ate_rmse_m') is None:
                    self.rows_by_dataset[dataset_name] = empty_row(dataset_name, 'missing', COLORS['orange'])
                    self.rows_by_dataset[dataset_name]['estimate'] = str(expected_output)
                    self.rows_by_dataset[dataset_name]['estimate_relpath'] = relative_path(expected_output, self.results_root)
                    self._update_table()
                else:
                    self.rows_by_dataset[dataset_name] = current
                    self._show_row(current)
                    self._update_table()
                self.append_log(f'Expected output missing: {expected_output}')
                if self.continue_queue_after_eval:
                    self._run_next_in_queue()
                return
            self._add_estimate_option(expected_output, select=True)
            self.start_evaluation(dataset_name, expected_output, automatic=True)

        def stop_current_run(self):
            self.stop_requested = True
            self.run_queue = []
            self.preview_timer.stop()
            if self.process is not None:
                self.append_log('Stopping active process...')
                self.process.terminate()
                QtCore.QTimer.singleShot(4000, self._kill_if_running)
            self._set_run_status('Stopped', COLORS['orange'])
            self._set_running_buttons(False)

        def _kill_if_running(self):
            if self.process is not None:
                self.process.kill()

        def _set_running_buttons(self, running):
            self.run_selected_button.setEnabled(not running)
            self.run_all_button.setEnabled(not running)
            self.stop_button.setEnabled(running)

        def evaluate_selected(self):
            estimate_path = self.current_estimate_path()
            if estimate_path is None:
                self.append_log('No estimate selected for evaluation.')
                self._set_eval_status('Missing Output', COLORS['orange'])
                return
            self.start_evaluation(self.dataset_combo.currentText(), estimate_path)

        def start_evaluation(self, dataset_name, estimate_path, automatic=False):
            self._set_eval_status('Evaluating', COLORS['orange'])
            existing = self.rows_by_dataset.get(dataset_name, empty_row(dataset_name))
            if existing.get('ate_rmse_m') is None:
                self.rows_by_dataset[dataset_name] = empty_row(dataset_name, 'candidate', COLORS['cyan'])
            self.rows_by_dataset[dataset_name]['estimate'] = str(estimate_path)
            self.rows_by_dataset[dataset_name]['estimate_relpath'] = relative_path(estimate_path, self.results_root)
            self._update_table()

            QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
            error = None
            row = None
            result = None
            try:
                row, result = evaluate_estimate(
                    dataset_name, estimate_path, self.datasets_root, self.results_root)
            except Exception as exc:  # Keep the GUI boundary explicit.
                error = str(exc)
            finally:
                QtWidgets.QApplication.restoreOverrideCursor()

            if error is not None:
                self._evaluation_failed(dataset_name, estimate_path, error)
                return
            self._evaluation_finished(dataset_name, estimate_path, row, result)

        def _clear_eval_thread(self):
            self.eval_thread = None
            self.eval_worker = None

        def _evaluation_finished(self, dataset_name, estimate_path, row, result):
            self.rows_by_dataset[dataset_name] = row
            self.previous_rows_before_run.pop(dataset_name, None)
            self.result_cache[str(estimate_path)] = result
            self._save_metrics()
            self._show_row(row)
            self._update_current_plots(result)
            self._update_table()
            self._update_bar_charts()
            self._set_eval_status('Fresh' if row.get('accepted') else row['status'].title(), row['color'])
            self.append_log(
                f'Evaluated {Path(estimate_path)} against {row["groundtruth_path"]}: '
                f'ATE RMSE={row["ate_rmse_m"]:.3f} m, '
                f'RPE 1s RMSE={metric_text(row["rpe_1s_rmse_m"])}')
            if self.continue_queue_after_eval:
                self.continue_queue_after_eval = False
                self._run_next_in_queue()

        def _evaluation_failed(self, dataset_name, estimate_path, message):
            previous = self.previous_rows_before_run.pop(dataset_name, None)
            if previous is not None and previous.get('ate_rmse_m') is not None:
                row = previous
            else:
                row = empty_row(dataset_name, 'failed', COLORS['red'])
                row['estimate'] = str(estimate_path)
                row['estimate_relpath'] = relative_path(estimate_path, self.results_root)
            self.rows_by_dataset[dataset_name] = row
            self._show_row(row)
            self._update_table()
            self._update_bar_charts()
            self._set_eval_status('Failed', COLORS['red'])
            self.append_log(f'Evaluation failed for {dataset_name}: {message}')
            if self.continue_queue_after_eval:
                self.continue_queue_after_eval = False
                self._run_next_in_queue()

        def refresh_results(self):
            self.results_root = Path(self.results_root_edit.text())
            self.metrics_path = self.results_root / METRICS_FILENAME
            self._set_eval_status('Refreshing', COLORS['cyan'])
            include_rejected = self.show_rejected_checkbox.isChecked()
            rows = []
            QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
            try:
                for dataset_name in DATASETS:
                    QtWidgets.QApplication.processEvents()
                    estimates = find_estimates(
                        self.results_root, dataset_name,
                        include_rejected=include_rejected)
                    if not estimates:
                        rows.append(empty_row(dataset_name, 'missing', COLORS['orange']))
                        continue

                    accepted_row = None
                    fallback_row = None
                    for estimate_path in estimates:
                        self.append_log(
                            f'Evaluating latest result for {dataset_name}: {estimate_path.name}')
                        try:
                            row, _ = evaluate_estimate(
                                dataset_name, estimate_path, self.datasets_root, self.results_root)
                        except Exception as exc:
                            fallback_row = empty_row(dataset_name, 'failed', COLORS['red'])
                            fallback_row['estimate'] = str(estimate_path)
                            fallback_row['estimate_relpath'] = relative_path(estimate_path, self.results_root)
                            self.append_log(f'Skipped {dataset_name}: {exc}')
                            continue

                        if include_rejected or row.get('accepted'):
                            accepted_row = row
                            break
                        if fallback_row is None:
                            fallback_row = row
                        self.append_log(
                            f'Skipped non-accepted result for {dataset_name}: '
                            f'{estimate_path.name} ({row["status"]}: {row["acceptance_reason"]})')

                    if accepted_row is not None:
                        rows.append(accepted_row)
                    elif fallback_row is not None:
                        rows.append(fallback_row)
                    else:
                        rows.append(empty_row(dataset_name, 'missing', COLORS['orange']))
            finally:
                QtWidgets.QApplication.restoreOverrideCursor()
            self._refresh_finished(rows)

        def _clear_refresh_thread(self):
            self.refresh_thread = None
            self.refresh_worker = None

        def _refresh_finished(self, rows):
            for row in rows:
                self.rows_by_dataset[row['dataset']] = row
            self._save_metrics()
            self._populate_estimates(self.current_dataset)
            self._show_row(self.rows_by_dataset.get(self.current_dataset, empty_row(self.current_dataset)))
            self._update_table()
            self._update_bar_charts()
            if self.process is None:
                self._set_run_status('Idle', COLORS['muted'])
                self._set_eval_status('Preview Loaded', COLORS['cyan'])

            current_row = self.rows_by_dataset.get(
                self.current_dataset,
                empty_row(self.current_dataset),
            )
            current_estimate_text = current_row.get('estimate')
            if current_estimate_text:
                current_estimate = Path(current_estimate_text)
                if current_estimate.exists():
                    self._add_estimate_option(current_estimate, select=True)
                    cached = self.result_cache.get(str(current_estimate))
                    if cached is not None:
                        self._update_current_plots(cached)
                    else:
                        QtCore.QTimer.singleShot(
                            0,
                            lambda dataset=self.current_dataset, estimate=current_estimate:
                                self.start_evaluation(dataset, estimate, automatic=True)
                        )

        def _show_row(self, row):
            color = row.get('color', COLORS['text'])
            self._set_status_values(
                dataset=row['dataset'],
                output=row.get('estimate_relpath') or '-')
            if hasattr(self, 'provenance_block'):
                self.provenance_block.show_row(row, self.results_root)
            values = {
                'ate_rmse_m': metric_text(row.get('ate_rmse_m')),
                'ate_mean_m': metric_text(row.get('ate_mean_m')),
                'ate_median_m': metric_text(row.get('ate_median_m')),
                'ate_max_m': metric_text(row.get('ate_max_m')),
                'rpe_1s_rmse_m': metric_text(row.get('rpe_1s_rmse_m')),
                'aligned_samples': metric_text(row.get('aligned_samples'), 'count'),
                'overlap_duration_s': metric_text(row.get('overlap_duration_s'), 's'),
            }
            for key, value in values.items():
                self.metric_cards[key].set_value(value, color)

        def _update_current_plots(self, result):
            times = result.timestamps - result.timestamps[0]
            estimate = result.estimate_aligned
            groundtruth = result.groundtruth_interpolated
            ate_errors = result.error_norms
            if hasattr(self.live_view, 'set_final_trajectory'):
                self.live_view.set_final_trajectory(estimate, groundtruth)
            else:
                self.live_view.set_groundtruth_preview(groundtruth)
            self.live_view.set_evaluation_curves(
                times,
                ate_errors,
                result.rpe_errors if result.rpe_errors is not None else None)
            self.xy_plot.draw_trajectory(estimate, groundtruth, (0, 1), 'Trajectory (XY)', COLORS['cyan'])
            self.xz_plot.draw_trajectory(estimate, groundtruth, (0, 2), 'Trajectory (XZ)', COLORS['cyan'])
            self.yz_plot.draw_trajectory(estimate, groundtruth, (1, 2), 'Trajectory (YZ)', COLORS['cyan'])
            self.ate_plot.draw_error(times, ate_errors, 'ATE Plot', COLORS['red'])
            if result.rpe_errors is not None and len(result.rpe_errors):
                x = np.arange(len(result.rpe_errors))
                self.rpe_plot.draw_error(x, result.rpe_errors, 'RPE/RTE Plot', COLORS['purple'])
            else:
                self.rpe_plot.draw_empty('No RPE pairs available')

        def _update_table(self):
            self.updating_table = True
            try:
                rows = [self.rows_by_dataset.get(name, empty_row(name)) for name in DATASETS]
                self.summary_model.set_rows(rows, self.results_root, display_path, metric_text, COLORS)
                self.summary_table.resizeRowsToContents()
            finally:
                self.updating_table = False

        def _update_bar_charts(self):
            rows = []
            for name in DATASETS:
                row = self.rows_by_dataset[name]
                if not self.show_rejected_checkbox.isChecked() and not row.get('accepted'):
                    continue
                rows.append(row)
            self.ate_bar_plot.draw_bar(rows, 'ate_rmse_m', 'ATE RMSE (m) - All Datasets')
            self.rpe_bar_plot.draw_bar(rows, 'rpe_1s_rmse_m', 'RPE 1s RMSE (m) - All Datasets')

        def _table_selection_changed(self, *args):
            if self.updating_table:
                return
            selected = self.summary_table.selectionModel().selectedRows()
            if not selected:
                return
            model_index = selected[0]
            item = self.summary_model.item(model_index.row(), 0)
            row = item.data(QtCore.Qt.UserRole) if item is not None else None
            if not row:
                return
            self.dataset_combo.blockSignals(True)
            self.dataset_combo.setCurrentText(row['dataset'])
            self.dataset_combo.blockSignals(False)
            self.current_dataset = row['dataset']
            self._load_camera_preview(row['dataset'])
            self._populate_estimates(row['dataset'])
            self._show_row(row)
            estimate_path = Path(row['estimate']) if row.get('estimate') else None
            if estimate_path and estimate_path.exists():
                self._add_estimate_option(estimate_path, select=True)
                cached = self.result_cache.get(str(estimate_path))
                if cached is not None:
                    self._update_current_plots(cached)
                else:
                    self.start_evaluation(row['dataset'], estimate_path, automatic=True)

        def current_plot_panel(self):
            widget = self.tabs.currentWidget()
            if isinstance(widget, PlotPanel):
                return widget
            return None

        def export_current_plot(self):
            panel = self.current_plot_panel()
            if panel is None:
                self.append_log('Select a static evaluation plot tab before exporting a plot.')
                return
            path, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, 'Export current plot',
                str(self.results_root / 'dashboard_plot.png'),
                'PNG files (*.png)')
            if not path:
                return
            panel.figure.savefig(path, dpi=150)
            self.append_log(f'Exported plot: {path}')

        def export_summary(self):
            path, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, 'Export dashboard summary',
                str(self.results_root / 'dashboard_summary.csv'),
                'CSV files (*.csv)')
            if not path:
                return
            with open(path, 'w', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow([
                    'dataset', 'status', 'output_file', 'ate_rmse_m',
                    'rpe_1s_rmse_m', 'ate_mean_m', 'aligned_samples',
                    'overlap_duration_s',
                ])
                for dataset_name in DATASETS:
                    row = self.rows_by_dataset[dataset_name]
                    writer.writerow([
                        row['dataset'], row['status'], row.get('estimate_relpath') or '-',
                        row.get('ate_rmse_m'), row.get('rpe_1s_rmse_m'),
                        row.get('ate_mean_m'), row.get('aligned_samples'),
                        row.get('overlap_duration_s'),
                    ])
            self.append_log(f'Exported summary: {path}')

        def open_results_folder(self):
            self.results_root.mkdir(parents=True, exist_ok=True)
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.results_root.resolve())))

        def closeEvent(self, event):
            if self.process is not None:
                self.process.terminate()
            self.preview_timer.stop()
            event.accept()

        def ui_self_check(self):
            screen = QtWidgets.QApplication.screenAt(self.frameGeometry().center())
            if screen is None:
                screen = QtWidgets.QApplication.primaryScreen()
            available = screen.availableGeometry()
            frame = self.frameGeometry()
            table_checks = table_self_check(self.summary_table)
            tab_bar = self.tabs.tabBar()
            tabs_not_clipped = True
            for index in range(tab_bar.count()):
                text = tab_bar.tabText(index)
                required = tab_bar.fontMetrics().horizontalAdvance(text) + 18
                if tab_bar.tabRect(index).width() < required:
                    tabs_not_clipped = False
                    break
            return {
                'frame_inside_available': (
                    frame.left() >= available.left()
                    and frame.top() >= available.top()
                    and frame.right() <= available.right()
                    and frame.bottom() <= available.bottom()
                ),
                'locked_size': self.minimumSize() == self.maximumSize(),
                **table_checks,
                'title_correct': self.windowTitle() == 'UAV-Airvision Evaluation Dashboard',
                'tabs_not_clipped': tabs_not_clipped,
                'sidebars_compact': (
                    self.left_sidebar.width() <= 285
                    and self.right_sidebar.width() <= 245
                ),
                'global_charts_tabbed': hasattr(self, 'global_chart_tabs'),
                'rejected_hidden_default': not self.show_rejected_checkbox.isChecked(),
                'provenance_visible': self.provenance_block.isVisible(),
                'bottom_not_behind_taskbar': frame.bottom() <= available.bottom(),
            }

    return DashboardWindow


def run_ui_self_check(args):
    try:
        load_gui_dependencies()
    except RuntimeError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    app.setApplicationName('UAV-Airvision Evaluation Dashboard')
    DashboardWindow = create_dashboard_classes()
    window = DashboardWindow(
        args.datasets_root,
        args.results_root,
        args.default_offset,
        auto_refresh=False,
        auto_evaluate=False,
    )
    results = {}

    def finish():
        window._apply_taskbar_safe_lock()
        app.processEvents()
        results.update(window.ui_self_check())
        for key, value in results.items():
            print(f'{key}={value}')
        window.close()
        app.quit()

    window.show()
    QtCore.QTimer.singleShot(700, finish)
    app.exec_()
    return 0 if all(bool(value) for value in results.values()) else 1


def main(argv=None):
    args = parse_args(argv)
    if args.validate_data_flow:
        return validate_data_flow(args)
    if args.ui_self_check:
        return run_ui_self_check(args)

    try:
        load_gui_dependencies()
    except RuntimeError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    app.setApplicationName('UAV-Airvision Evaluation Dashboard')
    DashboardWindow = create_dashboard_classes()
    window = DashboardWindow(args.datasets_root, args.results_root, args.default_offset)
    window.show()
    return app.exec_()


if __name__ == '__main__':
    raise SystemExit(main())
