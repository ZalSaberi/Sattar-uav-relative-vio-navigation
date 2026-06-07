from collections import deque
from concurrent.futures import ThreadPoolExecutor
import time

import cv2
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

pg.setConfigOptions(antialias=False)

try:
    import pyqtgraph.opengl as gl
    HAS_GL = True
except Exception:
    gl = None
    HAS_GL = False


class _ImageLoadNotifier(QtCore.QObject):
    loaded = QtCore.pyqtSignal(str, object)


class LiveRunView(QtWidgets.QWidget):
    """
    Viewer-style live dashboard panel.

    - Camera frames are loaded outside the GUI thread.
    - Old frames are dropped; only the newest frame is displayed.
    - Trajectory is rendered relative to the first pose, so it starts at origin.
    - 3D origin axes are drawn at (0, 0, 0).
    """

    def __init__(self, colors, history=6000, fps=24, parent=None):
        super().__init__(parent)
        self.colors = colors
        self.history = int(history)
        self._target_fps = max(1, int(fps))

        self._image_queue = deque(maxlen=1)
        self._pose_queue = deque(maxlen=4096)
        self._positions = deque(maxlen=self.history)
        self._timestamps = deque(maxlen=self.history)

        self._origin_position = None

        self._last_image_path = None
        self._last_drawn_image_path = None
        self._last_image_draw_at = 0.0
        self._last_pose_draw_at = 0.0
        self._last_title_update_at = 0.0
        self._last_camera_autorange = False

        self._frame_count = 0
        self._pose_count = 0
        self._draw_count = 0
        self._t0 = time.time()
        self._running = False
        self._final_trajectory_mode = False

        self._image_executor = ThreadPoolExecutor(max_workers=1)
        self._image_notifier = _ImageLoadNotifier()
        self._image_notifier.loaded.connect(self._on_image_loaded)
        self._image_load_busy = False
        self._pending_image_path = None
        self._image_draw_times = deque(maxlen=40)

        self._axis_scale = 1.0
        self._grid_step = 1.0
        self._final_trajectory_mode = False
        self._trajectory_points = np.zeros((0, 3), dtype=np.float32)
        self._groundtruth_points = np.zeros((0, 3), dtype=np.float32)
        self._trajectory_times = np.zeros((0,), dtype=np.float32)
        self._cursor_index = None
        self._inspector_follow_latest = True
        self._grid_step = 1.0
        self._trajectory_points = np.zeros((0, 3), dtype=np.float32)
        self._trajectory_times = np.zeros((0,), dtype=np.float32)
        self._cursor_index = None
        self._last_cursor_update_at = 0.0

        self._build_ui()
        self._apply_plot_style()

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(max(16, int(1000 / self._target_fps)))
        self.timer.timeout.connect(self._update_gui)
        self.timer.start()

    @property
    def frame_count(self):
        return self._frame_count

    @property
    def pose_count(self):
        return self._pose_count

    def _build_ui(self):
        layout = QtWidgets.QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        self.camera_title = QtWidgets.QLabel('Camera: cam0 (live)')
        self.camera_title.setObjectName('PanelTitle')

        self.image_view = pg.GraphicsLayoutWidget()
        self.image_item = pg.ImageItem()
        self.image_box = self.image_view.addViewBox(lockAspect=True)
        self.image_box.addItem(self.image_item)
        self.image_box.invertY(False)
        self.image_box.setMouseEnabled(x=False, y=False)

        self.traj_title = QtWidgets.QLabel('Trajectory 3D - Live')
        self.traj_title.setObjectName('PanelTitle')
        self.traj_title.setWordWrap(False)
        self.traj_title.setMaximumHeight(22)
        self.traj_title.setToolTip(
            'Red = estimate / VIO trajectory\n'
            'Green = ground truth / reference\n'
            'Axes: X red, Y green, Z blue\n'
            'Hover over the trajectory to inspect x, y, z, and time.'
        )

        if HAS_GL:
            self.traj_widget = gl.GLViewWidget()
            self.traj_widget.setBackgroundColor('k')
            self.traj_widget.opts['distance'] = 8
            self.traj_widget.opts['elevation'] = 22
            self.traj_widget.opts['azimuth'] = -55

            self.grid_xy = gl.GLGridItem()
            self.grid_xy.setSize(10, 10, 1)
            self.grid_xy.setSpacing(1, 1, 1)
            self.traj_widget.addItem(self.grid_xy)

            self.axis_x = gl.GLLinePlotItem(
                pos=np.array([[0, 0, 0], [1.0, 0, 0]], dtype=np.float32),
                color=(1.0, 0.15, 0.12, 1.0),
                width=2.5,
                antialias=False,
            )
            self.axis_y = gl.GLLinePlotItem(
                pos=np.array([[0, 0, 0], [0, 1.0, 0]], dtype=np.float32),
                color=(0.1, 0.9, 0.25, 1.0),
                width=2.5,
                antialias=False,
            )
            self.axis_z = gl.GLLinePlotItem(
                pos=np.array([[0, 0, 0], [0, 0, 1.0]], dtype=np.float32),
                color=(0.2, 0.45, 1.0, 1.0),
                width=2.5,
                antialias=False,
            )
            self.origin_point = gl.GLScatterPlotItem(
                pos=np.array([[0, 0, 0]], dtype=np.float32),
                color=(1.0, 1.0, 1.0, 1.0),
                size=7,
                pxMode=True,
            )

            self.traj_widget.addItem(self.axis_x)
            self.traj_widget.addItem(self.axis_y)
            self.traj_widget.addItem(self.axis_z)
            self.traj_widget.addItem(self.origin_point)

            self.live_line = gl.GLLinePlotItem(
                pos=np.zeros((0, 3), dtype=np.float32),
                color=(1.0, 0.08, 0.06, 1.0),
                width=2.2,
                antialias=False,
            )
            self.gt_line = gl.GLLinePlotItem(
                pos=np.zeros((0, 3), dtype=np.float32),
                color=(0.1, 0.9, 0.25, 0.75),
                width=1.4,
                antialias=False,
            )
            self.head_point = gl.GLScatterPlotItem(
                pos=np.zeros((0, 3), dtype=np.float32),
                color=(0.0, 0.9, 1.0, 1.0),
                size=7,
                pxMode=True,
            )
            self.cursor_point = gl.GLScatterPlotItem(
                pos=np.zeros((0, 3), dtype=np.float32),
                color=(1.0, 0.9, 0.05, 1.0),
                size=9,
                pxMode=True,
            )

            self.traj_widget.addItem(self.gt_line)
            self.traj_widget.addItem(self.live_line)
            self.traj_widget.addItem(self.head_point)
            self.traj_widget.addItem(self.cursor_point)
        else:
            self.traj_widget = pg.PlotWidget()
            self.traj_widget.setBackground(self.colors['panel_deep'])
            self.traj_widget.showGrid(x=True, y=True, alpha=0.18)
            self.live_curve_2d = self.traj_widget.plot(
                pen=pg.mkPen(self.colors['red'], width=2)
            )
            self.head_curve_2d = self.traj_widget.plot(
                [], [], pen=None, symbol='o', symbolSize=7,
                symbolBrush=pg.mkBrush(self.colors['cyan'])
            )
            self.gt_curve_2d = self.traj_widget.plot(
                pen=pg.mkPen(self.colors['green'], width=1, style=QtCore.Qt.DashLine)
            )
            self.cursor_curve_2d = self.traj_widget.plot(
                [], [], pen=None, symbol='o', symbolSize=9,
                symbolBrush=pg.mkBrush('#FDE047')
            )

        self.ate_plot = pg.PlotWidget()
        self.ate_curve = self.ate_plot.plot(
            pen=pg.mkPen(self.colors['red'], width=1.4)
        )
        self.ate_mean = self.ate_plot.plot(
            pen=pg.mkPen(self.colors['cyan'], width=1, style=QtCore.Qt.DashLine)
        )

        self.rpe_plot = pg.PlotWidget()
        self.rpe_curve = self.rpe_plot.plot(
            pen=pg.mkPen(self.colors['purple'], width=1.4)
        )
        self.rpe_mean = self.rpe_plot.plot(
            pen=pg.mkPen(self.colors['cyan'], width=1, style=QtCore.Qt.DashLine)
        )

        if hasattr(self.traj_widget, 'setMouseTracking'):
            self.traj_widget.setMouseTracking(True)
            self.traj_widget.installEventFilter(self)

        layout.addWidget(self._card_widget(self.camera_title, self.image_view), 0, 0)
        layout.addWidget(self._trajectory_card(), 0, 1)
        layout.addWidget(self._card('ATE (Absolute Trajectory Error)', self.ate_plot), 1, 0)
        layout.addWidget(self._card('RTE (Relative Translation Error) - w=1s', self.rpe_plot), 1, 1)

        layout.setRowStretch(0, 60)
        layout.setRowStretch(1, 40)
        layout.setColumnStretch(0, 50)
        layout.setColumnStretch(1, 50)

        self.set_message('No frame preview available')
        self.draw_evaluation_placeholders()

    def _trajectory_card(self):
        frame = QtWidgets.QFrame()
        frame.setObjectName('ChartCard')

        layout = QtWidgets.QVBoxLayout(frame)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(6)

        self.traj_title.setWordWrap(False)
        self.traj_title.setMaximumHeight(22)
        self.traj_title.setToolTip(
            'Red = estimate / VIO trajectory\n'
            'Green = ground truth / reference\n'
            'Axes: X red, Y green, Z blue\n'
            'Use the inspector slider to select an exact sample.'
        )

        layout.addWidget(self.traj_title)
        layout.addWidget(self.traj_widget, 1)

        inspector = QtWidgets.QFrame()
        inspector.setObjectName('TrajectoryInspector')

        inspector_layout = QtWidgets.QVBoxLayout(inspector)
        inspector_layout.setContentsMargins(0, 0, 0, 0)
        inspector_layout.setSpacing(3)

        controls = QtWidgets.QHBoxLayout()
        controls.setContentsMargins(0, 0, 0, 0)
        controls.setSpacing(6)

        self.inspect_prev_button = QtWidgets.QPushButton('Prev')
        self.inspect_next_button = QtWidgets.QPushButton('Next')
        self.inspect_latest_button = QtWidgets.QPushButton('Latest')

        self.inspect_prev_button.setToolTip('Previous sample')
        self.inspect_next_button.setToolTip('Next sample')
        self.inspect_latest_button.setToolTip('Latest sample')

        for button in (
            self.inspect_prev_button,
            self.inspect_next_button,
            self.inspect_latest_button,
        ):
            button.setFixedHeight(24)
            button.setMinimumWidth(58)
            button.setStyleSheet(
                'QPushButton {'
                'color: #EAF2FF;'
                'background-color: #0F1C2E;'
                'border: 1px solid #294061;'
                'border-radius: 7px;'
                'font-size: 10px;'
                'font-weight: 700;'
                'padding: 2px 8px;'
                '}'
                'QPushButton:hover {'
                'background-color: #16304F;'
                'border: 1px solid #3D6DFF;'
                '}'
                'QPushButton:pressed {'
                'background-color: #1B3C66;'
                '}'
                'QPushButton:disabled {'
                'color: #64748B;'
                'background-color: #0B1322;'
                'border: 1px solid #1E293B;'
                '}'
            )

        self.inspect_prev_button.setToolTip('Previous sample')
        self.inspect_next_button.setToolTip('Next sample')
        self.inspect_latest_button.setToolTip('Latest sample')

        self.inspect_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.inspect_slider.setRange(0, 0)
        self.inspect_slider.setValue(0)
        self.inspect_slider.setTracking(True)
        self.inspect_slider.setStyleSheet(
            'QSlider::groove:horizontal {'
            'height: 4px;'
            'background: #CBD5E1;'
            'border-radius: 2px;'
            '}'
            'QSlider::handle:horizontal {'
            'background: #3D6DFF;'
            'width: 12px;'
            'height: 16px;'
            'margin: -6px 0;'
            'border-radius: 3px;'
            '}'
            'QSlider::sub-page:horizontal {'
            'background: #3D6DFF;'
            'border-radius: 2px;'
            '}'
        )

        self.inspect_prev_button.clicked.connect(self._inspector_previous)
        self.inspect_next_button.clicked.connect(self._inspector_next)
        self.inspect_latest_button.clicked.connect(self._inspector_latest)
        self.inspect_slider.valueChanged.connect(self._inspector_slider_changed)

        controls.addWidget(self.inspect_prev_button)
        controls.addWidget(self.inspect_slider, 1)
        controls.addWidget(self.inspect_next_button)
        controls.addWidget(self.inspect_latest_button)

        self.inspector_label = QtWidgets.QLabel(
            'No trajectory samples yet'
        )
        self.inspector_label.setObjectName('InspectorLabel')
        self.inspector_label.setWordWrap(False)
        self.inspector_label.setMinimumWidth(0)
        self.inspector_label.setMaximumHeight(20)
        self.inspector_label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        self.inspector_label.setStyleSheet('color: #EAF2FF; background: transparent; font-size: 10px; font-weight: 600; padding-top: 2px;')
        self.inspector_label.setStyleSheet(
            'color: #EAF2FF;'
            'background: transparent;'
            'font-size: 10px;'
            'font-weight: 600;'
            'padding-top: 2px;'
        )
        self.inspector_label.setToolTip(
            'x,y,z are local metric coordinates in meters. '
            't is time from the start of the displayed trajectory.'
        )

        inspector_layout.addLayout(controls)
        inspector_layout.addWidget(self.inspector_label)

        layout.addWidget(inspector, 0)

        self._set_inspector_enabled(False)
        return frame

    def _card(self, title, widget):
        label = QtWidgets.QLabel(title)
        label.setObjectName('PanelTitle')
        return self._card_widget(label, widget)

    def _card_widget(self, title_widget, widget):
        frame = QtWidgets.QFrame()
        frame.setObjectName('ChartCard')
        layout = QtWidgets.QVBoxLayout(frame)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(6)
        layout.addWidget(title_widget)
        layout.addWidget(widget, 1)
        return frame

    def _set_inspector_enabled(self, enabled):
        for name in (
            'inspect_slider',
            'inspect_prev_button',
            'inspect_next_button',
            'inspect_latest_button',
        ):
            widget = getattr(self, name, None)
            if widget is not None:
                widget.setEnabled(bool(enabled))

    def _update_inspector_range(self, count, select_index=None, follow_latest=None):
        if not hasattr(self, 'inspect_slider'):
            return

        count = int(count)

        if count <= 0:
            self.inspect_slider.blockSignals(True)
            self.inspect_slider.setRange(0, 0)
            self.inspect_slider.setValue(0)
            self.inspect_slider.blockSignals(False)
            self._set_inspector_enabled(False)
            if hasattr(self, 'inspector_label'):
                self.inspector_label.setText('No trajectory samples yet')
            return

        self._set_inspector_enabled(True)

        if follow_latest is not None:
            self._inspector_follow_latest = bool(follow_latest)

        if select_index is None:
            if self._inspector_follow_latest or self._cursor_index is None:
                select_index = count - 1
            else:
                select_index = min(self._cursor_index, count - 1)

        select_index = int(max(0, min(count - 1, select_index)))

        self.inspect_slider.blockSignals(True)
        self.inspect_slider.setRange(0, count - 1)
        self.inspect_slider.setValue(select_index)
        self.inspect_slider.blockSignals(False)

        self._select_sample_by_index(select_index, update_slider=False)

    def _inspector_slider_changed(self, value):
        self._inspector_follow_latest = False
        self._select_sample_by_index(int(value), update_slider=False)

    def _inspector_previous(self):
        if not hasattr(self, 'inspect_slider'):
            return
        self._inspector_follow_latest = False
        value = max(self.inspect_slider.minimum(), self.inspect_slider.value() - 1)
        self._select_sample_by_index(value, update_slider=True)

    def _inspector_next(self):
        if not hasattr(self, 'inspect_slider'):
            return
        self._inspector_follow_latest = False
        value = min(self.inspect_slider.maximum(), self.inspect_slider.value() + 1)
        self._select_sample_by_index(value, update_slider=True)

    def _inspector_latest(self):
        count = len(getattr(self, '_trajectory_points', []))
        if count < 1:
            return
        self._inspector_follow_latest = True
        self._select_sample_by_index(count - 1, update_slider=True)

    def _select_sample_by_index(self, index, update_slider=True):
        points = getattr(self, '_trajectory_points', None)

        if points is None or len(points) < 1:
            return

        index = int(max(0, min(len(points) - 1, index)))
        self._cursor_index = index

        if update_slider and hasattr(self, 'inspect_slider'):
            self.inspect_slider.blockSignals(True)
            self.inspect_slider.setValue(index)
            self.inspect_slider.blockSignals(False)

        self._set_cursor_marker_at_index(index)

        full_text = self._format_inspector_text(index)
        visible_text = self._format_inspector_visible_text(index)

        if hasattr(self, 'inspector_label'):
            self.inspector_label.setText(visible_text)
            self.inspector_label.setToolTip(full_text)

        if hasattr(self, 'traj_widget'):
            self.traj_widget.setToolTip(full_text)
        if hasattr(self, 'traj_title'):
            self.traj_title.setToolTip(full_text)

        self._set_trajectory_title()


    def _format_inspector_text(self, index):
        points = getattr(self, '_trajectory_points', np.zeros((0, 3), dtype=np.float32))
        if len(points) < 1:
            return 'No trajectory samples yet'

        p = points[index]
        times = getattr(self, '_trajectory_times', np.zeros((0,), dtype=np.float32))
        t = float(times[index]) if index < len(times) else 0.0

        lines = [
            f'sample {index + 1}/{len(points)}',
            f't = {t:.3f} s',
            f'x = {p[0]:.3f} m',
            f'y = {p[1]:.3f} m',
            f'z = {p[2]:.3f} m',
        ]

        gt = getattr(self, '_groundtruth_points', None)
        if gt is not None and len(gt) == len(points) and index < len(gt):
            diff = p - gt[index]
            error = float(np.linalg.norm(diff))
            lines.extend([
                f'error = {error:.3f} m',
                f'dx = {diff[0]:+.3f} m',
                f'dy = {diff[1]:+.3f} m',
                f'dz = {diff[2]:+.3f} m',
            ])

        return '\n'.join(lines)

    def _format_inspector_visible_text(self, index):
        points = getattr(self, '_trajectory_points', np.zeros((0, 3), dtype=np.float32))
        if len(points) < 1:
            return 'No trajectory samples yet'

        p = points[index]
        times = getattr(self, '_trajectory_times', np.zeros((0,), dtype=np.float32))
        t = float(times[index]) if index < len(times) else 0.0

        text = (
            f'sample {index + 1}/{len(points)}   '
            f't={t:.2f}s   '
            f'x={p[0]:.2f}m   '
            f'y={p[1]:.2f}m   '
            f'z={p[2]:.2f}m'
        )

        gt = getattr(self, '_groundtruth_points', None)
        if gt is not None and len(gt) == len(points) and index < len(gt):
            error = float(np.linalg.norm(p - gt[index]))
            text += f'   err={error:.2f}m'

        return text


    def _set_cursor_marker_at_index(self, index, global_pos=None):
        points = getattr(self, '_trajectory_points', None)

        if points is None or len(points) < 1:
            return

        index = int(max(0, min(len(points) - 1, index)))
        point = points[index]

        if HAS_GL and hasattr(self, 'cursor_point'):
            self.cursor_point.setData(pos=point.reshape(1, 3).astype(np.float32))
        elif hasattr(self, 'cursor_curve_2d'):
            self.cursor_curve_2d.setData([point[0]], [point[1]])

    def _clear_data_cursor(self):
        self._cursor_index = None

        if HAS_GL and hasattr(self, 'cursor_point'):
            self.cursor_point.setData(pos=np.zeros((0, 3), dtype=np.float32))
        elif hasattr(self, 'cursor_curve_2d'):
            self.cursor_curve_2d.setData([], [])

        if hasattr(self, 'inspector_label'):
            self.inspector_label.setText('No sample selected')

        self._set_trajectory_title()

    def _apply_plot_style(self):
        for plot in (self.ate_plot, self.rpe_plot):
            plot.setBackground(self.colors['panel_deep'])
            plot.showGrid(x=True, y=True, alpha=0.18)
            plot.getAxis('bottom').setPen(self.colors['border'])
            plot.getAxis('left').setPen(self.colors['border'])
            plot.getAxis('bottom').setTextPen(self.colors['muted'])
            plot.getAxis('left').setTextPen(self.colors['muted'])
            plot.getViewBox().setMouseEnabled(x=True, y=True)

        self.ate_plot.setLabel('bottom', 'time [s]', color=self.colors['muted'])
        self.ate_plot.setLabel('left', 'error [m]', color=self.colors['muted'])
        self.rpe_plot.setLabel('bottom', 'sample', color=self.colors['muted'])
        self.rpe_plot.setLabel('left', 'error [m]', color=self.colors['muted'])

        if not HAS_GL:
            self.traj_widget.setLabel('bottom', 'x [m]', color=self.colors['muted'])
            self.traj_widget.setLabel('left', 'y [m]', color=self.colors['muted'])

    def reset_run(self):
        self._image_queue.clear()
        self._pose_queue.clear()
        self._positions.clear()
        self._timestamps.clear()

        self._origin_position = None

        self._last_image_path = None
        self._last_drawn_image_path = None
        self._last_image_draw_at = 0.0
        self._last_pose_draw_at = 0.0
        self._last_title_update_at = 0.0
        self._last_camera_autorange = False

        self._frame_count = 0
        self._pose_count = 0
        self._draw_count = 0
        self._image_draw_times.clear()
        self._t0 = time.time()
        self._running = True
        self._final_trajectory_mode = False

        self._clear_trajectory()
        self.draw_evaluation_placeholders()
        self.camera_title.setText('Camera: cam0 (live)')
        self.traj_title.setText('Trajectory 3D - Live' if HAS_GL else 'Trajectory (XY) - Live')
        self._update_origin_axes(1.0)

    def freeze_final_state(self):
        self._running = False

    def closeEvent(self, event):
        try:
            self._image_executor.shutdown(wait=False, cancel_futures=True)
        except TypeError:
            self._image_executor.shutdown(wait=False)
        super().closeEvent(event)

    def set_message(self, text):
        self.image_item.clear()
        self.camera_title.setText(f'Camera: cam0 (live) - {text}')

    def set_status(self, text):
        self.camera_title.setText(f'Camera: cam0 (live) - {text}')

    def update_image_from_path(self, path):
        path = str(path)
        if path == self._last_image_path:
            return

        self._last_image_path = path
        self._pending_image_path = path
        self._start_image_load_if_idle()

    def update_image_array(self, image):
        self._image_queue.append(('array', np.asarray(image).copy()))

    def append_pose(self, timestamp, position):
        position = np.asarray(position, dtype=float)
        if position.shape[0] < 3 or not np.all(np.isfinite(position[:3])):
            return

        raw = position[:3].copy()
        if self._origin_position is None:
            self._origin_position = raw.copy()

        relative = raw - self._origin_position
        self._pose_queue.append((float(timestamp), relative))

    def _to_xyz_array(self, values):
        values = np.asarray(values, dtype=float)

        if values.ndim != 2 or len(values) < 1:
            return np.zeros((0, 3), dtype=np.float32)

        if values.shape[1] == 2:
            z = np.zeros((len(values), 1), dtype=float)
            values = np.hstack([values[:, :2], z])
        elif values.shape[1] >= 3:
            values = values[:, :3]
        else:
            return np.zeros((0, 3), dtype=np.float32)

        finite = np.all(np.isfinite(values), axis=1)
        values = values[finite]
        return values.astype(np.float32)

    def set_final_trajectory(self, estimate, groundtruth, times=None):
        estimate = self._to_xyz_array(estimate)
        groundtruth = self._to_xyz_array(groundtruth)

        if len(estimate) < 2 or len(groundtruth) < 2:
            self._clear_trajectory()
            return

        count = min(len(estimate), len(groundtruth))
        estimate = estimate[:count]
        groundtruth = groundtruth[:count]

        if times is None:
            times = np.arange(count, dtype=float)
        else:
            times = np.asarray(times, dtype=float)[:count]
            if len(times) < count:
                times = np.arange(count, dtype=float)

        times = times - times[0]

        origin = groundtruth[0].copy()
        estimate_rel = estimate - origin
        groundtruth_rel = groundtruth - origin

        estimate_rel, groundtruth_rel, times = self._sample_aligned_trajectory(
            estimate_rel,
            groundtruth_rel,
            times,
            max_points=2600,
        )

        self._positions.clear()
        self._timestamps.clear()
        self._pose_queue.clear()

        self._trajectory_points = estimate_rel.astype(np.float32)
        self._groundtruth_points = groundtruth_rel.astype(np.float32)
        self._trajectory_times = times.astype(np.float32)

        self._pose_count = len(self._trajectory_points)
        self._final_trajectory_mode = True
        self._inspector_follow_latest = True

        if HAS_GL:
            self.live_line.setData(pos=self._trajectory_points)
            self.gt_line.setData(pos=self._groundtruth_points)
            self.head_point.setData(pos=self._trajectory_points[-1:].astype(np.float32))

            combined = np.vstack([
                self._trajectory_points,
                self._groundtruth_points,
                np.zeros((1, 3), dtype=np.float32),
            ])
            self._update_3d_camera_and_origin(combined)
        else:
            self.live_curve_2d.setData(self._trajectory_points[:, 0], self._trajectory_points[:, 1])
            self.head_curve_2d.setData([self._trajectory_points[-1, 0]], [self._trajectory_points[-1, 1]])
            self.gt_curve_2d.setData(self._groundtruth_points[:, 0], self._groundtruth_points[:, 1])

        self._update_inspector_range(
            len(self._trajectory_points),
            select_index=len(self._trajectory_points) - 1,
            follow_latest=True,
        )
        self._set_trajectory_title()


    def set_groundtruth_preview(self, positions):
        positions = self._to_xyz_array(positions)
        if len(positions) < 2:
            self._set_gt_line(np.zeros((0, 3), dtype=np.float32))
            return

        positions = positions - positions[0]
        positions = self._sample_positions(positions, max_points=1800).astype(np.float32)
        self._set_gt_line(positions)

    def draw_evaluation_placeholders(self):
        self.ate_curve.setData([], [])
        self.ate_mean.setData([], [])
        self.rpe_curve.setData([], [])
        self.rpe_mean.setData([], [])
        self.ate_plot.setTitle('Awaiting final evaluation', color=self.colors['muted'])
        self.rpe_plot.setTitle('Awaiting final evaluation', color=self.colors['muted'])

    def set_evaluation_curves(self, times, ate_errors, rpe_errors=None):
        times = np.asarray(times, dtype=float)
        ate_errors = np.asarray(ate_errors, dtype=float)

        if len(times) and len(ate_errors):
            times, ate_errors = self._sample_xy(times, ate_errors, max_points=1600)
            self.ate_curve.setData(times, ate_errors)
            mean = float(np.mean(ate_errors))
            self.ate_mean.setData([times[0], times[-1]], [mean, mean])
            self.ate_plot.setTitle(
                f'ATE (Absolute Trajectory Error) - mean {mean:.3f} m',
                color=self.colors['text']
            )
        else:
            self.ate_plot.setTitle('No ATE samples available', color=self.colors['muted'])

        if rpe_errors is not None and len(rpe_errors):
            rpe_errors = np.asarray(rpe_errors, dtype=float)
            x = np.arange(len(rpe_errors), dtype=float)
            x, rpe_errors = self._sample_xy(x, rpe_errors, max_points=1600)
            self.rpe_curve.setData(x, rpe_errors)
            mean = float(np.mean(rpe_errors))
            self.rpe_mean.setData([x[0], x[-1]], [mean, mean])
            self.rpe_plot.setTitle(
                f'RTE (Relative Translation Error) - mean {mean:.3f} m',
                color=self.colors['text']
            )
        else:
            self.rpe_plot.setTitle('No RPE pairs available', color=self.colors['muted'])

    def _update_gui(self):
        self._drain_image_queue()

        pose_updated = self._drain_pose_queue()
        now = time.perf_counter()

        if pose_updated and now - self._last_pose_draw_at >= 0.033:
            self._redraw_live_path()
            self._last_pose_draw_at = now

        if now - self._last_title_update_at >= 0.35:
            self._update_fps_title()
            self._last_title_update_at = now

    def _start_image_load_if_idle(self):
        if self._image_load_busy:
            return
        if not self._pending_image_path:
            return

        path = self._pending_image_path
        self._pending_image_path = None
        self._image_load_busy = True

        future = self._image_executor.submit(self._load_image_file, path)

        def done_callback(fut, image_path=path):
            try:
                image = fut.result()
            except Exception:
                image = None
            self._image_notifier.loaded.emit(image_path, image)

        future.add_done_callback(done_callback)

    @staticmethod
    def _load_image_file(path):
        return cv2.imread(path, cv2.IMREAD_GRAYSCALE)

    def _on_image_loaded(self, path, image):
        self._image_load_busy = False

        if self._pending_image_path is not None and self._pending_image_path != path:
            self._start_image_load_if_idle()
            return

        if image is None:
            self.set_message(f'Could not load frame: {path}')
        else:
            self._display_image(image, path)

        self._start_image_load_if_idle()

    def _drain_image_queue(self):
        if not self._image_queue:
            return

        kind, payload = self._image_queue.pop()
        self._image_queue.clear()

        if kind == 'array':
            self._display_image(payload, None)

    def _display_image(self, image, path=None):
        if path is not None and path == self._last_drawn_image_path:
            return

        prepared = self._prepare_image(image)
        self.image_item.setImage(prepared, autoLevels=False, levels=(0, 255))

        if not self._last_camera_autorange:
            self.image_box.autoRange()
            self._last_camera_autorange = True

        self._last_drawn_image_path = path
        self._frame_count += 1
        now = time.perf_counter()
        self._last_image_draw_at = now
        self._image_draw_times.append(now)

    def _drain_pose_queue(self):
        if not self._pose_queue:
            return False

        updated = False
        max_per_tick = 300

        if len(self._pose_queue) > max_per_tick * 3:
            newest = list(self._pose_queue)[-max_per_tick:]
            self._pose_queue.clear()
            self._pose_queue.extend(newest)

        count = 0
        while self._pose_queue and count < max_per_tick:
            timestamp, position = self._pose_queue.popleft()
            self._timestamps.append(timestamp)
            self._positions.append(position)
            self._pose_count += 1
            updated = True
            count += 1

        return updated

    def _redraw_live_path(self):
        if len(self._positions) < 1:
            return

        positions = np.asarray(self._positions, dtype=np.float32)
        times = np.asarray(self._timestamps, dtype=np.float32)

        positions, times = self._sample_positions_with_times(
            positions,
            times,
            max_points=2600,
        )

        if len(times):
            times = times - times[0]

        self._trajectory_points = positions.astype(np.float32)
        self._groundtruth_points = np.zeros((0, 3), dtype=np.float32)
        self._trajectory_times = times.astype(np.float32)

        if HAS_GL:
            self.live_line.setData(pos=self._trajectory_points)
            self.head_point.setData(pos=self._trajectory_points[-1:].astype(np.float32))
            self._update_3d_camera_and_origin(self._trajectory_points)
        else:
            self.live_curve_2d.setData(self._trajectory_points[:, 0], self._trajectory_points[:, 1])
            self.head_curve_2d.setData([self._trajectory_points[-1, 0]], [self._trajectory_points[-1, 1]])

        self._draw_count += 1
        self._update_inspector_range(len(self._trajectory_points))


    def _update_3d_camera_and_origin(self, positions):
        if len(positions) < 2:
            return

        all_points = np.vstack([positions, np.zeros((1, 3), dtype=np.float32)])
        mins = np.min(all_points, axis=0)
        maxs = np.max(all_points, axis=0)

        center = (mins + maxs) * 0.5
        span = float(np.max(maxs - mins))
        span = max(span, 1.5)

        axis_len = max(0.7, min(4.0, span * 0.35))
        if abs(axis_len - self._axis_scale) > 0.25:
            self._update_origin_axes(axis_len)

        if self._draw_count % 18 != 0:
            return

        self.traj_widget.opts['center'] = pg.Vector(center[0], center[1], center[2])
        self.traj_widget.setCameraPosition(distance=span * 1.9)

    def _update_origin_axes(self, length):
        self._axis_scale = float(length)

        if not HAS_GL:
            return

        l = self._axis_scale
        self.axis_x.setData(pos=np.array([[0, 0, 0], [l, 0, 0]], dtype=np.float32))
        self.axis_y.setData(pos=np.array([[0, 0, 0], [0, l, 0]], dtype=np.float32))
        self.axis_z.setData(pos=np.array([[0, 0, 0], [0, 0, l]], dtype=np.float32))

        grid_size = max(4.0, l * 6.0)
        self._grid_step = max(0.5, grid_size / 10.0)
        self.grid_xy.setSize(grid_size, grid_size, 1)
        self.grid_xy.setSpacing(self._grid_step, self._grid_step, 1)

    def _set_gt_line(self, positions):
        if HAS_GL:
            self.gt_line.setData(pos=np.asarray(positions, dtype=np.float32))
        else:
            if len(positions):
                self.gt_curve_2d.setData(positions[:, 0], positions[:, 1])
            else:
                self.gt_curve_2d.setData([], [])
            self.cursor_curve_2d.setData([], [])

    def _clear_trajectory(self):
        empty = np.zeros((0, 3), dtype=np.float32)

        self._trajectory_points = empty.copy()
        self._groundtruth_points = empty.copy()
        self._trajectory_times = np.zeros((0,), dtype=np.float32)
        self._cursor_index = None
        self._inspector_follow_latest = True

        if HAS_GL:
            self.live_line.setData(pos=empty)
            self.gt_line.setData(pos=empty)
            self.head_point.setData(pos=empty)
            if hasattr(self, 'cursor_point'):
                self.cursor_point.setData(pos=empty)
            self.origin_point.setData(pos=np.array([[0, 0, 0]], dtype=np.float32))
        else:
            self.live_curve_2d.setData([], [])
            self.head_curve_2d.setData([], [])
            self.gt_curve_2d.setData([], [])
            if hasattr(self, 'cursor_curve_2d'):
                self.cursor_curve_2d.setData([], [])

        self._update_inspector_range(0)


    def _update_fps_title(self):
        fps = self._display_fps()
        self.camera_title.setText(f'Camera: cam0 (live) - FPS {fps:.1f}')
        self._set_trajectory_title()

    def _set_trajectory_title(self):
        mode = 'Final Aligned' if getattr(self, '_final_trajectory_mode', False) else 'Live'
        count = len(getattr(self, '_trajectory_points', []))

        if HAS_GL:
            title = f'{mode} 3D Trajectory'
        else:
            title = f'{mode} Trajectory (XY)'

        if count:
            title += f' | samples {count}'

        self.traj_title.setText(title)


    def _set_trajectory_title(self, cursor_text=None):
        mode = 'Final Aligned' if getattr(self, '_final_trajectory_mode', False) else 'Live'

        if HAS_GL:
            title = f'{mode} 3D Trajectory'
        else:
            title = f'{mode} Trajectory (XY)'

        count = len(getattr(self, '_trajectory_points', []))
        if count:
            title += f' | samples {count}'

        # Keep the visible title short. Detailed x/y/z/t information belongs
        # only in hover tooltip, not in the title bar.
        self.traj_title.setText(title)
        self.traj_title.setToolTip(
            'Red = estimate / VIO trajectory\n'
            'Green = ground truth / reference\n'
            'Axes: X red, Y green, Z blue\n'
            'Hover over the trajectory to inspect x, y, z, and time.'
        )


    def eventFilter(self, obj, event):
        if obj is getattr(self, 'traj_widget', None):
            if event.type() == QtCore.QEvent.MouseButtonPress:
                if event.button() == QtCore.Qt.RightButton:
                    self._clear_data_cursor()
                    QtWidgets.QToolTip.showText(
                        event.globalPos(),
                        'Inspector selection cleared',
                        self.traj_widget,
                    )
                    return True

                QtWidgets.QToolTip.showText(
                    event.globalPos(),
                    'Use the Trajectory Inspector slider for exact sample selection.',
                    self.traj_widget,
                )
                return True

        return super().eventFilter(obj, event)


    def _select_data_cursor(self, event):
        points = getattr(self, '_trajectory_points', None)

        if points is None or len(points) < 1:
            QtWidgets.QToolTip.showText(
                event.globalPos(),
                'No trajectory samples available yet',
                self.traj_widget,
            )
            return

        if HAS_GL:
            index, distance_px = self._nearest_projected_trajectory_index(event.pos())

            if index is None:
                QtWidgets.QToolTip.showText(
                    event.globalPos(),
                    'Could not project trajectory points for selection',
                    self.traj_widget,
                )
                return

            # Prevent accidental far-away selections.
            # If user clicks too far from the projected trajectory, do not pin a point.
            if distance_px is not None and distance_px > 35:
                QtWidgets.QToolTip.showText(
                    event.globalPos(),
                    'Click closer to the red trajectory line',
                    self.traj_widget,
                )
                return
        else:
            index = self._nearest_2d_trajectory_index(event.pos())
            if index is None:
                return

        self._set_cursor_marker_at_index(index, event.globalPos())


    def _nearest_projected_trajectory_index(self, mouse_pos):
        points = np.asarray(getattr(self, '_trajectory_points', []), dtype=np.float32)
        if len(points) < 1:
            return None, None

        try:
            projection = self.traj_widget.projectionMatrix()
            view = self.traj_widget.viewMatrix()
        except Exception:
            return self._nearest_index_by_x_ratio(mouse_pos), None

        width = max(1, int(self.traj_widget.width()))
        height = max(1, int(self.traj_widget.height()))

        mouse_x = float(mouse_pos.x())
        mouse_y = float(mouse_pos.y())

        best_index = None
        best_dist2 = float('inf')

        # Picking does not need all 2600 points every click if the path is large.
        # But keep the returned index mapped to the real sample index.
        count = len(points)
        if count > 3500:
            sample_indices = np.linspace(0, count - 1, 3500).astype(int)
        else:
            sample_indices = np.arange(count)

        for index in sample_indices:
            x, y, z = points[index]
            clip = projection * view * QtGui.QVector4D(float(x), float(y), float(z), 1.0)
            w = clip.w()

            if abs(w) < 1e-9:
                continue

            ndc_x = clip.x() / w
            ndc_y = clip.y() / w
            ndc_z = clip.z() / w

            # Ignore points behind the camera or far outside clip volume.
            if ndc_z < -1.5 or ndc_z > 1.5:
                continue

            screen_x = (ndc_x + 1.0) * 0.5 * width
            screen_y = (1.0 - ndc_y) * 0.5 * height

            dx = screen_x - mouse_x
            dy = screen_y - mouse_y
            dist2 = dx * dx + dy * dy

            if dist2 < best_dist2:
                best_dist2 = dist2
                best_index = int(index)

        if best_index is None:
            return self._nearest_index_by_x_ratio(mouse_pos), None

        return best_index, best_dist2 ** 0.5

    def _nearest_2d_trajectory_index(self, mouse_pos):
        points = np.asarray(getattr(self, '_trajectory_points', []), dtype=np.float32)
        if len(points) < 1:
            return None

        if not hasattr(self, 'traj_widget') or not hasattr(self.traj_widget, 'plotItem'):
            return self._nearest_index_by_x_ratio(mouse_pos)

        try:
            view_pos = self.traj_widget.plotItem.vb.mapSceneToView(mouse_pos)
            mouse_x = float(view_pos.x())
            mouse_y = float(view_pos.y())
        except Exception:
            return self._nearest_index_by_x_ratio(mouse_pos)

        dx = points[:, 0] - mouse_x
        dy = points[:, 1] - mouse_y
        dist2 = dx * dx + dy * dy
        return int(np.argmin(dist2))

    def _nearest_index_by_x_ratio(self, mouse_pos):
        points = getattr(self, '_trajectory_points', None)
        if points is None or len(points) < 1:
            return None

        width = max(1, self.traj_widget.width() - 1)
        ratio = float(mouse_pos.x()) / float(width)
        ratio = max(0.0, min(1.0, ratio))
        index = int(round(ratio * (len(points) - 1)))
        return max(0, min(len(points) - 1, index))


    def _update_data_cursor(self, event):
        # Backward-compatible alias.
        # Do not call this from MouseMove; selection should happen on click only.
        self._select_data_cursor(event)



    def _display_fps(self):
        if len(self._image_draw_times) < 2:
            return 0.0
        dt = self._image_draw_times[-1] - self._image_draw_times[0]
        if dt <= 1e-6:
            return 0.0
        return (len(self._image_draw_times) - 1) / dt

    def _prepare_image(self, image):
        image = np.asarray(image)

        if image.ndim == 2:
            prepared = image
        elif image.ndim == 3 and image.shape[2] == 1:
            prepared = image[:, :, 0]
        elif image.ndim == 3 and image.shape[2] == 3:
            prepared = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        elif image.ndim == 3 and image.shape[2] == 4:
            prepared = cv2.cvtColor(image, cv2.COLOR_BGRA2RGBA)
        else:
            raise ValueError(f'Unsupported image shape: {image.shape}')

        return np.ascontiguousarray(np.rot90(prepared, -1))

    @staticmethod
    def _sample_positions_with_times(positions, times, max_points=2600):
        positions = np.asarray(positions)
        times = np.asarray(times)

        count = min(len(positions), len(times))
        positions = positions[:count]
        times = times[:count]

        if count <= max_points:
            return positions, times

        idx = np.linspace(0, count - 1, max_points).astype(int)
        return positions[idx], times[idx]

    @staticmethod
    def _sample_aligned_trajectory(estimate, groundtruth, times, max_points=2600):
        estimate = np.asarray(estimate)
        groundtruth = np.asarray(groundtruth)
        times = np.asarray(times)

        count = min(len(estimate), len(groundtruth), len(times))
        estimate = estimate[:count]
        groundtruth = groundtruth[:count]
        times = times[:count]

        if count <= max_points:
            return estimate, groundtruth, times

        indices = np.linspace(0, count - 1, max_points).astype(int)
        return estimate[indices], groundtruth[indices], times[indices]

    @staticmethod
    def _sample_positions(positions, max_points=2600):
        positions = np.asarray(positions)
        if len(positions) <= max_points:
            return positions
        idx = np.linspace(0, len(positions) - 1, max_points).astype(int)
        return positions[idx]

    @staticmethod
    def _sample_xy(x, y, max_points=1600):
        if len(x) <= max_points:
            return x, y
        idx = np.linspace(0, len(x) - 1, max_points).astype(int)
        return x[idx], y[idx]
