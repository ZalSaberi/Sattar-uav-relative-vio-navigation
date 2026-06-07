from collections import deque
from concurrent.futures import ThreadPoolExecutor
import time

import cv2
import numpy as np
from PyQt5 import QtCore, QtWidgets
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

            self.traj_widget.addItem(self.gt_line)
            self.traj_widget.addItem(self.live_line)
            self.traj_widget.addItem(self.head_point)
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

        layout.addWidget(self._card_widget(self.camera_title, self.image_view), 0, 0)
        layout.addWidget(self._card_widget(self.traj_title, self.traj_widget), 0, 1)
        layout.addWidget(self._card('ATE (Absolute Trajectory Error)', self.ate_plot), 1, 0)
        layout.addWidget(self._card('RTE (Relative Translation Error) - w=1s', self.rpe_plot), 1, 1)

        layout.setRowStretch(0, 60)
        layout.setRowStretch(1, 40)
        layout.setColumnStretch(0, 50)
        layout.setColumnStretch(1, 50)

        self.set_message('No frame preview available')
        self.draw_evaluation_placeholders()

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

    def set_final_trajectory(self, estimate, groundtruth):
        """
        Show the final evaluated trajectory pair in the 3D Live View.

        Red: estimate_aligned
        Green: groundtruth_interpolated

        Both are normalized with the same origin, so the visual comparison is
        meaningful and the green/red paths do not drift apart because of
        different local origins.
        """
        estimate = self._to_xyz_array(estimate)
        groundtruth = self._to_xyz_array(groundtruth)

        if len(estimate) < 2 or len(groundtruth) < 2:
            self._clear_trajectory()
            return

        count = min(len(estimate), len(groundtruth))
        estimate = estimate[:count]
        groundtruth = groundtruth[:count]

        # Use one shared origin. Since estimate is already aligned to groundtruth
        # by evaluate_trajectory, this preserves the real visual error while
        # putting the pair near the coordinate origin.
        origin = groundtruth[0].copy()
        estimate_rel = estimate - origin
        groundtruth_rel = groundtruth - origin

        estimate_rel = self._sample_positions(estimate_rel, max_points=2600).astype(np.float32)
        groundtruth_rel = self._sample_positions(groundtruth_rel, max_points=2600).astype(np.float32)

        self._positions.clear()
        self._timestamps.clear()
        self._pose_queue.clear()
        self._pose_count = len(estimate_rel)
        self._final_trajectory_mode = True

        if HAS_GL:
            self.live_line.setData(pos=estimate_rel)
            self.gt_line.setData(pos=groundtruth_rel)
            self.head_point.setData(pos=estimate_rel[-1:].astype(np.float32))

            combined = np.vstack([
                estimate_rel,
                groundtruth_rel,
                np.zeros((1, 3), dtype=np.float32),
            ])
            self._update_3d_camera_and_origin(combined)
        else:
            self.live_curve_2d.setData(estimate_rel[:, 0], estimate_rel[:, 1])
            self.head_curve_2d.setData([estimate_rel[-1, 0]], [estimate_rel[-1, 1]])
            self.gt_curve_2d.setData(groundtruth_rel[:, 0], groundtruth_rel[:, 1])

        self.traj_title.setText(
            'Final Aligned 3D Trajectory | red=estimate, green=ground truth'
            if HAS_GL else
            'Final Aligned Trajectory | red=estimate, green=ground truth'
        )

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
        positions = self._sample_positions(positions, max_points=2600)

        if HAS_GL:
            self.live_line.setData(pos=positions)
            self.head_point.setData(pos=positions[-1:].astype(np.float32))
            self._update_3d_camera_and_origin(positions)
        else:
            self.live_curve_2d.setData(positions[:, 0], positions[:, 1])
            self.head_curve_2d.setData([positions[-1, 0]], [positions[-1, 1]])

        self._draw_count += 1

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
        self.grid_xy.setSize(grid_size, grid_size, 1)
        self.grid_xy.setSpacing(max(0.5, grid_size / 10.0), max(0.5, grid_size / 10.0), 1)

    def _set_gt_line(self, positions):
        if HAS_GL:
            self.gt_line.setData(pos=np.asarray(positions, dtype=np.float32))
        else:
            if len(positions):
                self.gt_curve_2d.setData(positions[:, 0], positions[:, 1])
            else:
                self.gt_curve_2d.setData([], [])

    def _clear_trajectory(self):
        empty = np.zeros((0, 3), dtype=np.float32)

        if HAS_GL:
            self.live_line.setData(pos=empty)
            self.gt_line.setData(pos=empty)
            self.head_point.setData(pos=empty)
            self.origin_point.setData(pos=np.array([[0, 0, 0]], dtype=np.float32))
        else:
            self.live_curve_2d.setData([], [])
            self.head_curve_2d.setData([], [])
            self.gt_curve_2d.setData([], [])

    def _update_fps_title(self):
        fps = self._display_fps()
        self.camera_title.setText(f'Camera: cam0 (live) - FPS {fps:.1f}')

        if getattr(self, '_final_trajectory_mode', False):
            self.traj_title.setText(
                f'Final Aligned 3D Trajectory | samples {self._pose_count} | origin (0,0,0)'
                if HAS_GL else
                f'Final Aligned Trajectory | samples {self._pose_count}'
            )
            return

        self.traj_title.setText(
            f'Trajectory 3D - Live | poses {self._pose_count} | origin (0,0,0)'
            if HAS_GL else
            f'Trajectory (XY) - Live | poses {self._pose_count}'
        )

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
