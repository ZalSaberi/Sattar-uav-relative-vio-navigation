import queue
import sys
import threading
import time

import cv2
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
import pyqtgraph.opengl as gl


class SimpleViewer(QtWidgets.QMainWindow):
    def __init__(self, history=1000, video_path=None, fps=30):
        super().__init__()
        self.setWindowTitle('UAV-Airvision Viewer')
        self.resize(1280, 720)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.setCentralWidget(splitter)

        self.img_widget = pg.GraphicsLayoutWidget()
        splitter.addWidget(self.img_widget)
        self.img_item = pg.ImageItem()
        view = self.img_widget.addViewBox(lockAspect=True)
        view.addItem(self.img_item)

        self.gl = gl.GLViewWidget()
        self.gl.opts['distance'] = 6
        splitter.addWidget(self.gl)
        self.gl.addItem(gl.GLAxisItem(size=QtGui.QVector3D(1, 1, 1)))

        self.est_line = gl.GLLinePlotItem(color=(1, 0, 0, 1), width=2, antialias=True)
        self.pts_item = gl.GLScatterPlotItem(size=4, color=(1, 1, 0, 1))
        self.gl.addItem(self.est_line)
        self.gl.addItem(self.pts_item)

        self.history = history
        self.est_buf = np.empty((0, 3))
        self._img_q = queue.Queue(1)
        self._est_q = queue.Queue()
        self._pts_q = queue.Queue()
        self._running = True

        self.status = QtWidgets.QLabel()
        self.statusBar().addPermanentWidget(self.status)
        self.statusBar().addPermanentWidget(QtWidgets.QLabel(
            "<font color='red'>trajectory</font>  "
            "<font color='yellow'>points</font>"))
        self._cnt = 0
        self._t0 = time.time()

        self.video = None
        self._record_start = time.time()
        self._record_len = 50.0
        if video_path:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video = cv2.VideoWriter(
                video_path, fourcc, fps, (self.width(), self.height()))

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update_gui)
        self.timer.start(int(1000 / fps))

    def update_image(self, img):
        if not self._running:
            return
        try:
            if self._img_q.full():
                self._img_q.get_nowait()
            self._img_q.put_nowait(np.asarray(img).copy())
        except (queue.Empty, queue.Full):
            pass

    def update_pose(self, T):
        if self._running:
            self._est_q.put(np.asarray(T.t, dtype=float))

    def update_points(self, pts):
        if self._running:
            self._pts_q.put(np.asarray(pts, dtype=float))

    def _prepare_image(self, img):
        img = np.asarray(img)
        if img.ndim == 2:
            return np.ascontiguousarray(np.rot90(img, -1)), True
        if img.ndim == 3 and img.shape[2] == 1:
            gray = img[:, :, 0]
            return np.ascontiguousarray(np.rot90(gray, -1)), True
        if img.ndim == 3 and img.shape[2] == 3:
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            return np.ascontiguousarray(np.rot90(rgb, -1)), False
        if img.ndim == 3 and img.shape[2] == 4:
            rgba = cv2.cvtColor(img, cv2.COLOR_BGRA2RGBA)
            return np.ascontiguousarray(np.rot90(rgba, -1)), False
        raise ValueError(f'Unsupported viewer image shape: {img.shape}')

    def _update_gui(self):
        try:
            img = self._img_q.get_nowait()
            img, is_grayscale = self._prepare_image(img)
            self.img_item.setImage(img, autoLevels=is_grayscale)
        except queue.Empty:
            pass
        except ValueError as exc:
            self.status.setText(str(exc))

        while not self._est_q.empty():
            point = self._est_q.get()
            self.est_buf = np.vstack([self.est_buf, point])
            if self.est_buf.shape[0] > self.history:
                self.est_buf = self.est_buf[-self.history:]

        if self.est_buf.shape[0] >= 2:
            self.est_line.setData(pos=self.est_buf)
            cx, cy, cz = self.est_buf[-1]
            self.gl.setCameraPosition(pos=QtGui.QVector3D(cx, cy, cz + 3))

        try:
            pts = self._pts_q.get_nowait()
            self.pts_item.setData(pos=pts)
        except queue.Empty:
            pass

        self._cnt += 1
        now = time.time()
        if now - self._t0 > 0.5:
            fps_val = self._cnt / (now - self._t0)
            self.status.setText(f'FPS {fps_val:.1f}')
            self._cnt = 0
            self._t0 = now

        if self.video is not None and self.video.isOpened() and (now - self._record_start) < self._record_len:
            qimg = self.grab().toImage().convertToFormat(QtGui.QImage.Format.Format_RGB888)
            width, height = qimg.width(), qimg.height()
            ptr = qimg.bits()
            ptr.setsize(height * width * 3)
            frame = np.frombuffer(ptr, np.uint8).reshape(height, width, 3)
            self.video.write(frame[..., ::-1])

    def closeEvent(self, event):
        self._running = False
        if self.video is not None and self.video.isOpened():
            self.video.release()
        event.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    viewer = SimpleViewer()
    viewer.show()

    def demo():
        capture = cv2.VideoCapture(0)
        t = 0.0
        while viewer._running and capture.isOpened():
            ok, frame = capture.read()
            if not ok:
                break
            viewer.update_image(frame)
            viewer.update_pose(type('Pose', (), {
                't': np.array([np.cos(t), np.sin(t), 0.1 * t])
            })())
            viewer.update_points(
                np.random.randn(500, 3) * 0.1 + [np.cos(t), np.sin(t), 0.1 * t])
            t += 0.05
            time.sleep(1 / 30)
        capture.release()

    threading.Thread(target=demo, daemon=True).start()
    sys.exit(app.exec_())
