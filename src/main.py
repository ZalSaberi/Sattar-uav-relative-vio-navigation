import os
import time
import argparse
from queue import Queue
from threading import Thread

from .config import ConfigEuRoC
from .streaming.dataset import DatasetValidationError, EuRoCDataset
from .streaming.publisher import DataPublisher


def create_viewer():
    try:
        from PyQt5 import QtWidgets
        from .viewer import SimpleViewer
    except ImportError as exc:
        raise RuntimeError(
            "Viewer mode requires PyQt5 and pyqtgraph. Install them with "
            "`python -m pip install PyQt5 pyqtgraph`, or run without --view."
        ) from exc

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    viewer = SimpleViewer()
    viewer.show()
    return app, viewer


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--path',   default='./datasets/V2_03_difficult')
    parser.add_argument('--offset', type=float, default=10.)
    parser.add_argument('--view',   action='store_true')
    parser.add_argument(
        '--output-dir',
        default=os.path.join('results', 'txts'),
        help='Directory for trajectory text output. Default: results/txts')
    parser.add_argument(
        '--append-output',
        action='store_true',
        help='Append to an existing output file instead of replacing it.')
    args = parser.parse_args(argv)

    try:
        dataset = EuRoCDataset(args.path)
    except DatasetValidationError as exc:
        parser.error(str(exc))
    dataset.set_starttime(offset=args.offset)

    name = os.path.basename(os.path.normpath(args.path))
    os.environ['DATASET_NAME'] = name
    os.environ['TIME_OFFSET']  = str(int(args.offset))
    os.environ['OUTPUT_DIR'] = args.output_dir
    os.environ['APPEND_OUTPUT'] = '1' if args.append_output else '0'

    from .modules.vio import VIO

    img_q, imu_q = Queue(), Queue()
    app, viewer = (None, None)
    if args.view:
        try:
            app, viewer = create_viewer()
        except RuntimeError as exc:
            parser.error(str(exc))

    vio = VIO(ConfigEuRoC(), img_q, imu_q, viewer)
    vio.start()

    now = time.time()
    publishers = [
        DataPublisher(dataset.imu, imu_q, duration=float('inf'), ratio=0.4),
        DataPublisher(dataset.stereo, img_q, duration=float('inf'), ratio=0.4),
    ]
    for publisher in publishers:
        publisher.start(now)

    def wait_for_completion():
        for publisher in publishers:
            publisher.join()
        vio.join()

    if app is not None:
        def wait_then_quit():
            wait_for_completion()
            app.quit()

        Thread(target=wait_then_quit, daemon=True).start()
        return app.exec_()

    wait_for_completion()
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
