import argparse
import importlib
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

CORE_MODULES = [
    "numpy",
    "cv2",
    "scipy",
    "src.main",
    "src.streaming.dataset",
    "src.image_processing",
    "src.msckf",
]


def check_import(module_name):
    try:
        importlib.import_module(module_name)
    except Exception as exc:
        print(f"FAIL import {module_name}: {exc}", file=sys.stderr)
        return False
    print(f"OK import {module_name}")
    return True


def check_dataset(path):
    from src.streaming.dataset import DatasetValidationError, validate_euroc_dataset

    try:
        resolved = validate_euroc_dataset(path)
    except DatasetValidationError as exc:
        print(f"FAIL dataset: {exc}", file=sys.stderr)
        return False
    print(f"OK dataset {resolved}")
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(description="Lightweight runtime smoke check.")
    parser.add_argument(
        "--dataset",
        help="Optional EuRoC dataset path to validate without running VIO.")
    parser.add_argument(
        "--check-viewer",
        action="store_true",
        help="Also import optional PyQt5/pyqtgraph viewer dependencies.")
    args = parser.parse_args(argv)

    ok = True
    for module_name in CORE_MODULES:
        ok = check_import(module_name) and ok

    if args.check_viewer:
        ok = check_import("src.viewer") and ok

    if args.dataset:
        ok = check_dataset(args.dataset) and ok

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
