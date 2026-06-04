# Runtime Runbook

This runbook covers the Phase 1 runtime baseline for Windows.

## Install

Use Python 3.10 or newer from the repository root.

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

The core install is headless. Viewer mode is optional and needs extra packages:

```powershell
python -m pip install PyQt5 pyqtgraph
```

## Dataset Layout

Pass the path to one EuRoC sequence directory, for example `datasets\MH_01_easy`.
The runner validates this layout before processing:

```text
<dataset>\mav0\imu0\data.csv
<dataset>\mav0\state_groundtruth_estimate0\data.csv
<dataset>\mav0\cam0\data\*.png
<dataset>\mav0\cam1\data\*.png
```

Datasets are intentionally ignored by git.

## Smoke Check

Check imports only:

```powershell
python tools\smoke_check.py
```

Check imports and a dataset path:

```powershell
python tools\smoke_check.py --dataset .\datasets\MH_01_easy
```

Check optional viewer dependencies too:

```powershell
python tools\smoke_check.py --check-viewer
```

## Run

Supported command from the repository root:

```powershell
python main.py --path .\datasets\MH_01_easy --offset 10
```

Equivalent package command:

```powershell
python -m src.main --path .\datasets\MH_01_easy --offset 10
```

Optional viewer mode:

```powershell
python main.py --path .\datasets\MH_01_easy --offset 10 --view
```

By default, each run replaces the output file for the same dataset and offset. To append intentionally:

```powershell
python main.py --path .\datasets\MH_01_easy --offset 10 --append-output
```

To write output somewhere else:

```powershell
python main.py --path .\datasets\MH_01_easy --offset 10 --output-dir .\outputs\txts
```

## Expected Outputs

The default trajectory file is created automatically:

```text
results\txts\output_<dataset>_offset<offset>.txt
```

The file starts with:

```text
# timestamp p_x p_y p_z q_x q_y q_z q_w
```

Viewer mode may also create `output.mp4` in the current working directory.

## Troubleshooting

If the command says `Dataset path does not exist`, check that `--path` points at a sequence directory such as `MH_01_easy`, not the parent `datasets` directory.

If the command says `Invalid EuRoC dataset structure`, compare the missing paths in the error with the dataset layout above.

If `--view` fails with missing `PyQt5` or `pyqtgraph`, install the optional viewer packages or run without `--view`.

If imports fail in the smoke check, activate the virtual environment and reinstall `requirements.txt`.

If a repeated run seems to lose old output, that is expected in Phase 1. Use `--append-output` when appending is intentional.
