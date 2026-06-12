#!/usr/bin/env bash
cd "$(dirname "$0")"
source .venv/Scripts/activate

python tools/evaluation_dashboard.py \
  --datasets-root ./datasets \
  --results-root ./results \
  --warmup-skip-seconds 20
