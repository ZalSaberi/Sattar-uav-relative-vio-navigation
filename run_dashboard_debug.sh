#!/usr/bin/env bash
cd "$(dirname "$0")"
source .venv/Scripts/activate

DASHBOARD_SYNC_DEBUG=1 python tools/evaluation_dashboard.py \
  --datasets-root ./datasets \
  --results-root ./results \
  --warmup-skip-seconds 20
