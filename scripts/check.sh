#!/usr/bin/env bash
set -euo pipefail

python -m compileall -q src tests scripts
python -m pytest
