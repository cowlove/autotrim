#!/bin/bash
set -euo pipefail

exec python3 tests/csim/run.py "$@"
