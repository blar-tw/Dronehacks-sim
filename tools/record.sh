#!/usr/bin/env bash
# DEVELOPMENT ONLY. Rule 6.4: data recorded in flight is never fed to a later run. Bags stay in
# the development environment; flight code never reads bags.
# Records every topic into ${LOG_DIR:-<workspace>/logs/bags}/<YYYYmmdd-HHMMSS>.
# Extra arguments go to `ros2 bag record` (e.g. `-x <regex>` to skip image topics).
set -euo pipefail
dir="${LOG_DIR:-$(cd "$(dirname "$0")/.." && pwd)/logs/bags}"
mkdir -p "$dir"
exec ros2 bag record -a -o "$dir/$(date +%Y%m%d-%H%M%S)" "$@"
