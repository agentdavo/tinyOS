#!/usr/bin/env bash
# SPDX-License-Identifier: MIT OR Apache-2.0
# Serve the UI editor over plain HTTP so Chrome stops refusing sub-
# resource loads from the unique-origin `file://` scheme. WSL paths
# (file://wsl.localhost/...) are the worst offenders — Chrome blocks
# everything including vendor scripts and the folder-drop APIs.
#
# Usage:
#   bash tools/ui_editor/serve.sh          # port 8766
#   UI_EDITOR_PORT=9001 bash serve.sh      # custom port
#
# Then browse to http://localhost:8766/ — File->Open, load/save all work.

set -euo pipefail

PORT="${UI_EDITOR_PORT:-8766}"
DIR="$(cd "$(dirname "$0")" && pwd)"

echo "[ui-editor] serving $DIR on http://localhost:$PORT/"
echo "[ui-editor] Ctrl-C to stop"
cd "$DIR"
exec python3 -m http.server "$PORT" --bind 127.0.0.1
