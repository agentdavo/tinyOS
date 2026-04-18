#!/usr/bin/env bash
# SPDX-License-Identifier: MIT OR Apache-2.0
# Serve the machine editor + its sibling TSV / STL / OBJ files over plain
# HTTP. Same-folder file:// access works for relative <script src=> tags in
# most browsers, but:
#   - Chrome still treats every file:// URL as a unique security origin, so
#     fetch("./kinematic_mx850.tsv") is blocked unless you run Chrome with
#     --allow-file-access-from-files.
#   - File System Access API (showOpenFilePicker) requires a secure context;
#     file:// is not one.
#   - webkitGetAsEntry (folder drop) likewise refuses without a secure origin.
#   - WSL paths (file://wsl.localhost/...) are blocked particularly hard.
#
# Easiest answer: run this script and browse http://localhost:8765/ — all
# fetches, drops, pickers, and folder loads just work.
#
# Usage:
#   bash machines/serve.sh                # http://localhost:8765/
#   MACHINE_EDITOR_PORT=9000 bash serve.sh

set -euo pipefail

PORT="${MACHINE_EDITOR_PORT:-8765}"
DIR="$(cd "$(dirname "$0")" && pwd)"

echo "[machine-editor] serving $DIR on http://localhost:$PORT/"
echo "[machine-editor] Ctrl-C to stop"
cd "$DIR"
exec python3 -m http.server "$PORT" --bind 127.0.0.1
