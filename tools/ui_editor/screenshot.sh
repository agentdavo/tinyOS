#!/usr/bin/env bash
# SPDX-License-Identifier: MIT OR Apache-2.0
#
# screenshot.sh — render a UI editor page via headless Chrome and save it.
#
# Usage:
#   bash tools/ui_editor/screenshot.sh <page_id> [out_path] [tsv_path]
#
# Examples:
#   bash tools/ui_editor/screenshot.sh ethercat
#   bash tools/ui_editor/screenshot.sh dashboard /tmp/dash.png
#   bash tools/ui_editor/screenshot.sh ethercat /tmp/ec.png devices/embedded_ui.tsv
#
# Defaults:
#   out_path = /tmp/ui_<page_id>.png
#   tsv_path = devices/embedded_ui.tsv
#
# How it works:
#   - Spins up python3 -m http.server rooted at the REPO ROOT (so the
#     editor can fetch ../devices/*.tsv via a server-absolute URL).
#   - Opens http://localhost:$PORT/tools/ui_editor/index.html?tsv=...&page=...&ready=1
#     in headless Chrome with a 1080x1920 window matching the kernel canvas.
#   - Waits up to 15 s for the editor to paint the #__editor_ready marker,
#     then saves a PNG and tears down the server.
#
# Requires:
#   - python3
#   - google-chrome (or pass CHROME=/path/to/chromium-binary)

set -euo pipefail

PAGE="${1:-}"
if [[ -z "$PAGE" ]]; then
  echo "usage: $0 <page_id> [out_path] [tsv_path]" >&2
  exit 2
fi
OUT="${2:-/tmp/ui_${PAGE}.png}"
TSV_REL="${3:-devices/embedded_ui.tsv}"
PORT="${UI_EDITOR_PORT:-8767}"
CHROME="${CHROME:-google-chrome}"

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
if [[ ! -f "$ROOT/$TSV_REL" ]]; then
  echo "error: $ROOT/$TSV_REL not found" >&2
  exit 2
fi
if ! command -v "$CHROME" >/dev/null 2>&1; then
  echo "error: $CHROME not on PATH (set CHROME=/path/to/chromium)" >&2
  exit 2
fi

# Free the port if a stray instance is still bound to it.
fuser -k "${PORT}/tcp" >/dev/null 2>&1 || true

cd "$ROOT"
python3 -m http.server "$PORT" --bind 127.0.0.1 >/tmp/ui_editor_server.log 2>&1 &
SERVER_PID=$!
trap 'kill "$SERVER_PID" >/dev/null 2>&1 || true' EXIT

# Wait for the server to be ready.
for _ in 1 2 3 4 5 6 7 8 9 10; do
  if curl -fsS "http://127.0.0.1:${PORT}/tools/ui_editor/index.html" >/dev/null 2>&1; then
    break
  fi
  sleep 0.2
done

URL="http://127.0.0.1:${PORT}/tools/ui_editor/index.html?tsv=${TSV_REL}&page=${PAGE}&canvas_only=1&ready=1"
echo "[screenshot] $URL -> $OUT"

# --virtual-time-budget gives the JS time to fetch + parse the TSV and
# then idle until #__editor_ready is appended. 15 s is comfortable; the
# whole render is well under a second on modern hardware.
"$CHROME" \
  --headless=new \
  --hide-scrollbars \
  --disable-gpu \
  --no-sandbox \
  --window-size=1080,1920 \
  --virtual-time-budget=15000 \
  --screenshot="$OUT" \
  "$URL" >/tmp/ui_editor_chrome.log 2>&1

if [[ -s "$OUT" ]]; then
  echo "[screenshot] wrote $OUT ($(stat -c %s "$OUT") bytes)"
else
  echo "[screenshot] FAILED — see /tmp/ui_editor_chrome.log" >&2
  exit 1
fi
