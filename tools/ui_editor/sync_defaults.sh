#!/usr/bin/env bash
# SPDX-License-Identifier: MIT OR Apache-2.0
# Regenerate the DEFAULT_UI_TSV block inside index.html from the kernel's
# shipped devices/embedded_ui.tsv. Run this whenever that file changes so
# File -> New opens with the current canonical operator surface.

set -euo pipefail

ROOT=$(cd "$(dirname "$0")/../.." && pwd)
INDEX="$ROOT/tools/ui_editor/index.html"

read_file() {
  sed -e 's/\\/\\\\/g' -e 's/`/\\`/g' -e 's/\${/\\${/g' "$1"
}

UI=$(read_file "$ROOT/devices/embedded_ui.tsv")

python3 - "$INDEX" <<PY
import re, sys
path = sys.argv[1]
with open(path, 'r') as f:
    html = f.read()

ui = """$UI"""

block = (
    "// ---- BEGIN SYNCED DEFAULTS (regenerate via tools/ui_editor/sync_defaults.sh) ----\n"
    "const DEFAULT_UI_TSV = \`" + ui + "\`;\n"
    "// ---- END SYNCED DEFAULTS ----"
)

pat = re.compile(r"// ---- BEGIN SYNCED DEFAULTS.*?// ---- END SYNCED DEFAULTS ----", re.DOTALL)
if pat.search(html):
    html = pat.sub(block, html)
else:
    idx = html.find("function newDoc()")
    if idx < 0:
        sys.exit("newDoc() anchor not found in index.html")
    line_start = html.rfind("\n", 0, idx) + 1
    html = html[:line_start] + block + "\n\n" + html[line_start:]

with open(path, 'w') as f:
    f.write(html)
print("synced defaults into", path)
PY
