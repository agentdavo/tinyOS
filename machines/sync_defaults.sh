#!/usr/bin/env bash
# SPDX-License-Identifier: MIT OR Apache-2.0
# Regenerate the DEFAULT_KINEMATIC_TSV / DEFAULT_PODS_TSV / DEFAULT_OBJS
# blocks inside index.html from files in this flat machines/ directory
# and the kernel's devices/ defaults. Run after any TSV / mesh change so
# File -> New opens with the current canonical machine.
#
# Since the editor, the kinematic TSVs, and the STL/OBJ files all live as
# siblings here, "sync" just inlines whatever's alongside.

set -euo pipefail

MACHINES_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$MACHINES_DIR/.." && pwd)
INDEX="$MACHINES_DIR/index.html"

read_file() {
  # JS template-literal safe: escape backslash, backtick, ${ sequences.
  sed -e 's/\\/\\\\/g' -e 's/`/\\`/g' -e 's/\${/\\${/g' "$1"
}

# Prefer machines/ copies; fall back to devices/ for files that still
# live there (demo_box.obj, demo_part.stl, embedded_toolpods.tsv).
pick() {
  if [[ -f "$MACHINES_DIR/$1" ]]; then echo "$MACHINES_DIR/$1"
  elif [[ -f "$ROOT/devices/$1" ]]; then echo "$ROOT/devices/$1"
  else echo ""
  fi
}

KIN_SRC=$(pick "kinematic_mill3.tsv")
PODS_SRC=$(pick "embedded_toolpods.tsv")
OBJ_DEMO_SRC=$(pick "demo_box.obj")
STL_DEMO_SRC=$(pick "demo_part.stl")

[[ -n "$KIN_SRC" ]] || { echo "kinematic_mill3.tsv not found"; exit 1; }

KIN=$(read_file "$KIN_SRC")
PODS=$([[ -n "$PODS_SRC" ]] && read_file "$PODS_SRC" || echo "")
OBJ_DEMO=$([[ -n "$OBJ_DEMO_SRC" ]] && read_file "$OBJ_DEMO_SRC" || echo "")
STL_DEMO=$([[ -n "$STL_DEMO_SRC" ]] && read_file "$STL_DEMO_SRC" || echo "")

python3 - "$INDEX" <<PY
import re, sys
path = sys.argv[1]
with open(path, 'r') as f:
    html = f.read()

kin = """$KIN"""
pods = """$PODS"""
obj_demo = """$OBJ_DEMO"""
stl_demo = """$STL_DEMO"""

block = (
    "// ---- BEGIN SYNCED DEFAULTS (regenerate via machines/sync_defaults.sh) ----\n"
    "const DEFAULT_KINEMATIC_TSV = \`" + kin + "\`;\n"
    "const DEFAULT_PODS_TSV = \`" + pods + "\`;\n"
    "const DEFAULT_OBJS = {\n"
    "  \"demo_box.obj\": \`" + obj_demo + "\`,\n"
    "  \"demo_part.stl\": \`" + stl_demo + "\`,\n"
    "};\n"
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
