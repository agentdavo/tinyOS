#!/usr/bin/env bash
# SPDX-License-Identifier: MIT OR Apache-2.0
# Build a FAT32 sdcard.img from the flat machines/ authoring directory.
#
# Layout on disk (flat — matches how the machine editor expects to find
# sibling files without needing path prefixes):
#
#   /system/machine/kinematic_mill3.tsv
#   /system/machine/kinematic_millturn.tsv
#   /system/machine/kinematic_mx850.tsv
#   /system/machine/MX850 - DOC*.STL
#   /system/machine/demo_box.obj                 (from devices/)
#   /system/machine/demo_part.stl                (from devices/)
#   /system/machine/embedded_toolpods.tsv        (from devices/)
#   /system/ui/embedded_ui.tsv                   (from devices/)
#
# The OBJ registry rewrites an axis's `obj_file=foo.stl` reference to
# `system/machine/foo.stl`, so a flat layout means the TSV's obj_file
# column is just the basename — no `mx850/` prefix needed.

set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
IMG="$ROOT/sdcard.img"
SIZE_MB=${IMG_SIZE_MB:-64}

command -v mkfs.vfat >/dev/null || {
    echo "error: mkfs.vfat missing (apt install dosfstools)"; exit 1; }
python3 -c "from pyfatfs.PyFatFS import PyFatFS" 2>/dev/null || {
    echo "error: pyfatfs missing (pip3 install --user --break-system-packages pyfatfs)"; exit 1; }

echo "[mkimg] preparing $IMG (${SIZE_MB} MiB FAT32)"
truncate -s "${SIZE_MB}M" "$IMG"
mkfs.vfat -F 32 -n MINIOSSD "$IMG" >/dev/null

python3 - "$IMG" "$ROOT" <<'PY'
import os, sys, glob
from pyfatfs.PyFatFS import PyFatFS

img, root = sys.argv[1], sys.argv[2]
fs = PyFatFS(img, read_only=False, preserve_case=True)

def mkdir_p(path):
    parts = [p for p in path.strip("/").split("/") if p]
    cur = "/"
    for p in parts:
        cur = cur + p + "/"
        if not fs.exists(cur):
            fs.makedir(cur)

def put(src, dst):
    if not os.path.isfile(src): return
    # Skip Windows ADS metadata files if they end up in the source tree.
    if src.endswith(":Zone.Identifier"): return
    mkdir_p(os.path.dirname(dst))
    with open(src, "rb") as f:
        fs.writebytes(dst, f.read())
    print(f"  {dst}  ({os.path.getsize(src)} bytes)")

# Flat machines/ directory: every .tsv + .obj + .stl lands in
# /system/machine/. Ignores subdirectories, the vendor/ folder, Zone.Identifier
# files, and the editor tooling.
machines_dir = os.path.join(root, "machines")
if os.path.isdir(machines_dir):
    for fname in sorted(os.listdir(machines_dir)):
        src = os.path.join(machines_dir, fname)
        if not os.path.isfile(src): continue
        lower = fname.lower()
        if lower.endswith(":zone.identifier"): continue
        if not (lower.endswith(".tsv") or lower.endswith(".obj") or lower.endswith(".stl")):
            continue
        put(src, f"/system/machine/{fname}")

# Demo meshes + toolpods TSV still live in devices/ — the kernel's
# embedded-defaults pool points at them. Ship them on SD too so the card
# is a complete replacement for the embedded ROM.
for name in ("demo_box.obj", "demo_part.stl", "embedded_toolpods.tsv"):
    put(os.path.join(root, "devices", name), f"/system/machine/{name}")

# Operator UI TSV.
put(os.path.join(root, "devices/embedded_ui.tsv"), "/system/ui/embedded_ui.tsv")

fs.close()
PY

echo "[mkimg] done. boot with: make run"
