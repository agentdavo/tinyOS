#!/usr/bin/env bash
# SPDX-License-Identifier: MIT OR Apache-2.0
# Build sdcard.img (64 MiB FAT32) from the flat machines/ authoring directory.
#
# Thin wrapper over scripts/mkimg.py, which writes the FAT32 image itself
# (no dosfstools / pyfatfs) and is byte-for-byte reproducible — CI checks the
# committed sdcard.img against it. Layout:
#
#   /system/machine/<every machines/*.tsv|*.obj|*.stl>
#   /system/machine/demo_box.obj, demo_part.stl, embedded_toolpods.tsv  (devices/)
#   /system/ui/embedded_ui.tsv                                          (devices/)
#
# The OBJ registry rewrites an axis's `obj_file=foo.stl` reference to
# `system/machine/foo.stl`, so the TSV's obj_file column is just the basename.
# Files on the card SHADOW the kernel's embedded copies: regenerate after
# touching machines/ or devices/embedded_ui.tsv.

set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
exec python3 "$ROOT/scripts/mkimg.py" "${1:-$ROOT/sdcard.img}"
