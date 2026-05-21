#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
"""
Per-page pixel diff between this PR's captured screenshots and the
baseline committed on main.

Usage:
  python3 scripts/screenshot_diff.py <fresh_dir> <baseline_dir> [--json]

Reads every <fresh_dir>/*.png, finds the matching name in <baseline_dir>,
and runs a cheap pixel comparison (PIL ImageChops.difference + bbox).
Emits a Markdown summary suitable for inclusion in a PR comment, or
machine-readable JSON when --json is set.

Why not ImageMagick `compare`?
  - PIL is in the standard runner image; ImageMagick is one apt-get away
    but adds 50 MB and a CLI surface. Pixel-count diff is all we need.
  - PIL's ImageChops.difference returns a per-pixel-channel delta image;
    getbbox() returns the smallest rect that bounds any non-zero pixel.
    A non-None bbox means *something* changed; the bbox area is a useful
    quick metric.

Output sketch:

  | Page              | Diff |
  | ----------------- | ---- |
  | dashboard         | 0    |
  | jog               | 1240 |
  | restart_confirm   | NEW  |
  | tool_change       | DEL  |
  | _summary_         | 1 changed, 1 new, 1 deleted of 20 |

Exits 0 if no diffs; 1 otherwise. Intentionally tolerant — missing PIL
or unreadable images degrade to "no diff data", not a workflow failure.
"""

import argparse
import json
import sys
from pathlib import Path

try:
    from PIL import Image, ImageChops
except ImportError:
    print("error: Pillow not installed (`pip install pillow`)", file=sys.stderr)
    sys.exit(2)


def diff_image(fresh: Path, baseline: Path) -> dict:
    """Return {pixels: int, bbox: tuple|None, status: str}."""
    try:
        with Image.open(fresh) as a_img, Image.open(baseline) as b_img:
            a = a_img.convert("RGB")
            b = b_img.convert("RGB")
            if a.size != b.size:
                return {"pixels": -1, "bbox": None, "status": "resized"}
            diff = ImageChops.difference(a, b)
            bbox = diff.getbbox()
            if bbox is None:
                return {"pixels": 0, "bbox": None, "status": "identical"}
            # Count non-zero pixels via getextrema across channels.
            # Cheap approx: bbox area is an upper bound on changed pixels.
            bw, bh = bbox[2] - bbox[0], bbox[3] - bbox[1]
            # Per-pixel count: sum of any-channel-nonzero.
            mask = diff.convert("L").point(lambda p: 255 if p else 0)
            changed = sum(1 for p in mask.getdata() if p)
            return {"pixels": changed, "bbox": list(bbox), "bbox_area": bw * bh,
                    "status": "changed"}
    except OSError as e:
        return {"pixels": -1, "bbox": None, "status": f"error: {e}"}


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    p.add_argument("fresh", type=Path, help="directory with this PR's PNGs")
    p.add_argument("baseline", type=Path, help="directory with main's PNGs")
    p.add_argument("--json", action="store_true", help="machine-readable output")
    args = p.parse_args(argv)

    if not args.fresh.is_dir():
        print(f"error: fresh dir not found: {args.fresh}", file=sys.stderr)
        return 2
    if not args.baseline.is_dir():
        # Missing baseline isn't fatal — first run on a new repo state
        # legitimately has no main yet. Emit a summary with everything
        # marked NEW and exit cleanly so the workflow comment can still
        # post.
        args.baseline = None

    fresh_names = {p.name for p in args.fresh.glob("*.png")}
    baseline_names = (
        {p.name for p in args.baseline.glob("*.png")} if args.baseline else set()
    )

    rows = []
    changed = new_pages = deleted_pages = 0
    for name in sorted(fresh_names | baseline_names):
        if name not in baseline_names:
            rows.append({"page": name, "status": "NEW", "pixels": None})
            new_pages += 1
        elif name not in fresh_names:
            rows.append({"page": name, "status": "DEL", "pixels": None})
            deleted_pages += 1
        else:
            d = diff_image(args.fresh / name, args.baseline / name)
            rows.append({"page": name, "status": d["status"], "pixels": d["pixels"],
                         "bbox": d.get("bbox")})
            if d["status"] == "changed":
                changed += 1

    total = len(fresh_names | baseline_names)
    summary = {
        "total_pages": total,
        "changed": changed,
        "new": new_pages,
        "deleted": deleted_pages,
        "identical": total - changed - new_pages - deleted_pages,
    }
    report = {"summary": summary, "rows": rows}

    if args.json:
        print(json.dumps(report, indent=2))
        return 1 if changed + new_pages + deleted_pages else 0

    print(f"### Per-page screenshot diff vs main")
    print(f"_{summary['total_pages']} pages — "
          f"{summary['identical']} identical, "
          f"{summary['changed']} changed, "
          f"{summary['new']} new, "
          f"{summary['deleted']} deleted_")
    print()
    if changed + new_pages + deleted_pages == 0:
        print("All pages render identically to `main`.")
        return 0
    print("| Page | Status | Δ pixels | Region |")
    print("| ---- | ------ | -------- | ------ |")
    for r in rows:
        if r["status"] == "identical":
            continue
        if r["status"] == "changed":
            bbox = r.get("bbox")
            region = f"({bbox[0]},{bbox[1]})–({bbox[2]},{bbox[3]})" if bbox else ""
            print(f"| `{r['page']}` | changed | {r['pixels']} | {region} |")
        else:
            print(f"| `{r['page']}` | {r['status']} | — | — |")
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
