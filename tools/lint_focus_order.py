#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
"""
Lint focus-order on the operator UI: walk devices/embedded_ui.tsv,
collect every `focus=N` value per page, report duplicates and gaps.

Hand-numbered focus= values drift as pages evolve. PR #20 noted gaps
around 700-740 on the network page; reviewers can't see drift
without scanning every record. This script gives them one go/no-go
signal:

  $ python3 tools/lint_focus_order.py
  Focus order vs devices/embedded_ui.tsv
    21 pages — 190 focusable widgets
    page dashboard:    10 focus values, no duplicates, gaps OK
    page jog:          14 focus values, no duplicates, gaps OK
    page network:      9 focus values, 1 duplicate (706 on net_pgw_in, net_pgw_v)
    ...

Exits non-zero if any page has a duplicate. Gap reporting is
informational (gaps of >100 between consecutive values get flagged
as suspicious — usually means a chunk of widgets lost their order).

Usage:
  python3 tools/lint_focus_order.py
  python3 tools/lint_focus_order.py --json
  python3 tools/lint_focus_order.py --strict   (also fail on big gaps)
"""

import argparse
import json
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_TSV = REPO_ROOT / "devices" / "embedded_ui.tsv"

PAGE_RE = re.compile(r'^(page|dialog)\s+id=(\S+)')
FOCUS_RE = re.compile(r'\bfocus=(-?\d+)\b')
ID_RE = re.compile(r'\bid=(\S+)')


def parse_tsv(path: Path) -> dict:
    pages = []        # list of {id, kind, focus: [(widget_id, value), ...]}
    current = None
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.rstrip("\r")
        if not line or line.startswith("#"):
            continue
        m = PAGE_RE.match(line)
        if m:
            current = {"id": m.group(2), "kind": m.group(1), "focus": []}
            pages.append(current)
            continue
        if current is None:
            continue
        fm = FOCUS_RE.search(line)
        if not fm:
            continue
        im = ID_RE.search(line)
        wid = im.group(1) if im else "?"
        try:
            current["focus"].append((wid, int(fm.group(1))))
        except ValueError:
            pass
    return pages


def analyse(pages: list, strict_gap_threshold: int) -> dict:
    summary = {
        "total_pages": len(pages),
        "total_focusables": sum(len(p["focus"]) for p in pages),
        "pages_with_dupes": 0,
        "pages_with_big_gaps": 0,
    }
    per_page = []
    for p in pages:
        seen = {}
        dupes = []
        for wid, val in p["focus"]:
            if val in seen:
                dupes.append((val, seen[val], wid))
            else:
                seen[val] = wid
        # Gaps: walk the sorted focus values, flag any consecutive jump
        # bigger than `strict_gap_threshold` between non-trivial values.
        sorted_vals = sorted(seen.keys())
        big_gaps = []
        for a, b in zip(sorted_vals, sorted_vals[1:]):
            if b - a > strict_gap_threshold and a > 0:
                big_gaps.append((a, b))
        page_entry = {
            "id": p["id"],
            "kind": p["kind"],
            "count": len(p["focus"]),
            "duplicates": [{"value": v, "widgets": [a, b]} for v, a, b in dupes],
            "big_gaps": [{"from": a, "to": b} for a, b in big_gaps],
        }
        per_page.append(page_entry)
        if dupes:
            summary["pages_with_dupes"] += 1
        if big_gaps:
            summary["pages_with_big_gaps"] += 1
    return {"summary": summary, "pages": per_page}


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    p.add_argument("--tsv", type=Path, default=DEFAULT_TSV,
                   help=f"path to embedded_ui.tsv (default: {DEFAULT_TSV})")
    p.add_argument("--strict", action="store_true",
                   help="exit non-zero on big gaps too, not just duplicates")
    p.add_argument("--gap-threshold", type=int, default=200,
                   help="report consecutive gap > this as suspicious (default: 200)")
    p.add_argument("--json", action="store_true",
                   help="machine-readable JSON")
    args = p.parse_args(argv)

    if not args.tsv.exists():
        print(f"error: TSV not found: {args.tsv}", file=sys.stderr)
        return 2

    pages = parse_tsv(args.tsv)
    report = analyse(pages, args.gap_threshold)

    if args.json:
        print(json.dumps(report, indent=2))
    else:
        s = report["summary"]
        print(f"Focus order vs {args.tsv.relative_to(REPO_ROOT)}")
        print(f"  {s['total_pages']} pages — {s['total_focusables']} focusable widgets")
        for page in report["pages"]:
            if page["count"] == 0:
                continue
            line = f"  {page['kind']} {page['id']:24s}  {page['count']:3d} focus values"
            if page["duplicates"]:
                line += f", {len(page['duplicates'])} duplicate(s)"
            if page["big_gaps"]:
                line += f", {len(page['big_gaps'])} suspicious gap(s)"
            if not page["duplicates"] and not page["big_gaps"]:
                line += "  OK"
            print(line)
            for d in page["duplicates"]:
                print(f"      duplicate focus={d['value']} on {d['widgets'][0]} and {d['widgets'][1]}")
            for g in page["big_gaps"]:
                print(f"      gap focus {g['from']}→{g['to']} (Δ {g['to']-g['from']})")

    drift = report["summary"]["pages_with_dupes"] > 0
    if args.strict:
        drift = drift or report["summary"]["pages_with_big_gaps"] > 0
    return 1 if drift else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
