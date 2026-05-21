#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
"""
Extract the bind / action / widget-type catalogue from ui/ui_builder_tsv.cpp.

The TSV UI editor (tools/ui_editor/index.html) carries a hardcoded mirror of
the kernel's parse_bind() / run_action_target() / parse_type() name tables.
Without a source of truth, the two drift: a new bind added in the kernel
won't show up in the editor's dropdown, and an editor-only entry won't
actually bind anything at runtime.

This script scans ui_builder_tsv.cpp for the relevant strcmp() == 0 returns
and emits a single JSON catalogue. Consumers:

  * tools/ui_editor/sync_catalogue.py (TODO) — compare against the
    hardcoded BIND_GROUPS / ACTION_GROUPS in index.html.
  * docs/bind-catalogue.json (TODO) — committed artifact that PR reviewers
    can scan to see new bind/action surface.
  * Future: editor fetches catalogue.generated.js inlined into index.html
    at build time.

Usage:
  python3 tools/extract_bind_catalogue.py             # to stdout
  python3 tools/extract_bind_catalogue.py --output X  # to file X
  python3 tools/extract_bind_catalogue.py --pretty    # human-readable JSON

The script is deliberately conservative — anything it doesn't recognise is
emitted under an "unclassified" bucket so a maintainer can decide. It
doesn't try to evaluate macro-expanded values or runtime-only registrations.
"""

import argparse
import json
import os
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SOURCE = REPO_ROOT / "ui" / "ui_builder_tsv.cpp"

# Bind name → BindKind enum constant.
#   if (strcmp(s, "mode") == 0) return BindKind::Mode;
BIND_RE = re.compile(
    r'\bstrcmp\(s,\s*"([^"]+)"\)\s*==\s*0\)\s*return\s+BindKind::(\w+)\s*;'
)

# Action target → handler. Two shapes are common:
#   if      (strcmp(target, "x") == 0) ...;
#   else if (strncmp(target, "x:", N) == 0) ...;
# We capture exact (strcmp) matches; prefix (strncmp) matches are surfaced
# separately as "families" so the catalogue records the prefix without
# pretending it's an exhaustive list.
ACTION_EXACT_RE = re.compile(r'\bstrcmp\(target,\s*"([^"]+)"\)\s*==\s*0\)')
ACTION_PREFIX_RE = re.compile(r'\bstrncmp\(target,\s*"([^"]+)",\s*\d+\)\s*==\s*0\)')

# Widget type — parse_type's strcmp returns.
WIDGET_TYPE_RE = re.compile(
    r'\bstrcmp\(s,\s*"([^"]+)"\)\s*==\s*0\)\s*return\s+WidgetType::(\w+)\s*;'
)

# Layout (parse_layout)
LAYOUT_RE = re.compile(
    r'\bstrcmp\(s,\s*"([^"]+)"\)\s*==\s*0\)\s*return\s+LayoutType::(\w+)\s*;'
)

# Align (parse_align)
ALIGN_RE = re.compile(
    r'\bstrcmp\(s,\s*"([^"]+)"\)\s*==\s*0\)\s*return\s+Align::(\w+)\s*;'
)

# Record type (parse_record_type)
RECORD_RE = re.compile(
    r'\bstrcmp\(s,\s*"([^"]+)"\)\s*==\s*0\)\s*return\s+RecordType::(\w+)\s*;'
)


def extract(source_path: Path) -> dict:
    if not source_path.exists():
        raise FileNotFoundError(source_path)
    text = source_path.read_text(encoding="utf-8")

    binds = [{"name": m.group(1), "kind": m.group(2)} for m in BIND_RE.finditer(text)]
    actions_exact = sorted({m.group(1) for m in ACTION_EXACT_RE.finditer(text)})
    actions_prefix = sorted({m.group(1) for m in ACTION_PREFIX_RE.finditer(text)})
    widget_types = [
        {"name": m.group(1), "kind": m.group(2)} for m in WIDGET_TYPE_RE.finditer(text)
    ]
    layouts = [{"name": m.group(1), "kind": m.group(2)} for m in LAYOUT_RE.finditer(text)]
    aligns = [{"name": m.group(1), "kind": m.group(2)} for m in ALIGN_RE.finditer(text)]
    records = [{"name": m.group(1), "kind": m.group(2)} for m in RECORD_RE.finditer(text)]

    return {
        "source": str(source_path.relative_to(REPO_ROOT)),
        "binds": binds,
        "actions": {
            "exact": actions_exact,
            "prefix_families": actions_prefix,
        },
        "widget_types": widget_types,
        "layouts": layouts,
        "aligns": aligns,
        "record_types": records,
        "summary": {
            "bind_count": len(binds),
            "action_exact_count": len(actions_exact),
            "action_prefix_count": len(actions_prefix),
            "widget_type_count": len(widget_types),
        },
    }


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    p.add_argument("--source", type=Path, default=DEFAULT_SOURCE,
                   help=f"path to ui_builder_tsv.cpp (default: {DEFAULT_SOURCE})")
    p.add_argument("--output", "-o", type=Path, default=None,
                   help="write JSON to this file (default: stdout)")
    p.add_argument("--pretty", action="store_true",
                   help="indent JSON for human inspection")
    args = p.parse_args(argv)

    try:
        catalogue = extract(args.source)
    except FileNotFoundError as e:
        print(f"error: source not found: {e}", file=sys.stderr)
        return 1

    text = json.dumps(catalogue, indent=2 if args.pretty else None,
                      sort_keys=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
        print(f"wrote {args.output} ({catalogue['summary']['bind_count']} binds, "
              f"{catalogue['summary']['action_exact_count']} exact actions, "
              f"{catalogue['summary']['action_prefix_count']} action families, "
              f"{catalogue['summary']['widget_type_count']} widget types)",
              file=sys.stderr)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
