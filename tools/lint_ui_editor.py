#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
"""
Lint tools/ui_editor/index.html against the bind / action catalogue
extracted from ui/ui_builder_tsv.cpp.

The editor's BIND_GROUPS and ACTION_GROUPS JavaScript object literals
must stay in sync with the kernel's parse_bind() / run_action_target()
tables — otherwise the autocomplete drops bindings that the kernel still
accepts, or the editor offers bindings the kernel will silently reject.

This script:

  1. Extracts the live catalogue via tools/extract_bind_catalogue.py
  2. Parses BIND_GROUPS and ACTION_GROUPS out of index.html
  3. Reports name-set deltas in both directions

Exit code:
  0 — no drift, or only the explicit prefix-family expansions that the
      editor enumerates by index (e.g. `program:0:name`..`program:7:name`)
  1 — at least one bind / action exists in one side and not the other

Usage:
  python3 tools/lint_ui_editor.py                   # default report
  python3 tools/lint_ui_editor.py --strict          # fail on any drift
  python3 tools/lint_ui_editor.py --json            # machine-readable
"""

import argparse
import json
import re
import sys
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
EDITOR_PATH = REPO_ROOT / "tools" / "ui_editor" / "index.html"
EXTRACTOR = REPO_ROOT / "tools" / "extract_bind_catalogue.py"

# Match the JS object literal `const BIND_GROUPS = { ... };` and
# `const ACTION_GROUPS = { ... };`. We don't actually parse JS — just
# rip out every quoted string between the braces.
GROUP_BLOCK_RE = re.compile(
    r'const\s+(BIND|ACTION)_GROUPS\s*=\s*\{(.*?)\};',
    re.DOTALL,
)
QUOTED_RE = re.compile(r'"([^"]+)"')


def extract_editor_sets(html: str) -> tuple[set[str], set[str]]:
    binds: set[str] = set()
    actions: set[str] = set()
    for m in GROUP_BLOCK_RE.finditer(html):
        which = m.group(1)
        body = m.group(2)
        names = set(QUOTED_RE.findall(body))
        if which == "BIND":
            binds |= names
        else:
            actions |= names
    return binds, actions


def extract_kernel_catalogue() -> dict:
    proc = subprocess.run(
        [sys.executable, str(EXTRACTOR)],
        check=True,
        capture_output=True,
        text=True,
    )
    return json.loads(proc.stdout)


def strip_indexed(names: set[str], family_prefixes: set[str]) -> set[str]:
    """Remove names that look like `<family>N[:...]` for any prefix family.

    The editor enumerates a fixed number of program slots / tool slots /
    alarm history rows etc. as concrete bind names like
    `program:0:name`..`program:7:name`. The kernel handles the same via
    a runtime prefix dispatch (`program:` action family), so the catalogue
    only carries the family prefix. Drop those explicit enumerations from
    the editor side when comparing so we don't get spurious deltas.
    """
    out = set()
    for n in names:
        keep = True
        for prefix in family_prefixes:
            if n.startswith(prefix):
                # Trim family prefix; check if the remainder starts with a
                # digit (i.e. it's an indexed slot).
                tail = n[len(prefix):]
                if tail and tail[0].isdigit():
                    keep = False
                    break
        if keep:
            out.add(n)
    return out


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    p.add_argument("--editor", type=Path, default=EDITOR_PATH,
                   help=f"path to ui_editor/index.html (default: {EDITOR_PATH})")
    p.add_argument("--strict", action="store_true",
                   help="exit nonzero on any drift, including indexed-slot expansions")
    p.add_argument("--json", action="store_true",
                   help="emit a machine-readable JSON report")
    args = p.parse_args(argv)

    if not args.editor.exists():
        print(f"error: editor not found: {args.editor}", file=sys.stderr)
        return 2
    editor_html = args.editor.read_text(encoding="utf-8")
    editor_binds, editor_actions = extract_editor_sets(editor_html)
    catalogue = extract_kernel_catalogue()

    kernel_binds = {b["name"] for b in catalogue["binds"]}
    kernel_actions_exact = set(catalogue["actions"]["exact"])
    kernel_action_families = set(catalogue["actions"]["prefix_families"])

    # Strip the kernel's indexed enumerations from the editor side so the
    # `program:0:name`..`program:7:name` rows don't all show up as drift.
    bind_families = {p for p in kernel_action_families} | {
        # Bind families that the kernel resolves at runtime via prefix.
        "program:", "tool:T", "alarm:", "wcs:G", "axis:", "cal:pec:",
    }
    editor_binds_collapsed = strip_indexed(editor_binds, bind_families) if not args.strict else editor_binds
    editor_actions_collapsed = strip_indexed(editor_actions, kernel_action_families) if not args.strict else editor_actions

    def covered_by_family(name: str, families: set[str]) -> bool:
        return any(name.startswith(p) for p in families)

    binds_only_in_editor = {
        b for b in editor_binds_collapsed
        if b not in kernel_binds and not covered_by_family(b, bind_families)
    }
    binds_only_in_kernel = kernel_binds - editor_binds
    actions_only_in_editor = {
        a for a in editor_actions_collapsed
        if a not in kernel_actions_exact and not covered_by_family(a, kernel_action_families)
    }
    actions_only_in_kernel = kernel_actions_exact - editor_actions

    report = {
        "editor": str(args.editor.relative_to(REPO_ROOT)),
        "kernel_source": catalogue["source"],
        "summary": {
            "editor_binds": len(editor_binds),
            "editor_actions": len(editor_actions),
            "kernel_binds": len(kernel_binds),
            "kernel_actions_exact": len(kernel_actions_exact),
            "kernel_action_families": len(kernel_action_families),
            "drift_binds_editor_only": len(binds_only_in_editor),
            "drift_binds_kernel_only": len(binds_only_in_kernel),
            "drift_actions_editor_only": len(actions_only_in_editor),
            "drift_actions_kernel_only": len(actions_only_in_kernel),
        },
        "binds_editor_only": sorted(binds_only_in_editor),
        "binds_kernel_only": sorted(binds_only_in_kernel),
        "actions_editor_only": sorted(actions_only_in_editor),
        "actions_kernel_only": sorted(actions_only_in_kernel),
    }

    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print(f"UI editor catalogue lint vs {catalogue['source']}")
        s = report["summary"]
        print(f"  editor: {s['editor_binds']} binds, {s['editor_actions']} actions")
        print(f"  kernel: {s['kernel_binds']} binds, {s['kernel_actions_exact']} exact actions, "
              f"{s['kernel_action_families']} action families")
        print()
        if binds_only_in_editor:
            print(f"BINDS in editor but not in kernel ({len(binds_only_in_editor)}):")
            for n in sorted(binds_only_in_editor):
                print(f"  - {n}")
        if binds_only_in_kernel:
            print(f"BINDS in kernel but not in editor ({len(binds_only_in_kernel)}):")
            for n in sorted(binds_only_in_kernel):
                print(f"  + {n}")
        if actions_only_in_editor:
            print(f"ACTIONS in editor but not in kernel ({len(actions_only_in_editor)}):")
            for n in sorted(actions_only_in_editor):
                print(f"  - {n}")
        if actions_only_in_kernel:
            print(f"ACTIONS in kernel but not in editor ({len(actions_only_in_kernel)}):")
            for n in sorted(actions_only_in_kernel):
                print(f"  + {n}")
        if (not binds_only_in_editor and not binds_only_in_kernel
                and not actions_only_in_editor and not actions_only_in_kernel):
            print("OK — editor catalogue is in sync.")

    drift = (binds_only_in_editor or binds_only_in_kernel
             or actions_only_in_editor or actions_only_in_kernel)
    return 1 if drift else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
