#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
"""
Render UI.md from a directory of per-page screenshots.

Inputs:
  - <screenshots_dir>/pages.tsv   (page_id<TAB>title, in display order)
  - <screenshots_dir>/<page>.png  (one per row in pages.tsv)

The pages.tsv file is written by scripts/qemu_dump_ui_pages.sh. If it is
missing the generator falls back to globbing *.png in the screenshots dir
and using the page id as the title.
"""
from __future__ import annotations

import argparse
import datetime
import os
import sys
from pathlib import Path


HEADER = """\
# miniOS Operator UI

Auto-generated catalogue of every TSV-defined operator page in `devices/embedded_ui.tsv`.
Refresh by running the **UI screenshots** GitHub Actions workflow (or
`bash scripts/qemu_dump_ui_pages.sh screenshots && python3 scripts/generate_ui_md.py screenshots UI.md`
locally).

Each shot below is the guest framebuffer rendered by the `arm64` kernel,
captured via the CLI `ui_page <id>` + `ui_dump <scale>` commands and
downscaled by the factor noted at capture time.
"""

FOOTER_TEMPLATE = """\

---

*Generated {timestamp} from `{tsv_path}` ({page_count} pages).*
"""


def load_pages(screenshots_dir: Path) -> list[tuple[str, str]]:
    tsv = screenshots_dir / "pages.tsv"
    pages: list[tuple[str, str]] = []
    if tsv.is_file():
        for raw in tsv.read_text(encoding="utf-8").splitlines():
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split("\t", 1)
            if len(parts) == 2:
                pages.append((parts[0].strip(), parts[1].strip()))
            elif parts:
                pages.append((parts[0].strip(), parts[0].strip()))
        return pages
    # Fallback: glob PNGs.
    for png in sorted(screenshots_dir.glob("*.png")):
        pages.append((png.stem, png.stem))
    return pages


def slugify_anchor(page_id: str) -> str:
    return page_id.lower().replace("_", "-")


def render(pages: list[tuple[str, str]], screenshots_dir: Path, output_path: Path,
           image_prefix: str) -> None:
    repo_root = output_path.parent.resolve()
    rel_screens = os.path.relpath(screenshots_dir.resolve(), repo_root)
    rel_screens = rel_screens.replace(os.sep, "/")
    if image_prefix:
        rel_screens = image_prefix.rstrip("/")

    have_pages = [(pid, title) for pid, title in pages
                  if (screenshots_dir / f"{pid}.png").is_file()]
    missing = [pid for pid, _ in pages
               if not (screenshots_dir / f"{pid}.png").is_file()]

    out: list[str] = [HEADER, ""]

    if have_pages:
        out.append("## Pages")
        out.append("")
        for pid, title in have_pages:
            out.append(f"- [{title}](#{slugify_anchor(pid)}) — `{pid}`")
        out.append("")

    if missing:
        out.append("> Missing screenshots (page registered in TSV but no PNG produced): "
                   + ", ".join(f"`{p}`" for p in missing))
        out.append("")

    for pid, title in have_pages:
        anchor = slugify_anchor(pid)
        out.append(f'<a id="{anchor}"></a>')
        out.append(f"### {title}")
        out.append("")
        out.append(f"`ui_page {pid}` — defined in `devices/embedded_ui.tsv`.")
        out.append("")
        out.append(f"![{title}]({rel_screens}/{pid}.png)")
        out.append("")

    out.append(FOOTER_TEMPLATE.format(
        timestamp=datetime.datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S UTC"),
        tsv_path="devices/embedded_ui.tsv",
        page_count=len(have_pages),
    ))

    output_path.write_text("\n".join(out), encoding="utf-8")


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("screenshots_dir", type=Path,
                   help="directory containing <page>.png and pages.tsv")
    p.add_argument("output", type=Path,
                   help="path to write UI.md")
    p.add_argument("--image-prefix", default="",
                   help="override the relative path used in image links "
                        "(default: relative path from UI.md to screenshots_dir)")
    args = p.parse_args(argv)

    if not args.screenshots_dir.is_dir():
        print(f"error: {args.screenshots_dir} is not a directory", file=sys.stderr)
        return 1

    pages = load_pages(args.screenshots_dir)
    if not pages:
        print(f"error: no pages discovered in {args.screenshots_dir}", file=sys.stderr)
        return 1

    render(pages, args.screenshots_dir, args.output, args.image_prefix)
    print(f"wrote {args.output} ({len(pages)} pages)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
