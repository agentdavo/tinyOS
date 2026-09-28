#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
"""
Render the operator UI gallery from a directory of per-page screenshots:
HTML for the Pages site (output ending .html) or Markdown otherwise.

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

Each shot below is the guest framebuffer at native 1080×1920 (1:1), rendered
by the kernel and captured via the CLI `ui_page <id>` + `ui_dump 1` commands.

Operator notes:
- **Keyboard:** Tab / Down move focus through the controls of the visible
  page (Up goes back); Enter or Space activates the focused control; Esc
  drops focus and returns the arrow / WASD keys to the on-screen pointer.
  Hold-to-confirm buttons (E-STOP, CONFIRM RESTART) must be held for their
  hold time on the keyboard too.
- **Fields:** typing replaces the shown value; Enter commits. An amber border
  means an uncommitted edit is parked — tap the field to resume it.
- **Alarms:** alarm indicators are green when clear and red when an alarm
  (drive fault or EtherCAT deadline fault) is active, on every page.
- **Live preview:** the editor's *Push to kernel* (WebSocket 5001) is accepted
  from a local file, localhost, or the origin set by `hmi key=ws_origin`, and
  refused while a cycle or homing runs (`hmi key=ui_upload_enable`).
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
        timestamp=datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC"),
        tsv_path="devices/embedded_ui.tsv",
        page_count=len(have_pages),
    ))

    output_path.write_text("\n".join(out), encoding="utf-8")


HTML_TEMPLATE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>miniOS Operator UI</title>
<style>
  :root {{ --bg:#0A0F1A; --panel:#111827; --line:#334155; --ink:#F8FAFC;
          --muted:#94A3B8; --accent:#7DD3FC; }}
  * {{ box-sizing: border-box; }}
  body {{ margin:0; background:var(--bg); color:var(--ink);
         font:15px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif; }}
  header, main, footer {{ max-width:1200px; margin:0 auto; padding:0 16px; }}
  header {{ padding-top:32px; }}
  h1 {{ margin:0 0 4px; font-size:26px; }}
  a {{ color:var(--accent); }}
  .sub {{ color:var(--muted); margin:0 0 16px; }}
  details {{ background:var(--panel); border:1px solid var(--line); border-radius:8px;
            padding:10px 14px; margin:0 0 24px; }}
  summary {{ cursor:pointer; font-weight:600; }}
  details li {{ color:var(--muted); margin:4px 0; }}
  .grid {{ display:grid; gap:18px; grid-template-columns:repeat(auto-fill,minmax(200px,1fr)); }}
  figure {{ margin:0; background:var(--panel); border:1px solid var(--line);
           border-radius:8px; overflow:hidden; }}
  figure img {{ display:block; width:100%; height:auto; aspect-ratio:9/16;
               background:#000; }}
  figcaption {{ padding:8px 10px; }}
  figcaption b {{ display:block; }}
  figcaption code {{ color:var(--muted); font-size:12px; }}
  footer {{ color:var(--muted); font-size:13px; padding:32px 16px; }}
</style>
</head>
<body>
<header>
  <h1>miniOS Operator UI</h1>
  <p class="sub">Every page defined in <code>devices/embedded_ui.tsv</code>, rendered by the
  arm64 kernel under QEMU and captured 1:1 (1080&times;1920) with <code>ui_page</code> +
  <code>ui_dump 1 rle</code>. Click a page for the full-size image.
  &middot; <a href="../editor/">UI editor</a>
  &middot; <a href="https://github.com/agentdavo/tinyOS">source</a></p>
  <details>
    <summary>Operator notes</summary>
    <ul>
      <li><b>Keyboard:</b> Tab / Down move focus through the controls of the visible page (Up goes
        back); Enter or Space activates the focused control; Esc drops focus and returns the arrow /
        WASD keys to the on-screen pointer. Hold-to-confirm buttons (E-STOP, CONFIRM RESTART) must
        be held for their hold time on the keyboard too.</li>
      <li><b>Fields:</b> typing replaces the shown value; Enter commits. An amber border means an
        uncommitted edit is parked &mdash; tap the field to resume it.</li>
      <li><b>Alarms:</b> alarm indicators are green when clear and red when an alarm (drive fault or
        EtherCAT deadline fault) is active, on every page.</li>
      <li><b>Live preview:</b> the editor's <i>Push to kernel</i> (WebSocket 5001) is accepted from a
        local file, localhost, or the origin set by <code>hmi key=ws_origin</code>, and refused
        while a cycle or homing runs (<code>hmi key=ui_upload_enable</code>).</li>
    </ul>
  </details>
</header>
<main>
<div class="grid">
{figures}
</div>
{missing}
</main>
<footer>Generated {timestamp} from <code>devices/embedded_ui.tsv</code> ({page_count} pages)
by <code>.github/workflows/pages.yml</code>.</footer>
</body>
</html>
"""


def render_html(pages: list[tuple[str, str]], screenshots_dir: Path, output_path: Path,
                image_prefix: str) -> None:
    import html
    prefix = (image_prefix or "screenshots").rstrip("/")
    have = [(pid, title) for pid, title in pages if (screenshots_dir / f"{pid}.png").is_file()]
    missing = [pid for pid, _ in pages if not (screenshots_dir / f"{pid}.png").is_file()]
    figs = []
    for pid, title in have:
        src = f"{prefix}/{pid}.png"
        figs.append(
            f'<figure id="{slugify_anchor(pid)}"><a href="{html.escape(src)}">'
            f'<img src="{html.escape(src)}" alt="{html.escape(title)}" loading="lazy" '
            f'width="1080" height="1920"></a>'
            f'<figcaption><b>{html.escape(title)}</b><code>ui_page {html.escape(pid)}</code>'
            f'</figcaption></figure>')
    miss = ""
    if missing:
        miss = ('<p class="sub">Missing screenshots: '
                + ", ".join(f"<code>{html.escape(m)}</code>" for m in missing) + "</p>")
    output_path.write_text(HTML_TEMPLATE.format(
        figures="\n".join(figs), missing=miss,
        timestamp=datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d %H:%M UTC"),
        page_count=len(have)), encoding="utf-8")


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

    if args.output.suffix.lower() == ".html":
        render_html(pages, args.screenshots_dir, args.output, args.image_prefix)
    else:
        render(pages, args.screenshots_dir, args.output, args.image_prefix)
    print(f"wrote {args.output} ({len(pages)} pages)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
