# miniOS Operator UI

This file is regenerated automatically by the **UI screenshots** GitHub Actions
workflow (`.github/workflows/ui-screenshots.yml`).

When the workflow runs it:

1. Cross-builds `build/arm64/miniOS_kernel_arm64.elf`.
2. Boots the kernel under `qemu-system-aarch64 -nographic`.
3. Drives the CLI through every page registered in
   [`devices/embedded_ui.tsv`](./devices/embedded_ui.tsv) using `ui_page <id>` +
   `ui_dump 6`.
4. Converts each downscaled PPM to PNG with `ffmpeg`.
5. Regenerates this file from the captures via
   [`scripts/generate_ui_md.py`](./scripts/generate_ui_md.py).
6. On `push` to `main` or a manual dispatch, commits the refreshed
   `screenshots/` directory and `UI.md` back to the branch. On pull requests
   the artifacts are uploaded to the workflow run instead of being committed.

## Refreshing locally

```bash
make TARGET=arm64
bash scripts/qemu_dump_ui_pages.sh "$PWD/screenshots"
python3 scripts/generate_ui_md.py screenshots UI.md
```

The script needs `qemu-system-aarch64`, `ffmpeg`, and Python 3.

## Pages

The TSV registers these operator surfaces (in flow order):

| Page id | Title |
| --- | --- |
| `dashboard` | Dashboard |
| `jog` | Jog |
| `mdi` | MDI |
| `machine_view` | Machine View |
| `program` | Program |
| `offsets` | Offsets |
| `service` | Service |
| `ethercat` | EtherCAT |
| `homing` | Homing |
| `alarms` | Alarms |
| `macros` | Macros |
| `probe` | Probe |
| `pec` | Pitch Error |
| `geometry` | Geometry Comp |
| `sphere` | Volumetric Comp |
| `network` | Network |
| `axis_status` | Axis Detail |
| `tool_change` | Tool Change |

The shared `bottom_nav` template included on every real page is intentionally
excluded — it is layout, not a navigable page.

> **No screenshots committed yet.** Run the workflow once
> (`Actions → UI screenshots → Run workflow`) to populate `screenshots/` and
> overwrite this stub with embedded images for every page above.
