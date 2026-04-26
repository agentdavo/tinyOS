# miniOS UI Editor

Browser-based drag-and-drop editor for `devices/embedded_ui.tsv`, the TSV
format parsed by `ui/ui_builder_tsv.cpp` at kernel boot.

## Opening

```bash
bash tools/ui_editor/serve.sh          # http://localhost:8766/
# or: UI_EDITOR_PORT=9001 bash tools/ui_editor/serve.sh
```

Run over HTTP. Opening via `file://` works for the single-file editor in
most browsers, but Chrome treats every `file://` URL as a unique origin —
if you see "Unsafe attempt to load URL ... from frame" in the console, that
is the cause. WSL paths (`file://wsl.localhost/...`) are especially strict.
`serve.sh` spins up `python3 -m http.server` on 127.0.0.1 and sidesteps
the whole category.

No build step, no CDN fetches at runtime. Everything ships under
`tools/ui_editor/`.

## Supported browsers

- **Chrome / Edge / Chromium 86+** — full File System Access API (open +
  save in place).
- **Firefox / Safari** — open via `<input type=file>` fallback; Save As
  downloads a new file.

## Features

- 1080 x 1920 portrait canvas, 10 px snap grid, 25 %–200 % zoom.
- Drag to move, 8-handle resize, arrow-key nudge (Shift = 10 px).
- Inspector with grouped `bind=` and `action=` dropdowns; unknown values
  go red.
- Include-other-page chips; included pages render dimmed behind the
  current page.
- Actions table for per-page `action widget= event= target=` records.
- Undo / redo (`Ctrl+Z` / `Ctrl+Y`), auto-save to `localStorage`.

## Bind catalogue

Grouped exactly as in the inspector:

- **Machine** mode, alarm, prompt, units, wcs, cycle_progress, torque,
  feed, spindle, spindle_rpm, spindle_load, block_current, block_next,
  runtime, parts, page, page_name, selected_axis, jog_increment,
  operator_mode (0=AUTO, 1=MDI, 2=JOG, 3=SETUP)
- **DRO** axis:{x,y,z,a}, axis:{x,y,z,a}:{cmd,act,dtg,homed,near_limit}
  (`near_limit` is 1 when the axis sits within 10% of either soft limit;
  pair with `active_if=axis:x:near_limit:1` on a label to colour the
  digit fault-red)
- **Program** program_name, program_count, program_blocks, program_bytes,
  preview_points, plus per-row file browser tokens
  program:0..7:{name,size,selected,loaded} (size renders as `B` or `kB`,
  loaded paints `*` when bound to channel 0)
- **Homing** homing:{axis,method,state,message,fast,slow} (state cycles
  IDLE / READY / SEARCH / APPROACH / DONE / FAULT; method renders the
  human-readable strategy name set by the wizard buttons)
- **Offsets** work_name, work_offset:{x,y,z,a}, plus per-slot
  wcs:G54..G59:{x,y,z,a} (3-decimal mm or deg)
- **Tool** tool_name, tool_length, tool_radius, tool_wear, active_tool,
  plus per-slot tool:T1..T8:{length,radius,wear}
- **Macro** macro_name, macro_active, macro_status, macro_step,
  macro_count, macro_message
- **Probe** probe_{x,y,z,done,stylus,center_x,center_y,size_x,size_y,
  shift_x,shift_y,shift_z,sphere_ready,sphere_points}
- **MDI** mdi:{input,last,status,message,depth}
- **EtherCAT** ec:{state,fault,slaves,miss,trips,p99,max,period,cycles,tx,rx}
- **View** view_toolpath, view_toolpods (machine_view page overlay flags)

## Action catalogue

- `goto:<page_id>` (auto-completed from current document)
- `page:{position,program,offsets,ethercat,alarms,setup}`
- `demo:{cycle,hold,reset,home,jog+,jog-,estop}`
- `program:{prev,next,simulate}`
- `program:select:<n>` (n=0..7) — load program from row N of the file
  browser
- `macro:{prev,next,run,abort}` and `macro:run:<name>`
- `homing:axis:<n>` (n=0..3) — pick axis for the wizard
- `homing:method:<id>:<name>` — pick CiA-402 method (Touch+ = 1,
  Touch- = 2, Index Mark = 33, Hardstop+ = -2, Hardstop- = -1, Set Here
  = 35); the optional `name` is the label rendered in the wizard
- `homing:start` / `homing:abort` — kick / stop the configured cycle.
  `homing:start` is a no-op while either EtherCAT master holds a
  deadline-fault latch.
- `setup:{save,load}`
- `offset:work:<n>` (0..5), `offset:tool:<n>` (0..7), `offset:nudge:<axis><delta>`
- `mdi:{submit,clear,abort}`
- `jog:axis:<n>` (0..3 = X/Y/Z/A), `jog:inc:<counts>` (1/10/100/1000 = 0.001..1 mm)
- `jog:hold:<axis>:<sign>` or `jog:hold:sel:<sign>` — press-to-toggle
  continuous jog (first tap starts axis motion at jog_feed_cps in the
  given direction, second tap stops). The `sel` form resolves to
  selected_axis at click time.
- `jog:stop:<axis>` — explicit stop (drives axis velocity to 0)
- `spindle:{start,stop,rev}` — spindle control. `rev` negates the
  current commanded RPM and re-issues start.
- `mode:{auto,mdi,jog,setup}` — set the operator's declared mode
  (display-only today; no behaviour gating yet)
- Input commit targets: `commit:offset:{x,y,z,a}`,
  `commit:tool:{length,radius,wear}`, `commit:restart:line`,
  `commit:spindle:rpm`
- `ec:{estop,clear_fault}` — broadcast QuickStop / clear deadline-fault latch
- `view:{toolpath,toolpods}:toggle` — flip overlay flags on machine_view
- `view:{reset,zoom:in,zoom:out}` — camera primitives on machine_view
- `alarm:ack:N` (N=0..3) — acknowledge active alarm in row N
- `alarm:clear_history` — wipe the alarm history queue

## Attributes

- `scale=<1..4>` — integer pixel scale for label/button text (kernel uses
  `Framebuffer::draw_text_scaled`). Dashboard DRO command digits use
  scale=3; primary action buttons use scale=2.
- `active_if=<bind>:<int>` — on button widgets only. When the binder
  returns the given integer for the named bind, the button's background
  is brightened and a white outline is drawn. Used on the Offsets page
  (G54..G59, T1..T8) and Jog page (axis + increment selectors) so the
  operator can see which slot is currently active.
  Example: `active_if=wcs:0` highlights when G54 is active;
  `active_if=selected_axis:2` highlights when Z is the jog axis.
  The inspector exposes this as a bind-picker + integer value pair.

## URL params

The editor accepts a few query-string parameters used by the screenshot
pipeline (`tools/ui_editor/screenshot.sh`) and useful for deep-links:

- `?tsv=<server-relative-path>` — fetch + load that TSV on boot
  (e.g. `?tsv=devices/embedded_ui.tsv`). Path is resolved against the
  HTTP server root, not the editor URL — use `screenshot.sh` which
  serves from the repo root, or prefix with `/`.
- `?page=<page_id>` — switch the active tab to that page after load.
- `?canvas_only=1` — hide the editor chrome (toolbar / palette /
  inspector / tabs) so the 1080×1920 canvas fills the viewport. Also
  disables include-dimming so the bottom_nav renders at full opacity,
  matching what the kernel framebuffer paints.
- `?ready=1` — appends `<div id="__editor_ready">` once the canvas
  finishes painting; headless screenshotters poll for it before
  capturing.

## Active-tab highlight

The editor (and the kernel) auto-flag any button whose `goto:<page_id>`
target points to the currently active page. They render brightened with
a 3 px white outline — the operator sees "you are here" at a glance on
the bottom_nav. No TSV change needed; it's automatic.

## Known limitations

- No rendering of `image src=gles1:...` previews — kernel-side only.
- Slider thumb / input caret are approximations (no interaction).
- Child records (`child parent= widget=`) are preserved on round-trip
  but not exposed in the UI; most pages use the `parent=` field on the
  widget record itself.
- `font=small|medium|large|xlarge` is stored as text; canvas uses
  `scale` for glyph size, matching the kernel's 8 x 16 font semantics.

## Round-trip

Loading `devices/embedded_ui.tsv` and saving back produces TSV
functionally identical to the source: canonical field order, tab
separators, comments / blank lines preserved in position. Normalisations
(CRLF -> LF, collapsed multiple blank lines) are documented at the top
of `index.html`.

The kernel parser accepts `title=` and `src=` as aliases for `text=`
(`ui/ui_builder_tsv.cpp` `parse_widget`). The editor normalises these
into a single editable `text` field and tags the record with an
internal `__textKey` so the original keyword is re-emitted on save —
panels written with `title=STATUS` round-trip as `title=STATUS`, not
as `text=STATUS`.

`child parent= widget=` records are preserved verbatim on round-trip
(parser + serialiser handle them as opaque records). The inspector
doesn't edit them, since `embedded_ui.tsv` uses `parent=` on widgets
directly — if you need a `child`-based layout, hand-edit the TSV.

`offset:work:0..5` covers G54–G59, matching `OffsetsSnapshot::work` in
`ui/operator_api.hpp` (6 slots).
