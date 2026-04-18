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

## Bind catalogue (78 keys)

Grouped exactly as in the inspector:

- **Machine** mode, alarm, prompt, units, wcs, cycle_progress, torque,
  feed, spindle, spindle_rpm, spindle_load, block_current, block_next,
  runtime, parts, page, page_name, selected_axis, jog_increment
- **DRO** axis:{x,y,z,a}, axis:{x,y,z,a}:{cmd,act,dtg,homed}
- **Program** program_name, program_count, program_blocks, program_bytes,
  preview_points
- **Offsets** work_name, work_offset:{x,y,z,a}
- **Tool** tool_name, tool_length, tool_radius, tool_wear, active_tool
- **Macro** macro_name, macro_active, macro_status, macro_step,
  macro_count, macro_message
- **Probe** probe_{x,y,z,done,stylus,center_x,center_y,size_x,size_y,
  shift_x,shift_y,shift_z,sphere_ready,sphere_points}
- **MDI** mdi:{input,last,status,message,depth}

## Action catalogue

- `goto:<page_id>` (auto-completed from current document)
- `page:{position,program,offsets,ethercat,alarms,setup}`
- `demo:{cycle,hold,reset,home,jog+,jog-,estop}`
- `program:{prev,next,simulate}`
- `macro:{prev,next,run,abort}` and `macro:run:<name>`
- `setup:{save,load}`
- `offset:work:<n>` (0..5), `offset:tool:<n>` (0..7), `offset:nudge:<axis><delta>`
- `mdi:{submit,clear,abort}`
- `jog:axis:<n>` (0..3 = X/Y/Z/A), `jog:inc:<counts>` (1/10/100/1000 = 0.001..1 mm)
- Input commit targets: `commit:offset:{x,y,z,a}`,
  `commit:tool:{length,radius,wear}`, `commit:restart:line`

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
