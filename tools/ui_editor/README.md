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
  current page. Click a chip to edit the include + its `${arg}` values
  inline. Chips go red when the target page expects args the include
  doesn't provide.
- Actions table for per-page `action widget= event= target=` records.
  `event=` is a dropdown of `click` / `long_press` (D15).
- Page tab badges: a `modal` chip marks pages declared with the
  `dialog` record type (A4). Click the **D** / **P** affordance on a
  tab to flip between regular page and dialog. Only dialog pages
  appear in the `action target=dialog:show:<id>` picker.
- Theme editor (top toolbar): edit the `theme name=#hex` palette
  used by `$name` color tokens in widget fields. Add / remove tokens
  with live colour-swatch preview; the inspector's color pickers
  resolve `$name` references via this palette.
- Undo / redo (`Ctrl+Z` / `Ctrl+Y`), auto-save to `localStorage`.

## Bind catalogue

Grouped exactly as in the inspector:

- **Machine** mode, alarm, prompt, units, wcs, cycle_progress, torque,
  feed, spindle, spindle_rpm, spindle_load, spindle_override,
  block_current, block_next, runtime, parts, page, page_name,
  selected_axis, jog_increment, operator_mode (0=AUTO, 1=MDI, 2=JOG, 3=SETUP)
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
- **Probe wizard** probe:wizard:{state,cycle,step,total,message,
  result_x,result_y,result_z,result_valid}. State cycles IDLE / SELECT /
  CONFIRM / RUNNING / INSPECT / FAULT; cycle renders the human-readable
  cycle name; result_valid renders VALID or `---`. ACCEPT in Inspecting
  also commits the captured result into the active WCS axis offsets per
  cycle (Z surface -> Z, X edge -> X, Y edge -> Y, bore center -> XY,
  3D pocket -> XYZ; qualify and reference sphere never touch the WCS).
- **Compensation (PEC)** cal:pec:{axis,enabled,count,pending_pos,
  pending_err}, cal:pec:0..7:{pos,err}, cal:rotary:offset (axis A index
  offset in counts). PEC bind tokens reflect the operator-selected axis
  in the wizard, not all axes at once.
- **Compensation (geometry)** cal:geom:{xy,xz,yz,enabled} (squareness
  errors in microradians; XY = +urad means Y is tilted +urad relative
  to X).
- **Compensation (sphere)** cal:sphere:{enabled,diameter,probe_um,points,
  computed,err_pos_x,err_pos_y,err_pos_z,err_sq_xy,err_sq_xz,err_sq_yz}.
  `computed` renders READY when the last fit succeeded, PENDING
  otherwise; `points` is the count of measured stars in the volumetric
  fit (sphere_compute_errors needs >= 7 to succeed).
- **MDI** mdi:{input,last,status,message,depth}
- **EtherCAT** ec:{state,fault,slaves,miss,trips,p99,max,period,cycles,tx,rx,
  dc_fault,dc_drift_max,dc_drift_last,dc_samples,dc_trips}. The `dc_*`
  group surfaces the EtherCAT distributed-clock drift sentinel: `dc_fault`
  cycles OK/DRIFT/LOST, the two drift values are in ns, `dc_samples` is
  the rolling window size, `dc_trips` counts how many times the master
  has tripped on excessive drift.
- **Channel overrides** channel:{0,1}:{feed,spindle,barrier_ms}. `feed` /
  `spindle` are permille (1000 = 100%); `barrier_ms` is the dual-channel
  sync barrier wait time in ms (0 = no barrier).
- **Restart wizard** restart:{pending,line,tool,wcs,feed,spindle,coolant}.
  `pending` is the staged restart state (1 = an entry exists); the others
  are the staged values that `commit:restart:line` applies when fired.
- **Lights-out scheduler** scheduler:{state,active,jobcount} and
  pallet:0..3:{id,status,program,cycles}. `state` cycles
  IDLE / RUNNING / PAUSED / FAULT; `active` is the index of the running
  job (or -1); `jobcount` is the queue depth. Pallet rows show the
  4-slot pallet rotation with cycle counters.
- **Diagnostic** lookahead:{0,1,max}, klog:tail, tcp:status. The
  lookahead values are the per-channel block-buffer depth; `klog:tail`
  is the most-recent kernel-log line (single-line, truncated to fit a
  label); `tcp:status` reports the TCP listener / connection table state.
- **View** view_toolpath, view_toolpods (machine_view page overlay flags)
- **Network** net:{nic,ip,gateway,mac,dhcp,link,pending_ip,pending_gateway,
  pending_ping,ping_target,ping_result,ping_rtt,rx_requests,tx_responses,
  uptime}. dhcp cycles IDLE/DISCOVERING/BOUND/TIMEOUT/STATIC; link cycles
  DOWN/UP/PROBING; ping_result is one of OK/BUSY/BAD ADDR/TIMEOUT/SEND FAIL
  or `---` when no ping has been issued. IPs format dotted-quad; mac
  formats colon-hex; uptime renders HH:MM:SS.
- **Axis detail** axis:detail:{state,traj,mode,enabled,fault,homed,err_code,
  fe,fe_max,vmax,accel,jerk,cmd,act,sw,cw}. All bindings read the
  operator-selected axis (machine_snapshot().selected_axis); change axis
  via the picker buttons or any `jog:axis:N` action. State decodes the
  CiA-402 FSA name; traj cycles IDLE/READY/ACCEL/CRUISE/DECEL/HOLDING;
  mode is the active 402 op-mode (CSP/CSV/HM/...). Counts render mm
  (1000 cnt/mm); err_code/sw/cw render hex.
- **Tool change** tc:{current,target,state,msg,current_pod,current_st,
  current_lbl,target_pod,target_st,target_lbl,step,total}. State cycles
  IDLE/RELEASING/MOVING/PICKING/VERIFYING/DONE/FAULTED. The wizard is
  first-cut a confirmation flow (request → Moving → ACCEPT commits via
  toolpods::Service::accept_tool_change which calls select_station).

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
- `probe:wizard:select:<kind>` — pick a probing cycle. `kind` is one of
  `qualify`, `z`, `edgex`, `edgey`, `bore`, `pocket`, `sphere`. Drops the
  wizard into Confirming.
- `probe:wizard:{start,abort,accept,reject}` — wizard transport.
  `start` requires Confirming/Inspecting; `accept` / `reject` only do
  something while the wizard is in Inspecting; `start` is a no-op while
  either EtherCAT master holds a deadline-fault latch. Sphere routes to
  probe::Runtime; the other six cycles execute through the macro
  runtime (probe_qualify / probe_z_surface / probe_x_edge / probe_y_edge
  / probe_bore_xy / probe_calibrate_3d_pocket).
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
- `cal:pec:axis:<n>` (n=0..3) — pick axis for the PEC table
- `cal:pec:add` / `cal:pec:clear` / `cal:pec:enable:toggle` — flush the
  pending pos/err pair into the table, drop all rows, or flip the
  axis-local enable flag. All gated by master deadline-fault.
- `cal:geom:enable:toggle` — flip the geometry compensation enable flag.
  Per-pair urad values are committed via the inputs (see commit targets
  below).
- `cal:sphere:{compute,enable:toggle,clear}` — run the volumetric fit
  on the existing point set, flip the enable flag, or drop all measured
  points. Compute requires >= 7 points.
- Input commit targets: `commit:offset:{x,y,z,a}`,
  `commit:tool:{length,radius,wear}`, `commit:restart:line`,
  `commit:spindle:rpm`, `commit:cal:pec:{pos,err}`,
  `commit:cal:geom:{xy,xz,yz}`, `commit:net:{ip,gateway,ping_target}`
  (dotted-quad IPv4), `commit:tc:target` (integer tool number)
- `net:{dhcp:on,dhcp:off,commit_static,ping}` — network setup actions.
  `dhcp:off` latches the live IP into the static-config slot so the
  operator's view doesn't blank when toggled; `commit_static` pushes the
  pending IP / gateway buffer live; `ping` fires async at the pending
  target. All gated by master deadline-fault.
- `axis:detail:{enable,disable,fault_reset}` — operate on the SELECTED
  axis (whatever `set_selected_axis` last picked). Routes through
  motion::Kernel::{enable_axis,disable_axis,fault_reset}. Master
  deadline-fault gated.
- `tc:{start,abort,accept}` — tool change wizard transport. `start`
  picks up the operator-typed target tool, resolves it against the pod
  registry, and transitions Idle → Moving with a confirm message.
  `accept` commits the swap via toolpods::Service::select_station;
  `abort` reverts to Idle without touching the active station. Master
  deadline-fault gated.
- `ec:{estop,clear_fault}` — broadcast QuickStop / clear deadline-fault latch
- `view:{toolpath,toolpods}:toggle` — flip overlay flags on machine_view
- `view:{reset,zoom:in,zoom:out}` — camera primitives on machine_view
- `alarm:ack:N` (N=0..3) — acknowledge active alarm in row N
- `alarm:clear_history` — wipe the alarm history queue
- `dialog:show:<page_id>` / `dialog:close` (A4) — show / hide a page
  declared with the `dialog` record type. Dialog pages render on top of
  the active page in a modal-ish Z-layer; `close` hides whichever one
  is active. The editor exposes one `dialog:show:<id>` per defined page
  in the action picker — pick the page that is meant to behave modally.
- `jobs:{run,pause,resume,skip,abort}` — lights-out scheduler transport
  (mirrors the CLI `job_*` verbs). All gated by master deadline-fault.
- `restart:{confirm,cancel}` — restart-wizard transport. `confirm`
  commits the staged restart row (line / tool / wcs / feed / spindle /
  coolant) to the active program; `cancel` discards it. The staged
  values themselves are loaded via the `commit:restart:line` input
  target.

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
- `hold=<ms>` (D18) — buttons only. The user must keep the button pressed
  continuously for `<ms>` milliseconds before the action fires; release
  before that cancels. The kernel paints a red "keep holding" overlay
  while the timer counts up. Used on destructive actions (E-STOP, ABORT,
  clear-fault). 0 or unset = instant tap.
- `cols=<n>`, `rows=<n>`, `spacing=<px>` (alias `gap=`), `pad=<px>` (B8) —
  panel / container layout overrides. With layout=grid, `cols` × `rows`
  defines the cell matrix (defaults to ceil(sqrt(child_count))). `spacing`
  is the inter-child gap; `pad` is the container's inner padding. -1 in
  any of these means "use the kernel default" (12 spacing, 16 pad).
- `event=<name>` (D15) — on action records. `click` is the default tap;
  `long_press` fires when the user holds the button >= 500 ms without
  releasing. Distinct from `hold=` — `hold=` gates the click itself,
  while `long_press` is a second action wired to the same widget that
  fires only on a long hold.

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
- Dialog pages render in the editor canvas the same as regular pages
  (no z-overlay preview). The `modal` badge on the page tab and the
  scoped `dialog:show:` action picker are the cues; the kernel does
  the actual modal compositing at runtime.
- Theme tokens: alpha channel of `#RRGGBBAA` values is preserved by the
  parser/serializer but the colour picker UI is RGB-only — open the
  hex value in the theme editor to see/change alpha by hand.

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
