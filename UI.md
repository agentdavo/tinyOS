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


## Pages

- [Dashboard](#dashboard) — `dashboard`
- [Jog](#jog) — `jog`
- [MDI](#mdi) — `mdi`
- [Machine View](#machine-view) — `machine_view`
- [Program](#program) — `program`
- [Offsets](#offsets) — `offsets`
- [Service](#service) — `service`
- [EtherCAT](#ethercat) — `ethercat`
- [Homing](#homing) — `homing`
- [Alarms](#alarms) — `alarms`
- [Macros](#macros) — `macros`
- [Probe](#probe) — `probe`
- [Pitch Error](#pec) — `pec`
- [Geometry Comp](#geometry) — `geometry`
- [Volumetric Comp](#sphere) — `sphere`
- [Network](#network) — `network`
- [Axis Detail](#axis-status) — `axis_status`
- [Tool Change](#tool-change) — `tool_change`
- [Restart Confirm](#restart-confirm) — `restart_confirm`
- [Lights-Out](#lights-out) — `lights_out`

<a id="dashboard"></a>
### Dashboard

`ui_page dashboard` — defined in `devices/embedded_ui.tsv`.

![Dashboard](screenshots/dashboard.png)

<a id="jog"></a>
### Jog

`ui_page jog` — defined in `devices/embedded_ui.tsv`.

![Jog](screenshots/jog.png)

<a id="mdi"></a>
### MDI

`ui_page mdi` — defined in `devices/embedded_ui.tsv`.

![MDI](screenshots/mdi.png)

<a id="machine-view"></a>
### Machine View

`ui_page machine_view` — defined in `devices/embedded_ui.tsv`.

![Machine View](screenshots/machine_view.png)

<a id="program"></a>
### Program

`ui_page program` — defined in `devices/embedded_ui.tsv`.

![Program](screenshots/program.png)

<a id="offsets"></a>
### Offsets

`ui_page offsets` — defined in `devices/embedded_ui.tsv`.

![Offsets](screenshots/offsets.png)

<a id="service"></a>
### Service

`ui_page service` — defined in `devices/embedded_ui.tsv`.

![Service](screenshots/service.png)

<a id="ethercat"></a>
### EtherCAT

`ui_page ethercat` — defined in `devices/embedded_ui.tsv`.

![EtherCAT](screenshots/ethercat.png)

<a id="homing"></a>
### Homing

`ui_page homing` — defined in `devices/embedded_ui.tsv`.

![Homing](screenshots/homing.png)

<a id="alarms"></a>
### Alarms

`ui_page alarms` — defined in `devices/embedded_ui.tsv`.

![Alarms](screenshots/alarms.png)

<a id="macros"></a>
### Macros

`ui_page macros` — defined in `devices/embedded_ui.tsv`.

![Macros](screenshots/macros.png)

<a id="probe"></a>
### Probe

`ui_page probe` — defined in `devices/embedded_ui.tsv`.

![Probe](screenshots/probe.png)

<a id="pec"></a>
### Pitch Error

`ui_page pec` — defined in `devices/embedded_ui.tsv`.

![Pitch Error](screenshots/pec.png)

<a id="geometry"></a>
### Geometry Comp

`ui_page geometry` — defined in `devices/embedded_ui.tsv`.

![Geometry Comp](screenshots/geometry.png)

<a id="sphere"></a>
### Volumetric Comp

`ui_page sphere` — defined in `devices/embedded_ui.tsv`.

![Volumetric Comp](screenshots/sphere.png)

<a id="network"></a>
### Network

`ui_page network` — defined in `devices/embedded_ui.tsv`.

![Network](screenshots/network.png)

<a id="axis-status"></a>
### Axis Detail

`ui_page axis_status` — defined in `devices/embedded_ui.tsv`.

![Axis Detail](screenshots/axis_status.png)

<a id="tool-change"></a>
### Tool Change

`ui_page tool_change` — defined in `devices/embedded_ui.tsv`.

![Tool Change](screenshots/tool_change.png)

<a id="restart-confirm"></a>
### Restart Confirm

`ui_page restart_confirm` — defined in `devices/embedded_ui.tsv`.

![Restart Confirm](screenshots/restart_confirm.png)

<a id="lights-out"></a>
### Lights-Out

`ui_page lights_out` — defined in `devices/embedded_ui.tsv`.

![Lights-Out](screenshots/lights_out.png)


---

*Generated 2026-09-24 11:51:14 UTC from `devices/embedded_ui.tsv` (20 pages).*
