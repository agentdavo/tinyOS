# miniOS Machine Editor + Machine Files

Authoring surface for the kernel's kinematic chains and machine meshes. The
browser editor, the kinematic TSVs, and the STL/OBJ parts all live flat in
this directory so the editor can fetch its siblings at startup and the SD
build script can ship them straight to the card.

Kernel consumers:
- `machines/kinematic_*.tsv` — parsed by `render::kinematic::load_chain_from_tsv`
- `machines/*.STL` / `*.obj` — resolved by `render::obj::lookup` via the VFS
  path `system/machine/<name>`.
- `devices/embedded_toolpods.tsv` — parsed by `machine::toolpods::Service`

Three paths get the meshes into the kernel, in priority order:
1. **SD card** (`sdcard.img` built by `scripts/mkimg.sh`) — preferred for
   anything beyond a few hundred kB.
2. **Embedded defaults** — small files `.incbin`'d into `.rodata` from
   this folder (`devices/embedded_kinematics.S`).
3. **HMI upload** — future, not implemented.

## Run the editor

```bash
bash machines/serve.sh          # http://localhost:8765/
# or: MACHINE_EDITOR_PORT=9000 bash machines/serve.sh
```

`machines/vendor/` already carries Three.js r128 (UMD), OrbitControls,
OBJLoader, STLLoader, and JSZip 3.10 — no download step. r128 is chosen
deliberately: newer releases ship examples only as ES modules, which
would need an import-map-aware server.

With the editor served from localhost, on first load it auto-fetches any
`obj_file` references in the active chain from this directory — so the
MX850 chain opens already rendered, no drops required.

Chrome still treats `file://` as a unique security origin (especially
WSL's `file://wsl.localhost/`) and blocks `fetch()`, `webkitGetAsEntry`
(folder drops), and `showOpenFilePicker` on it — `serve.sh` sidesteps
the whole category.

### Opening from `file://` (not recommended)

If you really want to bypass the server, Chrome needs
`--allow-file-access-from-files --user-data-dir=/tmp/chrome-files` or similar.
On WSL this usually still fails because of the `file://wsl.localhost/` prefix
Chrome treats as a unique origin. Just use `serve.sh`.

## Coordinate system

Both renderers (the Three.js editor here and the kernel's GLES1 live view)
use the **Z-up, SolidWorks / ISO 841 CNC convention**, right-handed:

- **+X** lateral (machine table left → right, red axis in the gizmo)
- **+Y** transverse / into the machine (front → back, green)
- **+Z** vertical (bed → spindle, blue)

Preset views in the editor match SolidWorks:

- **Top**    camera on +Z looking −Z, screen-up ≈ +Y
- **Front**  camera on −Y looking +Y, screen-up = +Z
- **Side**   camera on +X looking −X, screen-up = +Z (SolidWorks Right)
- **Iso**    camera in the +X / −Y / +Z octant (default SolidWorks iso)

`dir_x/y/z` in the kinematic TSV is a plain world-space unit vector, so
a typical 3-axis mill reads `X=(1,0,0)`, `Y=(0,1,0)`, `Z=(0,0,1)` and
motion directly matches the machine's physical axes. STL files exported
from SolidWorks / Fusion / FreeCAD drop in at their authored position —
no rotation, no coordinate-swap.

## Kinematic TSV schema

15 comma-separated columns (existing 14-column files still load):

```
name, type, parent, dir_x, dir_y, dir_z, off_x, off_y, off_z,
min, max, mesh, channel, motion_axis [, obj_file]
```

| Inspector field | Column          | Notes                                               |
|-----------------|-----------------|-----------------------------------------------------|
| name            | 0 name          | Axis identifier; children reference by name.        |
| type            | 1 type          | `Linear` / `Rotary` / `Fixed`.                      |
| parent          | 2 parent        | `-1` for root; otherwise another axis name.         |
| dir preset +    | 3..5 dir_x/y/z  | Axis direction (unit vector).                       |
| dir x / y / z   |                 | Editable raw components.                            |
| offset x / y / z| 6..8 off_x/y/z  | Per-parent origin offset.                           |
| min / max       | 9..10           | Travel limits (mm for linear, deg for rotary).      |
| mesh            | 11              | Built-in slot / `box` / `cylinder` / `obj` / `none`.|
| channel         | 12              | 0 or 1 (mill vs lathe on mill-turn).                |
| motion_axis     | 13              | 0..31 or -1 (unbound).                              |
| obj_file        | 14              | Path relative to `devices/`; only saved if set.     |

Forward kinematics match the kernel's implementation exactly (translate by
offset, then translate-by-dir for linear or rotate-around-dir for rotary,
degrees). The DRO panel at the bottom of the viewport shows each axis's
current position, a slider bound to `[min, max]`, and the resolved world-space
direction vector after parent rotations compose.

## Toolpods TSV schema

Line-based, tab-separated, `key=value` fields. Two record kinds:

```
pod     id=.. title=.. axis=.. motion=linear|rotary clamp_required=0|1
station pod=.. index=.. position=.. physical_tool=.. virtual_tool=..
        x=.. y=.. z=.. a=.. name=..
```

Field order is preserved on save (per-record `__order` array). Comments and
blank lines stick with the following record.

## Features

- Live Three.js viewport with OrbitControls, ground grid, world-axis gizmo,
  and floating labels above each link.
- Left tree view with drag-to-reparent; cycle attempts refuse with an alert.
- Inspector editing name, type, parent, direction, offset, travel, mesh slot,
  channel, motion_axis, and `obj_file`.
- Bottom DRO panel with per-axis slider + world-direction + world-origin
  readout. Drag the slider to watch the link move in the viewport.
- OBJ import via drag-drop onto the viewport or `...` button in the inspector;
  kernel import limits (4096 verts, 8192 indices, 1024 positions/normals/uvs
  per `render/obj_importer.hpp`) are counted and flagged in red if exceeded.
- Test-cycle sweep (`T`): each non-Fixed axis sinusoidally visits its full
  travel over ~8 seconds.
- Collision hint: any two axes whose world AABBs overlap (and that aren't
  direct parent/child) are flagged pink in the DRO.
- Auto-save to `localStorage` under `miniOS.machineEditor.doc.v1`. OBJs under
  256 kB are stashed with the doc; larger ones must be re-dropped on reload.
- Export bundle: JSZip-packed `.tsv`s + referenced OBJs + `manifest.json`.

## Keyboard shortcuts

| Key          | Action                              |
|--------------|-------------------------------------|
| Del / Back   | Delete selected axis / pod / station|
| Ctrl+S       | Save both TSVs                      |
| Ctrl+Z / Y   | Undo / redo                         |
| F            | Frame viewport on the chain         |
| T            | Start test-cycle sweep              |
| Esc          | Clear selection                     |

## Round-trip

```bash
node tools/machine_editor/roundtrip_test.js
```

Expected output:

```
[ok] devices/kinematic_mill3.tsv
[ok] devices/kinematic_millturn.tsv
[ok] devices/embedded_toolpods.tsv
[ok] forward kinematics: Z@100 -> (0.000, 0.000, 100.000)
```

The test harness extracts the inline `<script>` from `index.html`, evaluates it
in a fresh `Function()` scope with a stubbed `window`/`document`, and parses +
re-serialises the shipped files to assert byte equality.

## Known limitations

- The toolpods TSV has no per-pod XYZ field, so pod gizmos anchor at the
  origin and stations sit at their local (x, y, z). If the kernel later grows
  a `pod x=...` convention, lift the fields in `rebuildPods`.
- OBJLoader renders triangles and quads; materials and groups are ignored.
- Click-drag in 3D is not wired up; use the DRO sliders or the inspector
  offset fields to move things.
- Changing the `obj_file=` column requires a kernel parser that accepts 15
  columns — the editor assumes that extension lands in parallel. Existing
  14-column behaviour still round-trips.
