# Render Starter

This directory is the isolated starter for a kernel-side graphics pipeline. It is intentionally separate from the current UI path so the renderer can evolve without destabilizing the framebuffer boot flow.

## Target shape

The first milestone is a minimal OpenGL ES 1.1-style fixed-function path:

- `obj_importer.*` parses a small Wavefront OBJ subset into caller-owned fixed buffers.
- `gles1.*` owns the draw-facing API: framebuffer binding, matrix state, flat color state, and indexed triangle submission.
- The current raster path is wireframe-only. That keeps the first integration cheap while proving the object import, transform, clip-space, and framebuffer plumbing.

## Intended kernel pipeline

1. Load or embed an OBJ asset as text.
2. Parse it into fixed vertex/index buffers through `render::obj::ObjImporter`.
3. Bind a framebuffer from the display backend or virtio-gpu scanout backing store.
4. Set model, view, and projection matrices in `render::gles1::Renderer`.
5. Draw the mesh through `draw_mesh_wireframe()`.
6. Later replace or augment the wireframe path with filled triangles, depth buffering, texture state, and DMA-backed uploads.

## Why this is structured this way

- No heap dependency: the importer writes into caller-provided storage.
- No STL dependency: the code stays usable in the freestanding kernel build.
- HAL-adaptable: framebuffer ownership can stay in the platform layer while the renderer remains device-agnostic.
- Future DMA fit: vertex uploads, texture staging, and scanout transfers can route through the DMA abstraction without changing renderer-facing APIs.

## Current limits

- OBJ subset only: `v`, `vn`, `vt`, and `f`.
- Faces support triangles and quads only.
- Material files, smoothing groups, and polygon triangulation beyond quads are not implemented.
- Rasterizer is currently wireframe, not filled.
