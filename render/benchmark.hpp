// SPDX-License-Identifier: MIT OR Apache-2.0
// Benchmark: 1000 spinning cubes

#ifndef RENDER_BENCHMARK_HPP
#define RENDER_BENCHMARK_HPP

#include <cstdint>

namespace render { namespace gles1 { class Renderer; struct MeshView; } }

namespace render::benchmark {

void init();
void run(gles1::Renderer& renderer, const gles1::MeshView& cube_mesh, uint32_t ticks);
uint64_t get_frame_count();

} // namespace render::benchmark

#endif
