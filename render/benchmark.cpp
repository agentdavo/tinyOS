// SPDX-License-Identifier: MIT OR Apache-2.0
// Benchmark: 1000 spinning cubes with Phong lighting

#include "render/gles1.hpp"
#include "render/machine_model.hpp"
#include <cstdint>
#include <cstring>

namespace render::benchmark {

namespace {

constexpr size_t kNumCubes = 1000;

struct CubeInstance {
    float pos_x;
    float pos_y;
    float pos_z;
    float rot_x;
    float rot_y;
    float rot_z;
    float scale;
    float shininess;
    float r, g, b;
};

CubeInstance g_cubes[kNumCubes];
bool g_initialized = false;
uint64_t g_frame_count = 0;

void init_cubes() {
    if (g_initialized) return;
    
    constexpr uint8_t colors[][3] = {
        {255, 50, 50},   {50, 255, 50},   {50, 50, 255},
        {255, 255, 50}, {255, 50, 255}, {50, 255, 255},
        {255, 128, 50}, {128, 50, 255}, {255, 100, 200}, {100, 200, 255},
        {255, 180, 80}, {80, 180, 255}, {180, 80, 255}, {80, 255, 180}, {255, 80, 180},
        {200, 255, 100}, {100, 200, 255}, {255, 200, 100}, {200, 100, 255}, {150, 100, 255}
    };
    constexpr size_t kNumColors = sizeof(colors) / sizeof(colors[0]);
    
    for (size_t i = 0; i < kNumCubes; ++i) {
        int grid_x = static_cast<int>(i) % 10 - 5;
        int grid_y = static_cast<int>(i / 10) % 10 - 5;
        int grid_z = static_cast<int>(i / 100) % 10 - 5;
        
        g_cubes[i].pos_x = static_cast<float>(grid_x) * 1.5f;
        g_cubes[i].pos_y = static_cast<float>(grid_y) * 1.5f;
        g_cubes[i].pos_z = static_cast<float>(grid_z) * 1.5f - 5.0f;
        
        g_cubes[i].rot_x = static_cast<float>(i * 3) * 0.1f;
        g_cubes[i].rot_y = static_cast<float>(i * 7) * 0.1f;
        g_cubes[i].rot_z = 0.0f;
        g_cubes[i].scale = 0.4f;
        
        size_t color_idx = i % kNumColors;
        g_cubes[i].r = static_cast<float>(colors[color_idx][0]) / 255.0f;
        g_cubes[i].g = static_cast<float>(colors[color_idx][1]) / 255.0f;
        g_cubes[i].b = static_cast<float>(colors[color_idx][2]) / 255.0f;
        
        g_cubes[i].shininess = 32.0f;
    }
    
    g_initialized = true;
}

} // namespace

void init() {
    init_cubes();
}

void run(gles1::Renderer& renderer, const gles1::MeshView& cube_mesh, uint32_t ticks) {
    if (!g_initialized) init_cubes();
    
    constexpr int FRAMES_PER_CALL = 60;
    for (int f = 0; f < FRAMES_PER_CALL; ++f) {
        g_frame_count++;
    }
    
    float time = static_cast<float>(ticks) * 0.01f;
    
    gles1::Mat4 proj = gles1::make_perspective(1.2f, 0.5625f, 0.1f, 100.0f);
    gles1::Mat4 view = gles1::make_look_at(
        {0.0f, 0.0f, 4.0f},
        {0.0f, 0.0f, -5.0f},
        {0.0f, 1.0f, 0.0f}
    );
    
    renderer.set_projection_matrix(proj);
    renderer.set_view_matrix(view);
    
    gles1::Light light{
        {2.0f, 4.0f, 4.0f},
        {0.2f, 0.2f, 0.2f, 1.0f},
        {1.0f, 1.0f, 1.0f, 1.0f},
        {1.0f, 1.0f, 1.0f, 1.0f}
    };
    renderer.set_light(light);
    
    for (size_t i = 0; i < kNumCubes; ++i) {
        const CubeInstance& cube = g_cubes[i];
        
        float rx = cube.rot_x + time * 2.0f;
        float ry = cube.rot_y + time * 3.0f;
        
        gles1::Mat4 model = gles1::make_translation(cube.pos_x, cube.pos_y, cube.pos_z);
        
        gles1::Mat4 rot_x = gles1::make_rotation_x(rx);
        gles1::Mat4 rot_y = gles1::make_rotation_y(ry);
        gles1::Mat4 scale = gles1::make_scale(cube.scale, cube.scale, cube.scale);
        
        model = gles1::multiply(model, rot_x);
        model = gles1::multiply(model, rot_y);
        model = gles1::multiply(model, scale);
        
        renderer.set_model_matrix(model);
        
        gles1::Material mat;
        mat.ambient = {
            cube.r * 0.2f,
            cube.g * 0.2f,
            cube.b * 0.2f,
            1.0f
        };
        mat.diffuse = {
            cube.r,
            cube.g,
            cube.b,
            1.0f
        };
        mat.specular = {0.5f, 0.5f, 0.5f, 1.0f};
        mat.shininess = cube.shininess;
        renderer.set_material(mat);
        
        renderer.draw_mesh_solid(cube_mesh);
    }
}

uint64_t get_frame_count() {
    return g_frame_count;
}

} // namespace render::benchmark
