// SPDX-License-Identifier: MIT OR Apache-2.0
// Minimal fixed-function renderer scaffold for kernel bring-up.

#ifndef RENDER_GLES1_HPP
#define RENDER_GLES1_HPP

#include <cstddef>
#include <cstdint>

namespace render::gles1 {

struct Vec2f {
    float x;
    float y;
};

struct Vec3f {
    float x;
    float y;
    float z;
};

struct Vec4f {
    float x;
    float y;
    float z;
    float w;
};

struct Color4u8 {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;
};

struct Color4f {
    float r;
    float g;
    float b;
    float a;
};

struct Mat4 {
    float m[16];

    static Mat4 identity();
};

struct Vertex {
    Vec3f position;
    Vec3f normal;
    Vec2f uv;
    Color4u8 color;
};

struct Material {
    Color4f ambient{0.2f, 0.2f, 0.2f, 1.0f};
    Color4f diffuse{0.8f, 0.8f, 0.8f, 1.0f};
    Color4f specular{0.0f, 0.0f, 0.0f, 1.0f};
    float shininess = 0.0f;
};

struct Light {
    Vec3f position{0.0f, 1.0f, 1.0f};
    Color4f ambient{0.2f, 0.2f, 0.2f, 1.0f};
    Color4f diffuse{0.8f, 0.8f, 0.8f, 1.0f};
    Color4f specular{0.5f, 0.5f, 0.5f, 1.0f};
};

struct MeshView {
    const Vertex* vertices = nullptr;
    size_t vertex_count = 0;
    const uint16_t* indices = nullptr;
    size_t index_count = 0;
};

struct FramebufferView {
    uint32_t* pixels = nullptr;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride_pixels = 0;
};

class Renderer {
public:
    void bind_framebuffer(FramebufferView fb);
    void clear(uint32_t argb);

    void set_model_matrix(const Mat4& mat);
    void set_view_matrix(const Mat4& mat);
    void set_projection_matrix(const Mat4& mat);
    void set_flat_color(Color4u8 color);

    void set_material(const Material& mat);
    void set_light(const Light& light);

    bool draw_mesh_wireframe(const MeshView& mesh);
    bool draw_mesh_solid(const MeshView& mesh);
    bool draw_polyline(const Vec3f* points, size_t point_count, Color4u8 color);

private:
    struct ScreenPoint {
        int32_t x;
        int32_t y;
        float z;
        bool valid;
    };

    struct RasterVertex {
        float x;
        float y;
        float z;
        Color4f color;
        bool valid;
    };

    FramebufferView framebuffer_{};
    Mat4 model_ = Mat4::identity();
    Mat4 view_ = Mat4::identity();
    Mat4 projection_ = Mat4::identity();
    Color4u8 flat_color_{0xff, 0xff, 0xff, 0xff};
    Material material_{};
    Light light_{};

    void put_pixel(int32_t x, int32_t y, uint32_t argb);
    void draw_line(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t argb);
    RasterVertex shade_vertex(const Mat4& model_view, const Mat4& mvp, const Vertex& vertex) const;
    bool rasterize_triangle(const RasterVertex& v0, const RasterVertex& v1, const RasterVertex& v2);
    static uint32_t pack_argb(Color4u8 color);
    static uint32_t pack_argb_f(Color4f color);
};

Mat4 multiply(const Mat4& a, const Mat4& b);
Vec4f multiply(const Mat4& m, const Vec4f& v);
Mat4 make_perspective(float fov_y_radians, float aspect, float z_near, float z_far);
Mat4 make_translation(float x, float y, float z);
Mat4 make_rotation_y(float radians);
Mat4 make_rotation_x(float radians);
Mat4 make_rotation_z(float radians);
Mat4 make_scale(float x, float y, float z);
Mat4 make_look_at(const Vec3f& eye, const Vec3f& center, const Vec3f& up);

} // namespace render::gles1

#endif
