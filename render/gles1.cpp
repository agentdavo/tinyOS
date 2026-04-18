// SPDX-License-Identifier: MIT OR Apache-2.0
// GLES1.1 software renderer - fixed

#include "render/gles1.hpp"
#include <cstdio>

namespace render::gles1 {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float k2Pi = kPi * 2.0f;
constexpr float kHalfPi = kPi * 0.5f;

#define GLES_INLINE __attribute__((always_inline)) inline

GLES_INLINE float absf(float v) {
    union { float f; uint32_t u; } val = {v};
    val.u &= 0x7FFFFFFFU;
    return val.f;
}

GLES_INLINE float minf(float a, float b) { return a < b ? a : b; }
GLES_INLINE float maxf(float a, float b) { return a > b ? a : b; }
GLES_INLINE float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
GLES_INLINE int32_t mini(int32_t a, int32_t b) { return a < b ? a : b; }
GLES_INLINE int32_t maxi(int32_t a, int32_t b) { return a > b ? a : b; }

GLES_INLINE float wrap_pi(float radians) {
    while (radians > kPi) radians -= k2Pi;
    while (radians < -kPi) radians += k2Pi;
    return radians;
}

GLES_INLINE float sin_approx(float x) {
    x = wrap_pi(x);
    const float x2 = x * x;
    const float x3 = x2 * x;
    const float x5 = x3 * x2;
    const float x7 = x5 * x2;
    return x - x3 / 6.0f + x5 / 120.0f - x7 / 5040.0f;
}

GLES_INLINE float cos_approx(float x) {
    return sin_approx(x + kHalfPi);
}

GLES_INLINE float rsqrt_approx(float v) {
    if (v <= 1e-6f) return 0.0f;
    float x = 1.0f / v;
    x = 0.5f * x * (3.0f - v * x * x);
    x = 0.5f * x * (3.0f - v * x * x);
    return x;
}

GLES_INLINE void sincos_approx(float x, float& s, float& c) {
    x = wrap_pi(x);
    const float x2 = x * x;
    const float x3 = x2 * x;
    const float x4 = x2 * x2;
    const float x5 = x4 * x;
    s = x - x3 / 6.0f + x5 / 120.0f;
    c = 1.0f - x2 / 2.0f + x4 / 24.0f;
}

GLES_INLINE float dot_v3v3(const Vec3f& a, const Vec3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

GLES_INLINE Vec3f normalize_v3_fast(Vec3f v) {
    float len_sq = v.x * v.x + v.y * v.y + v.z * v.z;
    if (len_sq > 1e-8f) {
        float inv_len = rsqrt_approx(len_sq);
        v.x *= inv_len;
        v.y *= inv_len;
        v.z *= inv_len;
    }
    return v;
}

GLES_INLINE Vec3f add_v3(const Vec3f& a, const Vec3f& b) {
    return Vec3f{a.x + b.x, a.y + b.y, a.z + b.z};
}

GLES_INLINE Vec3f sub_v3(const Vec3f& a, const Vec3f& b) {
    return Vec3f{a.x - b.x, a.y - b.y, a.z - b.z};
}

GLES_INLINE Vec3f mul_v3f(const Vec3f& v, float s) {
    return Vec3f{v.x * s, v.y * s, v.z * s};
}

GLES_INLINE Color4f mul_c4(const Color4f& a, const Color4f& b) {
    return Color4f{a.r * b.r, a.g * b.g, a.b * b.b, a.a * b.a};
}

GLES_INLINE Color4f add_c4(const Color4f& a, const Color4f& b) {
    return Color4f{a.r + b.r, a.g + b.g, a.b + b.b, a.a + b.a};
}

GLES_INLINE Color4f scale_c4(const Color4f& c, float s) {
    return Color4f{c.r * s, c.g * s, c.b * s, c.a * s};
}

GLES_INLINE float edge_function(float ax, float ay, float bx, float by, float px, float py) {
    return (px - ax) * (by - ay) - (py - ay) * (bx - ax);
}

GLES_INLINE float pow_approx(float base, float exp) {
    if (base <= 0.0f) return 0.0f;
    const int32_t whole = static_cast<int32_t>(exp);
    float out = 1.0f;
    for (int32_t i = 0; i < whole; ++i) {
        out *= base;
    }
    const float frac = exp - static_cast<float>(whole);
    if (frac > 0.001f) {
        out *= 1.0f + frac * (base - 1.0f);
    }
    return out;
}

GLES_INLINE Vec3f cross_v3v3(const Vec3f& a, const Vec3f& b) {
    return Vec3f{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

} // namespace

Mat4 Mat4::identity() {
    Mat4 out{};
    out.m[0] = 1.0f; out.m[1] = 0.0f; out.m[2] = 0.0f; out.m[3] = 0.0f;
    out.m[4] = 0.0f; out.m[5] = 1.0f; out.m[6] = 0.0f; out.m[7] = 0.0f;
    out.m[8] = 0.0f; out.m[9] = 0.0f; out.m[10] = 1.0f; out.m[11] = 0.0f;
    out.m[12] = 0.0f; out.m[13] = 0.0f; out.m[14] = 0.0f; out.m[15] = 1.0f;
    return out;
}

Mat4 multiply(const Mat4& a, const Mat4& b) {
    Mat4 out{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            out.m[i * 4 + j] = 
                a.m[0 * 4 + i] * b.m[j * 4 + 0] +
                a.m[1 * 4 + i] * b.m[j * 4 + 1] +
                a.m[2 * 4 + i] * b.m[j * 4 + 2] +
                a.m[3 * 4 + i] * b.m[j * 4 + 3];
        }
    }
    return out;
}

Vec4f multiply(const Mat4& m, const Vec4f& v) {
    return Vec4f{
        m.m[0] * v.x + m.m[4] * v.y + m.m[8] * v.z + m.m[12] * v.w,
        m.m[1] * v.x + m.m[5] * v.y + m.m[9] * v.z + m.m[13] * v.w,
        m.m[2] * v.x + m.m[6] * v.y + m.m[10] * v.z + m.m[14] * v.w,
        m.m[3] * v.x + m.m[7] * v.y + m.m[11] * v.z + m.m[15] * v.w
    };
}

Mat4 make_perspective(float fov_y_radians, float aspect, float z_near, float z_far) {
    const float half_fov = fov_y_radians * 0.5f;
    const float tan_half = sin_approx(half_fov) / cos_approx(half_fov);
    const float f = 1.0f / tan_half;
    const float range_inv = 1.0f / (z_near - z_far);

    Mat4 out = Mat4::identity();
    out.m[0] = f / aspect;
    out.m[5] = f;
    out.m[10] = (z_far + z_near) * range_inv;
    out.m[11] = -1.0f;
    out.m[14] = 2.0f * z_far * z_near * range_inv;
    out.m[15] = 0.0f;
    return out;
}

Mat4 make_translation(float x, float y, float z) {
    Mat4 out = Mat4::identity();
    out.m[12] = x;
    out.m[13] = y;
    out.m[14] = z;
    return out;
}

Mat4 make_rotation_y(float radians) {
    Mat4 out = Mat4::identity();
    float s, c;
    sincos_approx(radians, s, c);
    out.m[0] = c;  out.m[2] = -s;
    out.m[8] = s;  out.m[10] = c;
    return out;
}

Mat4 make_rotation_x(float radians) {
    Mat4 out = Mat4::identity();
    float s, c;
    sincos_approx(radians, s, c);
    out.m[5] = c;  out.m[6] = s;
    out.m[9] = -s; out.m[10] = c;
    return out;
}

Mat4 make_rotation_z(float radians) {
    Mat4 out = Mat4::identity();
    float s, c;
    sincos_approx(radians, s, c);
    out.m[0] = c;  out.m[1] = -s;
    out.m[4] = s;  out.m[5] = c;
    return out;
}

Mat4 make_scale(float x, float y, float z) {
    Mat4 out = Mat4::identity();
    out.m[0] = x;
    out.m[5] = y;
    out.m[10] = z;
    return out;
}

Mat4 make_look_at(const Vec3f& eye, const Vec3f& center, const Vec3f& up) {
    Vec3f f = normalize_v3_fast(Vec3f{center.x - eye.x, center.y - eye.y, center.z - eye.z});
    Vec3f s = normalize_v3_fast(cross_v3v3(f, up));
    Vec3f u = cross_v3v3(s, f);

    Mat4 out = Mat4::identity();
    out.m[0] = s.x; out.m[1] = s.y; out.m[2] = s.z;
    out.m[4] = u.x; out.m[5] = u.y; out.m[6] = u.z;
    out.m[8] = -f.x; out.m[9] = -f.y; out.m[10] = -f.z;
    out.m[12] = -dot_v3v3(s, eye);
    out.m[13] = -dot_v3v3(u, eye);
    out.m[14] = dot_v3v3(f, eye);
    return out;
}

void Renderer::bind_framebuffer(FramebufferView fb) {
    framebuffer_ = fb;
}

void Renderer::clear(uint32_t argb) {
    if (!framebuffer_.pixels || framebuffer_.stride_pixels == 0) return;
    uint32_t* p = framebuffer_.pixels;
    const uint32_t count = framebuffer_.width * framebuffer_.height;
    for (uint32_t i = 0; i < count; ++i) {
        p[i] = argb;
    }
}

void Renderer::set_model_matrix(const Mat4& mat) { model_ = mat; }
void Renderer::set_view_matrix(const Mat4& mat) { view_ = mat; }
void Renderer::set_projection_matrix(const Mat4& mat) { projection_ = mat; }
void Renderer::set_flat_color(Color4u8 color) { flat_color_ = color; }
void Renderer::set_material(const Material& mat) { material_ = mat; }
void Renderer::set_light(const Light& light) { light_ = light; }

void Renderer::put_pixel(int32_t x, int32_t y, uint32_t argb) {
    if (!framebuffer_.pixels) return;
    if (x < 0 || y < 0) return;
    if (static_cast<uint32_t>(x) >= framebuffer_.width ||
        static_cast<uint32_t>(y) >= framebuffer_.height) return;
    framebuffer_.pixels[static_cast<uint32_t>(y) * framebuffer_.stride_pixels + static_cast<uint32_t>(x)] = argb;
}

void Renderer::draw_line(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t argb) {
    int32_t dx = x1 > x0 ? x1 - x0 : x0 - x1;
    int32_t sx = x0 < x1 ? 1 : -1;
    int32_t dy = y1 > y0 ? -(y1 - y0) : -(y0 - y1);
    int32_t sy = y0 < y1 ? 1 : -1;
    int32_t err = dx + dy;

    while (true) {
        put_pixel(x0, y0, argb);
        if (x0 == x1 && y0 == y1) break;
        int32_t e2 = err * 2;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

bool Renderer::draw_mesh_wireframe(const MeshView& mesh) {
    if (!framebuffer_.pixels || !mesh.vertices || !mesh.indices) return false;
    if ((mesh.index_count % 3) != 0) return false;

    const Mat4 mvp = multiply(projection_, multiply(view_, model_));
    const uint32_t color = pack_argb(flat_color_);

    const size_t tri_count = mesh.index_count / 3;
    for (size_t t = 0; t < tri_count; ++t) {
        const uint16_t i0 = mesh.indices[t * 3 + 0];
        const uint16_t i1 = mesh.indices[t * 3 + 1];
        const uint16_t i2 = mesh.indices[t * 3 + 2];
        
        const Vec3f& v0 = mesh.vertices[i0].position;
        const Vec3f& v1 = mesh.vertices[i1].position;
        const Vec3f& v2 = mesh.vertices[i2].position;

        Vec4f c0 = multiply(mvp, Vec4f{v0.x, v0.y, v0.z, 1.0f});
        Vec4f c1 = multiply(mvp, Vec4f{v1.x, v1.y, v1.z, 1.0f});
        Vec4f c2 = multiply(mvp, Vec4f{v2.x, v2.y, v2.z, 1.0f});

        if (c0.w > 0.001f && c1.w > 0.001f && c2.w > 0.001f) {
            float iw0 = 1.0f / c0.w;
            float iw1 = 1.0f / c1.w;
            float iw2 = 1.0f / c2.w;

            int32_t x0 = (int32_t)((c0.x * iw0 * 0.5f + 0.5f) * framebuffer_.width);
            int32_t y0 = (int32_t)((1.0f - (c0.y * iw0 * 0.5f + 0.5f)) * framebuffer_.height);
            int32_t x1 = (int32_t)((c1.x * iw1 * 0.5f + 0.5f) * framebuffer_.width);
            int32_t y1 = (int32_t)((1.0f - (c1.y * iw1 * 0.5f + 0.5f)) * framebuffer_.height);
            int32_t x2 = (int32_t)((c2.x * iw2 * 0.5f + 0.5f) * framebuffer_.width);
            int32_t y2 = (int32_t)((1.0f - (c2.y * iw2 * 0.5f + 0.5f)) * framebuffer_.height);

            draw_line(x0, y0, x1, y1, color);
            draw_line(x1, y1, x2, y2, color);
            draw_line(x2, y2, x0, y0, color);
        }
    }
    return true;
}

bool Renderer::draw_mesh_solid(const MeshView& mesh) {
    if (!framebuffer_.pixels || !mesh.vertices || !mesh.indices) return false;
    if ((mesh.index_count % 3) != 0) return false;

    const Mat4 mv = multiply(view_, model_);
    const Mat4 mvp = multiply(projection_, mv);

    const size_t tri_count = mesh.index_count / 3;
    size_t drawn = 0;

    for (size_t t = 0; t < tri_count; ++t) {
        const uint16_t i0 = mesh.indices[t * 3 + 0];
        const uint16_t i1 = mesh.indices[t * 3 + 1];
        const uint16_t i2 = mesh.indices[t * 3 + 2];

        const Vertex& v0 = mesh.vertices[i0];
        const Vertex& v1 = mesh.vertices[i1];
        const Vertex& v2 = mesh.vertices[i2];
        const RasterVertex rv0 = shade_vertex(mv, mvp, v0);
        const RasterVertex rv1 = shade_vertex(mv, mvp, v1);
        const RasterVertex rv2 = shade_vertex(mv, mvp, v2);
        if (rasterize_triangle(rv0, rv1, rv2)) {
            ++drawn;
        }
    }
    return drawn > 0;
}

bool Renderer::draw_polyline(const Vec3f* points, size_t point_count, Color4u8 color) {
    if (!framebuffer_.pixels || !points || point_count < 2) return false;
    const Mat4 mvp = multiply(projection_, multiply(view_, model_));
    const uint32_t argb = pack_argb(color);
    size_t drawn = 0;
    for (size_t i = 1; i < point_count; ++i) {
        const Vec4f a = multiply(mvp, Vec4f{points[i - 1].x, points[i - 1].y, points[i - 1].z, 1.0f});
        const Vec4f b = multiply(mvp, Vec4f{points[i].x, points[i].y, points[i].z, 1.0f});
        if (a.w <= 0.001f || b.w <= 0.001f) continue;
        const float ia = 1.0f / a.w;
        const float ib = 1.0f / b.w;
        const int32_t x0 = static_cast<int32_t>((a.x * ia * 0.5f + 0.5f) * framebuffer_.width);
        const int32_t y0 = static_cast<int32_t>((1.0f - (a.y * ia * 0.5f + 0.5f)) * framebuffer_.height);
        const int32_t x1 = static_cast<int32_t>((b.x * ib * 0.5f + 0.5f) * framebuffer_.width);
        const int32_t y1 = static_cast<int32_t>((1.0f - (b.y * ib * 0.5f + 0.5f)) * framebuffer_.height);
        draw_line(x0, y0, x1, y1, argb);
        ++drawn;
    }
    return drawn > 0;
}

Renderer::RasterVertex Renderer::shade_vertex(const Mat4& model_view, const Mat4& mvp,
                                              const Vertex& vertex) const {
    RasterVertex out{};
    out.valid = false;

    const Vec4f clip = multiply(mvp, Vec4f{
        vertex.position.x, vertex.position.y, vertex.position.z, 1.0f});
    if (clip.w <= 0.001f) {
        return out;
    }

    const float inv_w = 1.0f / clip.w;
    out.x = (clip.x * inv_w * 0.5f + 0.5f) * static_cast<float>(framebuffer_.width);
    out.y = (1.0f - (clip.y * inv_w * 0.5f + 0.5f)) * static_cast<float>(framebuffer_.height);
    out.z = clip.z * inv_w;

    const Vec4f eye_pos4 = multiply(model_view, Vec4f{
        vertex.position.x, vertex.position.y, vertex.position.z, 1.0f});
    const Vec3f eye_pos{eye_pos4.x, eye_pos4.y, eye_pos4.z};

    Vec4f eye_n4 = multiply(model_view, Vec4f{
        vertex.normal.x, vertex.normal.y, vertex.normal.z, 0.0f});
    Vec3f normal = normalize_v3_fast(Vec3f{eye_n4.x, eye_n4.y, eye_n4.z});
    if (normal.x == 0.0f && normal.y == 0.0f && normal.z == 0.0f) {
        normal = Vec3f{0.0f, 0.0f, 1.0f};
    }

    Vec3f light_dir = normalize_v3_fast(light_.position);
    light_dir = mul_v3f(light_dir, -1.0f);
    const Vec3f view_dir = normalize_v3_fast(mul_v3f(eye_pos, -1.0f));
    const Vec3f half_dir = normalize_v3_fast(add_v3(light_dir, view_dir));

    const float ndotl = maxf(0.0f, dot_v3v3(normal, light_dir));
    const float ndoth = maxf(0.0f, dot_v3v3(normal, half_dir));
    const float spec_power = material_.shininess > 1.0f
        ? material_.shininess * 0.25f
        : material_.shininess;
    const float specular = spec_power > 0.0f ? pow_approx(ndoth, spec_power) : 0.0f;

    Color4f base{
        static_cast<float>(vertex.color.r) * (1.0f / 255.0f),
        static_cast<float>(vertex.color.g) * (1.0f / 255.0f),
        static_cast<float>(vertex.color.b) * (1.0f / 255.0f),
        static_cast<float>(vertex.color.a) * (1.0f / 255.0f)
    };

    Color4f lit = add_c4(
        mul_c4(base, mul_c4(material_.ambient, light_.ambient)),
        add_c4(
            mul_c4(base, scale_c4(mul_c4(material_.diffuse, light_.diffuse), ndotl)),
            scale_c4(mul_c4(material_.specular, light_.specular), specular)));

    lit.r = clampf(lit.r, 0.0f, 1.0f);
    lit.g = clampf(lit.g, 0.0f, 1.0f);
    lit.b = clampf(lit.b, 0.0f, 1.0f);
    lit.a = clampf(base.a * material_.diffuse.a, 0.0f, 1.0f);
    out.color = lit;
    out.valid = true;
    return out;
}

bool Renderer::rasterize_triangle(const RasterVertex& v0, const RasterVertex& v1,
                                  const RasterVertex& v2) {
    if (!v0.valid || !v1.valid || !v2.valid) return false;

    const float area = edge_function(v0.x, v0.y, v1.x, v1.y, v2.x, v2.y);
    if (absf(area) < 0.001f) return false;

    const int32_t width = static_cast<int32_t>(framebuffer_.width);
    const int32_t height = static_cast<int32_t>(framebuffer_.height);
    if (width <= 0 || height <= 0) return false;

    int32_t min_x = static_cast<int32_t>(v0.x);
    int32_t max_x = static_cast<int32_t>(v0.x);
    int32_t min_y = static_cast<int32_t>(v0.y);
    int32_t max_y = static_cast<int32_t>(v0.y);

    const int32_t vx1 = static_cast<int32_t>(v1.x);
    const int32_t vy1 = static_cast<int32_t>(v1.y);
    const int32_t vx2 = static_cast<int32_t>(v2.x);
    const int32_t vy2 = static_cast<int32_t>(v2.y);

    min_x = maxi(0, mini(min_x, mini(vx1, vx2)));
    min_y = maxi(0, mini(min_y, mini(vy1, vy2)));
    max_x = mini(width - 1, maxi(max_x, maxi(vx1, vx2)));
    max_y = mini(height - 1, maxi(max_y, maxi(vy1, vy2)));

    const float inv_area = 1.0f / area;
    for (int32_t y = min_y; y <= max_y; ++y) {
        for (int32_t x = min_x; x <= max_x; ++x) {
            const float px = static_cast<float>(x) + 0.5f;
            const float py = static_cast<float>(y) + 0.5f;
            const float w0 = edge_function(v1.x, v1.y, v2.x, v2.y, px, py) * inv_area;
            const float w1 = edge_function(v2.x, v2.y, v0.x, v0.y, px, py) * inv_area;
            const float w2 = edge_function(v0.x, v0.y, v1.x, v1.y, px, py) * inv_area;
            if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) continue;

            const Color4f shade{
                clampf(v0.color.r * w0 + v1.color.r * w1 + v2.color.r * w2, 0.0f, 1.0f),
                clampf(v0.color.g * w0 + v1.color.g * w1 + v2.color.g * w2, 0.0f, 1.0f),
                clampf(v0.color.b * w0 + v1.color.b * w1 + v2.color.b * w2, 0.0f, 1.0f),
                clampf(v0.color.a * w0 + v1.color.a * w1 + v2.color.a * w2, 0.0f, 1.0f)
            };
            put_pixel(x, y, pack_argb_f(shade));
        }
    }
    return true;
}

uint32_t Renderer::pack_argb(Color4u8 color) {
    return (static_cast<uint32_t>(color.a) << 24) |
           (static_cast<uint32_t>(color.r) << 16) |
           (static_cast<uint32_t>(color.g) << 8) |
            static_cast<uint32_t>(color.b);
}

uint32_t Renderer::pack_argb_f(Color4f color) {
    auto clamp01 = [](float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); };
    return (static_cast<uint32_t>(clamp01(color.a) * 255.0f) << 24) |
           (static_cast<uint32_t>(clamp01(color.r) * 255.0f) << 16) |
           (static_cast<uint32_t>(clamp01(color.g) * 255.0f) << 8) |
            static_cast<uint32_t>(clamp01(color.b) * 255.0f);
}

} // namespace render::gles1
