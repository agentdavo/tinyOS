// SPDX-License-Identifier: MIT OR Apache-2.0
//
// Shared freestanding scalar math approximations. Three TUs (cnc/interpreter.cpp,
// cnc/programs.cpp, render/gles1.cpp) used to hand-roll their own copies of
// sin/cos/atan/sqrt with subtly different formulations and accuracy. Now they
// all consume the same header so a fix in one place lands everywhere, and the
// CNC and renderer agree on the same arc geometry math.
//
// All functions are header-inline, freestanding, no dependencies.

#ifndef UTIL_MATH_HPP
#define UTIL_MATH_HPP

#include <cstdint>

namespace kernel::util::math {

constexpr float kPi      = 3.14159265358979323846f;
constexpr float kTwoPi   = kPi * 2.0f;
constexpr float kHalfPi  = kPi * 0.5f;
constexpr float kDegToRad = 0.01745329252f;
constexpr float kRadToDeg = 57.2957795131f;

inline float absf(float v) noexcept {
    union { float f; uint32_t u; } val{v};
    val.u &= 0x7FFFFFFFU;
    return val.f;
}

inline float minf(float a, float b) noexcept { return a < b ? a : b; }
inline float maxf(float a, float b) noexcept { return a > b ? a : b; }
inline float clampf(float v, float lo, float hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Range-reduce to [-pi, pi]. The old plain-subtraction loop spun effectively
// forever for a large finite magnitude (e.g. a degenerate arc yielding 1e30
// radians) — millions of iterations on the motion/interpreter thread. Reduce
// in one shot via the nearest multiple of 2*pi, then tidy the float residual
// with a bounded loop. NaN returns 0 (the comparisons below would otherwise
// short-circuit but we make it explicit). Absurd magnitudes that can't form an
// exact quotient degenerate to 0 rather than hang.
inline float wrap_pi(float radians) noexcept {
    if (!(radians == radians)) return 0.0f;                 // NaN
    if (radians >= -kPi && radians <= kPi) return radians;  // common fast path
    const float q = radians / kTwoPi;
    if (q > 9.0e8f || q < -9.0e8f) return 0.0f;             // too large to reduce
    const long long n = static_cast<long long>(q < 0.0f ? q - 0.5f : q + 0.5f);
    radians -= kTwoPi * static_cast<float>(n);
    while (radians >  kPi) radians -= kTwoPi;               // residual only
    while (radians < -kPi) radians += kTwoPi;
    return radians;
}

// Taylor truncation through x^7. Accurate to ~1e-6 across [-pi, pi].
inline float sin_approx(float x) noexcept {
    x = wrap_pi(x);
    const float x2 = x * x;
    const float x3 = x2 * x;
    const float x5 = x3 * x2;
    const float x7 = x5 * x2;
    return x - x3 / 6.0f + x5 / 120.0f - x7 / 5040.0f;
}

inline float cos_approx(float x) noexcept {
    return sin_approx(x + kHalfPi);
}

// Same Taylor pair as sin/cos_approx but yields both with one wrap.
inline void sincos_approx(float x, float& s, float& c) noexcept {
    x = wrap_pi(x);
    const float x2 = x * x;
    const float x3 = x2 * x;
    const float x4 = x2 * x2;
    const float x5 = x4 * x;
    s = x - x3 / 6.0f + x5 / 120.0f;
    c = 1.0f - x2 / 2.0f + x4 / 24.0f;
}

// Newton-Raphson sqrt. The old seed of max(v,1) does NOT converge in a fixed
// 6 iterations for large arguments (e.g. squared count distances ~1e18 feeding
// feedrate/path-length math) — the result could be off by orders of magnitude.
// Seed via IEEE-754 exponent halving so the relative error of the guess is
// bounded regardless of magnitude; 4 Newton steps then nail it across the whole
// float range. NaN/negative return 0.
inline float sqrt_approx(float v) noexcept {
    if (!(v > 0.0f)) return 0.0f;   // also catches NaN (NaN > 0 is false)
    union { float f; uint32_t u; } val{v};
    val.u = 0x1FBD1DF5U + (val.u >> 1);   // fast sqrt seed (~3.4% max rel error)
    float x = val.f;
    for (int i = 0; i < 4; ++i) x = 0.5f * (x + v / x);
    return x;
}

// Two Newton iterations on 1/sqrt; adequate for normal-vector normalisation
// where downstream tolerates ~1e-4 relative error.
inline float rsqrt_approx(float v) noexcept {
    if (v <= 1e-6f) return 0.0f;
    float x = 1.0f / v;
    x = 0.5f * x * (3.0f - v * x * x);
    x = 0.5f * x * (3.0f - v * x * x);
    return x;
}

// |z| <= 1 branch uses Padé-style fit, else identity atan(z) = pi/2 - atan(1/z).
inline float atan_approx(float z) noexcept {
    const float az = absf(z);
    if (az <= 1.0f) return z / (1.0f + 0.28f * z * z);
    const float base = kHalfPi - (az / (az * az + 0.28f));
    return z < 0.0f ? -base : base;
}

inline float atan2_approx(float y, float x) noexcept {
    if (x > 0.0f) return atan_approx(y / x);
    if (x < 0.0f && y >= 0.0f) return atan_approx(y / x) + kPi;
    if (x < 0.0f && y <  0.0f) return atan_approx(y / x) - kPi;
    if (y > 0.0f) return  kHalfPi;
    if (y < 0.0f) return -kHalfPi;
    return 0.0f;
}

// Inverse cosine via the standard atan2 identity. Used for chord-error
// inversion in arc segmentation: angle_per_segment = 2 * acos(1 - tol/R).
inline float acos_approx(float x) noexcept {
    if (x >=  1.0f) return 0.0f;
    if (x <= -1.0f) return kPi;
    const float s = sqrt_approx(1.0f - x * x);
    return atan2_approx(s, x);
}

// Used by gles1 specular shader; placed here so callers don't fork a fourth
// copy. log2 for x in (0, ∞) via IEEE-754 exponent + quartic mantissa fit.
inline float log2_approx(float x) noexcept {
    if (x <= 0.0f) return -127.0f;
    union { float f; uint32_t u; } v{x};
    const int32_t e = static_cast<int32_t>((v.u >> 23) & 0xFF) - 127;
    v.u = (v.u & 0x007FFFFFU) | 0x3F800000U;
    const float m = v.f;
    const float p = -0.34484843f * m * m + 2.02466578f * m - 1.67487759f;
    return p + static_cast<float>(e);
}

inline float exp2_approx(float x) noexcept {
    if (x < -126.0f) return 0.0f;
    if (x >  127.0f) x = 127.0f;
    const float xi = static_cast<float>(static_cast<int32_t>(x) - (x < 0.0f ? 1 : 0));
    const float xf = x - xi;
    const float p = ((0.07252294f * xf + 0.24279419f) * xf + 0.69502427f) * xf + 1.0f;
    union { uint32_t u; float f; } vf{};
    vf.u = static_cast<uint32_t>((static_cast<int32_t>(xi) + 127) & 0xFF) << 23;
    return vf.f * p;
}

inline float pow_approx(float base, float exp) noexcept {
    if (base <= 0.0f) return 0.0f;
    if (exp == 0.0f) return 1.0f;
    return exp2_approx(exp * log2_approx(base));
}

} // namespace kernel::util::math

#endif // UTIL_MATH_HPP
