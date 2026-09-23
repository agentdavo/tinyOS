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

// Fold a wrapped angle into [-pi/2, pi/2], where the Taylor series below
// converge fast: sin(pi - x) = sin(x), cos(pi - x) = -cos(x). Returns the
// sign cos must be multiplied by.
inline float fold_half_pi(float& x) noexcept {
    x = wrap_pi(x);
    if (x >  kHalfPi) { x =  kPi - x; return -1.0f; }
    if (x < -kHalfPi) { x = -kPi - x; return -1.0f; }
    return 1.0f;
}

// Series on |x| <= pi/2 after folding: sin through x^11, cos through x^12.
// Max error ~7e-7 (float rounding of the range reduction) over +-10 rad. The
// old version Taylor-expanded over the whole [-pi, pi] and stopped at x^7,
// so it was off by 0.075 near +-pi (and sincos_approx, stopping at x^5,
// by up to 1.1) — enough to put G-code arc points millimetres off the arc.
inline float sin_poly(float x) noexcept {
    const float x2 = x * x;
    return x * (1.0f + x2 * (-1.0f / 6.0f + x2 * (1.0f / 120.0f + x2 * (-1.0f / 5040.0f +
               x2 * (1.0f / 362880.0f + x2 * (-1.0f / 39916800.0f))))));
}
inline float cos_poly(float x) noexcept {
    const float x2 = x * x;
    return 1.0f + x2 * (-0.5f + x2 * (1.0f / 24.0f + x2 * (-1.0f / 720.0f + x2 * (1.0f / 40320.0f +
               x2 * (-1.0f / 3628800.0f + x2 * (1.0f / 479001600.0f))))));
}

inline float sin_approx(float x) noexcept {
    (void)fold_half_pi(x);
    return sin_poly(x);
}

inline float cos_approx(float x) noexcept {
    const float sign = fold_half_pi(x);
    return sign * cos_poly(x);
}

// Both at once with a single range reduction.
inline void sincos_approx(float x, float& s, float& c) noexcept {
    const float sign = fold_half_pi(x);
    s = sin_poly(x);
    c = sign * cos_poly(x);
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

// 1/sqrt(v): IEEE-754 exponent-halving seed (~3.4% error) plus three Newton
// steps, good to float precision for any magnitude. The old 1/v seed only
// converged for v near 1 — normalising a 10 mm vector gave length 0.22, which
// skewed make_look_at's camera basis once the scene moved to millimetres.
inline float rsqrt_approx(float v) noexcept {
    if (!(v > 1e-30f)) return 0.0f;   // also catches NaN and negatives
    union { float f; uint32_t u; } val{v};
    val.u = 0x5F3759DFU - (val.u >> 1);
    float x = val.f;
    for (int i = 0; i < 3; ++i) x = x * (1.5f - 0.5f * v * x * x);
    return x;
}

// Abramowitz & Stegun 4.4.49 on |z| <= 1 (max error ~2e-8; ~1e-7 in float),
// with atan(z) = pi/2 - atan(1/z) outside. The old z/(1 + 0.28 z^2) fit was
// only good to ~5e-3 rad (0.28 deg), which skewed arc start/end angles.
inline float atan_unit(float x) noexcept {
    const float x2 = x * x;
    return x * (1.0f + x2 * (-0.3333314528f + x2 * (0.1999355085f + x2 * (-0.1420889944f +
               x2 * (0.1065626393f + x2 * (-0.0752896400f + x2 * (0.0429096138f +
               x2 * (-0.0161657367f + x2 * 0.0028662257f))))))));
}
inline float atan_approx(float z) noexcept {
    const float az = absf(z);
    const float r = az <= 1.0f ? atan_unit(az) : kHalfPi - atan_unit(1.0f / az);
    return z < 0.0f ? -r : r;
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
