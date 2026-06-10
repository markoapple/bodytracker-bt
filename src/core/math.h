#pragma once

#include <array>
#include <algorithm>
#include <cmath>

namespace bt {

struct Vec2f {
    float x = 0.0f;
    float y = 0.0f;
};

struct Vec3f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Quatf {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;
};

struct Pose3f {
    Vec3f position{};
    Quatf orientation{};
};

struct Rect2f {
    float x = 0.0f;
    float y = 0.0f;
    float width = 0.0f;
    float height = 0.0f;
};

struct Mat3f {
    std::array<float, 9> m{};
};

struct Mat34f {
    std::array<float, 12> m{};
};

inline float Dot(const Vec3f& a, const Vec3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3f Sub(const Vec3f& a, const Vec3f& b) {
    return Vec3f{a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3f Add(const Vec3f& a, const Vec3f& b) {
    return Vec3f{a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3f Scale(const Vec3f& v, float s) {
    return Vec3f{v.x * s, v.y * s, v.z * s};
}

inline float Lerp(float a, float b, float t) {
    t = std::max(0.0f, std::min(1.0f, t));
    return a + (b - a) * t;
}

inline Vec3f Lerp(const Vec3f& a, const Vec3f& b, float t) {
    t = std::max(0.0f, std::min(1.0f, t));
    return Add(Scale(a, 1.0f - t), Scale(b, t));
}

inline float Length(const Vec3f& v) {
    return std::sqrt(Dot(v, v));
}

inline float Distance(const Vec3f& a, const Vec3f& b) {
    return Length(Sub(a, b));
}

inline float Distance(const Vec2f& a, const Vec2f& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

inline Vec3f Cross(const Vec3f& a, const Vec3f& b) {
    return Vec3f{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

inline Vec3f NormalizeOr(const Vec3f& v, const Vec3f& fallback) {
    const float len = Length(v);
    if (len < 1e-6f || !std::isfinite(len)) {
        return fallback;
    }
    return Scale(v, 1.0f / len);
}

inline Vec3f Normalize(const Vec3f& v) {
    return NormalizeOr(v, Vec3f{});
}

inline Vec3f ProjectPoint(const Mat34f& p, const Vec3f& x) {
    const float u = p.m[0] * x.x + p.m[1] * x.y + p.m[2] * x.z + p.m[3];
    const float v = p.m[4] * x.x + p.m[5] * x.y + p.m[6] * x.z + p.m[7];
    const float w = p.m[8] * x.x + p.m[9] * x.y + p.m[10] * x.z + p.m[11];
    if (std::abs(w) < 1e-6f) {
        return {};
    }
    return Vec3f{u / w, v / w, 1.0f};
}

inline bool IsFinite(const Vec3f& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline Vec3f Rotate(const Quatf& q, const Vec3f& v) {
    const Vec3f u{q.x, q.y, q.z};
    const float s = q.w;
    const Vec3f uv{
        u.y * v.z - u.z * v.y,
        u.z * v.x - u.x * v.z,
        u.x * v.y - u.y * v.x
    };
    const Vec3f uuv{
        u.y * uv.z - u.z * uv.y,
        u.z * uv.x - u.x * uv.z,
        u.x * uv.y - u.y * uv.x
    };
    return Add(v, Add(Scale(uv, 2.0f * s), Scale(uuv, 2.0f)));
}

inline Quatf Normalize(const Quatf& q) {
    const float len = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (len < 1e-6f || !std::isfinite(len)) {
        return {};
    }
    return Quatf{q.x / len, q.y / len, q.z / len, q.w / len};
}

inline float Dot(const Quatf& a, const Quatf& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline Quatf Slerp(Quatf from, Quatf to, float t) {
    t = std::max(0.0f, std::min(1.0f, t));
    from = Normalize(from);
    to = Normalize(to);

    float dot = Dot(from, to);
    if (dot < 0.0f) {
        to = Quatf{-to.x, -to.y, -to.z, -to.w};
        dot = -dot;
    }

    if (dot > 0.9995f) {
        return Normalize(Quatf{
            from.x + t * (to.x - from.x),
            from.y + t * (to.y - from.y),
            from.z + t * (to.z - from.z),
            from.w + t * (to.w - from.w)});
    }

    dot = std::max(-1.0f, std::min(1.0f, dot));
    const float theta = std::acos(dot);
    const float sin_theta = std::sin(theta);
    if (std::abs(sin_theta) < 1e-6f) {
        return to;
    }

    const float a = std::sin((1.0f - t) * theta) / sin_theta;
    const float b = std::sin(t * theta) / sin_theta;
    return Normalize(Quatf{
        a * from.x + b * to.x,
        a * from.y + b * to.y,
        a * from.z + b * to.z,
        a * from.w + b * to.w});
}

inline Quatf Multiply(const Quatf& a, const Quatf& b) {
    return Normalize(Quatf{
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    });
}

inline Quatf Conjugate(const Quatf& q) {
    return Quatf{-q.x, -q.y, -q.z, q.w};
}

inline Quatf QuatFromAngularVelocity(const Vec3f& angular_velocity, float dt_seconds) {
    const float angle = Length(angular_velocity) * dt_seconds;
    if (angle < 1e-6f) {
        return {};
    }
    const Vec3f axis = Normalize(angular_velocity);
    const float half = 0.5f * angle;
    const float s = std::sin(half);
    return Normalize(Quatf{axis.x * s, axis.y * s, axis.z * s, std::cos(half)});
}

inline Vec3f AngularVelocityBetween(const Quatf& from, const Quatf& to, float dt_seconds) {
    if (dt_seconds <= 1e-6f) {
        return {};
    }
    Quatf delta = Multiply(to, Conjugate(from));
    if (delta.w < 0.0f) {
        delta = Quatf{-delta.x, -delta.y, -delta.z, -delta.w};
    }
    const float vector_len = std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
    if (vector_len < 1e-6f) {
        return {};
    }
    const float angle = 2.0f * std::atan2(vector_len, delta.w);
    const Vec3f axis{delta.x / vector_len, delta.y / vector_len, delta.z / vector_len};
    return Scale(axis, angle / dt_seconds);
}

inline Quatf QuatFromBasis(Vec3f right, Vec3f up, Vec3f forward) {
    right = NormalizeOr(right, Vec3f{1.0f, 0.0f, 0.0f});
    up = NormalizeOr(up, Vec3f{0.0f, 1.0f, 0.0f});
    forward = NormalizeOr(forward, Vec3f{0.0f, 0.0f, 1.0f});

    const float m00 = right.x;
    const float m01 = up.x;
    const float m02 = forward.x;
    const float m10 = right.y;
    const float m11 = up.y;
    const float m12 = forward.y;
    const float m20 = right.z;
    const float m21 = up.z;
    const float m22 = forward.z;

    const float trace = m00 + m11 + m22;
    Quatf q;
    if (trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f;
        q.w = 0.25f * s;
        q.x = (m21 - m12) / s;
        q.y = (m02 - m20) / s;
        q.z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        const float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        q.w = (m21 - m12) / s;
        q.x = 0.25f * s;
        q.y = (m01 + m10) / s;
        q.z = (m02 + m20) / s;
    } else if (m11 > m22) {
        const float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        q.w = (m02 - m20) / s;
        q.x = (m01 + m10) / s;
        q.y = 0.25f * s;
        q.z = (m12 + m21) / s;
    } else {
        const float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        q.w = (m10 - m01) / s;
        q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s;
        q.z = 0.25f * s;
    }

    return Normalize(q);
}

inline bool Vec3Finite(const Vec3f& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

template <std::size_t N>
inline bool TrackerSpaceTransformFinite(
    const Vec3f& position,
    const Quatf& rotation,
    float scale,
    const std::array<Vec3f, N>& role_offsets) {

    const float len2 = rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z + rotation.w * rotation.w;
    return Vec3Finite(position) &&
        std::isfinite(rotation.x) && std::isfinite(rotation.y) &&
        std::isfinite(rotation.z) && std::isfinite(rotation.w) &&
        std::isfinite(len2) && len2 >= 1e-12f &&
        std::isfinite(scale) && scale > 0.0f &&
        std::all_of(role_offsets.begin(), role_offsets.end(), Vec3Finite);
}

} // namespace bt
