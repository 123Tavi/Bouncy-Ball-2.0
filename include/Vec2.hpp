#pragma once

#include <cmath>

namespace bouncy {

struct Vec2 {
    float x{0.0f};
    float y{0.0f};

    constexpr Vec2() = default;
    constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}

    [[nodiscard]] constexpr Vec2 operator+(const Vec2& rhs) const noexcept { return {x + rhs.x, y + rhs.y}; }
    [[nodiscard]] constexpr Vec2 operator-(const Vec2& rhs) const noexcept { return {x - rhs.x, y - rhs.y}; }
    [[nodiscard]] constexpr Vec2 operator-() const noexcept { return {-x, -y}; }
    [[nodiscard]] constexpr Vec2 operator*(float s) const noexcept { return {x * s, y * s}; }
    [[nodiscard]] constexpr Vec2 operator/(float s) const noexcept { return {x / s, y / s}; }

    constexpr Vec2& operator+=(const Vec2& rhs) noexcept {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    constexpr Vec2& operator-=(const Vec2& rhs) noexcept {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    constexpr Vec2& operator*=(float s) noexcept {
        x *= s;
        y *= s;
        return *this;
    }

    [[nodiscard]] constexpr float dot(const Vec2& rhs) const noexcept {
        return x * rhs.x + y * rhs.y;
    }

    [[nodiscard]] constexpr float cross(const Vec2& rhs) const noexcept {
        return x * rhs.y - y * rhs.x;
    }

    [[nodiscard]] constexpr float lengthSq() const noexcept {
        return x * x + y * y;
    }

    [[nodiscard]] float length() const noexcept {
        return std::sqrt(lengthSq());
    }

    [[nodiscard]] Vec2 normalized(Vec2 fallback = {0.0f, 1.0f}) const noexcept {
        const float lenSq = lengthSq();
        if (lenSq <= 1e-12f) {
            return fallback;
        }
        const float inv = 1.0f / std::sqrt(lenSq);
        return {x * inv, y * inv};
    }

    // Counter-clockwise 90-degree perpendicular vector
    [[nodiscard]] constexpr Vec2 perp() const noexcept {
        return {-y, x};
    }
};

[[nodiscard]] inline constexpr Vec2 operator*(float s, const Vec2& v) noexcept {
    return {v.x * s, v.y * s};
}

} // namespace bouncy
