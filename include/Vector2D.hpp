#pragma once

#include <cmath>

// Vector2D
// A 2D vector used for positions, velocities, forces and
// directions. Provides arithmetic, dot and cross products,
// length, and safe normalization.
struct Vector2D {
  float x = 0.0f;
  float y = 0.0f;

  Vector2D() = default;
  Vector2D(float xCoordinate, float yCoordinate) : x(xCoordinate), y(yCoordinate) {}

  Vector2D operator+(const Vector2D& otherVector) const {
    return { x + otherVector.x, y + otherVector.y };
  }
  Vector2D operator-(const Vector2D& otherVector) const {
    return { x - otherVector.x, y - otherVector.y };
  }
  Vector2D operator*(float scalar) const { return { x * scalar, y * scalar }; }
  Vector2D operator-() const { return { -x, -y }; }

  Vector2D& operator+=(const Vector2D& otherVector) {
    x += otherVector.x;
    y += otherVector.y;
    return *this;
  }
  Vector2D& operator-=(const Vector2D& otherVector) {
    x -= otherVector.x;
    y -= otherVector.y;
    return *this;
  }

  // Projection of one vector onto another (scaled by both lengths)
  float dot(const Vector2D& otherVector) const { return x * otherVector.x + y * otherVector.y; }

  // Z component of the 3D cross product; its sign tells which side a point lies on
  float cross(const Vector2D& otherVector) const {
    return x * otherVector.y - y * otherVector.x;
  }

  float lengthSquared() const { return x * x + y * y; }
  float length() const { return std::sqrt(lengthSquared()); }

  // Unit vector in the same direction; a zero vector stays zero instead of dividing by 0
  Vector2D normalized() const {
    float vectorLength = length();
    if (vectorLength < 1e-6f) return { 0.0f, 0.0f };
    return { x / vectorLength, y / vectorLength };
  }
};
