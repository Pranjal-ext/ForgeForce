#pragma once

#include <vector>

#include "Vector2D.hpp"

// Shape types
// The numeric values are written to scene files, so the order
// must not change (Circle = 0, Box = 1, Polygon = 2).
enum class ShapeType { Circle, Box, Polygon };

// Body
// One simulated object. The shape fields used depend on
// shapeType:
// Circle -> radius
// Box -> halfExtents (axis-aligned)
// Polygon -> localVertices (convex, in order, relative to position)
// Bodies are plain values, so a whole scene can be copied
// (for undo/redo and saving) with a single assignment.
struct Body {
  Vector2D position;
  Vector2D velocity;
  Vector2D force; // total force gathered this step; cleared after integration

  float mass = 1.0f;
  float invMass = 1.0f; // 1 / mass; 0 means static (floor, walls, anchors)

  ShapeType shapeType = ShapeType::Circle;

  float radius = 10.0f;
  Vector2D halfExtents{ 10.0f, 10.0f };
  std::vector<Vector2D> localVertices;

  // Construction helpers
  static Body makeCircle(const Vector2D& position, float radius, float mass = 1.0f);
  static Body makeBox(const Vector2D& position, const Vector2D& halfExtents, float mass = 1.0f);
  static Body makePolygon(const Vector2D& position, const std::vector<Vector2D>& localVertices,
    float mass = 1.0f);
  static Body makeRegularPolygon(const Vector2D& position, int sides, float radius,
    float mass = 1.0f);

  // Vertices of a regular polygon with a flat bottom edge, centred on the origin
  static std::vector<Vector2D> regularPolygonVertices(int sides, float radius);

  // Mass and forces
  // Sets mass and keeps invMass consistent; mass <= 0 makes the body static
  void setMass(float newMass);
  bool isStatic() const { return invMass == 0.0f; }

  void applyForce(const Vector2D& f) { force += f; }
  void clearForce() { force = { 0.0f, 0.0f }; }

  // Geometry
  Vector2D boxMin() const { return position - halfExtents; }
  Vector2D boxMax() const { return position + halfExtents; }

  // World-space bounding box for any shape type
  Vector2D aabbMin() const;
  Vector2D aabbMax() const;

  // Distance from position to the furthest point of the shape
  float boundingRadius() const;

  // World-space corners for Box and Polygon (empty for Circle)
  std::vector<Vector2D> worldVertices() const;

  // True if the point lies inside the shape
  bool containsPoint(const Vector2D& point) const;
};
