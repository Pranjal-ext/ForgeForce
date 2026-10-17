#include "Body.hpp"

#include <algorithm>
#include <cmath>

// Construction helpers

Body Body::makeCircle(const Vector2D& position, float radius, float mass) {
  Body body;
  body.shapeType = ShapeType::Circle;
  body.position = position;
  body.radius = radius;
  body.setMass(mass);
  return body;
}

Body Body::makeBox(const Vector2D& position, const Vector2D& halfExtents, float mass) {
  Body body;
  body.shapeType = ShapeType::Box;
  body.position = position;
  body.halfExtents = halfExtents;
  body.setMass(mass);
  return body;
}

Body Body::makePolygon(const Vector2D& position, const std::vector<Vector2D>& localVertices,
  float mass) {
  Body body;
  body.shapeType = ShapeType::Polygon;
  body.position = position;
  body.localVertices = localVertices;
  body.setMass(mass);
  return body;
}

Body Body::makeRegularPolygon(const Vector2D& position, int sides, float radius, float mass) {
  return makePolygon(position, regularPolygonVertices(sides, radius), mass);
}

std::vector<Vector2D> Body::regularPolygonVertices(int sides, float radius) {
  const float pi = 3.14159265f;

  std::vector<Vector2D> vertices;
  if (sides < 3) return vertices;
  vertices.reserve(sides);

  // Screen y points down, so pi/2 is straight down. Starting half a side
  // past it makes the bottom edge horizontal (squares axis-aligned,
  // triangles pointing up).
  float startAngle = pi / 2.0f + pi / sides;

  for (int i = 0; i < sides; ++i) {
    float angle = startAngle + (2.0f * pi * i) / sides;
    vertices.push_back({ std::cos(angle) * radius, std::sin(angle) * radius });
  }
  return vertices;
}

// Mass

void Body::setMass(float newMass) {
  if (newMass > 0.0f) {
    mass = newMass;
    invMass = 1.0f / newMass;
  } else {
    mass = 0.0f;
    invMass = 0.0f;
  }
}

// Geometry

Vector2D Body::aabbMin() const {
  switch (shapeType) {
    case ShapeType::Circle:
      return { position.x - radius, position.y - radius };
    case ShapeType::Box:
      return boxMin();
    case ShapeType::Polygon: {
      if (localVertices.empty()) return position;
      Vector2D lo = localVertices[0];
      for (const Vector2D& v : localVertices) {
        lo.x = std::min(lo.x, v.x);
        lo.y = std::min(lo.y, v.y);
      }
      return position + lo;
    }
  }
  return position;
}

Vector2D Body::aabbMax() const {
  switch (shapeType) {
    case ShapeType::Circle:
      return { position.x + radius, position.y + radius };
    case ShapeType::Box:
      return boxMax();
    case ShapeType::Polygon: {
      if (localVertices.empty()) return position;
      Vector2D hi = localVertices[0];
      for (const Vector2D& v : localVertices) {
        hi.x = std::max(hi.x, v.x);
        hi.y = std::max(hi.y, v.y);
      }
      return position + hi;
    }
  }
  return position;
}

float Body::boundingRadius() const {
  switch (shapeType) {
    case ShapeType::Circle:
      return radius;
    case ShapeType::Box:
      return halfExtents.length();
    case ShapeType::Polygon: {
      float r = 0.0f;
      for (const Vector2D& v : localVertices) r = std::max(r, v.length());
      return r;
    }
  }
  return 0.0f;
}

std::vector<Vector2D> Body::worldVertices() const {
  std::vector<Vector2D> result;

  if (shapeType == ShapeType::Box) {
    Vector2D lo = boxMin();
    Vector2D hi = boxMax();
    result = { { lo.x, lo.y }, { hi.x, lo.y }, { hi.x, hi.y }, { lo.x, hi.y } };
  }
  else if (shapeType == ShapeType::Polygon) {
    result.reserve(localVertices.size());
    for (const Vector2D& v : localVertices) result.push_back(position + v);
  }

  return result;
}

bool Body::containsPoint(const Vector2D& point) const {
  if (shapeType == ShapeType::Circle) {
    return (point - position).lengthSquared() <= radius * radius;
  }

  if (shapeType == ShapeType::Box) {
    Vector2D lo = boxMin(), hi = boxMax();
    return point.x >= lo.x && point.x <= hi.x && point.y >= lo.y && point.y <= hi.y;
  }

  // Convex polygon: the point is inside if it is on the same side of every edge
  std::vector<Vector2D> verts = worldVertices();
  if (verts.size() < 3) return false;

  bool hasPositive = false, hasNegative = false;
  for (std::size_t i = 0; i < verts.size(); ++i) {
    Vector2D edge = verts[(i + 1) % verts.size()] - verts[i];
    float side = edge.cross(point - verts[i]);
    if (side > 0.0f) hasPositive = true;
    if (side < 0.0f) hasNegative = true;
    if (hasPositive && hasNegative) return false;
  }
  return true;
}
