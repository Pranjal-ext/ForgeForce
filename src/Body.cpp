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

  for (int sideIndex = 0; sideIndex < sides; ++sideIndex) {
    float angle = startAngle + (2.0f * pi * sideIndex) / sides;
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
      Vector2D minimumVertex = localVertices[0];
      for (const Vector2D& vertex : localVertices) {
        minimumVertex.x = std::min(minimumVertex.x, vertex.x);
        minimumVertex.y = std::min(minimumVertex.y, vertex.y);
      }
      return position + minimumVertex;
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
      Vector2D maximumVertex = localVertices[0];
      for (const Vector2D& vertex : localVertices) {
        maximumVertex.x = std::max(maximumVertex.x, vertex.x);
        maximumVertex.y = std::max(maximumVertex.y, vertex.y);
      }
      return position + maximumVertex;
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
      float maximumRadius = 0.0f;
      for (const Vector2D& vertex : localVertices) {
        maximumRadius = std::max(maximumRadius, vertex.length());
      }
      return maximumRadius;
    }
  }
  return 0.0f;
}

std::vector<Vector2D> Body::worldVertices() const {
  std::vector<Vector2D> result;

  if (shapeType == ShapeType::Box) {
    Vector2D minimum = boxMin();
    Vector2D maximum = boxMax();
    result = { { minimum.x, minimum.y }, { maximum.x, minimum.y },
      { maximum.x, maximum.y }, { minimum.x, maximum.y } };
  }
  else if (shapeType == ShapeType::Polygon) {
    result.reserve(localVertices.size());
    for (const Vector2D& vertex : localVertices) result.push_back(position + vertex);
  }

  return result;
}

bool Body::containsPoint(const Vector2D& point) const {
  if (shapeType == ShapeType::Circle) {
    return (point - position).lengthSquared() <= radius * radius;
  }

  if (shapeType == ShapeType::Box) {
    Vector2D minimum = boxMin(), maximum = boxMax();
    return point.x >= minimum.x && point.x <= maximum.x &&
      point.y >= minimum.y && point.y <= maximum.y;
  }

  // Convex polygon: the point is inside if it is on the same side of every edge
  std::vector<Vector2D> vertices = worldVertices();
  if (vertices.size() < 3) return false;

  bool hasPositive = false, hasNegative = false;
  for (std::size_t edgeIndex = 0; edgeIndex < vertices.size(); ++edgeIndex) {
    Vector2D edge = vertices[(edgeIndex + 1) % vertices.size()] - vertices[edgeIndex];
    float edgeSide = edge.cross(point - vertices[edgeIndex]);
    if (edgeSide > 0.0f) hasPositive = true;
    if (edgeSide < 0.0f) hasNegative = true;
    if (hasPositive && hasNegative) return false;
  }
  return true;
}
