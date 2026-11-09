#include "CollisionSystem.hpp"

#include <algorithm>
#include <limits>

// Narrow-phase tests

Manifold testCircleCircle(const Body& firstBody, const Body& secondBody) {
  Manifold manifold;

  Vector2D separation = secondBody.position - firstBody.position;
  float distanceSquared = separation.lengthSquared();
  float radiusSum = firstBody.radius + secondBody.radius;

  if (distanceSquared > radiusSum * radiusSum) return manifold;

  float distance = std::sqrt(distanceSquared);

  manifold.colliding = true;
  // Identical centres have no direction, so pick one
  manifold.normal = (distance > 1e-6f) ? separation * (1.0f / distance) : Vector2D{ 1.0f, 0.0f };
  manifold.penetration = radiusSum - distance;
  return manifold;
}

Manifold testCircleBox(const Body& circle, const Body& box) {
  Manifold manifold;

  Vector2D boxMinimum = box.boxMin();
  Vector2D boxMaximum = box.boxMax();

  // Closest point on the box to the circle's centre
  Vector2D closest{
    std::clamp(circle.position.x, boxMinimum.x, boxMaximum.x),
    std::clamp(circle.position.y, boxMinimum.y, boxMaximum.y)
  };

  Vector2D separation = closest - circle.position;
  float distanceSquared = separation.lengthSquared();

  if (distanceSquared > circle.radius * circle.radius) return manifold;

  float distance = std::sqrt(distanceSquared);
  manifold.colliding = true;

  if (distance > 1e-6f) {
    manifold.normal = separation * (1.0f / distance);
    manifold.penetration = circle.radius - distance;
    return manifold;
  }

  // Centre is inside the box: push out through the nearest face
  float distanceToLeft = circle.position.x - boxMinimum.x;
  float distanceToRight = boxMaximum.x - circle.position.x;
  float distanceToTop = circle.position.y - boxMinimum.y;
  float distanceToBottom = boxMaximum.y - circle.position.y;

  float nearestFaceDistance = distanceToLeft;
  Vector2D outward{ -1.0f, 0.0f };
  if (distanceToRight < nearestFaceDistance) { nearestFaceDistance = distanceToRight; outward = { 1.0f, 0.0f }; }
  if (distanceToTop < nearestFaceDistance) { nearestFaceDistance = distanceToTop; outward = { 0.0f, -1.0f }; }
  if (distanceToBottom < nearestFaceDistance) { nearestFaceDistance = distanceToBottom; outward = { 0.0f, 1.0f }; }

  // Normal points circle -> box, i.e. opposite to the face the circle exits through
  manifold.normal = -outward;
  manifold.penetration = circle.radius + nearestFaceDistance;
  return manifold;
}

Manifold testBoxBox(const Body& firstBody, const Body& secondBody) {
  Manifold manifold;

  Vector2D firstMinimum = firstBody.boxMin(), firstMaximum = firstBody.boxMax();
  Vector2D secondMinimum = secondBody.boxMin(), secondMaximum = secondBody.boxMax();

  float overlapX = std::min(firstMaximum.x, secondMaximum.x) - std::max(firstMinimum.x, secondMinimum.x);
  float overlapY = std::min(firstMaximum.y, secondMaximum.y) - std::max(firstMinimum.y, secondMinimum.y);

  if (overlapX <= 0.0f || overlapY <= 0.0f) return manifold;

  manifold.colliding = true;

  // Separate along the axis of least overlap
  if (overlapX < overlapY) {
    manifold.normal = (firstBody.position.x < secondBody.position.x) ? Vector2D{ 1.0f, 0.0f } : Vector2D{ -1.0f, 0.0f };
    manifold.penetration = overlapX;
  } else {
    manifold.normal = (firstBody.position.y < secondBody.position.y) ? Vector2D{ 0.0f, 1.0f } : Vector2D{ 0.0f, -1.0f };
    manifold.penetration = overlapY;
  }
  return manifold;
}

namespace {

// Projects vertices onto an axis and returns the covered interval
void projectVertices(const std::vector<Vector2D>& vertices, const Vector2D& axis,
  float& projectedMinimum, float& projectedMaximum) {
  projectedMinimum = projectedMaximum = vertices[0].dot(axis);
  for (const Vector2D& vertex : vertices) {
    float projection = vertex.dot(axis);
    projectedMinimum = std::min(projectedMinimum, projection);
    projectedMaximum = std::max(projectedMaximum, projection);
  }
}

// Unit normal of edge i of a polygon (zero for degenerate edges)
Vector2D edgeNormal(const std::vector<Vector2D>& vertices, std::size_t edgeIndex) {
  Vector2D edge = vertices[(edgeIndex + 1) % vertices.size()] - vertices[edgeIndex];
  return Vector2D{ -edge.y, edge.x }.normalized();
}

} // namespace

Manifold testPolygonPolygon(const Body& firstBody, const Body& secondBody) {
  Manifold manifold;

  std::vector<Vector2D> firstVertices = firstBody.worldVertices();
  std::vector<Vector2D> secondVertices = secondBody.worldVertices();
  if (firstVertices.size() < 3 || secondVertices.size() < 3) return manifold;

  float minimumOverlap = std::numeric_limits<float>::max();
  Vector2D bestAxis;

  // Separating Axis Theorem: test every edge normal of both shapes.
  // A gap on any axis means the shapes are not touching.
  auto testAxesOf = [&](const std::vector<Vector2D>& vertices) {
    for (std::size_t edgeIndex = 0; edgeIndex < vertices.size(); ++edgeIndex) {
      Vector2D axis = edgeNormal(vertices, edgeIndex);
      if (axis.lengthSquared() < 0.5f) continue; // degenerate edge

      float firstMinimum, firstMaximum, secondMinimum, secondMaximum;
      projectVertices(firstVertices, axis, firstMinimum, firstMaximum);
      projectVertices(secondVertices, axis, secondMinimum, secondMaximum);

      float overlap = std::min(firstMaximum, secondMaximum) -
        std::max(firstMinimum, secondMinimum);
      if (overlap <= 0.0f) return false;

      if (overlap < minimumOverlap) {
        minimumOverlap = overlap;
        bestAxis = axis;
      }
    }
    return true;
  };

  if (!testAxesOf(firstVertices) || !testAxesOf(secondVertices)) return manifold;

  // Make the normal point from the first body toward the second
  if (bestAxis.dot(secondBody.position - firstBody.position) < 0.0f) bestAxis = -bestAxis;

  manifold.colliding = true;
  manifold.normal = bestAxis;
  manifold.penetration = minimumOverlap;
  return manifold;
}

Manifold testCirclePolygon(const Body& circle, const Body& polygon) {
  Manifold manifold;

  std::vector<Vector2D> polygonVertices = polygon.worldVertices();
  if (polygonVertices.size() < 3) return manifold;

  float minimumOverlap = std::numeric_limits<float>::max();
  Vector2D bestAxis;

  auto testAxis = [&](const Vector2D& axis) {
    float polygonMinimum, polygonMaximum;
    projectVertices(polygonVertices, axis, polygonMinimum, polygonMaximum);

    float circleCenterProjection = circle.position.dot(axis);
    float circleMinimum = circleCenterProjection - circle.radius;
    float circleMaximum = circleCenterProjection + circle.radius;

    float overlap = std::min(polygonMaximum, circleMaximum) -
      std::max(polygonMinimum, circleMinimum);
    if (overlap <= 0.0f) return false;

    if (overlap < minimumOverlap) {
      minimumOverlap = overlap;
      bestAxis = axis;
    }
    return true;
  };

  // Polygon edge normals
  for (std::size_t edgeIndex = 0; edgeIndex < polygonVertices.size(); ++edgeIndex) {
    Vector2D axis = edgeNormal(polygonVertices, edgeIndex);
    if (axis.lengthSquared() < 0.5f) continue;
    if (!testAxis(axis)) return manifold;
  }

  // Axis from the circle's centre to the nearest polygon vertex (catches corner contacts)
  Vector2D nearestVertex = polygonVertices[0];
  for (const Vector2D& vertex : polygonVertices) {
    if ((vertex - circle.position).lengthSquared() < (nearestVertex - circle.position).lengthSquared()) {
      nearestVertex = vertex;
    }
  }
  Vector2D nearestCornerAxis = (nearestVertex - circle.position).normalized();
  if (nearestCornerAxis.lengthSquared() > 0.5f && !testAxis(nearestCornerAxis)) return manifold;

  // Normal points circle -> polygon
  if (bestAxis.dot(polygon.position - circle.position) < 0.0f) bestAxis = -bestAxis;

  manifold.colliding = true;
  manifold.normal = bestAxis;
  manifold.penetration = minimumOverlap;
  return manifold;
}

Manifold testCollision(const Body& firstBody, const Body& secondBody) {
  const bool firstIsCircle = (firstBody.shapeType == ShapeType::Circle);
  const bool secondIsCircle = (secondBody.shapeType == ShapeType::Circle);
  const bool firstIsBox = (firstBody.shapeType == ShapeType::Box);
  const bool secondIsBox = (secondBody.shapeType == ShapeType::Box);

  if (firstIsCircle && secondIsCircle) return testCircleCircle(firstBody, secondBody);
  if (firstIsBox && secondIsBox) return testBoxBox(firstBody, secondBody);
  if (firstIsCircle && secondIsBox) return testCircleBox(firstBody, secondBody);

  if (firstIsBox && secondIsCircle) {
    Manifold manifold = testCircleBox(secondBody, firstBody);
    manifold.normal = -manifold.normal; // flip so it points first -> second
    return manifold;
  }

  if (firstIsCircle) return testCirclePolygon(firstBody, secondBody);

  if (secondIsCircle) {
    Manifold manifold = testCirclePolygon(secondBody, firstBody);
    manifold.normal = -manifold.normal;
    return manifold;
  }

  // Box-Polygon or Polygon-Polygon
  return testPolygonPolygon(firstBody, secondBody);
}

// Collision response

void resolveCollision(Body& firstBody, Body& secondBody, const Vector2D& normal,
  float restitution, float restitutionThreshold, float friction) {
  Vector2D relativeVelocity = secondBody.velocity - firstBody.velocity;
  float relativeSpeedAlongNormal = relativeVelocity.dot(normal);

  // Already separating
  if (relativeSpeedAlongNormal > 0.0f) return;

  float inverseMassSum = firstBody.invMass + secondBody.invMass;
  if (inverseMassSum <= 0.0f) return; // both static

  // Slow impacts are resting contact: no bounce
  float effectiveRestitution =
    (-relativeSpeedAlongNormal < restitutionThreshold) ? 0.0f : restitution;

  // Normal impulse: j = -(1 + e) * vRel·n / (1/mA + 1/mB)
  float normalImpulseMagnitude =
    -(1.0f + effectiveRestitution) * relativeSpeedAlongNormal / inverseMassSum;

  Vector2D normalImpulse = normal * normalImpulseMagnitude;
  firstBody.velocity -= normalImpulse * firstBody.invMass;
  secondBody.velocity += normalImpulse * secondBody.invMass;

  // Friction impulse along the contact tangent, capped by Coulomb's law (|jt| <= mu * j)
  relativeVelocity = secondBody.velocity - firstBody.velocity;
  Vector2D tangent = relativeVelocity - normal * relativeVelocity.dot(normal);
  float tangentLength = tangent.length();
  if (tangentLength < 1e-6f) return;
  tangent = tangent * (1.0f / tangentLength);

  float tangentImpulseMagnitude = -relativeVelocity.dot(tangent) / inverseMassSum;
  float maxFrictionImpulse = friction * normalImpulseMagnitude;
  tangentImpulseMagnitude =
    std::clamp(tangentImpulseMagnitude, -maxFrictionImpulse, maxFrictionImpulse);

  Vector2D frictionImpulse = tangent * tangentImpulseMagnitude;
  firstBody.velocity -= frictionImpulse * firstBody.invMass;
  secondBody.velocity += frictionImpulse * secondBody.invMass;
}

void correctPosition(Body& firstBody, Body& secondBody, const Vector2D& normal, float penetration) {
  const float correctionPercent = 0.2f; // correct 20% of the overlap per frame
  const float penetrationSlop = 0.01f; // ignore negligible overlaps

  float inverseMassSum = firstBody.invMass + secondBody.invMass;
  if (inverseMassSum <= 0.0f) return;

  float correctionMagnitude =
    std::max(penetration - penetrationSlop, 0.0f) / inverseMassSum * correctionPercent;
  Vector2D correction = normal * correctionMagnitude;

  firstBody.position -= correction * firstBody.invMass;
  secondBody.position += correction * secondBody.invMass;
}

void applyVelocitySleep(Body& body, float velocityThreshold) {
  if (body.velocity.length() < velocityThreshold) {
    body.velocity = { 0.0f, 0.0f };
  }
}

// SpatialGrid

SpatialGrid::SpatialGrid(float cellSize) : cellSize(cellSize) {}

int SpatialGrid::cellCoord(float coordinateValue) const {
  // floor (not truncation) so negative coordinates map to the correct cell
  return static_cast<int>(std::floor(coordinateValue / cellSize));
}

void SpatialGrid::clear() {
  cells.clear();
}

void SpatialGrid::insert(const std::vector<Body>& bodies) {
  for (int bodyIndex = 0; bodyIndex < static_cast<int>(bodies.size()); ++bodyIndex) {
    Vector2D boundsMinimum = bodies[bodyIndex].aabbMin();
    Vector2D boundsMaximum = bodies[bodyIndex].aabbMax();

    // Skip bodies with invalid positions instead of corrupting the grid
    if (!std::isfinite(boundsMinimum.x) || !std::isfinite(boundsMinimum.y) ||
      !std::isfinite(boundsMaximum.x) || !std::isfinite(boundsMaximum.y)) {
      continue;
    }

    int firstCellX = cellCoord(boundsMinimum.x), lastCellX = cellCoord(boundsMaximum.x);
    int firstCellY = cellCoord(boundsMinimum.y), lastCellY = cellCoord(boundsMaximum.y);

    for (int cellX = firstCellX; cellX <= lastCellX; ++cellX) {
      for (int cellY = firstCellY; cellY <= lastCellY; ++cellY) {
        cells[{ cellX, cellY }].push_back(bodyIndex);
      }
    }
  }
}

std::vector<std::pair<int, int>> SpatialGrid::getCandidatePairs() const {
  // Bodies sharing several cells would be paired more than once; the set removes duplicates
  std::set<std::pair<int, int>> uniquePairs;

  for (const auto& cellEntry : cells) {
    const std::vector<int>& bodyIndices = cellEntry.second;
    for (std::size_t firstIndex = 0; firstIndex < bodyIndices.size(); ++firstIndex) {
      for (std::size_t secondIndex = firstIndex + 1; secondIndex < bodyIndices.size(); ++secondIndex) {
        int firstBodyIndex = bodyIndices[firstIndex], secondBodyIndex = bodyIndices[secondIndex];
        uniquePairs.insert({ std::min(firstBodyIndex, secondBodyIndex),
          std::max(firstBodyIndex, secondBodyIndex) });
      }
    }
  }

  return std::vector<std::pair<int, int>>(uniquePairs.begin(), uniquePairs.end());
}

// CollisionSystem

CollisionSystem::CollisionSystem(float cellSize, float restitution)
  : grid(cellSize), restitution(restitution) {}

void CollisionSystem::step(std::vector<Body>& bodies) {
  // Broad phase
  grid.clear();
  grid.insert(bodies);
  std::vector<std::pair<int, int>> candidates = grid.getCandidatePairs();

  // Narrow phase: gather every touching pair
  contacts.clear();
  for (const auto& pair : candidates) {
    const Body& firstBody = bodies[pair.first];
    const Body& secondBody = bodies[pair.second];

    if (firstBody.invMass <= 0.0f && secondBody.invMass <= 0.0f) continue; // two static bodies

    Manifold manifold = testCollision(firstBody, secondBody);
    if (manifold.colliding) contacts.push_back({ pair.first, pair.second, manifold });
  }

  // Velocity solver: several passes let impulses propagate through stacks
  for (int iteration = 0; iteration < solverIterations; ++iteration) {
    for (const Contact& contact : contacts) {
      resolveCollision(bodies[contact.a], bodies[contact.b], contact.manifold.normal,
        restitution, restitutionThreshold, friction);
    }
  }

  // Position correction: once per contact
  for (const Contact& contact : contacts) {
    correctPosition(bodies[contact.a], bodies[contact.b], contact.manifold.normal,
      contact.manifold.penetration);
  }

  // Settle near-stationary bodies
  for (Body& body : bodies) {
    applyVelocitySleep(body, sleepThreshold);
  }
}
