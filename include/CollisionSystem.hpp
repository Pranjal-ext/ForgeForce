#pragma once

#include <cstddef>
#include <functional>
#include <vector>
#include <unordered_map>
#include <utility>
#include <set>
#include <cmath>
#include <algorithm>

#include "Body.hpp"

// Manifold
// Result of a narrow-phase test.
struct Manifold {
  bool colliding = false;
  Vector2D normal; // points from the first body toward the second
  float penetration = 0.0f; // overlap depth along normal
};

// Narrow-phase tests
Manifold testCircleCircle(const Body& firstBody, const Body& secondBody);
Manifold testCircleBox(const Body& circle, const Body& box);
Manifold testBoxBox(const Body& firstBody, const Body& secondBody);
Manifold testCirclePolygon(const Body& circle, const Body& polygon); // polygon may be Box or Polygon
Manifold testPolygonPolygon(const Body& firstBody, const Body& secondBody); // either may be Box or Polygon

// Picks the right test for any pair of shape types; normal always points first -> second
Manifold testCollision(const Body& firstBody, const Body& secondBody);

// Spatial hash grid (broad phase)
// Each body is inserted into every cell its bounding box covers,
// so large bodies (like a full-width floor) are found from any
// cell they touch.
struct PairHash {
  std::size_t operator()(const std::pair<int, int>& cell) const {
    const std::size_t firstCoordinateHash = std::hash<int>{}(cell.first);
    const std::size_t secondCoordinateHash = std::hash<int>{}(cell.second);
    return firstCoordinateHash ^
      (secondCoordinateHash * static_cast<std::size_t>(0x9e3779b9U));
  }
};

class SpatialGrid {
public:
  explicit SpatialGrid(float cellSize);

  void clear();
  void insert(const std::vector<Body>& bodies);
  std::vector<std::pair<int, int>> getCandidatePairs() const;

private:
  float cellSize;
  std::unordered_map<std::pair<int, int>, std::vector<int>, PairHash> cells;

  int cellCoord(float coordinateValue) const;
};

// Collision response

// Normal impulse with restitution, plus Coulomb friction impulse.
// Impacts slower than restitutionThreshold are treated as resting
// contact (no bounce), so stacked/resting bodies settle to zero velocity.
void resolveCollision(Body& firstBody, Body& secondBody, const Vector2D& normal,
  float restitution, float restitutionThreshold, float friction);

// Pushes overlapping bodies apart by a fraction of the penetration
void correctPosition(Body& firstBody, Body& secondBody, const Vector2D& normal, float penetration);

// Snaps tiny velocities to exactly zero
void applyVelocitySleep(Body& body, float velocityThreshold);

// CollisionSystem
// Runs broad phase, narrow phase and response for one frame.
class CollisionSystem {
public:
  explicit CollisionSystem(float cellSize, float restitution = 0.5f);

  void step(std::vector<Body>& bodies);

  void setRestitution(float restitutionValue) { restitution = restitutionValue; }
  float getRestitution() const { return restitution; }
  // Should exceed gravity * timestep, e.g. 2 * gravity / 60 at 60 FPS
  void setRestitutionThreshold(float minimumImpactSpeed) { restitutionThreshold = minimumImpactSpeed; }
  float getRestitutionThreshold() const { return restitutionThreshold; }
  void setFriction(float frictionCoefficient) { friction = frictionCoefficient; }
  float getFriction() const { return friction; }
  void setSleepThreshold(float sleepSpeedThreshold) { sleepThreshold = sleepSpeedThreshold; }
  // More iterations give more stable stacks at a small CPU cost
  void setSolverIterations(int iterationCount) { solverIterations = std::max(1, iterationCount); }

private:
  struct Contact {
    int a;
    int b;
    Manifold manifold;
  };

  SpatialGrid grid;
  float restitution;
  float restitutionThreshold = 30.0f;
  float friction = 0.3f;
  float sleepThreshold = 0.05f;
  int solverIterations = 8;
  std::vector<Contact> contacts; // reused every frame to avoid reallocations
};
