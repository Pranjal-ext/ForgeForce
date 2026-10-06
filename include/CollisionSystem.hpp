#pragma once

#include <vector>
#include <unordered_map>
#include <utility>
#include <set>
#include <cmath>
#include <algorithm>

#include "Body.hpp"

// ============================================================
// Manifold
// Result of a narrow-phase test.
// ============================================================
struct Manifold {
    bool colliding = false;
    Vector2D normal;          // points from the first body toward the second
    float penetration = 0.0f; // overlap depth along normal
};

// ============================================================
// Narrow-phase tests
// ============================================================
Manifold testCircleCircle(const Body& a, const Body& b);
Manifold testCircleBox(const Body& circle, const Body& box);
Manifold testBoxBox(const Body& a, const Body& b);
Manifold testCirclePolygon(const Body& circle, const Body& polygon); // polygon may be Box or Polygon
Manifold testPolygonPolygon(const Body& a, const Body& b);           // either may be Box or Polygon

// Picks the right test for any pair of shape types; normal always points a -> b
Manifold testCollision(const Body& a, const Body& b);

// ============================================================
// Spatial hash grid (broad phase)
// Each body is inserted into every cell its bounding box covers,
// so large bodies (like a full-width floor) are found from any
// cell they touch.
// ============================================================
struct PairHash {
    size_t operator()(const std::pair<int, int>& cell) const {
        return (static_cast<size_t>(static_cast<unsigned int>(cell.first)) << 32)
             ^ static_cast<size_t>(static_cast<unsigned int>(cell.second));
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

    int cellCoord(float value) const;
};

// ============================================================
// Collision response
// ============================================================

// Normal impulse with restitution, plus Coulomb friction impulse.
// Impacts slower than restitutionThreshold are treated as resting
// contact (no bounce), so stacked/resting bodies settle to zero velocity.
void resolveCollision(Body& a, Body& b, const Vector2D& normal,
                      float restitution, float restitutionThreshold, float friction);

// Pushes overlapping bodies apart by a fraction of the penetration
void correctPosition(Body& a, Body& b, const Vector2D& normal, float penetration);

// Snaps tiny velocities to exactly zero
void applyVelocitySleep(Body& body, float threshold);

// ============================================================
// CollisionSystem
// Runs broad phase, narrow phase and response for one frame.
// ============================================================
class CollisionSystem {
public:
    explicit CollisionSystem(float cellSize, float restitution = 0.5f);

    void step(std::vector<Body>& bodies);

    void setRestitution(float value) { restitution = value; }
    float getRestitution() const { return restitution; }
    // Should exceed gravity * timestep, e.g. 2 * gravity / 60 at 60 FPS
    void setRestitutionThreshold(float value) { restitutionThreshold = value; }
    float getRestitutionThreshold() const { return restitutionThreshold; }
    void setFriction(float value) { friction = value; }
    float getFriction() const { return friction; }
    void setSleepThreshold(float value) { sleepThreshold = value; }
    // More iterations give more stable stacks at a small CPU cost
    void setSolverIterations(int value) { solverIterations = std::max(1, value); }

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
