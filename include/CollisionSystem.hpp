#pragma once

#include <vector>
#include <unordered_map>
#include <utility>
#include <set>
#include <cmath>
#include <algorithm>

// ============================================================
// Vector2D
// 2D vector with arithmetic, dot product, length and safe
// normalization (returns a zero vector instead of dividing by 0).
// ============================================================
struct Vector2D {
    float x = 0.0f;
    float y = 0.0f;

    Vector2D() = default;
    Vector2D(float x, float y) : x(x), y(y) {}

    Vector2D operator+(const Vector2D& other) const { return { x + other.x, y + other.y }; }
    Vector2D operator-(const Vector2D& other) const { return { x - other.x, y - other.y }; }
    Vector2D operator*(float scalar) const { return { x * scalar, y * scalar }; }
    Vector2D operator-() const { return { -x, -y }; }

    Vector2D& operator+=(const Vector2D& other) { x += other.x; y += other.y; return *this; }
    Vector2D& operator-=(const Vector2D& other) { x -= other.x; y -= other.y; return *this; }

    float dot(const Vector2D& other) const { return x * other.x + y * other.y; }
    float cross(const Vector2D& other) const { return x * other.y - y * other.x; }
    float lengthSquared() const { return x * x + y * y; }
    float length() const { return std::sqrt(lengthSquared()); }

    Vector2D normalized() const {
        float len = length();
        if (len < 1e-6f) return { 0.0f, 0.0f }; // no meaningful direction
        return { x / len, y / len };
    }
};

// ============================================================
// Shape types
// The numeric values are written to scene files, so the order
// must not change (Circle = 0, Box = 1, Polygon = 2).
// ============================================================
enum class ShapeType { Circle, Box, Polygon };

// ============================================================
// Body
// One simulated object. Which shape fields are used depends on
// shapeType:
//   Circle  -> radius
//   Box     -> halfExtents (axis-aligned)
//   Polygon -> localVertices (convex, ordered, relative to position)
// ============================================================
struct Body {
    Vector2D position;
    Vector2D velocity;

    float mass = 1.0f;
    float invMass = 1.0f; // 1 / mass; 0 means static (walls, floor)

    ShapeType shapeType = ShapeType::Circle;

    float radius = 10.0f;
    Vector2D halfExtents{ 10.0f, 10.0f };
    std::vector<Vector2D> localVertices;

    // Sets mass and keeps invMass consistent; mass <= 0 makes the body static
    void setMass(float newMass);

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
    // Should exceed gravity * timestep, e.g. 2 * gravity / 60 at 60 FPS
    void setRestitutionThreshold(float value) { restitutionThreshold = value; }
    void setFriction(float value) { friction = value; }
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
