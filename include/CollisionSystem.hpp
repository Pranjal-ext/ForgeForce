#pragma once

#include <vector>
#include <unordered_map>
#include <utility>
#include <set>
#include <cmath>
#include <algorithm>


// This struct is used to represent a 2D vector using x and y values.
// It provides basic operations like adding, subtracting and scaling vectors,
// along with finding their length, dot product and unit vector.
// It also handles very small vectors safely while normalizing them.

struct Vector2D {
    float x= 0.0f;
    float y= 0.0f;
    Vector2D operator-(const Vector2D& other) const{ return {x- other.x,y-other.y};}
    Vector2D operator+(const Vector2D& other) const{ return {x+ other.x,y+other.y};}
    Vector2D operator*(float scalar) const { return { x * scalar, y * scalar }; }
    Vector2D& operator-=(const Vector2D& other) { x -= other.x; y -= other.y; return *this; }
    Vector2D& operator+=(const Vector2D& other) { x += other.x; y += other.y; return *this; }
    float dot(const Vector2D& other) const { return x * other.x + y * other.y; }
    float length() const { return std::sqrt(x * x + y * y); }
    Vector2D normalized() const {
        float len = length();
        if (len < 1e-6f) return { 0.0f, 0.0f }; // two shapes sitting exactly on top of each other - no real direction to give
        return { x / len, y / len };
    }
};



enum class ShapeType { Circle, Box };
struct Body{
    Vector2D position;
    Vector2D velocity;
    float mass = 1.0f;
    float invMass = 1.0f;  // 1/mass. Set to 0 for walls/anchors so they never move.
    ShapeType shapeType = ShapeType::Circle; 
    float radius = 10.0f;                  // used when shapeType is a  Circle
    Vector2D halfExtents{ 10.0f, 10.0f };  // used when shapeType a box Box (half width and half height)
    Vector2D boxMin() const { return position - halfExtents; }
        Vector2D boxMax() const { return position + halfExtents; }
};



struct Manifold {
    bool colliding = false;
    Vector2D normal;           // always points from the FIRST body passed in, toward the SECOND
    float penetration = 0.0f;  // how deep the overlap is, measured along normal
};


Manifold testCircleCircle(const Body& a, const Body& b);
Manifold testCircleBox(const Body& circle, const Body& box);
Manifold testBoxBox(const Body& a, const Body& b);




struct PairHash {
    // std::unordered_map has no built-in hash for std::pair, so this is the
    // functor that lets us use a (cellX, cellY) pair as a map key directly.
    size_t operator()(const std::pair<int, int>& cell) const {
    // Shifting cellX into the top half of the size_t keeps (2,1) and (1,2)
        // from ever colliding to the same hash value.
        return (static_cast<size_t>(static_cast<unsigned int>(cell.first)) << 32)^  static_cast<size_t>(static_cast<unsigned int>(cell.second));
    }
};



class SpatialGrid {
public:
    explicit SpatialGrid(float cellSize);
    void clear();
    void insert(const std::vector<Body>& bodies);
    std::vector<std::pair<int, int>> getCandidatePairs(const std::vector<Body>& bodies) const;
    private:
    float cellSize;
    std::unordered_map<std::pair<int, int>, std::vector<int>, PairHash> cells;
 
    std::pair<int, int> cellOf(const Vector2D& position) const;
};




void resolveCollision(Body& a, Body& b, const Vector2D& normal, float restitution);
void correctPosition(Body& a, Body& b, const Vector2D& normal, float penetration);
void applyVelocitySleep(Body& body, float threshold);




class CollisionSystem {
public:
    explicit CollisionSystem(float cellSize, float restitution = 0.5f);
 
    void step(std::vector<Body>& bodies);
 
private:
    SpatialGrid grid;
    float restitution;
};

