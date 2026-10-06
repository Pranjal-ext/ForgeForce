#include "CollisionSystem.hpp"

#include <limits>

// ============================================================
// Narrow-phase tests
// ============================================================

Manifold testCircleCircle(const Body& a, const Body& b) {
    Manifold m;

    Vector2D delta = b.position - a.position;
    float distSquared = delta.lengthSquared();
    float radiusSum = a.radius + b.radius;

    if (distSquared > radiusSum * radiusSum) return m;

    float dist = std::sqrt(distSquared);

    m.colliding = true;
    // Identical centres have no direction, so pick one
    m.normal = (dist > 1e-6f) ? delta * (1.0f / dist) : Vector2D{ 1.0f, 0.0f };
    m.penetration = radiusSum - dist;
    return m;
}

Manifold testCircleBox(const Body& circle, const Body& box) {
    Manifold m;

    Vector2D lo = box.boxMin();
    Vector2D hi = box.boxMax();

    // Closest point on the box to the circle's centre
    Vector2D closest{
        std::clamp(circle.position.x, lo.x, hi.x),
        std::clamp(circle.position.y, lo.y, hi.y)
    };

    Vector2D delta = closest - circle.position;
    float distSquared = delta.lengthSquared();

    if (distSquared > circle.radius * circle.radius) return m;

    float dist = std::sqrt(distSquared);
    m.colliding = true;

    if (dist > 1e-6f) {
        m.normal = delta * (1.0f / dist);
        m.penetration = circle.radius - dist;
        return m;
    }

    // Centre is inside the box: push out through the nearest face
    float toLeft   = circle.position.x - lo.x;
    float toRight  = hi.x - circle.position.x;
    float toTop    = circle.position.y - lo.y;
    float toBottom = hi.y - circle.position.y;

    float nearest = toLeft;
    Vector2D outward{ -1.0f, 0.0f };
    if (toRight  < nearest) { nearest = toRight;  outward = {  1.0f, 0.0f }; }
    if (toTop    < nearest) { nearest = toTop;    outward = { 0.0f, -1.0f }; }
    if (toBottom < nearest) { nearest = toBottom; outward = { 0.0f,  1.0f }; }

    // Normal points circle -> box, i.e. opposite to the face the circle exits through
    m.normal = -outward;
    m.penetration = circle.radius + nearest;
    return m;
}

Manifold testBoxBox(const Body& a, const Body& b) {
    Manifold m;

    Vector2D aMin = a.boxMin(), aMax = a.boxMax();
    Vector2D bMin = b.boxMin(), bMax = b.boxMax();

    float overlapX = std::min(aMax.x, bMax.x) - std::max(aMin.x, bMin.x);
    float overlapY = std::min(aMax.y, bMax.y) - std::max(aMin.y, bMin.y);

    if (overlapX <= 0.0f || overlapY <= 0.0f) return m;

    m.colliding = true;

    // Separate along the axis of least overlap
    if (overlapX < overlapY) {
        m.normal = (a.position.x < b.position.x) ? Vector2D{ 1.0f, 0.0f } : Vector2D{ -1.0f, 0.0f };
        m.penetration = overlapX;
    } else {
        m.normal = (a.position.y < b.position.y) ? Vector2D{ 0.0f, 1.0f } : Vector2D{ 0.0f, -1.0f };
        m.penetration = overlapY;
    }
    return m;
}

namespace {

// Projects vertices onto an axis and returns the covered interval
void projectVertices(const std::vector<Vector2D>& verts, const Vector2D& axis,
                     float& outMin, float& outMax) {
    outMin = outMax = verts[0].dot(axis);
    for (const Vector2D& v : verts) {
        float p = v.dot(axis);
        outMin = std::min(outMin, p);
        outMax = std::max(outMax, p);
    }
}

// Unit normal of edge i of a polygon (zero for degenerate edges)
Vector2D edgeNormal(const std::vector<Vector2D>& verts, std::size_t i) {
    Vector2D edge = verts[(i + 1) % verts.size()] - verts[i];
    return Vector2D{ -edge.y, edge.x }.normalized();
}

} // namespace

Manifold testPolygonPolygon(const Body& a, const Body& b) {
    Manifold m;

    std::vector<Vector2D> va = a.worldVertices();
    std::vector<Vector2D> vb = b.worldVertices();
    if (va.size() < 3 || vb.size() < 3) return m;

    float minOverlap = std::numeric_limits<float>::max();
    Vector2D bestAxis;

    // Separating Axis Theorem: test every edge normal of both shapes.
    // A gap on any axis means the shapes are not touching.
    auto testAxesOf = [&](const std::vector<Vector2D>& verts) {
        for (std::size_t i = 0; i < verts.size(); ++i) {
            Vector2D axis = edgeNormal(verts, i);
            if (axis.lengthSquared() < 0.5f) continue; // degenerate edge

            float minA, maxA, minB, maxB;
            projectVertices(va, axis, minA, maxA);
            projectVertices(vb, axis, minB, maxB);

            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap <= 0.0f) return false;

            if (overlap < minOverlap) {
                minOverlap = overlap;
                bestAxis = axis;
            }
        }
        return true;
    };

    if (!testAxesOf(va) || !testAxesOf(vb)) return m;

    // Make the normal point from a toward b
    if (bestAxis.dot(b.position - a.position) < 0.0f) bestAxis = -bestAxis;

    m.colliding = true;
    m.normal = bestAxis;
    m.penetration = minOverlap;
    return m;
}

Manifold testCirclePolygon(const Body& circle, const Body& polygon) {
    Manifold m;

    std::vector<Vector2D> verts = polygon.worldVertices();
    if (verts.size() < 3) return m;

    float minOverlap = std::numeric_limits<float>::max();
    Vector2D bestAxis;

    auto testAxis = [&](const Vector2D& axis) {
        float polyMin, polyMax;
        projectVertices(verts, axis, polyMin, polyMax);

        float centre = circle.position.dot(axis);
        float circleMin = centre - circle.radius;
        float circleMax = centre + circle.radius;

        float overlap = std::min(polyMax, circleMax) - std::max(polyMin, circleMin);
        if (overlap <= 0.0f) return false;

        if (overlap < minOverlap) {
            minOverlap = overlap;
            bestAxis = axis;
        }
        return true;
    };

    // Polygon edge normals
    for (std::size_t i = 0; i < verts.size(); ++i) {
        Vector2D axis = edgeNormal(verts, i);
        if (axis.lengthSquared() < 0.5f) continue;
        if (!testAxis(axis)) return m;
    }

    // Axis from the circle's centre to the nearest polygon vertex (catches corner contacts)
    Vector2D nearestVertex = verts[0];
    for (const Vector2D& v : verts) {
        if ((v - circle.position).lengthSquared() < (nearestVertex - circle.position).lengthSquared()) {
            nearestVertex = v;
        }
    }
    Vector2D cornerAxis = (nearestVertex - circle.position).normalized();
    if (cornerAxis.lengthSquared() > 0.5f && !testAxis(cornerAxis)) return m;

    // Normal points circle -> polygon
    if (bestAxis.dot(polygon.position - circle.position) < 0.0f) bestAxis = -bestAxis;

    m.colliding = true;
    m.normal = bestAxis;
    m.penetration = minOverlap;
    return m;
}

Manifold testCollision(const Body& a, const Body& b) {
    const bool aCircle = (a.shapeType == ShapeType::Circle);
    const bool bCircle = (b.shapeType == ShapeType::Circle);
    const bool aBox = (a.shapeType == ShapeType::Box);
    const bool bBox = (b.shapeType == ShapeType::Box);

    if (aCircle && bCircle) return testCircleCircle(a, b);
    if (aBox && bBox) return testBoxBox(a, b);
    if (aCircle && bBox) return testCircleBox(a, b);

    if (aBox && bCircle) {
        Manifold m = testCircleBox(b, a);
        m.normal = -m.normal; // flip so it points a -> b
        return m;
    }

    if (aCircle) return testCirclePolygon(a, b);

    if (bCircle) {
        Manifold m = testCirclePolygon(b, a);
        m.normal = -m.normal;
        return m;
    }

    // Box-Polygon or Polygon-Polygon
    return testPolygonPolygon(a, b);
}

// ============================================================
// Collision response
// ============================================================

void resolveCollision(Body& a, Body& b, const Vector2D& normal,
                      float restitution, float restitutionThreshold, float friction) {
    Vector2D relativeVelocity = b.velocity - a.velocity;
    float velAlongNormal = relativeVelocity.dot(normal);

    // Already separating
    if (velAlongNormal > 0.0f) return;

    float invMassSum = a.invMass + b.invMass;
    if (invMassSum <= 0.0f) return; // both static

    // Slow impacts are resting contact: no bounce
    float e = (-velAlongNormal < restitutionThreshold) ? 0.0f : restitution;

    // Normal impulse: j = -(1 + e) * vRel·n / (1/mA + 1/mB)
    float j = -(1.0f + e) * velAlongNormal / invMassSum;

    Vector2D impulse = normal * j;
    a.velocity -= impulse * a.invMass;
    b.velocity += impulse * b.invMass;

    // Friction impulse along the contact tangent, capped by Coulomb's law (|jt| <= mu * j)
    relativeVelocity = b.velocity - a.velocity;
    Vector2D tangent = relativeVelocity - normal * relativeVelocity.dot(normal);
    float tangentLength = tangent.length();
    if (tangentLength < 1e-6f) return;
    tangent = tangent * (1.0f / tangentLength);

    float jt = -relativeVelocity.dot(tangent) / invMassSum;
    float maxFriction = friction * j;
    jt = std::clamp(jt, -maxFriction, maxFriction);

    Vector2D frictionImpulse = tangent * jt;
    a.velocity -= frictionImpulse * a.invMass;
    b.velocity += frictionImpulse * b.invMass;
}

void correctPosition(Body& a, Body& b, const Vector2D& normal, float penetration) {
    const float percent = 0.2f; // correct 20% of the overlap per frame
    const float slop = 0.01f;   // ignore negligible overlaps

    float invMassSum = a.invMass + b.invMass;
    if (invMassSum <= 0.0f) return;

    float magnitude = std::max(penetration - slop, 0.0f) / invMassSum * percent;
    Vector2D correction = normal * magnitude;

    a.position -= correction * a.invMass;
    b.position += correction * b.invMass;
}

void applyVelocitySleep(Body& body, float threshold) {
    if (body.velocity.length() < threshold) {
        body.velocity = { 0.0f, 0.0f };
    }
}

// ============================================================
// SpatialGrid
// ============================================================

SpatialGrid::SpatialGrid(float cellSize) : cellSize(cellSize) {}

int SpatialGrid::cellCoord(float value) const {
    // floor (not truncation) so negative coordinates map to the correct cell
    return static_cast<int>(std::floor(value / cellSize));
}

void SpatialGrid::clear() {
    cells.clear();
}

void SpatialGrid::insert(const std::vector<Body>& bodies) {
    for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
        Vector2D lo = bodies[i].aabbMin();
        Vector2D hi = bodies[i].aabbMax();

        // Skip bodies with invalid positions instead of corrupting the grid
        if (!std::isfinite(lo.x) || !std::isfinite(lo.y) ||
            !std::isfinite(hi.x) || !std::isfinite(hi.y)) {
            continue;
        }

        int x0 = cellCoord(lo.x), x1 = cellCoord(hi.x);
        int y0 = cellCoord(lo.y), y1 = cellCoord(hi.y);

        for (int cx = x0; cx <= x1; ++cx) {
            for (int cy = y0; cy <= y1; ++cy) {
                cells[{ cx, cy }].push_back(i);
            }
        }
    }
}

std::vector<std::pair<int, int>> SpatialGrid::getCandidatePairs() const {
    // Bodies sharing several cells would be paired more than once; the set removes duplicates
    std::set<std::pair<int, int>> uniquePairs;

    for (const auto& entry : cells) {
        const std::vector<int>& indices = entry.second;
        for (std::size_t i = 0; i < indices.size(); ++i) {
            for (std::size_t j = i + 1; j < indices.size(); ++j) {
                int a = indices[i], b = indices[j];
                uniquePairs.insert({ std::min(a, b), std::max(a, b) });
            }
        }
    }

    return std::vector<std::pair<int, int>>(uniquePairs.begin(), uniquePairs.end());
}

// ============================================================
// CollisionSystem
// ============================================================

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
        const Body& a = bodies[pair.first];
        const Body& b = bodies[pair.second];

        if (a.invMass <= 0.0f && b.invMass <= 0.0f) continue; // two static bodies

        Manifold m = testCollision(a, b);
        if (m.colliding) contacts.push_back({ pair.first, pair.second, m });
    }

    // Velocity solver: several passes let impulses propagate through stacks
    for (int iteration = 0; iteration < solverIterations; ++iteration) {
        for (const Contact& c : contacts) {
            resolveCollision(bodies[c.a], bodies[c.b], c.manifold.normal,
                             restitution, restitutionThreshold, friction);
        }
    }

    // Position correction: once per contact
    for (const Contact& c : contacts) {
        correctPosition(bodies[c.a], bodies[c.b], c.manifold.normal, c.manifold.penetration);
    }

    // Settle near-stationary bodies
    for (Body& body : bodies) {
        applyVelocitySleep(body, sleepThreshold);
    }
}
