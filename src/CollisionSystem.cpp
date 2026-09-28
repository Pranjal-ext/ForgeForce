// ================================================================
// CollisionSystem.cpp
// ================================================================

#include "CollisionSystem.h"

// ------------------------------------------------------------------
// Narrow-phase tests
// ------------------------------------------------------------------

Manifold testCircleCircle(const Body& a, const Body& b) {
    Manifold m; // starts as "not colliding" until we prove otherwise

    Vector2D delta = b.position - a.position; // vector from A's centre to B's centre
    float distSquared = delta.dot(delta); // squared distance - no sqrt needed yet
    float radiusSum = a.radius + b.radius;

    if (distSquared > radiusSum * radiusSum) {
        return m; // too far apart - bail out before paying for a sqrt we don't need
    }

    // We only need the real (non-squared) distance once we already know they
    // overlap, to build a normalized direction and a penetration depth.
    float dist = std::sqrt(distSquared);

    m.colliding = true;
    m.normal = (dist > 1e-6f) ? delta * (1.0f / dist) : Vector2D{ 1.0f, 0.0f };
    // ^ if dist is ~0 the circles are stacked exactly on top of each other -
    // there's no real direction to separate them along, so we just pick one.
    m.penetration = radiusSum - dist;

    return m;
}

Manifold testCircleBox(const Body& circle, const Body& box) {
    Manifold m;

    Vector2D boxMin = box.boxMin();
    Vector2D boxMax = box.boxMax();

    // Clamp the circle's centre onto the box's edges - this gives us the
    // point on (or inside) the box that sits closest to the circle's centre.
    float closestX = std::clamp(circle.position.x, boxMin.x, boxMax.x);
    float closestY = std::clamp(circle.position.y, boxMin.y, boxMax.y);
    Vector2D closestPoint{ closestX, closestY };

    Vector2D delta = closestPoint - circle.position; // circle's centre -> nearest point on the box
    float distSquared = delta.dot(delta);

    if (distSquared > circle.radius * circle.radius) {
        return m; // gap between the circle's edge and the box - no collision
    }

    float dist = std::sqrt(distSquared);

    m.colliding = true;
    // If dist is ~0 the circle's centre is already inside the box - there's
    // no clean edge direction, so we just push "up" rather than divide by 0.
    m.normal = (dist > 1e-6f) ? delta * (1.0f / dist) : Vector2D{ 0.0f, -1.0f };
    m.penetration = circle.radius - dist;

    return m;
}

Manifold testBoxBox(const Body& a, const Body& b) {
    Manifold m;

    Vector2D aMin = a.boxMin(), aMax = a.boxMax();
    Vector2D bMin = b.boxMin(), bMax = b.boxMax();

    // How far the boxes overlap on each axis on its own. Negative means
    // there's a gap on that axis, which (per SAT) means they can't be touching.
    float overlapX = std::min(aMax.x, bMax.x) - std::max(aMin.x, bMin.x);
    float overlapY = std::min(aMax.y, bMax.y) - std::max(aMin.y, bMin.y);

    if (overlapX <= 0.0f || overlapY <= 0.0f) {
        return m; // a gap exists on x or y - that's a separating axis, so no collision
    }

    m.colliding = true;

    // Push out along whichever axis has the SMALLER overlap - that's the
    // shortest path that actually separates the two boxes.
    if (overlapX < overlapY) {
        m.normal = (a.position.x < b.position.x) ? Vector2D{ 1.0f, 0.0f } : Vector2D{ -1.0f, 0.0f };
        m.penetration = overlapX;
    } else {
        m.normal = (a.position.y < b.position.y) ? Vector2D{ 0.0f, 1.0f } : Vector2D{ 0.0f, -1.0f };
        m.penetration = overlapY;
    }

    return m;
}

// ------------------------------------------------------------------
// Collision response
// ------------------------------------------------------------------

void resolveCollision(Body& a, Body& b, const Vector2D& normal, float restitution) {
    Vector2D relativeVelocity = b.velocity - a.velocity;
    float velAlongNormal = relativeVelocity.dot(normal); // closing speed - only the part along normal matters

    // Already moving apart? Then there's nothing to resolve - an impulse here
    // would suck them back together instead of letting them separate.
    if (velAlongNormal > 0.0f) return;

    float invMassSum = a.invMass + b.invMass;
    if (invMassSum <= 0.0f) return; // both bodies are immovable (invMass 0) - nothing can happen

    // j = -(1 + e) * (closing speed along normal) / (1/massA + 1/massB)
    float j = -(1.0f + restitution) * velAlongNormal / invMassSum;

    Vector2D impulse = normal * j;
    a.velocity -= impulse * a.invMass; // A gets pushed backward along normal
    b.velocity += impulse * b.invMass; // B gets pushed forward along normal
}

void correctPosition(Body& a, Body& b, const Vector2D& normal, float penetration) {
    // This is the fix for jitter: instead of relying only on
    // velocity changes to slowly push overlapping shapes apart over several
    // frames (which is what causes the twitching), we nudge their positions
    // apart directly, right now, by a slice of however deep they're overlapping.
    const float percent = 0.2f; // fix 20% of the overlap per frame, not all of it at once (fixing 100% causes its own jitter)
    const float slop = 0.01f;   // ignore overlaps smaller than this - not worth correcting, and stops micro-jitter from rounding

    float invMassSum = a.invMass + b.invMass;
    if (invMassSum <= 0.0f) return;

    float correctionMagnitude = std::max(penetration - slop, 0.0f) / invMassSum * percent;
    Vector2D correction = normal * correctionMagnitude;

    a.position -= correction * a.invMass;
    b.position += correction * b.invMass;
}

void applyVelocitySleep(Body& body, float threshold) {
    //  other jitter fix: a velocity this small isn't visible
    // anyway, so just round it down to exactly zero instead of letting it
    // twitch around near-zero forever.
    if (body.velocity.length() < threshold) {
        body.velocity = { 0.0f, 0.0f };
    }
}

// ------------------------------------------------------------------
// SpatialGrid
// ------------------------------------------------------------------

SpatialGrid::SpatialGrid(float cellSize) : cellSize(cellSize) {}

std::pair<int, int> SpatialGrid::cellOf(const Vector2D& position) const {
    // std::floor (not a plain cast to int) matters the moment a shape has a
    // negative x or y - a plain (int) cast truncates toward 0, which would
    // put -0.5 and 0.5 in the same cell. floor() puts them where they belong.
    int cellX = static_cast<int>(std::floor(position.x / cellSize));
    int cellY = static_cast<int>(std::floor(position.y / cellSize));
    return { cellX, cellY };
}

void SpatialGrid::clear() {
    cells.clear(); // wipe every cell so last frame's positions don't linger
}

void SpatialGrid::insert(const std::vector<Body>& bodies) {
    for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
        std::pair<int, int> cell = cellOf(bodies[i].position);
        cells[cell].push_back(i); // store the INDEX, not the body itself - cheap to copy, and lets us write back to the real body later
    }
}

std::vector<std::pair<int, int>>
SpatialGrid::getCandidatePairs(const std::vector<Body>& bodies) const {
    // (bodyIndexA, bodyIndexB) with A < B always, so the same pair found from
    // two different directions collapses into one set entry instead of being
    // checked twice.
    std::set<std::pair<int, int>> uniquePairs;

    for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
        std::pair<int, int> baseCell = cellOf(bodies[i].position);

        // Check this shape's own cell plus all 8 neighbours - a shape sitting
        // near a cell's edge can still be touching something just over the border.
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                std::pair<int, int> neighbourCell = { baseCell.first + dx, baseCell.second + dy };
                auto found = cells.find(neighbourCell);
                if (found == cells.end()) continue; // that neighbouring cell is empty - nothing to compare against

                for (int j : found->second) {
                    if (j == i) continue; // don't pair a shape with itself
                    uniquePairs.insert({ std::min(i, j), std::max(i, j) });
                }
            }
        }
    }

    return std::vector<std::pair<int, int>>(uniquePairs.begin(), uniquePairs.end());
}

// ------------------------------------------------------------------
// CollisionSystem
// ------------------------------------------------------------------

CollisionSystem::CollisionSystem(float cellSize, float restitution)
    : grid(cellSize), restitution(restitution) {}

void CollisionSystem::step(std::vector<Body>& bodies) {
    //Broad-phase
    grid.clear();
    grid.insert(bodies);
    std::vector<std::pair<int, int>> candidates = grid.getCandidatePairs(bodies);

    // Narrow-phase + response
    for (const auto& pair : candidates) {
        Body& a = bodies[pair.first];
        Body& b = bodies[pair.second];
        Manifold m;

        if (a.shapeType == ShapeType::Circle && b.shapeType == ShapeType::Circle) {
            m = testCircleCircle(a, b);
        } else if (a.shapeType == ShapeType::Box && b.shapeType == ShapeType::Box) {
            m = testBoxBox(a, b);
        } else if (a.shapeType == ShapeType::Circle && b.shapeType == ShapeType::Box) {
            m = testCircleBox(a, b); // already normal-points-a-to-b, matches what resolveCollision expects
        } else { // a is a Box and b is a Circle - opposite of what testCircleBox expects
            m = testCircleBox(b, a); // so test it as (circle=b, box=a) instead...
            m.normal = m.normal * -1.0f; // ...then flip the normal back to point a -> b
        }

        if (!m.colliding) continue;

        resolveCollision(a, b, m.normal, restitution);
        correctPosition(a, b, m.normal, m.penetration);
    }

    //settle anything that's basically already still
    for (Body& body : bodies) {
        applyVelocitySleep(body, 0.05f);
    }
}
