#include "ForceGenerator.hpp"

// ============================================================
// GravityForce
// ============================================================

GravityForce::GravityForce(const Vector2D& acceleration) : acceleration(acceleration) {}

void GravityForce::apply(std::vector<Body>& bodies) const {
    for (Body& body : bodies) {
        if (body.isStatic()) continue;
        body.applyForce(acceleration * body.mass);
    }
}

// ============================================================
// DragForce
// ============================================================

DragForce::DragForce(float linear, float quadratic) : linear(linear), quadratic(quadratic) {}

void DragForce::setCoefficients(float newLinear, float newQuadratic) {
    linear = newLinear;
    quadratic = newQuadratic;
}

void DragForce::apply(std::vector<Body>& bodies) const {
    for (Body& body : bodies) {
        if (body.isStatic()) continue;

        float speed = body.velocity.length();
        if (speed < 1e-6f) continue;

        // Always opposite to the velocity, growing with speed
        body.applyForce(body.velocity * -(linear + quadratic * speed));
    }
}

// ============================================================
// SpringForce
// ============================================================

SpringForce::SpringForce(int bodyA, int bodyB, float restLength, float stiffness, float damping)
    : bodyA(bodyA), bodyB(bodyB), restLength(restLength), stiffness(stiffness), damping(damping) {}

bool SpringForce::connects(std::size_t bodyCount) const {
    return bodyA >= 0 && bodyB >= 0 && bodyA != bodyB &&
           static_cast<std::size_t>(bodyA) < bodyCount &&
           static_cast<std::size_t>(bodyB) < bodyCount;
}

void SpringForce::apply(std::vector<Body>& bodies) const {
    if (!connects(bodies.size())) return;

    Body& a = bodies[bodyA];
    Body& b = bodies[bodyB];

    Vector2D delta = b.position - a.position;
    float length = delta.length();
    if (length < 1e-6f) return; // ends on top of each other: no defined direction

    Vector2D direction = delta * (1.0f / length);

    // Positive when stretched (pulls the ends together), negative when compressed
    float extension = length - restLength;

    // Rate at which the ends move apart along the spring; damping resists it
    float separatingSpeed = (b.velocity - a.velocity).dot(direction);

    Vector2D force = direction * (stiffness * extension + damping * separatingSpeed);

    a.applyForce(force);
    b.applyForce(-force);
}
