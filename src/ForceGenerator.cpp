#include "ForceGenerator.hpp"

// GravityForce

GravityForce::GravityForce(const Vector2D& initialAcceleration) : acceleration(initialAcceleration) {}

void GravityForce::apply(std::vector<Body>& bodies) const {
  for (Body& body : bodies) {
    if (body.isStatic()) continue;
    body.applyForce(acceleration * body.mass);
  }
}

// DragForce

DragForce::DragForce(float linearCoefficient, float quadraticCoefficient)
  : linear(linearCoefficient), quadratic(quadraticCoefficient) {}

void DragForce::setCoefficients(float linearCoefficient, float quadraticCoefficient) {
  linear = linearCoefficient;
  quadratic = quadraticCoefficient;
}

void DragForce::apply(std::vector<Body>& bodies) const {
  for (Body& body : bodies) {
    if (body.isStatic()) continue;

    float velocityMagnitude = body.velocity.length();
    if (velocityMagnitude < 1e-6f) continue;

    // Always opposite to the velocity, growing with speed
    body.applyForce(body.velocity * -(linear + quadratic * velocityMagnitude));
  }
}

// SpringForce

SpringForce::SpringForce(int firstBodyIndex, int secondBodyIndex, float restLength,
  float stiffness, float damping)
  : bodyA(firstBodyIndex), bodyB(secondBodyIndex), restLength(restLength),
    stiffness(stiffness), damping(damping) {}

bool SpringForce::connects(std::size_t bodyCount) const {
  return bodyA >= 0 && bodyB >= 0 && bodyA != bodyB &&
    static_cast<std::size_t>(bodyA) < bodyCount &&
    static_cast<std::size_t>(bodyB) < bodyCount;
}

void SpringForce::apply(std::vector<Body>& bodies) const {
  if (!connects(bodies.size())) return;

  Body& firstBody = bodies[bodyA];
  Body& secondBody = bodies[bodyB];

  Vector2D separation = secondBody.position - firstBody.position;
  float length = separation.length();
  if (length < 1e-6f) return; // ends on top of each other: no defined direction

  Vector2D direction = separation * (1.0f / length);

  // Positive when stretched (pulls the ends together), negative when compressed
  float extension = length - restLength;

  // Rate at which the ends move apart along the spring; damping resists it
  float separatingSpeed = (secondBody.velocity - firstBody.velocity).dot(direction);

  Vector2D force = direction * (stiffness * extension + damping * separatingSpeed);

  firstBody.applyForce(force);
  secondBody.applyForce(-force);
}
