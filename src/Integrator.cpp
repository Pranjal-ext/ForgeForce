#include "Integrator.hpp"

Integrator::Integrator(IntegrationMethod method) : method(method) {}

void Integrator::step(std::vector<Body>& bodies, float deltaTime) const {
  for (Body& body : bodies) {
    if (!body.isStatic()) {
      // Newton's second law: a = F / m  (invMass avoids a division)
      Vector2D acceleration = body.force * body.invMass;

      if (method == IntegrationMethod::SemiImplicitEuler) {
        body.velocity += acceleration * deltaTime;
        body.position += body.velocity * deltaTime;
      } else {
        body.position += body.velocity * deltaTime;
        body.velocity += acceleration * deltaTime;
      }
    }

    // Forces are recomputed from scratch every step
    body.clearForce();
  }
}
