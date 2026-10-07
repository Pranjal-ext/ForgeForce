#include "Integrator.hpp"

Integrator::Integrator(IntegrationMethod method) : method(method) {}

void Integrator::step(std::vector<Body>& bodies, float dt) const {
    for (Body& body : bodies) {
        if (!body.isStatic()) {
            // Newton's second law: a = F / m  (invMass avoids a division)
            Vector2D acceleration = body.force * body.invMass;

            if (method == IntegrationMethod::SemiImplicitEuler) {
                body.velocity += acceleration * dt;
                body.position += body.velocity * dt;
            } else {
                body.position += body.velocity * dt;
                body.velocity += acceleration * dt;
            }
        }

        // Forces are recomputed from scratch every step
        body.clearForce();
    }
}
