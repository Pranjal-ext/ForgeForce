#pragma once

#include <vector>

#include "Body.hpp"

// ============================================================
// Integration methods
//   SemiImplicitEuler: v += a*dt, then x += v*dt (uses the NEW velocity).
//                      Stable for springs and orbits; used by default.
//   ExplicitEuler:     x += v*dt, then v += a*dt (uses the OLD velocity).
//                      Simpler but gains energy over time; kept for comparison.
// ============================================================
enum class IntegrationMethod { SemiImplicitEuler, ExplicitEuler };

// ============================================================
// Integrator
// Advances every non-static body by one timestep using the
// force gathered in Body::force (a = F / m), then clears the
// force so the next step starts from zero.
// ============================================================
class Integrator {
public:
    explicit Integrator(IntegrationMethod method = IntegrationMethod::SemiImplicitEuler);

    void step(std::vector<Body>& bodies, float dt) const;

    void setMethod(IntegrationMethod value) { method = value; }
    IntegrationMethod getMethod() const { return method; }

private:
    IntegrationMethod method;
};
