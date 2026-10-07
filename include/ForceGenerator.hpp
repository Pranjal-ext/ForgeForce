#pragma once

#include <string>
#include <vector>

#include "Body.hpp"

// ============================================================
// ForceGenerator (abstract base class)
// Something that pushes on bodies. Each frame, every enabled
// generator adds its force to the bodies' force accumulators;
// the integrator then turns the total force into motion.
// New kinds of force are added by deriving from this class
// without changing any of the code that uses it.
// ============================================================
class ForceGenerator {
public:
    virtual ~ForceGenerator() = default;

    // Adds this generator's force to the affected bodies
    virtual void apply(std::vector<Body>& bodies) const = 0;

    // Human-readable name, used for display and debugging
    virtual std::string name() const = 0;

    void setEnabled(bool value) { enabled = value; }
    bool isEnabled() const { return enabled; }

private:
    bool enabled = true;
};

// ============================================================
// GravityForce
// Uniform field: F = m * g on every non-static body, so every
// body accelerates at exactly g regardless of its mass.
// Screen y points down, so a positive y value pulls downward.
// ============================================================
class GravityForce : public ForceGenerator {
public:
    explicit GravityForce(const Vector2D& acceleration = { 0.0f, 900.0f });

    void apply(std::vector<Body>& bodies) const override;
    std::string name() const override { return "Gravity"; }

    void setAcceleration(const Vector2D& value) { acceleration = value; }
    const Vector2D& getAcceleration() const { return acceleration; }

private:
    Vector2D acceleration;
};

// ============================================================
// DragForce
// Air resistance opposing motion: F = -(k1 + k2 * |v|) * v
//   k1 (linear)    dominates at low speed
//   k2 (quadratic) dominates at high speed
// Heavier bodies slow down less, because a = F / m.
// ============================================================
class DragForce : public ForceGenerator {
public:
    explicit DragForce(float linear = 0.02f, float quadratic = 0.0001f);

    void apply(std::vector<Body>& bodies) const override;
    std::string name() const override { return "Drag"; }

    void setCoefficients(float linear, float quadratic);
    float getLinear() const { return linear; }
    float getQuadratic() const { return quadratic; }

private:
    float linear;
    float quadratic;
};

// ============================================================
// SpringForce
// Damped spring (Hooke's law) between two bodies, identified by
// their index in the scene:
//   F = (k * (length - restLength) + c * closingSpeed) along the spring
// Equal and opposite forces act on both ends (Newton's third law).
// ============================================================
class SpringForce : public ForceGenerator {
public:
    SpringForce(int bodyA, int bodyB, float restLength, float stiffness, float damping = 0.0f);

    void apply(std::vector<Body>& bodies) const override;
    std::string name() const override { return "Spring"; }

    int getBodyA() const { return bodyA; }
    int getBodyB() const { return bodyB; }
    float getRestLength() const { return restLength; }
    float getStiffness() const { return stiffness; }
    float getDamping() const { return damping; }

    // True if both indices refer to existing, different bodies
    bool connects(std::size_t bodyCount) const;

private:
    int bodyA;
    int bodyB;
    float restLength;
    float stiffness;
    float damping;
};
