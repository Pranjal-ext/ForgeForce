// ============================================================
// Automated tests for every module: vector maths, bodies,
// collision detection and response, forces, integration,
// projectile and spring accuracy, the world, events,
// undo/redo, presets, the launcher, and scene save/load.
//
// Build from the project root (do NOT add src/main.cpp, this file has its own main).
// See the run guide or README for the exact command.
// Exit code 0 means every test passed.
// ============================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "Body.hpp"
#include "CollisionSystem.hpp"
#include "EventQueue.hpp"
#include "ForceGenerator.hpp"
#include "InteractionRenderer.hpp"
#include "Integrator.hpp"
#include "Scenarios.hpp"
#include "SceneHistory.hpp"
#include "World.hpp"

// ------------------------------------------------------------
// Minimal test helpers
// ------------------------------------------------------------
static int passed = 0;
static int failed = 0;

static void check(bool condition, const std::string& name, const std::string& detail = "") {
    if (condition) {
        ++passed;
        std::cout << "  [PASS] " << name << "\n";
    } else {
        ++failed;
        std::cout << "  [FAIL] " << name;
        if (!detail.empty()) std::cout << "  -> " << detail;
        std::cout << "\n";
    }
}

static bool near(float firstValue, float secondValue, float tolerance) {
    return std::fabs(firstValue - secondValue) <= tolerance;
}

static std::string fmt(float value) {
    std::ostringstream formattedValue;
    formattedValue << std::fixed << std::setprecision(3) << value;
    return formattedValue.str();
}

static void section(const std::string& title) { std::cout << "\n== " << title << " ==\n"; }

static const float WINDOW_WIDTH = 1280.0f;
static const float WINDOW_HEIGHT = 720.0f;
static const float FIXED_TIMESTEP = World::FIXED_TIMESTEP;

static void runSteps(World& world, int stepCount, float deltaTime = FIXED_TIMESTEP) {
    for (int stepIndex = 0; stepIndex < stepCount; ++stepIndex) world.step(deltaTime);
}

static bool isFinite(const Body& body) {
    return std::isfinite(body.position.x) && std::isfinite(body.position.y) &&
           std::isfinite(body.velocity.x) && std::isfinite(body.velocity.y);
}

// A world with drag off, for tests that need exact textbook motion
static void disableDrag(World& world) {
    PhysicsSettings settings = world.getSettings();
    settings.dragEnabled = false;
    world.setSettings(settings);
}

// ============================================================
static void testVector2D() {
    section("Vector2D");
    Vector2D vector(3.0f, 4.0f);
    check(near(vector.length(), 5.0f, 1e-5f), "length of (3,4) is 5");
    check(near(vector.normalized().length(), 1.0f, 1e-5f), "normalized vector has length 1");
    check(Vector2D(0, 0).normalized().length() == 0.0f, "normalizing a zero vector is safe");
    check(near(vector.dot(Vector2D(1, 0)), 3.0f, 1e-5f), "dot product");
    check(Vector2D(1, 0).cross(Vector2D(0, 1)) > 0.0f, "cross product sign");
}

static void testBody() {
    section("Body");
    Body circle = Body::makeCircle({ 0, 0 }, 10.0f, 2.0f);
    check(circle.mass == 2.0f && near(circle.invMass, 0.5f, 1e-6f), "setMass keeps invMass = 1/mass");

    Body staticBox = Body::makeBox({ 0, 0 }, { 5, 5 }, 0.0f);
    check(staticBox.isStatic() && staticBox.invMass == 0.0f, "mass 0 makes a body static");

    // Lowest point on screen = largest y; a flat bottom means two vertices share it
    std::vector<Vector2D> squareVertices = Body::regularPolygonVertices(4, 10.0f);
    float lowestY = -1e9f;
    for (const Vector2D& vertex : squareVertices) lowestY = std::max(lowestY, vertex.y);
    int verticesOnBottom = 0;
    for (const Vector2D& vertex : squareVertices) {
        if (near(vertex.y, lowestY, 1e-4f)) ++verticesOnBottom;
    }
    check(squareVertices.size() == 4 && verticesOnBottom == 2, "regular square has a flat bottom edge");
    check(Body::regularPolygonVertices(2, 10.0f).empty(), "fewer than 3 sides gives no polygon");

    Body hexagon = Body::makeRegularPolygon({ 100, 100 }, 6, 20.0f);
    check(hexagon.containsPoint({ 100, 100 }) && !hexagon.containsPoint({ 125, 100 }), "polygon containsPoint");
    check(near(hexagon.aabbMax().x - hexagon.aabbMin().x, 40.0f, 0.5f) ||
          near(hexagon.aabbMax().x - hexagon.aabbMin().x, 34.64f, 0.5f), "polygon bounding box width");

    Body forceBody = Body::makeCircle({ 0, 0 }, 5.0f);
    forceBody.applyForce({ 1, 2 });
    forceBody.applyForce({ 3, 4 });
    check(near(forceBody.force.x, 4.0f, 1e-6f) && near(forceBody.force.y, 6.0f, 1e-6f),
          "applyForce accumulates");
    forceBody.clearForce();
    check(forceBody.force.x == 0.0f && forceBody.force.y == 0.0f, "clearForce resets");
}

static void testNarrowPhase() {
    section("Collision detection");
    Manifold manifold = testCollision(Body::makeCircle({ 0, 0 }, 10), Body::makeCircle({ 15, 0 }, 10));
    check(manifold.colliding && near(manifold.normal.x, 1.0f, 1e-4f) &&
          near(manifold.penetration, 5.0f, 1e-4f),
          "circle-circle: normal points A->B, depth 5");
    check(!testCollision(Body::makeCircle({ 0, 0 }, 10), Body::makeCircle({ 25, 0 }, 10)).colliding,
          "circle-circle with gap: no collision");

    manifold = testCollision(Body::makeCircle({ 0, 0 }, 10), Body::makeCircle({ 0, 0 }, 10));
    check(manifold.colliding && std::isfinite(manifold.normal.x), "identical positions: no NaN");

    manifold = testCollision(Body::makeCircle({ 0, 0 }, 10), Body::makeBox({ 15, 0 }, { 10, 10 }));
    check(manifold.colliding && manifold.normal.x > 0.9f, "circle-box: normal points circle->box");
    manifold = testCollision(Body::makeBox({ 15, 0 }, { 10, 10 }), Body::makeCircle({ 0, 0 }, 10));
    check(manifold.colliding && manifold.normal.x < -0.9f, "box-circle (reversed order): normal flips");

    check(testCollision(Body::makeBox({ 0, 0 }, { 10, 10 }), Body::makeBox({ 15, 0 }, { 10, 10 })).colliding,
          "box-box overlap");
    check(!testCollision(Body::makeBox({ 0, 0 }, { 10, 10 }), Body::makeBox({ 25, 0 }, { 10, 10 })).colliding,
          "box-box with gap");

    manifold = testCollision(Body::makeRegularPolygon({ 0, 0 }, 6, 20),
      Body::makeRegularPolygon({ 30, 0 }, 6, 20));
    check(manifold.colliding && manifold.normal.x > 0.5f, "hexagon-hexagon (SAT) overlap");
    check(!testCollision(Body::makeRegularPolygon({ 0, 0 }, 6, 20),
                         Body::makeRegularPolygon({ 60, 0 }, 6, 20)).colliding,
          "hexagon-hexagon with gap");

    check(testCollision(Body::makeCircle({ 0, 0 }, 10), Body::makeRegularPolygon({ 20, 0 }, 3, 20)).colliding,
          "circle-triangle overlap");
    check(!testCollision(Body::makeCircle({ 0, 0 }, 10), Body::makeRegularPolygon({ 25, 0 }, 3, 20)).colliding,
          "circle-triangle near miss (edge 11.7 away, radius 10)");

    Body floor = Body::makeBox({ 640, 700 }, { 640, 20 }, 0.0f);
    manifold = testCollision(Body::makeCircle({ 200, 685 }, 20), floor);
    check(manifold.colliding && manifold.normal.y > 0.9f, "circle centre inside floor resolves upward");
}

static void testResponse() {
    section("Collision response");
    std::vector<Body> bodies = { Body::makeCircle({ 0, 0 }, 10), Body::makeCircle({ 19, 0 }, 10) };
    bodies[0].velocity = { 50, 0 };
    CollisionSystem elastic(80.0f, 1.0f);
    elastic.setRestitutionThreshold(0.0f);
    elastic.setFriction(0.0f);
    elastic.step(bodies);
    check(near(bodies[0].velocity.x, 0.0f, 1e-3f) && near(bodies[1].velocity.x, 50.0f, 1e-3f),
          "elastic equal masses swap velocities");
    check(near(bodies[0].velocity.x + bodies[1].velocity.x, 50.0f, 1e-3f), "momentum conserved");

    std::vector<Body> staticBodies = { Body::makeBox({ 0, 0 }, { 10, 10 }, 0.0f),
                                       Body::makeBox({ 5, 0 }, { 10, 10 }, 0.0f) };
    CollisionSystem collisionSystem(80.0f);
    collisionSystem.step(staticBodies);
    check(staticBodies[0].position.x == 0.0f && staticBodies[1].position.x == 5.0f,
          "static bodies never move");
}

static void testForces() {
    section("Forces");

    // Gravity: same acceleration for every mass
    std::vector<Body> bodies = { Body::makeCircle({ 0, 0 }, 5, 1.0f), Body::makeCircle({ 0, 0 }, 5, 10.0f),
                                 Body::makeCircle({ 0, 0 }, 5, 0.0f) };
    GravityForce gravity({ 0, 900 });
    gravity.apply(bodies);
    check(near(bodies[0].force.y * bodies[0].invMass, 900.0f, 1e-3f) &&
          near(bodies[1].force.y * bodies[1].invMass, 900.0f, 1e-3f),
          "gravity gives every mass the same acceleration (F = m g)");
    check(bodies[2].force.y == 0.0f, "gravity ignores static bodies");

    // Drag: opposes motion; heavier body decelerates less
    std::vector<Body> moving = { Body::makeCircle({ 0, 0 }, 5, 1.0f), Body::makeCircle({ 0, 0 }, 5, 4.0f) };
    moving[0].velocity = { 100, 0 };
    moving[1].velocity = { 100, 0 };
    DragForce drag(0.1f, 0.001f);
    drag.apply(moving);
    float lightBodyDeceleration = -moving[0].force.x * moving[0].invMass;
    float heavyBodyDeceleration = -moving[1].force.x * moving[1].invMass;
    check(moving[0].force.x < 0.0f, "drag opposes velocity");
    check(heavyBodyDeceleration < lightBodyDeceleration,
          "heavier body slows down less under the same drag");

    // Spring: stretched pulls together, compressed pushes apart, equal and opposite
    std::vector<Body> pair = { Body::makeCircle({ 0, 0 }, 5), Body::makeCircle({ 150, 0 }, 5) };
    SpringForce spring(0, 1, 100.0f, 10.0f);
    spring.apply(pair);
    check(pair[0].force.x > 0.0f && pair[1].force.x < 0.0f, "stretched spring pulls ends together");
    check(near(pair[0].force.x, 500.0f, 1e-2f), "spring force = k * extension (10 * 50)");
    check(near(pair[0].force.x, -pair[1].force.x, 1e-4f), "spring forces are equal and opposite");

    std::vector<Body> squeezed = { Body::makeCircle({ 0, 0 }, 5), Body::makeCircle({ 50, 0 }, 5) };
    spring.apply(squeezed);
    check(squeezed[0].force.x < 0.0f, "compressed spring pushes ends apart");

    check(!SpringForce(0, 5, 10, 1).connects(2) && !SpringForce(1, 1, 10, 1).connects(2),
          "spring with missing or identical ends is rejected");

    // Polymorphism: generators used through base-class pointers
    std::vector<std::unique_ptr<ForceGenerator>> generators;
    generators.push_back(std::make_unique<GravityForce>(Vector2D{ 0, 100 }));
    generators.push_back(std::make_unique<DragForce>(1.0f, 0.0f));
    std::vector<Body> singleBody = { Body::makeCircle({ 0, 0 }, 5, 2.0f) };
    singleBody[0].velocity = { 10, 0 };
    for (const auto& generator : generators) generator->apply(singleBody);
    check(near(singleBody[0].force.y, 200.0f, 1e-3f) &&
          near(singleBody[0].force.x, -10.0f, 1e-3f),
          "virtual apply() runs the right generator through a base pointer");
    check(generators[0]->name() == "Gravity" && generators[1]->name() == "Drag", "virtual name()");

    generators[0]->setEnabled(false);
    check(!generators[0]->isEnabled(), "generators can be disabled");
}

static void testIntegrator() {
    section("Integration");

    // Constant force: v = a t after n steps (exact for semi-implicit Euler)
    std::vector<Body> bodies = { Body::makeCircle({ 0, 0 }, 5, 2.0f) };
    Integrator integrator;
    for (int stepIndex = 0; stepIndex < 60; ++stepIndex) {
        bodies[0].applyForce({ 20.0f, 0.0f }); // a = 10
        integrator.step(bodies, 1.0f / 60.0f);
    }
    check(near(bodies[0].velocity.x, 10.0f, 1e-3f), "constant force: v = a t after 1 s");
    check(bodies[0].force.x == 0.0f, "integrator clears forces after each step");

    std::vector<Body> staticBodies = { Body::makeBox({ 5, 5 }, { 1, 1 }, 0.0f) };
    staticBodies[0].applyForce({ 1000, 1000 });
    integrator.step(staticBodies, 1.0f);
    check(staticBodies[0].position.x == 5.0f, "integrator never moves static bodies");

    // Energy: semi-implicit stays bounded on a spring, explicit grows
    auto springEnergy = [](IntegrationMethod method) {
        std::vector<Body> bodies = { Body::makeBox({ 0, 0 }, { 1, 1 }, 0.0f), Body::makeCircle({ 120, 0 }, 5, 1.0f) };
        SpringForce spring(0, 1, 100.0f, 100.0f);
        Integrator methodIntegrator(method);
        for (int stepIndex = 0; stepIndex < 600; ++stepIndex) {
            spring.apply(bodies);
            methodIntegrator.step(bodies, 1.0f / 60.0f);
        }
        float extension = bodies[1].position.x - 100.0f;
        return 0.5f * bodies[1].velocity.lengthSquared() +
          0.5f * 100.0f * extension * extension;
    };
    const float startEnergy = 0.5f * 100.0f * 20.0f * 20.0f;
    float semiImplicitEnergy = springEnergy(IntegrationMethod::SemiImplicitEuler);
    float explicitEnergy = springEnergy(IntegrationMethod::ExplicitEuler);
    check(std::fabs(semiImplicitEnergy - startEnergy) / startEnergy < 0.2f,
          "semi-implicit Euler keeps spring energy bounded (10 s)",
          "start " + fmt(startEnergy) + " end " + fmt(semiImplicitEnergy));
    check(explicitEnergy > 2.0f * startEnergy, "explicit Euler gains energy (why it is not used)",
          "end " + fmt(explicitEnergy));
}

// Flies the projectile preset's launch with no drag and returns the measured range
static float measureRange(float deltaTime) {
    World world(WINDOW_WIDTH, WINDOW_HEIGHT);
    world.loadPreset(static_cast<int>(Preset::Projectile));
    int projectileIndex = static_cast<int>(world.getBodies().size()) - 1;

    const float startX = world.getBodies()[projectileIndex].position.x;
    const float startY = world.getBodies()[projectileIndex].position.y;

    // The landing step can also contain the floor bounce, which flips the
    // velocity, so the crossing is detected from position once past the apex
    float previousX = startX, previousY = startY;
    bool pastApex = false;
    for (int stepIndex = 0; stepIndex < 100000; ++stepIndex) {
        world.step(deltaTime);
        const Body& projectile = world.getBodies()[projectileIndex];
        if (projectile.velocity.y > 0.0f) pastApex = true;
        if (pastApex && projectile.position.y >= startY) {
            // Interpolate the exact crossing of the launch height
            float interpolationFactor = (startY - previousY) /
              (projectile.position.y - previousY);
            return (previousX + interpolationFactor * (projectile.position.x - previousX)) - startX;
        }
        previousX = projectile.position.x;
        previousY = projectile.position.y;
    }
    return -1.0f;
}

static void testProjectile() {
    section("Projectile accuracy");
    const float gravityAcceleration = 900.0f;
    const float angle = PROJECTILE_ANGLE_DEGREES * 3.14159265f / 180.0f;
    const float expected = PROJECTILE_SPEED * PROJECTILE_SPEED *
      std::sin(2.0f * angle) / gravityAcceleration;

    float rangeAt60Hz = measureRange(1.0f / 60.0f);
    float rangeAt240Hz = measureRange(1.0f / 240.0f);
    float percentErrorAt60Hz = std::fabs(rangeAt60Hz - expected) / expected * 100.0f;
    float percentErrorAt240Hz = std::fabs(rangeAt240Hz - expected) / expected * 100.0f;

    std::cout << "         expected range " << fmt(expected) << " px; measured "
              << fmt(rangeAt60Hz) << " (1/60 s, " << fmt(percentErrorAt60Hz) << "% off), "
              << fmt(rangeAt240Hz) << " (1/240 s, " << fmt(percentErrorAt240Hz) << "% off)\n";
    check(percentErrorAt60Hz < 3.0f, "range matches v^2 sin(2a) / g within 3% at 60 steps/s");
    check(percentErrorAt240Hz < percentErrorAt60Hz && percentErrorAt240Hz < 1.0f,
          "error shrinks with a smaller timestep (within 1% at 240 steps/s)");
}

static void testSpringPeriod() {
    section("Spring oscillation");
    World world(WINDOW_WIDTH, WINDOW_HEIGHT);
    world.gravity().setEnabled(false);
    disableDrag(world);

    int anchor = world.addBody(Body::makeBox({ 400, 300 }, { 5, 5 }, 0.0f));
    int oscillatorBodyIndex = world.addBody(Body::makeCircle({ 520, 300 }, 8, 1.0f)); // stretched by 20
    world.addSpring(SpringForce(anchor, oscillatorBodyIndex, 100.0f, 100.0f));

    const float deltaTime = 1.0f / 240.0f;
    float lastVelocitySign = 0.0f, firstCrossingTime = -1.0f, thirdCrossingTime = -1.0f;
    int crossings = 0;
    for (int stepIndex = 1; stepIndex < 2400 && thirdCrossingTime < 0.0f; ++stepIndex) {
        world.step(deltaTime);
        float horizontalVelocity = world.getBodies()[oscillatorBodyIndex].velocity.x;
        float velocitySign = (horizontalVelocity > 0.0f) ? 1.0f :
          (horizontalVelocity < 0.0f ? -1.0f : 0.0f);
        if (lastVelocitySign != 0.0f && velocitySign != 0.0f &&
            velocitySign != lastVelocitySign) {
            ++crossings;
            if (crossings == 1) firstCrossingTime = stepIndex * deltaTime;
            if (crossings == 3) thirdCrossingTime = stepIndex * deltaTime;
        }
        if (velocitySign != 0.0f) lastVelocitySign = velocitySign;
    }
    float measuredPeriod = thirdCrossingTime - firstCrossingTime; // two half-periods
    float expectedPeriod = 2.0f * 3.14159265f * std::sqrt(1.0f / 100.0f);
    check(near(measuredPeriod, expectedPeriod, expectedPeriod * 0.02f),
          "period matches 2*pi*sqrt(m/k) within 2%",
          "measured " + fmt(measuredPeriod) + " s, expected " + fmt(expectedPeriod) + " s");
}

static void testWorld() {
    section("World");
    World world(WINDOW_WIDTH, WINDOW_HEIGHT);
    check(world.getBodies().size() == 3, "new world has floor and two walls");
    bool allBodiesStatic = true;
    for (const Body& body : world.getBodies()) {
        allBodiesStatic = allBodiesStatic && body.isStatic();
    }
    check(allBodiesStatic, "floor and walls are static");
    check(world.getHistory().undoCount() == 0, "starting scene is not on the undo stack");

    // Fixed timestep: velocity counts the steps taken (drag off, g*dt = 15 per step)
    disableDrag(world);
    int bodyIndex = world.addBody(Body::makeCircle({ 640, 100 }, 10));
    world.update(0.055f); // 3.3 steps' worth: 3 full steps, remainder carried over
    check(near(world.getBodies()[bodyIndex].velocity.y, 45.0f, 1e-2f),
          "update(0.055 s) runs 3 fixed steps",
          "vy " + fmt(world.getBodies()[bodyIndex].velocity.y));
    world.update(1.0f); // capped: at most 5 steps
    check(near(world.getBodies()[bodyIndex].velocity.y, 120.0f, 1e-2f),
          "a long frame is capped at 5 steps",
          "vy " + fmt(world.getBodies()[bodyIndex].velocity.y));

    check(world.bodyAt(world.getBodies()[bodyIndex].position) == bodyIndex, "bodyAt finds the body");
    check(world.bodyAt({ 640, 400 }) == -1, "bodyAt on empty space returns -1");

    world.dragBody(bodyIndex, { 300, 300 });
    check(near(world.getBodies()[bodyIndex].position.x, 300.0f, 1e-4f) &&
          world.getBodies()[bodyIndex].velocity.length() == 0.0f,
          "dragBody moves and stills the body");
    world.dragBody(0, { 0, 0 });
    check(world.getBodies()[0].position.x == WINDOW_WIDTH / 2.0f, "dragBody ignores static bodies");
    world.dragBody(99, { 0, 0 });
    check(true, "dragBody with a bad index is ignored safely");
}

static void testWorldBehaviour() {
    section("Simulation behaviour");
    const float ground = floorTop(WINDOW_HEIGHT);

    // Every shape lands on the floor across the whole width
    const char* shapeNames[] = { "circle", "box", "triangle", "hexagon" };
    float horizontalPositions[] = { 50, 400, 640, 900, 1230 };
    for (int shapeIndex = 0; shapeIndex < 4; ++shapeIndex) {
        bool shapeLandedAtEveryPosition = true;
        std::string failedPositions;
        for (float positionX : horizontalPositions) {
            World world(WINDOW_WIDTH, WINDOW_HEIGHT);
            Body spawnedBody = (shapeIndex == 0) ? Body::makeCircle({ positionX, 400 }, 20)
                               : (shapeIndex == 1) ? Body::makeBox({ positionX, 400 }, { 20, 20 })
                               : (shapeIndex == 2) ? Body::makeRegularPolygon({ positionX, 400 }, 3, 20)
                                                   : Body::makeRegularPolygon({ positionX, 400 }, 6, 20);
            int bodyIndex = world.addBody(spawnedBody);
            runSteps(world, 300);
            const Body& settledBody = world.getBodies()[bodyIndex];
            if (!(near(settledBody.aabbMax().y, ground, 3.0f) &&
                  std::fabs(settledBody.velocity.y) < 0.5f)) {
                shapeLandedAtEveryPosition = false;
                failedPositions += " x=" + fmt(positionX);
            }
        }
        check(shapeLandedAtEveryPosition,
              std::string(shapeNames[shapeIndex]) + " lands and rests on the floor at 5 positions",
              "failed at" + failedPositions);
    }

    // Bounce then settle
    {
        World world(WINDOW_WIDTH, WINDOW_HEIGHT);
        int bodyIndex = world.addBody(Body::makeCircle({ 300, 100 }, 20));
        bool bounced = false;
        for (int frameIndex = 0; frameIndex < 600; ++frameIndex) {
            world.step(FIXED_TIMESTEP);
            if (world.getBodies()[bodyIndex].velocity.y < -50.0f) bounced = true;
        }
        check(bounced, "a high drop bounces");
        check(std::fabs(world.getBodies()[bodyIndex].velocity.y) < 0.5f, "and then settles");
    }

    // Two-body stack
    {
        World world(WINDOW_WIDTH, WINDOW_HEIGHT);
        int boxBodyIndex = world.addBody(Body::makeBox({ 300, ground - 40 }, { 20, 20 }));
        int hexagonBodyIndex = world.addBody(Body::makeRegularPolygon({ 300, ground - 100 }, 6, 20));
        runSteps(world, 400);
        bool stable = std::fabs(world.getBodies()[boxBodyIndex].velocity.y) < 1.0f &&
                      std::fabs(world.getBodies()[hexagonBodyIndex].velocity.y) < 1.0f &&
                      world.getBodies()[boxBodyIndex].aabbMax().y < ground + 4.0f;
        check(stable, "a two-body stack is stable");
    }

    // Friction stops a sliding box
    {
        World world(WINDOW_WIDTH, WINDOW_HEIGHT);
        Body slidingBox = Body::makeBox({ 200, ground - 20 }, { 20, 20 });
        slidingBox.velocity = { 300, 0 };
        int bodyIndex = world.addBody(slidingBox);
        runSteps(world, 240);
        check(std::fabs(world.getBodies()[bodyIndex].velocity.x) < 0.5f, "friction stops a sliding box");
    }

    // Drag slows a body in flight
    {
        World worldWithDrag(WINDOW_WIDTH, WINDOW_HEIGHT);
        World worldWithoutDrag(WINDOW_WIDTH, WINDOW_HEIGHT);
        disableDrag(worldWithoutDrag);
        Body movingBody = Body::makeCircle({ 640, 100 }, 10);
        movingBody.velocity = { 400, 0 };
        int draggedBodyIndex = worldWithDrag.addBody(movingBody);
        int undraggedBodyIndex = worldWithoutDrag.addBody(movingBody);
        runSteps(worldWithDrag, 20);
        runSteps(worldWithoutDrag, 20);
        check(worldWithDrag.getBodies()[draggedBodyIndex].velocity.x <
              worldWithoutDrag.getBodies()[undraggedBodyIndex].velocity.x,
              "air drag reduces horizontal speed during flight");
    }

    // Walls keep fast bodies inside
    {
        World world(WINDOW_WIDTH, WINDOW_HEIGHT);
        Body fastBody = Body::makeCircle({ 640, ground - 20 }, 20);
        fastBody.velocity = { 1500, 0 };
        int bodyIndex = world.addBody(fastBody);
        runSteps(world, 240);
        float bodyPositionX = world.getBodies()[bodyIndex].position.x;
        check(bodyPositionX > 0.0f && bodyPositionX < WINDOW_WIDTH,
              "side walls keep a fast body inside the window", "x=" + fmt(bodyPositionX));
    }

    // Stress test
    {
        World world(WINDOW_WIDTH, WINDOW_HEIGHT);
        for (int bodyIndex = 0; bodyIndex < 100; ++bodyIndex) {
            Vector2D spawnPosition{ 100.0f + (bodyIndex % 20) * 54.0f,
                                    100.0f + (bodyIndex / 20) * 50.0f };
            int shapeIndex = bodyIndex % 3;
            world.addBody(shapeIndex == 0 ? Body::makeCircle(spawnPosition, 15)
                        : shapeIndex == 1 ? Body::makeBox(spawnPosition, { 15, 15 })
                                          : Body::makeRegularPolygon(spawnPosition, 5, 15));
        }
        auto simulationStart = std::chrono::high_resolution_clock::now();
        runSteps(world, 300);
        auto simulationEnd = std::chrono::high_resolution_clock::now();
        float averageStepTimeMilliseconds =
          std::chrono::duration<float, std::milli>(simulationEnd - simulationStart).count() / 300.0f;

        bool allBodiesFinite = true, allBodiesInside = true;
        for (const Body& body : world.getBodies()) {
            allBodiesFinite = allBodiesFinite && isFinite(body);
            allBodiesInside = allBodiesInside && body.position.y < WINDOW_HEIGHT &&
                              body.position.x > 0.0f && body.position.x < WINDOW_WIDTH;
        }
        check(allBodiesFinite, "100 bodies: no NaN or infinity");
        check(allBodiesInside, "100 bodies: none escape the floor or walls");
        check(averageStepTimeMilliseconds < 8.0f, "100 bodies: physics fits the frame budget",
              fmt(averageStepTimeMilliseconds) + " ms/step");
        std::cout << "         (" << fmt(averageStepTimeMilliseconds)
                  << " ms per step; 16.7 ms available per frame)\n";
    }
}

static void testEventsAndHistory() {
    section("Event queue");
    EventQueue eventQueue;
    eventQueue.push(SandboxEvent::preset(1));
    eventQueue.push(SandboxEvent::preset(2));
    eventQueue.push(SandboxEvent::preset(3));
    SandboxEvent event;
    bool eventsMaintainFifoOrder = eventQueue.pop(event) && event.value == 1 &&
      eventQueue.pop(event) && event.value == 2 &&
      eventQueue.pop(event) && event.value == 3;
    check(eventsMaintainFifoOrder, "events come out in the order they went in (FIFO)");
    check(!eventQueue.pop(event) && eventQueue.empty(), "pop on an empty queue returns false");
    eventQueue.push(SandboxEvent::undo());
    eventQueue.clear();
    check(eventQueue.empty(), "clear empties the queue");

    World world(WINDOW_WIDTH, WINDOW_HEIGHT);
    std::size_t initialBodyCount = world.getBodies().size();
    eventQueue.push(SandboxEvent::spawn(Body::makeCircle({ 300, 300 }, 10)));
    eventQueue.push(SandboxEvent::spawn(Body::makeCircle({ 400, 300 }, 10)));
    world.processEvents(eventQueue);
    check(world.getBodies().size() == initialBodyCount + 2 && eventQueue.empty(),
          "processEvents applies spawns and empties the queue");

    eventQueue.push(SandboxEvent::undo());
    world.processEvents(eventQueue);
    check(world.getBodies().size() == initialBodyCount + 1, "undo removes the last spawn");
    eventQueue.push(SandboxEvent::redo());
    world.processEvents(eventQueue);
    check(world.getBodies().size() == initialBodyCount + 2, "redo brings it back");

    eventQueue.push(SandboxEvent::reset());
    world.processEvents(eventQueue);
    check(world.getBodies().size() == initialBodyCount, "reset clears to floor and walls");
    eventQueue.push(SandboxEvent::undo());
    world.processEvents(eventQueue);
    check(world.getBodies().size() == initialBodyCount + 2, "reset can be undone");

    eventQueue.push(SandboxEvent::preset(99));
    std::size_t bodyCountBeforeInvalidPreset = world.getBodies().size();
    world.processEvents(eventQueue);
    check(world.getBodies().size() == bodyCountBeforeInvalidPreset,
          "an invalid preset number is ignored");

    WorldState invalidScene = world.getState();
    invalidScene.springs.emplace_back(0, 500, 10.0f, 1.0f);
    eventQueue.push(SandboxEvent::replaceScene(invalidScene));
    world.processEvents(eventQueue);
    check(world.getSprings().empty(), "replaceScene drops springs pointing at missing bodies");

    section("Undo / redo stacks");
    SceneHistory history(3);
    WorldState currentState;
    check(!history.undo(currentState), "undo with empty history returns false");
    for (int snapshotIndex = 1; snapshotIndex <= 5; ++snapshotIndex) {
        currentState.bodies.assign(snapshotIndex - 1, Body());
        history.record(currentState); // records states with 0..4 bodies
    }
    currentState.bodies.assign(5, Body());
    check(history.undoCount() == 3, "capacity limits the undo stack (oldest dropped)");
    history.undo(currentState);
    check(currentState.bodies.size() == 4, "undo restores the most recent snapshot (LIFO)");
    history.undo(currentState);
    history.undo(currentState);
    check(currentState.bodies.size() == 2 && !history.undo(currentState),
          "oldest kept snapshot is the limit");
    history.redo(currentState);
    check(currentState.bodies.size() == 3, "redo moves forward again");
    history.record(currentState);
    check(!history.canRedo(), "a new action clears the redo stack");
}

static void testPresets() {
    section("Presets");
    for (int presetNumber = 1; presetNumber <= PRESET_COUNT; ++presetNumber) {
        World world(WINDOW_WIDTH, WINDOW_HEIGHT);
        bool presetLoaded = world.loadPreset(presetNumber) && world.getBodies().size() > 3;
        check(presetLoaded, "preset " + std::to_string(presetNumber) + " loads: " +
              presetName(static_cast<Preset>(presetNumber)));
    }
    World emptyWorld(WINDOW_WIDTH, WINDOW_HEIGHT);
    check(!emptyWorld.loadPreset(0) && !emptyWorld.loadPreset(PRESET_COUNT + 1),
          "out-of-range presets are refused");

    // Elastic: velocities swap
    {
        World world(WINDOW_WIDTH, WINDOW_HEIGHT);
        world.loadPreset(static_cast<int>(Preset::ElasticCollision));
        runSteps(world, 120);
        const auto& bodies = world.getBodies();
        float movingBodySpeed = bodies[3].velocity.x, restingBodySpeed = bodies[4].velocity.x;
        check(near(movingBodySpeed, 0.0f, 3.0f) && near(restingBodySpeed, 300.0f, 3.0f),
              "elastic preset: moving ball stops, resting ball leaves at 300",
              "v1=" + fmt(movingBodySpeed) + " v2=" + fmt(restingBodySpeed));
    }

    // Inelastic: common velocity 4*300/5 = 240, momentum conserved
    {
        World world(WINDOW_WIDTH, WINDOW_HEIGHT);
        world.loadPreset(static_cast<int>(Preset::InelasticCollision));
        runSteps(world, 120);
        const auto& bodies = world.getBodies();
        float heavyBodySpeed = bodies[3].velocity.x, lightBodySpeed = bodies[4].velocity.x;
        float momentum = bodies[3].mass * heavyBodySpeed + bodies[4].mass * lightBodySpeed;
        check(near(heavyBodySpeed, 240.0f, 5.0f) && near(lightBodySpeed, 240.0f, 5.0f),
              "inelastic preset: both move at 240",
              "heavy=" + fmt(heavyBodySpeed) + " light=" + fmt(lightBodySpeed));
        check(near(momentum, 1200.0f, 12.0f), "inelastic preset: momentum conserved (1200)", fmt(momentum));
    }

    // Free fall: different masses released together stay level (a = g for all)
    {
        World world(WINDOW_WIDTH, WINDOW_HEIGHT);
        world.loadPreset(static_cast<int>(Preset::FreeFall));
        runSteps(world, 30); // 0.5 s, before any of them reaches the floor
        const auto& bodies = world.getBodies();
        bool bodiesStayLevel = near(bodies[3].position.y, bodies[4].position.y, 0.01f) &&
                               near(bodies[4].position.y, bodies[5].position.y, 0.01f);
        bool bodiesHaveSameSpeed = near(bodies[3].velocity.y, bodies[5].velocity.y, 0.01f);
        check(bodies.size() == 6 && bodies[3].mass != bodies[5].mass,
              "free fall preset: three bodies of different mass");
        check(bodiesStayLevel && bodiesHaveSameSpeed, "free fall preset: all masses fall together",
              "y=" + fmt(bodies[3].position.y) + "," + fmt(bodies[4].position.y) + "," +
              fmt(bodies[5].position.y));
        check(near(bodies[3].velocity.y, 450.0f, 2.0f), "free fall preset: v = g t (900 * 0.5 = 450)",
              fmt(bodies[3].velocity.y));
    }
}

static void testLauncher() {
    section("Launcher and editable values");
    using namespace sandbox;

    Vector2D upwardVelocity = launchVelocity(90.0f, 100.0f);
    Vector2D rightwardVelocity = launchVelocity(0.0f, 100.0f);
    check(near(upwardVelocity.x, 0.0f, 1e-3f) && near(upwardVelocity.y, -100.0f, 1e-3f),
          "90 degrees points straight up");
    check(near(rightwardVelocity.x, 100.0f, 1e-3f) && near(rightwardVelocity.y, 0.0f, 1e-3f),
          "0 degrees points right");

    AppSettings settings;
    settings.launchAngleDegrees = -10.0f;
    settings.launchSpeed = 5000.0f;
    settings.launchMass = 0.0f;
    clampLaunchSettings(settings);
    check(near(settings.launchAngleDegrees, 350.0f, 1e-3f), "angle wraps into 0-360");
    check(near(settings.launchSpeed, MAX_LAUNCH_SPEED, 1e-3f), "speed is limited to the feasible maximum");
    check(near(settings.launchMass, MIN_BODY_MASS, 1e-3f), "mass is limited to the feasible minimum");

    AppSettings shotSettings;
    shotSettings.currentShape = SpawnShape::Square;
    shotSettings.launchAngleDegrees = 30.0f;
    shotSettings.launchSpeed = 400.0f;
    shotSettings.launchMass = 7.5f;
    Body launchedBody = createLaunchedBody({ 100, 100 }, shotSettings);
    check(launchedBody.shapeType == ShapeType::Polygon && launchedBody.localVertices.size() == 4,
          "launched body has the chosen shape");
    check(near(launchedBody.mass, 7.5f, 1e-4f) && near(launchedBody.velocity.length(), 400.0f, 1e-2f),
          "launched body has the chosen mass and speed");

    AppSettings selectedBodySettings;
    readBodyIntoSettings(launchedBody, selectedBodySettings);
    check(near(selectedBodySettings.launchAngleDegrees, 30.0f, 0.01f) &&
          near(selectedBodySettings.launchSpeed, 400.0f, 0.01f) &&
          near(selectedBodySettings.launchMass, 7.5f, 1e-4f),
          "selecting a body reads back its angle, speed and mass");

    Vector2D launchPosition = launchPoint(WINDOW_HEIGHT, 20.0f);
    check(launchPosition.y + 20.0f < floorTop(WINDOW_HEIGHT) && launchPosition.x > WALL_THICKNESS,
          "launch point is inside the arena, above the floor");

    // Many shots in a row, fired whenever the launcher is clear (as the program does),
    // all fly and stay inside the arena
    World world(WINDOW_WIDTH, WINDOW_HEIGHT);
    world.loadPreset(static_cast<int>(Preset::Projectile));
    AppSettings rapidFireSettings;
    rapidFireSettings.launchSpeed = MAX_LAUNCH_SPEED;
    const Vector2D rapidFirePosition = launchPoint(WINDOW_HEIGHT, rapidFireSettings.spawnSize);
    int shotsFired = 0;
    for (int stepIndex = 0; stepIndex < 1200 && shotsFired < 20; ++stepIndex) {
        if (isAreaClear(world.getState(), rapidFirePosition, rapidFireSettings.spawnSize)) {
            rapidFireSettings.launchAngleDegrees =
              10.0f + 4.0f * static_cast<float>(shotsFired);
            world.addBody(createLaunchedBody(rapidFirePosition, rapidFireSettings));
            ++shotsFired;
        }
        world.step(FIXED_TIMESTEP);
    }
    check(!isAreaClear(world.getState(), world.getBodies().back().position, 1.0f),
          "the launcher reports blocked while a body sits on it");
    runSteps(world, 600);
    bool allBodiesInside = true;
    for (const Body& body : world.getBodies()) {
        if (body.isStatic()) continue;
        allBodiesInside = allBodiesInside && isFinite(body) && body.position.x > 0.0f &&
                          body.position.x < WINDOW_WIDTH && body.position.y < WINDOW_HEIGHT;
    }
    check(countDynamicBodies(world.getState()) == 21, "20 extra shots added (21 moving bodies)");
    check(allBodiesInside, "20 shots at maximum speed all stay inside the arena");
}

static void testSceneFiles() {
    section("Scene files and shape creation");
    using namespace sandbox;

    Body triangleBody = createShape({ 200, 100 }, SpawnShape::Triangle, 20);
    check(triangleBody.shapeType == ShapeType::Polygon && triangleBody.localVertices.size() == 3,
          "createShape: triangle");
    Body squareBody = createShape({ 200, 100 }, SpawnShape::Square, 20);
    check(squareBody.shapeType == ShapeType::Polygon && squareBody.localVertices.size() == 4,
          "createShape: square");
    check(createShape({ 0, 0 }, SpawnShape::Circle, 20).shapeType == ShapeType::Circle, "createShape: circle");
    check(shapeName(SpawnShape::Square) == "Square", "shapeName");

    // The program no longer creates springs, but the file format still supports them
    World world(WINDOW_WIDTH, WINDOW_HEIGHT);
    world.loadPreset(static_cast<int>(Preset::FreeFall));
    WorldState original = world.getState();
    original.springs.emplace_back(3, 4, 100.0f, 10.0f);
    original.settings.restitution = 0.7f;

    const std::string sceneFilePath = "test_scene.txt";
    check(saveScene(original, sceneFilePath), "saveScene writes the file");

    WorldState loaded;
    bool sceneLoaded = loadScene(loaded, sceneFilePath);
    check(sceneLoaded && loaded.bodies.size() == original.bodies.size(), "loadScene reads all bodies back");
    check(sceneLoaded && loaded.springs.size() == original.springs.size(), "springs survive save and load");
    check(sceneLoaded && near(loaded.settings.restitution, 0.7f, 1e-4f), "settings survive save and load");
    check(sceneLoaded && loaded.bodies[0].isStatic(), "static bodies stay static");

    { std::ofstream oldSceneFile("test_old.txt"); oldSceneFile << "1\n0 100 100 0 0 1 10 10 10 0\n"; }
    WorldState oldScene;
    check(loadScene(oldScene, "test_old.txt") && oldScene.bodies.size() == 1 && oldScene.springs.empty(),
          "files without spring/settings sections still load");

    { std::ofstream damagedSceneFile("test_bad.txt"); damagedSceneFile << "2\n0 1 2 3 4 1 10 10 10 0\n7 garbage\n"; }
    WorldState preservedState = original;
    check(!loadScene(preservedState, "test_bad.txt") &&
          preservedState.bodies.size() == original.bodies.size(),
          "a damaged file is rejected and the scene is kept");

    { std::ofstream invalidSpringSceneFile("test_badspring.txt"); invalidSpringSceneFile << "1\n0 1 1 0 0 1 10 10 10 0\nsprings 1\n0 9 10 1 0\n"; }
    check(!loadScene(preservedState, "test_badspring.txt"),
          "a spring pointing at a missing body is rejected");

    check(!loadScene(preservedState, "does_not_exist.txt"), "a missing file returns false");

    std::remove(sceneFilePath.c_str());
    std::remove("test_old.txt");
    std::remove("test_bad.txt");
    std::remove("test_badspring.txt");
}

// ============================================================
int main() {
    std::cout << "ForceForge automated tests\n";

    testVector2D();
    testBody();
    testNarrowPhase();
    testResponse();
    testForces();
    testIntegrator();
    testProjectile();
    testSpringPeriod();
    testWorld();
    testWorldBehaviour();
    testEventsAndHistory();
    testPresets();
    testLauncher();
    testSceneFiles();

    std::cout << "\n----------------------------------------\n";
    std::cout << "Passed: " << passed << "   Failed: " << failed << "\n";
    std::cout << (failed == 0 ? "ALL TESTS PASSED\n" : "SOME TESTS FAILED\n");
    return failed == 0 ? 0 : 1;
}
