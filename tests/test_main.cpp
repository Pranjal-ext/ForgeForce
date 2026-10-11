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

static bool near(float a, float b, float tolerance) { return std::fabs(a - b) <= tolerance; }

static std::string fmt(float value) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(3) << value;
    return s.str();
}

static void section(const std::string& title) { std::cout << "\n== " << title << " ==\n"; }

static const float W = 1280.0f;
static const float H = 720.0f;
static const float DT = World::FIXED_TIMESTEP;

static void runSteps(World& world, int steps, float dt = DT) {
    for (int i = 0; i < steps; ++i) world.step(dt);
}

static bool isFinite(const Body& b) {
    return std::isfinite(b.position.x) && std::isfinite(b.position.y) &&
           std::isfinite(b.velocity.x) && std::isfinite(b.velocity.y);
}

// A world with drag off, for tests that need exact textbook motion
static void disableDrag(World& world) {
    PhysicsSettings s = world.getSettings();
    s.dragEnabled = false;
    world.setSettings(s);
}

// ============================================================
static void testVector2D() {
    section("Vector2D");
    Vector2D a(3.0f, 4.0f);
    check(near(a.length(), 5.0f, 1e-5f), "length of (3,4) is 5");
    check(near(a.normalized().length(), 1.0f, 1e-5f), "normalized vector has length 1");
    check(Vector2D(0, 0).normalized().length() == 0.0f, "normalizing a zero vector is safe");
    check(near(a.dot(Vector2D(1, 0)), 3.0f, 1e-5f), "dot product");
    check(Vector2D(1, 0).cross(Vector2D(0, 1)) > 0.0f, "cross product sign");
}

static void testBody() {
    section("Body");
    Body c = Body::makeCircle({ 0, 0 }, 10.0f, 2.0f);
    check(c.mass == 2.0f && near(c.invMass, 0.5f, 1e-6f), "setMass keeps invMass = 1/mass");

    Body s = Body::makeBox({ 0, 0 }, { 5, 5 }, 0.0f);
    check(s.isStatic() && s.invMass == 0.0f, "mass 0 makes a body static");

    // Lowest point on screen = largest y; a flat bottom means two vertices share it
    std::vector<Vector2D> sq = Body::regularPolygonVertices(4, 10.0f);
    float lowest = -1e9f;
    for (const Vector2D& v : sq) lowest = std::max(lowest, v.y);
    int onBottom = 0;
    for (const Vector2D& v : sq) if (near(v.y, lowest, 1e-4f)) ++onBottom;
    check(sq.size() == 4 && onBottom == 2, "regular square has a flat bottom edge");
    check(Body::regularPolygonVertices(2, 10.0f).empty(), "fewer than 3 sides gives no polygon");

    Body hex = Body::makeRegularPolygon({ 100, 100 }, 6, 20.0f);
    check(hex.containsPoint({ 100, 100 }) && !hex.containsPoint({ 125, 100 }), "polygon containsPoint");
    check(near(hex.aabbMax().x - hex.aabbMin().x, 40.0f, 0.5f) ||
          near(hex.aabbMax().x - hex.aabbMin().x, 34.64f, 0.5f), "polygon bounding box width");

    Body f = Body::makeCircle({ 0, 0 }, 5.0f);
    f.applyForce({ 1, 2 });
    f.applyForce({ 3, 4 });
    check(near(f.force.x, 4.0f, 1e-6f) && near(f.force.y, 6.0f, 1e-6f), "applyForce accumulates");
    f.clearForce();
    check(f.force.x == 0.0f && f.force.y == 0.0f, "clearForce resets");
}

static void testNarrowPhase() {
    section("Collision detection");
    Manifold m = testCollision(Body::makeCircle({ 0, 0 }, 10), Body::makeCircle({ 15, 0 }, 10));
    check(m.colliding && near(m.normal.x, 1.0f, 1e-4f) && near(m.penetration, 5.0f, 1e-4f),
          "circle-circle: normal points A->B, depth 5");
    check(!testCollision(Body::makeCircle({ 0, 0 }, 10), Body::makeCircle({ 25, 0 }, 10)).colliding,
          "circle-circle with gap: no collision");

    m = testCollision(Body::makeCircle({ 0, 0 }, 10), Body::makeCircle({ 0, 0 }, 10));
    check(m.colliding && std::isfinite(m.normal.x), "identical positions: no NaN");

    m = testCollision(Body::makeCircle({ 0, 0 }, 10), Body::makeBox({ 15, 0 }, { 10, 10 }));
    check(m.colliding && m.normal.x > 0.9f, "circle-box: normal points circle->box");
    m = testCollision(Body::makeBox({ 15, 0 }, { 10, 10 }), Body::makeCircle({ 0, 0 }, 10));
    check(m.colliding && m.normal.x < -0.9f, "box-circle (reversed order): normal flips");

    check(testCollision(Body::makeBox({ 0, 0 }, { 10, 10 }), Body::makeBox({ 15, 0 }, { 10, 10 })).colliding,
          "box-box overlap");
    check(!testCollision(Body::makeBox({ 0, 0 }, { 10, 10 }), Body::makeBox({ 25, 0 }, { 10, 10 })).colliding,
          "box-box with gap");

    m = testCollision(Body::makeRegularPolygon({ 0, 0 }, 6, 20), Body::makeRegularPolygon({ 30, 0 }, 6, 20));
    check(m.colliding && m.normal.x > 0.5f, "hexagon-hexagon (SAT) overlap");
    check(!testCollision(Body::makeRegularPolygon({ 0, 0 }, 6, 20),
                         Body::makeRegularPolygon({ 60, 0 }, 6, 20)).colliding,
          "hexagon-hexagon with gap");

    check(testCollision(Body::makeCircle({ 0, 0 }, 10), Body::makeRegularPolygon({ 20, 0 }, 3, 20)).colliding,
          "circle-triangle overlap");
    check(!testCollision(Body::makeCircle({ 0, 0 }, 10), Body::makeRegularPolygon({ 25, 0 }, 3, 20)).colliding,
          "circle-triangle near miss (edge 11.7 away, radius 10)");

    Body floor = Body::makeBox({ 640, 700 }, { 640, 20 }, 0.0f);
    m = testCollision(Body::makeCircle({ 200, 685 }, 20), floor);
    check(m.colliding && m.normal.y > 0.9f, "circle centre inside floor resolves upward");
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

    std::vector<Body> statics = { Body::makeBox({ 0, 0 }, { 10, 10 }, 0.0f),
                                  Body::makeBox({ 5, 0 }, { 10, 10 }, 0.0f) };
    CollisionSystem sys(80.0f);
    sys.step(statics);
    check(statics[0].position.x == 0.0f && statics[1].position.x == 5.0f, "static bodies never move");
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
    float decelLight = -moving[0].force.x * moving[0].invMass;
    float decelHeavy = -moving[1].force.x * moving[1].invMass;
    check(moving[0].force.x < 0.0f, "drag opposes velocity");
    check(decelHeavy < decelLight, "heavier body slows down less under the same drag");

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
    std::vector<Body> one = { Body::makeCircle({ 0, 0 }, 5, 2.0f) };
    one[0].velocity = { 10, 0 };
    for (const auto& g : generators) g->apply(one);
    check(near(one[0].force.y, 200.0f, 1e-3f) && near(one[0].force.x, -10.0f, 1e-3f),
          "virtual apply() runs the right generator through a base pointer");
    check(generators[0]->name() == "Gravity" && generators[1]->name() == "Drag", "virtual name()");

    generators[0]->setEnabled(false);
    check(!generators[0]->isEnabled(), "generators can be disabled");
}

static void testIntegrator() {
    section("Integration");

    // Constant force: v = a t after n steps (exact for semi-implicit Euler)
    std::vector<Body> b = { Body::makeCircle({ 0, 0 }, 5, 2.0f) };
    Integrator integrator;
    for (int i = 0; i < 60; ++i) {
        b[0].applyForce({ 20.0f, 0.0f }); // a = 10
        integrator.step(b, 1.0f / 60.0f);
    }
    check(near(b[0].velocity.x, 10.0f, 1e-3f), "constant force: v = a t after 1 s");
    check(b[0].force.x == 0.0f, "integrator clears forces after each step");

    std::vector<Body> st = { Body::makeBox({ 5, 5 }, { 1, 1 }, 0.0f) };
    st[0].applyForce({ 1000, 1000 });
    integrator.step(st, 1.0f);
    check(st[0].position.x == 5.0f, "integrator never moves static bodies");

    // Energy: semi-implicit stays bounded on a spring, explicit grows
    auto springEnergy = [](IntegrationMethod method) {
        std::vector<Body> bodies = { Body::makeBox({ 0, 0 }, { 1, 1 }, 0.0f), Body::makeCircle({ 120, 0 }, 5, 1.0f) };
        SpringForce s(0, 1, 100.0f, 100.0f);
        Integrator in(method);
        for (int i = 0; i < 600; ++i) { s.apply(bodies); in.step(bodies, 1.0f / 60.0f); }
        float ext = bodies[1].position.x - 100.0f;
        return 0.5f * bodies[1].velocity.lengthSquared() + 0.5f * 100.0f * ext * ext;
    };
    const float startEnergy = 0.5f * 100.0f * 20.0f * 20.0f;
    float semi = springEnergy(IntegrationMethod::SemiImplicitEuler);
    float expl = springEnergy(IntegrationMethod::ExplicitEuler);
    check(std::fabs(semi - startEnergy) / startEnergy < 0.2f,
          "semi-implicit Euler keeps spring energy bounded (10 s)",
          "start " + fmt(startEnergy) + " end " + fmt(semi));
    check(expl > 2.0f * startEnergy, "explicit Euler gains energy (why it is not used)",
          "end " + fmt(expl));
}

// Flies the projectile preset's launch with no drag and returns the measured range
static float measureRange(float dt) {
    World world(W, H);
    world.loadPreset(static_cast<int>(Preset::Projectile));
    int ball = static_cast<int>(world.getBodies().size()) - 1;

    const float startX = world.getBodies()[ball].position.x;
    const float startY = world.getBodies()[ball].position.y;

    // The landing step can also contain the floor bounce, which flips the
    // velocity, so the crossing is detected from position once past the apex
    float prevX = startX, prevY = startY;
    bool pastApex = false;
    for (int i = 0; i < 100000; ++i) {
        world.step(dt);
        const Body& b = world.getBodies()[ball];
        if (b.velocity.y > 0.0f) pastApex = true;
        if (pastApex && b.position.y >= startY) {
            // Interpolate the exact crossing of the launch height
            float t = (startY - prevY) / (b.position.y - prevY);
            return (prevX + t * (b.position.x - prevX)) - startX;
        }
        prevX = b.position.x;
        prevY = b.position.y;
    }
    return -1.0f;
}

static void testProjectile() {
    section("Projectile accuracy");
    const float g = 900.0f;
    const float angle = PROJECTILE_ANGLE_DEGREES * 3.14159265f / 180.0f;
    const float expected = PROJECTILE_SPEED * PROJECTILE_SPEED * std::sin(2.0f * angle) / g;

    float r60 = measureRange(1.0f / 60.0f);
    float r240 = measureRange(1.0f / 240.0f);
    float e60 = std::fabs(r60 - expected) / expected * 100.0f;
    float e240 = std::fabs(r240 - expected) / expected * 100.0f;

    std::cout << "         expected range " << fmt(expected) << " px; measured "
              << fmt(r60) << " (1/60 s, " << fmt(e60) << "% off), "
              << fmt(r240) << " (1/240 s, " << fmt(e240) << "% off)\n";
    check(e60 < 3.0f, "range matches v^2 sin(2a) / g within 3% at 60 steps/s");
    check(e240 < e60 && e240 < 1.0f, "error shrinks with a smaller timestep (within 1% at 240 steps/s)");
}

static void testSpringPeriod() {
    section("Spring oscillation");
    World world(W, H);
    world.gravity().setEnabled(false);
    disableDrag(world);

    int anchor = world.addBody(Body::makeBox({ 400, 300 }, { 5, 5 }, 0.0f));
    int bob = world.addBody(Body::makeCircle({ 520, 300 }, 8, 1.0f)); // stretched by 20
    world.addSpring(SpringForce(anchor, bob, 100.0f, 100.0f));

    const float dt = 1.0f / 240.0f;
    float lastSign = 0.0f, firstCross = -1.0f, thirdCross = -1.0f;
    int crossings = 0;
    for (int i = 1; i < 2400 && thirdCross < 0.0f; ++i) {
        world.step(dt);
        float v = world.getBodies()[bob].velocity.x;
        float sign = (v > 0.0f) ? 1.0f : (v < 0.0f ? -1.0f : 0.0f);
        if (lastSign != 0.0f && sign != 0.0f && sign != lastSign) {
            ++crossings;
            if (crossings == 1) firstCross = i * dt;
            if (crossings == 3) thirdCross = i * dt;
        }
        if (sign != 0.0f) lastSign = sign;
    }
    float measured = thirdCross - firstCross; // two half-periods
    float expected = 2.0f * 3.14159265f * std::sqrt(1.0f / 100.0f);
    check(near(measured, expected, expected * 0.02f), "period matches 2*pi*sqrt(m/k) within 2%",
          "measured " + fmt(measured) + " s, expected " + fmt(expected) + " s");
}

static void testWorld() {
    section("World");
    World world(W, H);
    check(world.getBodies().size() == 3, "new world has floor and two walls");
    bool allStatic = true;
    for (const Body& b : world.getBodies()) allStatic = allStatic && b.isStatic();
    check(allStatic, "floor and walls are static");
    check(world.getHistory().undoCount() == 0, "starting scene is not on the undo stack");

    // Fixed timestep: velocity counts the steps taken (drag off, g*dt = 15 per step)
    disableDrag(world);
    int ball = world.addBody(Body::makeCircle({ 640, 100 }, 10));
    world.update(0.055f); // 3.3 steps' worth: 3 full steps, remainder carried over
    check(near(world.getBodies()[ball].velocity.y, 45.0f, 1e-2f), "update(0.055 s) runs 3 fixed steps",
          "vy " + fmt(world.getBodies()[ball].velocity.y));
    world.update(1.0f); // capped: at most 5 steps
    check(near(world.getBodies()[ball].velocity.y, 120.0f, 1e-2f), "a long frame is capped at 5 steps",
          "vy " + fmt(world.getBodies()[ball].velocity.y));

    check(world.bodyAt(world.getBodies()[ball].position) == ball, "bodyAt finds the body");
    check(world.bodyAt({ 640, 400 }) == -1, "bodyAt on empty space returns -1");

    world.dragBody(ball, { 300, 300 });
    check(near(world.getBodies()[ball].position.x, 300.0f, 1e-4f) &&
          world.getBodies()[ball].velocity.length() == 0.0f, "dragBody moves and stills the body");
    world.dragBody(0, { 0, 0 });
    check(world.getBodies()[0].position.x == W / 2.0f, "dragBody ignores static bodies");
    world.dragBody(99, { 0, 0 });
    check(true, "dragBody with a bad index is ignored safely");
}

static void testWorldBehaviour() {
    section("Simulation behaviour");
    const float ground = floorTop(H);

    // Every shape lands on the floor across the whole width
    const char* names[] = { "circle", "box", "triangle", "hexagon" };
    float xs[] = { 50, 400, 640, 900, 1230 };
    for (int s = 0; s < 4; ++s) {
        bool ok = true;
        std::string where;
        for (float x : xs) {
            World world(W, H);
            Body b = (s == 0) ? Body::makeCircle({ x, 400 }, 20)
                   : (s == 1) ? Body::makeBox({ x, 400 }, { 20, 20 })
                   : (s == 2) ? Body::makeRegularPolygon({ x, 400 }, 3, 20)
                              : Body::makeRegularPolygon({ x, 400 }, 6, 20);
            int i = world.addBody(b);
            runSteps(world, 300);
            const Body& r = world.getBodies()[i];
            if (!(near(r.aabbMax().y, ground, 3.0f) && std::fabs(r.velocity.y) < 0.5f)) {
                ok = false;
                where += " x=" + fmt(x);
            }
        }
        check(ok, std::string(names[s]) + " lands and rests on the floor at 5 positions", "failed at" + where);
    }

    // Bounce then settle
    {
        World world(W, H);
        int i = world.addBody(Body::makeCircle({ 300, 100 }, 20));
        bool bounced = false;
        for (int f = 0; f < 600; ++f) {
            world.step(DT);
            if (world.getBodies()[i].velocity.y < -50.0f) bounced = true;
        }
        check(bounced, "a high drop bounces");
        check(std::fabs(world.getBodies()[i].velocity.y) < 0.5f, "and then settles");
    }

    // Two-body stack
    {
        World world(W, H);
        int box = world.addBody(Body::makeBox({ 300, ground - 40 }, { 20, 20 }));
        int hex = world.addBody(Body::makeRegularPolygon({ 300, ground - 100 }, 6, 20));
        runSteps(world, 400);
        bool stable = std::fabs(world.getBodies()[box].velocity.y) < 1.0f &&
                      std::fabs(world.getBodies()[hex].velocity.y) < 1.0f &&
                      world.getBodies()[box].aabbMax().y < ground + 4.0f;
        check(stable, "a two-body stack is stable");
    }

    // Friction stops a sliding box
    {
        World world(W, H);
        Body b = Body::makeBox({ 200, ground - 20 }, { 20, 20 });
        b.velocity = { 300, 0 };
        int i = world.addBody(b);
        runSteps(world, 240);
        check(std::fabs(world.getBodies()[i].velocity.x) < 0.5f, "friction stops a sliding box");
    }

    // Drag slows a body in flight
    {
        World with(W, H), without(W, H);
        disableDrag(without);
        Body b = Body::makeCircle({ 640, 100 }, 10);
        b.velocity = { 400, 0 };
        int a = with.addBody(b), c = without.addBody(b);
        runSteps(with, 20);
        runSteps(without, 20);
        check(with.getBodies()[a].velocity.x < without.getBodies()[c].velocity.x,
              "air drag reduces horizontal speed during flight");
    }

    // Walls keep fast bodies inside
    {
        World world(W, H);
        Body b = Body::makeCircle({ 640, ground - 20 }, 20);
        b.velocity = { 1500, 0 };
        int i = world.addBody(b);
        runSteps(world, 240);
        float x = world.getBodies()[i].position.x;
        check(x > 0.0f && x < W, "side walls keep a fast body inside the window", "x=" + fmt(x));
    }

    // Stress test
    {
        World world(W, H);
        for (int i = 0; i < 100; ++i) {
            Vector2D p{ 100.0f + (i % 20) * 54.0f, 100.0f + (i / 20) * 50.0f };
            int k = i % 3;
            world.addBody(k == 0 ? Body::makeCircle(p, 15)
                        : k == 1 ? Body::makeBox(p, { 15, 15 })
                                 : Body::makeRegularPolygon(p, 5, 15));
        }
        auto t0 = std::chrono::high_resolution_clock::now();
        runSteps(world, 300);
        auto t1 = std::chrono::high_resolution_clock::now();
        float ms = std::chrono::duration<float, std::milli>(t1 - t0).count() / 300.0f;

        bool finite = true, inside = true;
        for (const Body& b : world.getBodies()) {
            finite = finite && isFinite(b);
            inside = inside && b.position.y < H && b.position.x > 0.0f && b.position.x < W;
        }
        check(finite, "100 bodies: no NaN or infinity");
        check(inside, "100 bodies: none escape the floor or walls");
        check(ms < 8.0f, "100 bodies: physics fits the frame budget", fmt(ms) + " ms/step");
        std::cout << "         (" << fmt(ms) << " ms per step; 16.7 ms available per frame)\n";
    }
}

static void testEventsAndHistory() {
    section("Event queue");
    EventQueue q;
    q.push(SandboxEvent::preset(1));
    q.push(SandboxEvent::preset(2));
    q.push(SandboxEvent::preset(3));
    SandboxEvent e;
    bool fifo = q.pop(e) && e.value == 1 && q.pop(e) && e.value == 2 && q.pop(e) && e.value == 3;
    check(fifo, "events come out in the order they went in (FIFO)");
    check(!q.pop(e) && q.empty(), "pop on an empty queue returns false");
    q.push(SandboxEvent::undo());
    q.clear();
    check(q.empty(), "clear empties the queue");

    World world(W, H);
    std::size_t base = world.getBodies().size();
    q.push(SandboxEvent::spawn(Body::makeCircle({ 300, 300 }, 10)));
    q.push(SandboxEvent::spawn(Body::makeCircle({ 400, 300 }, 10)));
    world.processEvents(q);
    check(world.getBodies().size() == base + 2 && q.empty(), "processEvents applies spawns and empties the queue");

    q.push(SandboxEvent::undo());
    world.processEvents(q);
    check(world.getBodies().size() == base + 1, "undo removes the last spawn");
    q.push(SandboxEvent::redo());
    world.processEvents(q);
    check(world.getBodies().size() == base + 2, "redo brings it back");

    q.push(SandboxEvent::reset());
    world.processEvents(q);
    check(world.getBodies().size() == base, "reset clears to floor and walls");
    q.push(SandboxEvent::undo());
    world.processEvents(q);
    check(world.getBodies().size() == base + 2, "reset can be undone");

    q.push(SandboxEvent::preset(99));
    std::size_t before = world.getBodies().size();
    world.processEvents(q);
    check(world.getBodies().size() == before, "an invalid preset number is ignored");

    WorldState bad = world.getState();
    bad.springs.emplace_back(0, 500, 10.0f, 1.0f);
    q.push(SandboxEvent::replaceScene(bad));
    world.processEvents(q);
    check(world.getSprings().empty(), "replaceScene drops springs pointing at missing bodies");

    section("Undo / redo stacks");
    SceneHistory history(3);
    WorldState current;
    check(!history.undo(current), "undo with empty history returns false");
    for (int i = 1; i <= 5; ++i) {
        current.bodies.assign(i - 1, Body());
        history.record(current); // records states with 0..4 bodies
    }
    current.bodies.assign(5, Body());
    check(history.undoCount() == 3, "capacity limits the undo stack (oldest dropped)");
    history.undo(current);
    check(current.bodies.size() == 4, "undo restores the most recent snapshot (LIFO)");
    history.undo(current);
    history.undo(current);
    check(current.bodies.size() == 2 && !history.undo(current), "oldest kept snapshot is the limit");
    history.redo(current);
    check(current.bodies.size() == 3, "redo moves forward again");
    history.record(current);
    check(!history.canRedo(), "a new action clears the redo stack");
}

static void testPresets() {
    section("Presets");
    for (int n = 1; n <= PRESET_COUNT; ++n) {
        World world(W, H);
        bool ok = world.loadPreset(n) && world.getBodies().size() > 3;
        check(ok, "preset " + std::to_string(n) + " loads: " + presetName(static_cast<Preset>(n)));
    }
    World none(W, H);
    check(!none.loadPreset(0) && !none.loadPreset(PRESET_COUNT + 1), "out-of-range presets are refused");

    // Elastic: velocities swap
    {
        World world(W, H);
        world.loadPreset(static_cast<int>(Preset::ElasticCollision));
        runSteps(world, 120);
        const auto& b = world.getBodies();
        float vMoving = b[3].velocity.x, vResting = b[4].velocity.x;
        check(near(vMoving, 0.0f, 3.0f) && near(vResting, 300.0f, 3.0f),
              "elastic preset: moving ball stops, resting ball leaves at 300",
              "v1=" + fmt(vMoving) + " v2=" + fmt(vResting));
    }

    // Inelastic: common velocity 4*300/5 = 240, momentum conserved
    {
        World world(W, H);
        world.loadPreset(static_cast<int>(Preset::InelasticCollision));
        runSteps(world, 120);
        const auto& b = world.getBodies();
        float vh = b[3].velocity.x, vl = b[4].velocity.x;
        float momentum = b[3].mass * vh + b[4].mass * vl;
        check(near(vh, 240.0f, 5.0f) && near(vl, 240.0f, 5.0f), "inelastic preset: both move at 240",
              "heavy=" + fmt(vh) + " light=" + fmt(vl));
        check(near(momentum, 1200.0f, 12.0f), "inelastic preset: momentum conserved (1200)", fmt(momentum));
    }

    // Free fall: different masses released together stay level (a = g for all)
    {
        World world(W, H);
        world.loadPreset(static_cast<int>(Preset::FreeFall));
        runSteps(world, 30); // 0.5 s, before any of them reaches the floor
        const auto& b = world.getBodies();
        bool level = near(b[3].position.y, b[4].position.y, 0.01f) &&
                     near(b[4].position.y, b[5].position.y, 0.01f);
        bool sameSpeed = near(b[3].velocity.y, b[5].velocity.y, 0.01f);
        check(b.size() == 6 && b[3].mass != b[5].mass, "free fall preset: three bodies of different mass");
        check(level && sameSpeed, "free fall preset: all masses fall together",
              "y=" + fmt(b[3].position.y) + "," + fmt(b[4].position.y) + "," + fmt(b[5].position.y));
        check(near(b[3].velocity.y, 450.0f, 2.0f), "free fall preset: v = g t (900 * 0.5 = 450)",
              fmt(b[3].velocity.y));
    }
}

static void testLauncher() {
    section("Launcher and editable values");
    using namespace sandbox;

    Vector2D up = launchVelocity(90.0f, 100.0f);
    Vector2D right = launchVelocity(0.0f, 100.0f);
    check(near(up.x, 0.0f, 1e-3f) && near(up.y, -100.0f, 1e-3f), "90 degrees points straight up");
    check(near(right.x, 100.0f, 1e-3f) && near(right.y, 0.0f, 1e-3f), "0 degrees points right");

    AppSettings s;
    s.launchAngleDegrees = -10.0f;
    s.launchSpeed = 5000.0f;
    s.launchMass = 0.0f;
    clampLaunchSettings(s);
    check(near(s.launchAngleDegrees, 350.0f, 1e-3f), "angle wraps into 0-360");
    check(near(s.launchSpeed, MAX_LAUNCH_SPEED, 1e-3f), "speed is limited to the feasible maximum");
    check(near(s.launchMass, MIN_BODY_MASS, 1e-3f), "mass is limited to the feasible minimum");

    AppSettings shot;
    shot.currentShape = SpawnShape::Square;
    shot.launchAngleDegrees = 30.0f;
    shot.launchSpeed = 400.0f;
    shot.launchMass = 7.5f;
    Body b = createLaunchedBody({ 100, 100 }, shot);
    check(b.shapeType == ShapeType::Polygon && b.localVertices.size() == 4, "launched body has the chosen shape");
    check(near(b.mass, 7.5f, 1e-4f) && near(b.velocity.length(), 400.0f, 1e-2f),
          "launched body has the chosen mass and speed");

    AppSettings read;
    readBodyIntoSettings(b, read);
    check(near(read.launchAngleDegrees, 30.0f, 0.01f) && near(read.launchSpeed, 400.0f, 0.01f) &&
          near(read.launchMass, 7.5f, 1e-4f), "selecting a body reads back its angle, speed and mass");

    Vector2D p = launchPoint(H, 20.0f);
    check(p.y + 20.0f < floorTop(H) && p.x > WALL_THICKNESS, "launch point is inside the arena, above the floor");

    // Many shots in a row, fired whenever the launcher is clear (as the program does),
    // all fly and stay inside the arena
    World world(W, H);
    world.loadPreset(static_cast<int>(Preset::Projectile));
    AppSettings fire;
    fire.launchSpeed = MAX_LAUNCH_SPEED;
    const Vector2D start = launchPoint(H, fire.spawnSize);
    int fired = 0;
    for (int step = 0; step < 1200 && fired < 20; ++step) {
        if (isAreaClear(world.getState(), start, fire.spawnSize)) {
            fire.launchAngleDegrees = 10.0f + 4.0f * static_cast<float>(fired);
            world.addBody(createLaunchedBody(start, fire));
            ++fired;
        }
        world.step(DT);
    }
    check(!isAreaClear(world.getState(), world.getBodies().back().position, 1.0f),
          "the launcher reports blocked while a body sits on it");
    runSteps(world, 600);
    bool inside = true;
    for (const Body& body : world.getBodies()) {
        if (body.isStatic()) continue;
        inside = inside && isFinite(body) && body.position.x > 0.0f && body.position.x < W &&
                 body.position.y < H;
    }
    check(countDynamicBodies(world.getState()) == 21, "20 extra shots added (21 moving bodies)");
    check(inside, "20 shots at maximum speed all stay inside the arena");
}

static void testSceneFiles() {
    section("Scene files and shape creation");
    using namespace sandbox;

    Body tri = createShape({ 200, 100 }, SpawnShape::Triangle, 20);
    check(tri.shapeType == ShapeType::Polygon && tri.localVertices.size() == 3, "createShape: triangle");
    Body sq = createShape({ 200, 100 }, SpawnShape::Square, 20);
    check(sq.shapeType == ShapeType::Polygon && sq.localVertices.size() == 4, "createShape: square");
    check(createShape({ 0, 0 }, SpawnShape::Circle, 20).shapeType == ShapeType::Circle, "createShape: circle");
    check(shapeName(SpawnShape::Square) == "Square", "shapeName");

    // The program no longer creates springs, but the file format still supports them
    World world(W, H);
    world.loadPreset(static_cast<int>(Preset::FreeFall));
    WorldState original = world.getState();
    original.springs.emplace_back(3, 4, 100.0f, 10.0f);
    original.settings.restitution = 0.7f;

    const std::string file = "test_scene.txt";
    check(saveScene(original, file), "saveScene writes the file");

    WorldState loaded;
    bool ok = loadScene(loaded, file);
    check(ok && loaded.bodies.size() == original.bodies.size(), "loadScene reads all bodies back");
    check(ok && loaded.springs.size() == original.springs.size(), "springs survive save and load");
    check(ok && near(loaded.settings.restitution, 0.7f, 1e-4f), "settings survive save and load");
    check(ok && loaded.bodies[0].isStatic(), "static bodies stay static");

    { std::ofstream old("test_old.txt"); old << "1\n0 100 100 0 0 1 10 10 10 0\n"; }
    WorldState oldScene;
    check(loadScene(oldScene, "test_old.txt") && oldScene.bodies.size() == 1 && oldScene.springs.empty(),
          "files without spring/settings sections still load");

    { std::ofstream bad("test_bad.txt"); bad << "2\n0 1 2 3 4 1 10 10 10 0\n7 garbage\n"; }
    WorldState keep = original;
    check(!loadScene(keep, "test_bad.txt") && keep.bodies.size() == original.bodies.size(),
          "a damaged file is rejected and the scene is kept");

    { std::ofstream bs("test_badspring.txt"); bs << "1\n0 1 1 0 0 1 10 10 10 0\nsprings 1\n0 9 10 1 0\n"; }
    check(!loadScene(keep, "test_badspring.txt"), "a spring pointing at a missing body is rejected");

    check(!loadScene(keep, "does_not_exist.txt"), "a missing file returns false");

    std::remove(file.c_str());
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
