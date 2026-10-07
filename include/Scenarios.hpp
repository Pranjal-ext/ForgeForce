#pragma once

#include <string>

#include "WorldState.hpp"

// ============================================================
// Demo presets
// Each preset builds a complete scene (bodies and settings)
// that demonstrates one physical idea.
// ============================================================
enum class Preset {
    Projectile = 1,      // launch at an angle, compare range with v^2 sin(2a) / g
    ElasticCollision,    // equal masses, e = 1: velocities swap
    InelasticCollision,  // masses 4 and 1, e = 0: they move off together
    FreeFall             // different masses dropped together, no drag: they land together
};

constexpr int PRESET_COUNT = 4;

// Launch values used by the projectile preset (also the launcher's starting values)
constexpr float PROJECTILE_SPEED = 600.0f;
constexpr float PROJECTILE_ANGLE_DEGREES = 45.0f;

// Static floor and side walls that enclose every scene
constexpr float WALL_THICKNESS = 20.0f;

// Horizontal position of the launcher (bottom-left of the arena)
constexpr float LAUNCH_X = 120.0f;

// Feasible ranges for values the user can set.
// With g = 900 px/s^2, 1000 px/s at 45 degrees lands about 1110 px away,
// which still fits inside a 1280 px wide window.
constexpr float MIN_LAUNCH_SPEED = 0.0f;
constexpr float MAX_LAUNCH_SPEED = 1000.0f;
constexpr float MIN_BODY_MASS = 0.5f;
constexpr float MAX_BODY_MASS = 20.0f;

// Upper limit on moving bodies, so repeated shots cannot slow the program down
constexpr int MAX_DYNAMIC_BODIES = 80;

// Height at which the free fall preset releases its bodies
constexpr float FREE_FALL_HEIGHT = 150.0f;

std::string presetName(Preset preset);

// Floor and walls only, with default settings
WorldState buildEmptyScene(float width, float height);

// A complete preset scene
WorldState buildPreset(Preset preset, float width, float height);

// Height of the floor's top surface for a window of the given height
float floorTop(float height);

// Centre of a body of the given size resting at the launcher, just above the floor
Vector2D launchPoint(float height, float bodySize);

// Velocity for a launch at angleDegrees above the horizontal
// (0 = right, 90 = straight up; screen y points down)
Vector2D launchVelocity(float angleDegrees, float speed);

// Number of bodies that are not static (excludes floor and walls)
int countDynamicBodies(const WorldState& scene);

// True if no moving body overlaps a circle of this radius at this point.
// Spawning onto a fully overlapping body can push one of them through the floor.
bool isAreaClear(const WorldState& scene, const Vector2D& point, float radius);
