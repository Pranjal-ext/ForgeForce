#include "Scenarios.hpp"

#include <cmath>

namespace {
const float PI = 3.14159265f;
}

float floorTop(float height) {
  return height - 2.0f * WALL_THICKNESS;
}

Vector2D launchPoint(float height, float bodySize) {
  // 1 px gap so the body does not start overlapping the floor
  return { LAUNCH_X, floorTop(height) - bodySize - 1.0f };
}

Vector2D launchVelocity(float angleDegrees, float speed) {
  const float angle = angleDegrees * PI / 180.0f;
  // Screen y points down, so "upward" is negative y
  return { speed * std::cos(angle), -speed * std::sin(angle) };
}

int countDynamicBodies(const WorldState& worldState) {
  int dynamicBodyCount = 0;
  for (const Body& body : worldState.bodies) {
    if (!body.isStatic()) ++dynamicBodyCount;
  }
  return dynamicBodyCount;
}

bool isAreaClear(const WorldState& worldState, const Vector2D& spawnPoint, float radius) {
  for (const Body& body : worldState.bodies) {
    if (body.isStatic()) continue;
    if ((body.position - spawnPoint).length() < body.boundingRadius() + radius) return false;
  }
  return true;
}

std::string presetName(Preset preset) {
  switch (preset) {
    case Preset::Projectile: return "Projectile motion";
    case Preset::ElasticCollision: return "Elastic collision";
    case Preset::InelasticCollision: return "Inelastic collision";
    case Preset::FreeFall: return "Free fall";
  }
  return "Unknown";
}

WorldState buildEmptyScene(float width, float height) {
  WorldState scene;

  // Mass 0 makes all three static
  scene.bodies.push_back(Body::makeBox({ width / 2.0f, height - WALL_THICKNESS },
    { width / 2.0f, WALL_THICKNESS }, 0.0f));
  scene.bodies.push_back(Body::makeBox({ WALL_THICKNESS / 2.0f, height / 2.0f },
    { WALL_THICKNESS / 2.0f, height / 2.0f }, 0.0f));
  scene.bodies.push_back(Body::makeBox({ width - WALL_THICKNESS / 2.0f, height / 2.0f },
    { WALL_THICKNESS / 2.0f, height / 2.0f }, 0.0f));
  return scene;
}

WorldState buildPreset(Preset preset, float width, float height) {
  WorldState scene = buildEmptyScene(width, height);
  const float ground = floorTop(height);

  switch (preset) {
    case Preset::Projectile: {
      // No air resistance, so the flight matches the textbook parabola
      scene.settings.dragEnabled = false;

      const float radius = 12.0f;
      Body projectileBody = Body::makeCircle(launchPoint(height, radius), radius);
      projectileBody.velocity = launchVelocity(PROJECTILE_ANGLE_DEGREES, PROJECTILE_SPEED);
      scene.bodies.push_back(projectileBody);
      break;
    }

    case Preset::ElasticCollision: {
      scene.settings.restitution = 1.0f;
      scene.settings.friction = 0.0f;
      scene.settings.dragEnabled = false;

      const float radius = 22.0f;
      Body moving = Body::makeCircle({ 300.0f, ground - radius }, radius, 1.0f);
      moving.velocity = { 300.0f, 0.0f };
      Body resting = Body::makeCircle({ 700.0f, ground - radius }, radius, 1.0f);

      scene.bodies.push_back(moving);
      scene.bodies.push_back(resting);
      break;
    }

    case Preset::InelasticCollision: {
      scene.settings.restitution = 0.0f;
      scene.settings.friction = 0.0f;
      scene.settings.dragEnabled = false;

      // Same size so the contact is head-on; only the masses differ
      const float radius = 22.0f;
      Body heavy = Body::makeCircle({ 300.0f, ground - radius }, radius, 4.0f);
      heavy.velocity = { 300.0f, 0.0f };
      Body light = Body::makeCircle({ 700.0f, ground - radius }, radius, 1.0f);

      scene.bodies.push_back(heavy);
      scene.bodies.push_back(light);
      break;
    }

    case Preset::FreeFall: {
      // No air resistance: every body accelerates at g whatever its mass,
      // so bodies released together from the same height land together
      scene.settings.dragEnabled = false;
      scene.settings.restitution = 0.3f;

      const float radius = 20.0f;
      const float masses[] = { 1.0f, 5.0f, 15.0f };
      for (int bodyIndex = 0; bodyIndex < 3; ++bodyIndex) {
        // Kept right of centre so the on-screen help text does not cover them
        float positionX = width * (0.45f + 0.15f * static_cast<float>(bodyIndex));
        scene.bodies.push_back(Body::makeCircle(
          { positionX, FREE_FALL_HEIGHT }, radius, masses[bodyIndex]));
      }
      break;
    }
  }

  return scene;
}
