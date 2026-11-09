#include "World.hpp"

#include <algorithm>

#include "Scenarios.hpp"

World::World(float width, float height)
  : width(width), height(height), collisionSystem(80.0f, 0.5f), history(50) {
  auto gravityGenerator = std::make_unique<GravityForce>(Vector2D{ 0.0f, 900.0f });
  auto dragGenerator = std::make_unique<DragForce>();
  gravityForce = gravityGenerator.get();
  dragForce = dragGenerator.get();
  forces.push_back(std::move(gravityGenerator));
  forces.push_back(std::move(dragGenerator));

  reset();
  history.clear(); // the starting scene is not an undoable action
}

// Editing the scene

int World::addBody(const Body& body) {
  state.bodies.push_back(body);
  return static_cast<int>(state.bodies.size()) - 1;
}

bool World::addSpring(const SpringForce& spring) {
  if (!spring.connects(state.bodies.size())) return false;
  state.springs.push_back(spring);
  return true;
}

void World::setSettings(const PhysicsSettings& settings) {
  state.settings = settings;
  applySettings();
}

void World::reset() {
  state = buildEmptyScene(width, height);
  timeAccumulator = 0.0f;
  applySettings();
}

bool World::loadPreset(int presetNumber) {
  if (presetNumber < 1 || presetNumber > PRESET_COUNT) return false;
  state = buildPreset(static_cast<Preset>(presetNumber), width, height);
  timeAccumulator = 0.0f;
  applySettings();
  return true;
}

void World::replaceState(const WorldState& replacementState) {
  state = replacementState;

  // Drop springs that would point at bodies that no longer exist
  state.springs.erase(
    std::remove_if(state.springs.begin(), state.springs.end(),
      [&](const SpringForce& spring) { return !spring.connects(state.bodies.size()); }),
    state.springs.end());

  timeAccumulator = 0.0f;
  applySettings();
}

void World::applySettings() {
  collisionSystem.setRestitution(state.settings.restitution);
  collisionSystem.setFriction(state.settings.friction);
  dragForce->setEnabled(state.settings.dragEnabled);
}

// Mouse interaction

int World::bodyAt(const Vector2D& worldPoint) const {
  // From the end: the most recently added body is drawn on top
  for (int bodyIndex = static_cast<int>(state.bodies.size()) - 1; bodyIndex >= 0; --bodyIndex) {
    if (state.bodies[bodyIndex].containsPoint(worldPoint)) return bodyIndex;
  }
  return -1;
}

void World::dragBody(int bodyIndex, const Vector2D& position) {
  if (bodyIndex < 0 || bodyIndex >= static_cast<int>(state.bodies.size())) return;

  Body& body = state.bodies[bodyIndex];
  if (body.isStatic()) return;

  // Keep the dragged body inside the walls and above the floor, so it
  // cannot be pushed deep into a wall and then shot out by the solver
  const float bodyRadius = body.boundingRadius();
  const float minX = WALL_THICKNESS + bodyRadius;
  const float maxX = width - WALL_THICKNESS - bodyRadius;
  const float maxY = floorTop(height) - bodyRadius;
  Vector2D target = position;
  if (minX <= maxX) target.x = std::clamp(target.x, minX, maxX);
  target.y = std::min(target.y, maxY);

  body.position = target;
  body.velocity = { 0.0f, 0.0f }; // the hand holds it still
}

// Undo / redo

void World::saveUndoPoint() {
  history.record(state);
}

bool World::undo() {
  if (!history.undo(state)) return false;
  timeAccumulator = 0.0f;
  applySettings();
  return true;
}

bool World::redo() {
  if (!history.redo(state)) return false;
  timeAccumulator = 0.0f;
  applySettings();
  return true;
}

// Events

void World::processEvents(EventQueue& queue) {
  SandboxEvent event;
  while (queue.pop(event)) {
    switch (event.type) {
      case EventType::SpawnBody:
        saveUndoPoint();
        addBody(event.body);
        break;
      case EventType::BeginDrag:
        saveUndoPoint(); // so the move can be undone
        break;
      case EventType::Reset:
        saveUndoPoint();
        reset();
        break;
      case EventType::Undo:
        undo();
        break;
      case EventType::Redo:
        redo();
        break;
      case EventType::LoadPreset:
        if (event.value >= 1 && event.value <= PRESET_COUNT) {
          saveUndoPoint();
          loadPreset(event.value);
        }
        break;
      case EventType::ReplaceScene:
        saveUndoPoint();
        replaceState(event.scene);
        break;
    }
  }
}

// Simulation

void World::addForce(std::unique_ptr<ForceGenerator> generator) {
  if (generator) forces.push_back(std::move(generator));
}

void World::update(float frameTime) {
  // Ignore huge gaps (window dragged, debugger pause) instead of
  // trying to catch up with hundreds of steps
  timeAccumulator += std::min(frameTime, 0.25f);

  int steps = 0;
  const int maxSteps = 5;
  while (timeAccumulator >= FIXED_TIMESTEP && steps < maxSteps) {
    step(FIXED_TIMESTEP);
    timeAccumulator -= FIXED_TIMESTEP;
    ++steps;
  }

  // Still behind after the limit: drop the backlog rather than spiral
  if (steps == maxSteps) timeAccumulator = 0.0f;
}

void World::step(float deltaTime) {
  if (deltaTime <= 0.0f) return;

  // 1. Forces. Every generator is used through the base class:
  // each one's own apply() runs (polymorphism).
  for (const auto& generator : forces) {
    if (generator->isEnabled()) generator->apply(state.bodies);
  }
  for (const SpringForce& spring : state.springs) {
    if (spring.isEnabled()) spring.apply(state.bodies);
  }

  // 2. Motion
  integration.step(state.bodies, deltaTime);

  // 3. Contacts. Impacts slower than about two steps of gravity are
  // treated as resting contact so stacks settle instead of jittering.
  float gravityStrength = gravityForce->isEnabled() ? gravityForce->getAcceleration().length() : 0.0f;
  collisionSystem.setRestitutionThreshold(std::max(1.0f, 2.0f * gravityStrength * deltaTime));
  collisionSystem.step(state.bodies);
}
