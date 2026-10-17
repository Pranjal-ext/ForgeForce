#include "World.hpp"

#include <algorithm>

#include "Scenarios.hpp"

World::World(float width, float height)
  : width(width), height(height), collisionSystem(80.0f, 0.5f), history(50) {
  auto gravityPtr = std::make_unique<GravityForce>(Vector2D{ 0.0f, 900.0f });
  auto dragPtr = std::make_unique<DragForce>();
  gravityForce = gravityPtr.get();
  dragForce = dragPtr.get();
  forces.push_back(std::move(gravityPtr));
  forces.push_back(std::move(dragPtr));

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

bool World::loadPreset(int number) {
  if (number < 1 || number > PRESET_COUNT) return false;
  state = buildPreset(static_cast<Preset>(number), width, height);
  timeAccumulator = 0.0f;
  applySettings();
  return true;
}

void World::replaceState(const WorldState& newState) {
  state = newState;

  // Drop springs that would point at bodies that no longer exist
  state.springs.erase(
    std::remove_if(state.springs.begin(), state.springs.end(),
      [&](const SpringForce& s) { return !s.connects(state.bodies.size()); }),
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

int World::bodyAt(const Vector2D& point) const {
  // From the end: the most recently added body is drawn on top
  for (int i = static_cast<int>(state.bodies.size()) - 1; i >= 0; --i) {
    if (state.bodies[i].containsPoint(point)) return i;
  }
  return -1;
}

void World::dragBody(int index, const Vector2D& position) {
  if (index < 0 || index >= static_cast<int>(state.bodies.size())) return;

  Body& body = state.bodies[index];
  if (body.isStatic()) return;

  body.position = position;
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

void World::step(float dt) {
  if (dt <= 0.0f) return;

  // 1. Forces. Every generator is used through the base class:
  // each one's own apply() runs (polymorphism).
  for (const auto& generator : forces) {
    if (generator->isEnabled()) generator->apply(state.bodies);
  }
  for (const SpringForce& spring : state.springs) {
    if (spring.isEnabled()) spring.apply(state.bodies);
  }

  // 2. Motion
  integration.step(state.bodies, dt);

  // 3. Contacts. Impacts slower than about two steps of gravity are
  // treated as resting contact so stacks settle instead of jittering.
  float gravityStrength = gravityForce->isEnabled() ? gravityForce->getAcceleration().length() : 0.0f;
  collisionSystem.setRestitutionThreshold(std::max(1.0f, 2.0f * gravityStrength * dt));
  collisionSystem.step(state.bodies);
}
