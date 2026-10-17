#pragma once

#include <memory>
#include <vector>

#include "Body.hpp"
#include "CollisionSystem.hpp"
#include "EventQueue.hpp"
#include "ForceGenerator.hpp"
#include "Integrator.hpp"
#include "SceneHistory.hpp"
#include "WorldState.hpp"

// World
// Owns the scene and runs the simulation. Each step:
// 1. every enabled force generator adds its force
// 2. the integrator turns force into velocity and position
// 3. the collision system detects and resolves contacts
// The scene itself (bodies, springs, settings) is kept private
// and changed only through World's methods, so its rules
// (snapshot before every edit, settings kept in sync) always hold.
class World {
public:
  // Physics always advances in steps of this size, independent of frame rate
  static constexpr float FIXED_TIMESTEP = 1.0f / 60.0f;

  World(float width, float height);

  // Owns force generators through unique_ptr, so it cannot be copied
  World(const World&) = delete;
  World& operator=(const World&) = delete;

  // Read access
  float getWidth() const { return width; }
  float getHeight() const { return height; }
  const WorldState& getState() const { return state; }
  const std::vector<Body>& getBodies() const { return state.bodies; }
  const std::vector<SpringForce>& getSprings() const { return state.springs; }
  const PhysicsSettings& getSettings() const { return state.settings; }

  // Editing the scene
  int addBody(const Body& body); // returns the new body's index
  bool addSpring(const SpringForce& spring); // false if it refers to missing bodies
  void setSettings(const PhysicsSettings& settings);
  void reset(); // floor and walls only, default settings
  bool loadPreset(int number); // 1 to PRESET_COUNT
  void replaceState(const WorldState& newState);

  // Mouse interaction
  int bodyAt(const Vector2D& point) const; // topmost body under the point, or -1
  void dragBody(int index, const Vector2D& position);

  // Undo / redo
  void saveUndoPoint();
  bool undo();
  bool redo();
  const SceneHistory& getHistory() const { return history; }

  // Events
  // Applies every queued event in arrival order, then leaves the queue empty
  void processEvents(EventQueue& queue);

  // Simulation
  // Advances by real elapsed time using as many fixed steps as fit
  void update(float frameTime);
  // Advances by exactly dt seconds
  void step(float dt);

  // Components
  GravityForce& gravity() { return *gravityForce; }
  DragForce& drag() { return *dragForce; }
  void addForce(std::unique_ptr<ForceGenerator> generator);
  CollisionSystem& collisions() { return collisionSystem; }
  Integrator& integrator() { return integration; }

private:
  float width;
  float height;

  WorldState state;

  // Global force fields; gravityForce and dragForce point into this list
  std::vector<std::unique_ptr<ForceGenerator>> forces;
  GravityForce* gravityForce = nullptr;
  DragForce* dragForce = nullptr;

  Integrator integration;
  CollisionSystem collisionSystem;
  SceneHistory history;

  float timeAccumulator = 0.0f;

  // Pushes the scene settings into the force and collision objects
  void applySettings();
};
