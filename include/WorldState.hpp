#pragma once

#include <vector>

#include "Body.hpp"
#include "ForceGenerator.hpp"

// ============================================================
// PhysicsSettings
// Scene-wide options that presets change and undo restores.
// ============================================================
struct PhysicsSettings {
    float restitution = 0.5f; // bounciness: 0 = no bounce, 1 = perfectly elastic
    float friction = 0.3f;    // Coulomb friction coefficient at contacts
    bool dragEnabled = true;  // air resistance on or off
};

// ============================================================
// WorldState
// Everything that defines a scene at one moment: the bodies,
// the springs connecting them, and the physics settings.
// All members are plain values, so copying a WorldState takes
// a complete snapshot (used for undo/redo, presets, save/load).
// ============================================================
struct WorldState {
    std::vector<Body> bodies;
    std::vector<SpringForce> springs;
    PhysicsSettings settings;
};
