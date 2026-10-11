#pragma once

// ============================================================
// SFML window, input handling, drawing and scene save/load.
// Works with SFML 2.6 and SFML 3: the few places where the two
// versions differ are selected at compile time with
// SFML_VERSION_MAJOR. Everything is inside the "sandbox"
// namespace so these names cannot clash with other code.
//
// saveScene / loadScene remain available to code and tests,
// but no key in the program calls them.
// ============================================================

#include <SFML/Graphics.hpp>
#include <string>
#include <vector>

#include "EventQueue.hpp"
#include "Scenarios.hpp"
#include "World.hpp"

namespace sandbox {

// ------------------------------------------------------------
// Shape options for spawning and firing
// ------------------------------------------------------------
enum class SpawnShape {
    Circle,
    Triangle,
    Square
};

// ------------------------------------------------------------
// Frontend state that is not part of the physics scene
// ------------------------------------------------------------
struct AppSettings {
    bool paused = false;
    bool showVelocityVectors = true;
    bool showBodyInfo = true;
    bool showInstructions = false; // H toggles the controls list

    bool dragging = false;
    int draggedBodyIndex = -1;

    // A press on a body becomes a drag only after the mouse moves a few
    // pixels, so a plain click selects without stopping the body
    int pressedBodyIndex = -1;
    Vector2D pressPoint;

    // Body whose values Enter replaces and Delete removes (-1 = none)
    int selectedBodyIndex = -1;

    SpawnShape currentShape = SpawnShape::Circle;
    float spawnSize = 20.0f;

    // Values used for every new body (shots and clicks) and by Enter
    float launchAngleDegrees = PROJECTILE_ANGLE_DEGREES; // 0 = right, 90 = up
    float launchSpeed = PROJECTILE_SPEED;                // pixels per second
    float launchMass = 1.0f;

    std::string message; // feedback line shown on screen
};

// ------------------------------------------------------------
// One input action, independent of the SFML version
// ------------------------------------------------------------
struct RawInput {
    enum class Kind { Close, MouseDown, MouseUp, Key };

    Kind kind = Kind::Key;
    Vector2D mouse;                                      // for MouseDown / MouseUp (world coordinates)
    sf::Keyboard::Key key = sf::Keyboard::Key::Unknown;  // for Key
    bool control = false;                                // Ctrl held during Key
    bool shift = false;                                  // Shift held during Key (fine steps)
};

// ------------------------------------------------------------
// Conversions, shape creation and launch values
// ------------------------------------------------------------
sf::Vector2f toSFML(const Vector2D& v);
Body createShape(const Vector2D& position, SpawnShape shape, float size);
std::string shapeName(SpawnShape shape);

// Keeps angle in [0, 360), speed and mass inside the feasible ranges
void clampLaunchSettings(AppSettings& settings);

// A body of the current shape with the current mass and launch velocity
Body createLaunchedBody(const Vector2D& position, const AppSettings& settings);

// Copies a body's mass, speed and direction into the settings
void readBodyIntoSettings(const Body& body, AppSettings& settings);

// ------------------------------------------------------------
// Drawing
// ------------------------------------------------------------
sf::Text makeText(const sf::Font& font, const std::string& content, unsigned int size);
void drawBody(sf::RenderWindow& window, const Body& body);
void drawVelocityVector(sf::RenderWindow& window, const Body& body);
void drawBodyInfo(sf::RenderWindow& window, const Body& body, const sf::Font& font);
void drawSelection(sf::RenderWindow& window, const Body& body);
void drawLauncher(sf::RenderWindow& window, const AppSettings& settings,
                  const World& world, float gravity);
void drawStatus(sf::RenderWindow& window, const sf::Font& font,
                const AppSettings& settings, const World& world);

// ------------------------------------------------------------
// Input
// ------------------------------------------------------------
// Reads all pending window events and converts them to RawInput
std::vector<RawInput> pollInput(sf::RenderWindow& window);

// Turns one input into a queued scene change or a frontend setting change
void handleInput(const RawInput& input, World& world, EventQueue& queue,
                 AppSettings& settings, sf::RenderWindow& window);

// ------------------------------------------------------------
// Scene persistence (backend only: not bound to any key)
// ------------------------------------------------------------
bool saveScene(const WorldState& scene, const std::string& filename);
bool loadScene(WorldState& scene, const std::string& filename);

// ------------------------------------------------------------
// Font and main loop
// ------------------------------------------------------------
bool loadFont(sf::Font& font);

// Opens the window and runs until it is closed.
// Returns 0 on normal exit, -1 if no font could be loaded.
int run();

} // namespace sandbox
