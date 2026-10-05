#pragma once

// ============================================================
// SFML window, input handling, rendering and scene save/load.
// Everything is inside the "sandbox" namespace so these names
// cannot clash with identically named functions elsewhere.
// ============================================================

#include <SFML/Graphics.hpp>
#include <vector>
#include <string>

#include "CollisionSystem.hpp"

namespace sandbox {

// ------------------------------------------------------------
// Shape options for spawning
// ------------------------------------------------------------
enum class SpawnShape {
    Circle,
    Box,
    Triangle,
    Square,
    Pentagon,
    Hexagon
};

// ------------------------------------------------------------
// Frontend application state
// ------------------------------------------------------------
struct AppSettings {
    bool paused = false;
    bool showVelocityVectors = true;
    bool showBodyInfo = true;

    bool dragging = false;
    int draggedBodyIndex = -1;

    SpawnShape currentShape = SpawnShape::Circle;

    float gravityStrength = 900.0f;
    float spawnSize = 20.0f;
};

// ------------------------------------------------------------
// Vector conversion
// ------------------------------------------------------------
sf::Vector2f toSFML(const Vector2D& v);

// ------------------------------------------------------------
// Shape creation
// ------------------------------------------------------------
std::vector<Vector2D> createPolygonVertices(int sides, float radius);
Body createShape(const sf::Vector2f& mousePosition, SpawnShape shape, float size);
Body createFloor(float windowWidth, float windowHeight);

// ------------------------------------------------------------
// Drawing
// ------------------------------------------------------------
void drawBody(sf::RenderWindow& window, const Body& body);
void drawVelocityVector(sf::RenderWindow& window, const Body& body);
void drawBodyInfo(sf::RenderWindow& window, const Body& body, const sf::Font& font);
void drawStatus(sf::RenderWindow& window, const sf::Font& font,
                const AppSettings& settings, const std::vector<Body>& bodies);

// ------------------------------------------------------------
// Input
// ------------------------------------------------------------
int findBodyAt(const std::vector<Body>& bodies, const sf::Vector2f& mousePosition);

// ------------------------------------------------------------
// Scene persistence
// ------------------------------------------------------------
bool saveScene(const std::vector<Body>& bodies, const std::string& filename);
bool loadScene(std::vector<Body>& bodies, const std::string& filename);

// ------------------------------------------------------------
// Opens the window and runs the sandbox until it is closed.
// Returns 0 on normal exit, -1 if no font could be loaded.
// ------------------------------------------------------------
int run();

} // namespace sandbox
