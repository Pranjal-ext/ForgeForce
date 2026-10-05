#include "InteractionRenderer.hpp"

#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>

namespace sandbox {

// ------------------------------------------------------------
// Converts a Vector2D to an SFML vector
// ------------------------------------------------------------
sf::Vector2f toSFML(const Vector2D& v) {
    return sf::Vector2f(v.x, v.y);
}

// ------------------------------------------------------------
// Creates vertices for a regular polygon with a flat bottom edge
// ------------------------------------------------------------
std::vector<Vector2D> createPolygonVertices(int sides, float radius) {
    const float pi = 3.14159265f;

    std::vector<Vector2D> vertices;
    vertices.reserve(sides);

    // Start so the bottom edge is horizontal (squares axis-aligned, triangles point up)
    float startAngle = pi / 2.0f + pi / sides;

    for (int i = 0; i < sides; ++i) {
        float angle = startAngle + (2.0f * pi * i) / sides;

        vertices.push_back(Vector2D(
            std::cos(angle) * radius,
            std::sin(angle) * radius
        ));
    }

    return vertices;
}

// ------------------------------------------------------------
// Creates a new body based on selected shape
// ------------------------------------------------------------
Body createShape(
    const sf::Vector2f& mousePosition,
    SpawnShape shape,
    float size
) {
    Body body;

    body.position = Vector2D(mousePosition.x, mousePosition.y);
    body.velocity = Vector2D(0.0f, 0.0f);
    body.setMass(1.0f);

    switch (shape) {
        case SpawnShape::Circle:
            body.shapeType = ShapeType::Circle;
            body.radius = size;
            break;

        case SpawnShape::Box:
            body.shapeType = ShapeType::Box;
            body.halfExtents = Vector2D(size, size);
            break;

        case SpawnShape::Triangle:
            body.shapeType = ShapeType::Polygon;
            body.localVertices = createPolygonVertices(3, size);
            break;

        case SpawnShape::Square:
            body.shapeType = ShapeType::Polygon;
            body.localVertices = createPolygonVertices(4, size);
            break;

        case SpawnShape::Pentagon:
            body.shapeType = ShapeType::Polygon;
            body.localVertices = createPolygonVertices(5, size);
            break;

        case SpawnShape::Hexagon:
            body.shapeType = ShapeType::Polygon;
            body.localVertices = createPolygonVertices(6, size);
            break;
    }

    return body;
}

// ------------------------------------------------------------
// Creates a static floor
// ------------------------------------------------------------
Body createFloor(float windowWidth, float windowHeight) {
    Body floor;

    floor.position = Vector2D(windowWidth / 2.0f, windowHeight - 20.0f);
    floor.velocity = Vector2D(0.0f, 0.0f);
    floor.setMass(0.0f); // Static body: never moves

    floor.shapeType = ShapeType::Box;
    floor.halfExtents = Vector2D(windowWidth / 2.0f, 20.0f);

    return floor;
}

// ------------------------------------------------------------
// Draws any body: Circle, Box, or Polygon
// ------------------------------------------------------------
void drawBody(sf::RenderWindow& window, const Body& body) {
    if (body.shapeType == ShapeType::Circle) {
        sf::CircleShape circle(body.radius);

        circle.setOrigin(body.radius, body.radius);
        circle.setPosition(toSFML(body.position));
        circle.setFillColor(sf::Color(90, 160, 255));
        circle.setOutlineColor(sf::Color::White);
        circle.setOutlineThickness(1.5f);

        window.draw(circle);
    }
    else if (body.shapeType == ShapeType::Box) {
        sf::RectangleShape box;

        box.setSize(sf::Vector2f(
            body.halfExtents.x * 2.0f,
            body.halfExtents.y * 2.0f
        ));

        box.setOrigin(body.halfExtents.x, body.halfExtents.y);
        box.setPosition(toSFML(body.position));
        box.setFillColor(sf::Color(255, 150, 70));
        box.setOutlineColor(sf::Color::White);
        box.setOutlineThickness(1.5f);

        window.draw(box);
    }
    else if (body.shapeType == ShapeType::Polygon) {
        sf::ConvexShape polygon;
        polygon.setPointCount(body.localVertices.size());

        for (std::size_t i = 0; i < body.localVertices.size(); ++i) {
            polygon.setPoint(i, toSFML(body.localVertices[i]));
        }

        polygon.setPosition(toSFML(body.position));
        polygon.setFillColor(sf::Color(170, 110, 255));
        polygon.setOutlineColor(sf::Color::White);
        polygon.setOutlineThickness(1.5f);

        window.draw(polygon);
    }
}

// ------------------------------------------------------------
// Draws velocity arrow for a moving body
// ------------------------------------------------------------
void drawVelocityVector(sf::RenderWindow& window, const Body& body) {
    Vector2D velocity = body.velocity;

    if (velocity.length() < 1.0f) {
        return;
    }

    float scale = 0.15f;

    sf::Vector2f start = toSFML(body.position);
    sf::Vector2f end = start + toSFML(velocity * scale);

    sf::Vertex line[] = {
        sf::Vertex(start, sf::Color::Green),
        sf::Vertex(end, sf::Color::Green)
    };

    window.draw(line, 2, sf::Lines);

    Vector2D direction = velocity.normalized();
    Vector2D perpendicular(-direction.y, direction.x);

    Vector2D arrowBase = velocity * scale;

    Vector2D left = arrowBase
        - direction * 10.0f
        + perpendicular * 5.0f;

    Vector2D right = arrowBase
        - direction * 10.0f
        - perpendicular * 5.0f;

    sf::ConvexShape arrowHead;
    arrowHead.setPointCount(3);

    arrowHead.setPoint(0, toSFML(body.position + arrowBase));
    arrowHead.setPoint(1, toSFML(body.position + left));
    arrowHead.setPoint(2, toSFML(body.position + right));

    arrowHead.setFillColor(sf::Color::Green);

    window.draw(arrowHead);
}

// ------------------------------------------------------------
// Draws live information beside a body
// ------------------------------------------------------------
void drawBodyInfo(
    sf::RenderWindow& window,
    const Body& body,
    const sf::Font& font
) {
    std::ostringstream info;
    info << std::fixed << std::setprecision(1);

    info << "Mass: " << body.mass << "\n";
    info << "Vel: " << body.velocity.x << ", " << body.velocity.y;

    sf::Text text;
    text.setFont(font);
    text.setCharacterSize(12);
    text.setFillColor(sf::Color::White);

    // Place the label just outside the shape, whatever its type
    text.setPosition(
        body.position.x + body.boundingRadius() + 8.0f,
        body.position.y - 10.0f
    );

    text.setString(info.str());

    window.draw(text);
}

// ------------------------------------------------------------
// Draws controls and current status
// ------------------------------------------------------------
void drawStatus(
    sf::RenderWindow& window,
    const sf::Font& font,
    const AppSettings& settings,
    const std::vector<Body>& bodies
) {
    std::string shapeName;

    switch (settings.currentShape) {
        case SpawnShape::Circle:   shapeName = "Circle";   break;
        case SpawnShape::Box:      shapeName = "Box";      break;
        case SpawnShape::Triangle: shapeName = "Triangle"; break;
        case SpawnShape::Square:   shapeName = "Square";   break;
        case SpawnShape::Pentagon: shapeName = "Pentagon"; break;
        case SpawnShape::Hexagon:  shapeName = "Hexagon";  break;
    }

    std::ostringstream status;

    status << "ForceForge\n";
    status << "Bodies: " << bodies.size() << "\n";
    status << "State: " << (settings.paused ? "PAUSED" : "RUNNING") << "\n";
    status << "Selected Shape: " << shapeName << "\n\n";

    status << "Left Click: Spawn Shape\n";
    status << "Drag: Move Body\n";
    status << "1: Circle\n";
    status << "2: Box\n";
    status << "3: Triangle\n";
    status << "4: Square\n";
    status << "5: Pentagon\n";
    status << "6: Hexagon\n\n";

    status << "P: Pause / Resume\n";
    status << "R: Reset\n";
    status << "V: Toggle Velocity Vectors\n";
    status << "S: Save Scene\n";
    status << "L: Load Scene\n";
    status << "Esc: Exit";

    sf::Text text;
    text.setFont(font);
    text.setCharacterSize(14);
    text.setFillColor(sf::Color::White);
    text.setPosition(12.0f, 12.0f);
    text.setString(status.str());

    window.draw(text);
}

// ------------------------------------------------------------
// Finds the topmost body under the mouse
// ------------------------------------------------------------
int findBodyAt(
    const std::vector<Body>& bodies,
    const sf::Vector2f& mousePosition
) {
    Vector2D point(mousePosition.x, mousePosition.y);

    // Search from the end so the most recently spawned (drawn on top) body wins
    for (int i = static_cast<int>(bodies.size()) - 1; i >= 0; --i) {
        if (bodies[i].containsPoint(point)) {
            return i;
        }
    }

    return -1;
}

// ------------------------------------------------------------
// Saves all bodies to a file
// ------------------------------------------------------------
bool saveScene(
    const std::vector<Body>& bodies,
    const std::string& filename
) {
    std::ofstream file(filename);

    if (!file.is_open()) {
        return false;
    }

    file << bodies.size() << "\n";

    for (const Body& body : bodies) {
        file << static_cast<int>(body.shapeType) << " "
             << body.position.x << " " << body.position.y << " "
             << body.velocity.x << " " << body.velocity.y << " "
             << body.mass << " "
             << body.radius << " "
             << body.halfExtents.x << " "
             << body.halfExtents.y << " "
             << body.localVertices.size();

        for (const Vector2D& vertex : body.localVertices) {
            file << " " << vertex.x << " " << vertex.y;
        }

        file << "\n";
    }

    return true;
}

// ------------------------------------------------------------
// Loads bodies from a file; leaves the scene unchanged on failure
// ------------------------------------------------------------
bool loadScene(
    std::vector<Body>& bodies,
    const std::string& filename
) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        return false;
    }

    int count = 0;
    file >> count;

    if (!file || count < 0) {
        return false;
    }

    // Read into a temporary list so a damaged file leaves the current scene untouched
    std::vector<Body> loaded;
    loaded.reserve(count);

    for (int i = 0; i < count; ++i) {
        Body body;

        int shapeType = 0;
        int vertexCount = 0;
        float mass = 0.0f;

        file >> shapeType
             >> body.position.x >> body.position.y
             >> body.velocity.x >> body.velocity.y
             >> mass
             >> body.radius
             >> body.halfExtents.x
             >> body.halfExtents.y
             >> vertexCount;

        if (!file ||
            shapeType < static_cast<int>(ShapeType::Circle) ||
            shapeType > static_cast<int>(ShapeType::Polygon) ||
            vertexCount < 0) {
            return false;
        }

        body.shapeType = static_cast<ShapeType>(shapeType);
        body.setMass(mass);

        for (int j = 0; j < vertexCount; ++j) {
            Vector2D vertex;

            file >> vertex.x >> vertex.y;
            body.localVertices.push_back(vertex);
        }

        if (!file) {
            return false;
        }

        loaded.push_back(body);
    }

    bodies = std::move(loaded);
    return true;
}

// ------------------------------------------------------------
// Window, input, simulation and rendering loop
// ------------------------------------------------------------
int run() {
    const unsigned int windowWidth = 1280;
    const unsigned int windowHeight = 720;

    sf::RenderWindow window(
        sf::VideoMode(windowWidth, windowHeight),
        "ForceForge - Interactive Physics Sandbox"
    );

    window.setFramerateLimit(60);

    // Bundled font first, then common system locations on Windows, Linux and macOS
    sf::Font font;
    const char* fontPaths[] = {
        "arial.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/Library/Fonts/Arial.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf"
    };

    bool fontLoaded = false;
    for (const char* path : fontPaths) {
        if (font.loadFromFile(path)) {
            fontLoaded = true;
            break;
        }
    }

    if (!fontLoaded) {
        std::cerr << "Could not load a font. Place arial.ttf next to the executable.\n";
        return -1;
    }

    std::vector<Body> bodies;
    AppSettings settings;

    bodies.push_back(createFloor(
        static_cast<float>(windowWidth),
        static_cast<float>(windowHeight)
    ));

    CollisionSystem collisionSystem(80.0f, 0.5f);

    // Impacts slower than about two frames of gravity are treated as resting contact
    collisionSystem.setRestitutionThreshold(settings.gravityStrength * (2.0f / 60.0f));

    sf::Clock clock;

    while (window.isOpen()) {
        sf::Event event;

        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }

            // Mouse input
            if (event.type == sf::Event::MouseButtonPressed &&
                event.mouseButton.button == sf::Mouse::Left) {

                sf::Vector2f mousePosition(
                    static_cast<float>(event.mouseButton.x),
                    static_cast<float>(event.mouseButton.y)
                );

                int clickedIndex = findBodyAt(bodies, mousePosition);

                if (clickedIndex != -1) {
                    settings.dragging = true;
                    settings.draggedBodyIndex = clickedIndex;
                }
                else {
                    bodies.push_back(createShape(
                        mousePosition,
                        settings.currentShape,
                        settings.spawnSize
                    ));
                }
            }

            if (event.type == sf::Event::MouseButtonReleased &&
                event.mouseButton.button == sf::Mouse::Left) {
                settings.dragging = false;
                settings.draggedBodyIndex = -1;
            }

            // Keyboard input
            if (event.type == sf::Event::KeyPressed) {
                switch (event.key.code) {
                    case sf::Keyboard::Num1: settings.currentShape = SpawnShape::Circle;   break;
                    case sf::Keyboard::Num2: settings.currentShape = SpawnShape::Box;      break;
                    case sf::Keyboard::Num3: settings.currentShape = SpawnShape::Triangle; break;
                    case sf::Keyboard::Num4: settings.currentShape = SpawnShape::Square;   break;
                    case sf::Keyboard::Num5: settings.currentShape = SpawnShape::Pentagon; break;
                    case sf::Keyboard::Num6: settings.currentShape = SpawnShape::Hexagon;  break;

                    case sf::Keyboard::P:
                        settings.paused = !settings.paused;
                        break;

                    case sf::Keyboard::R:
                        bodies.clear();
                        bodies.push_back(createFloor(
                            static_cast<float>(windowWidth),
                            static_cast<float>(windowHeight)
                        ));
                        settings.dragging = false;
                        settings.draggedBodyIndex = -1;
                        break;

                    case sf::Keyboard::V:
                        settings.showVelocityVectors = !settings.showVelocityVectors;
                        break;

                    case sf::Keyboard::S:
                        if (!saveScene(bodies, "scene.txt")) {
                            std::cerr << "Could not save scene.txt\n";
                        }
                        break;

                    case sf::Keyboard::L:
                        if (loadScene(bodies, "scene.txt")) {
                            settings.dragging = false;
                            settings.draggedBodyIndex = -1;
                        } else {
                            std::cerr << "Could not load scene.txt\n";
                        }
                        break;

                    case sf::Keyboard::Escape:
                        window.close();
                        break;

                    default:
                        break;
                }
            }
        }

        // Drag selected body
        if (settings.dragging &&
            settings.draggedBodyIndex >= 0 &&
            settings.draggedBodyIndex < static_cast<int>(bodies.size())) {

            sf::Vector2i mousePixel = sf::Mouse::getPosition(window);

            Body& draggedBody = bodies[settings.draggedBodyIndex];

            if (draggedBody.invMass != 0.0f) {
                draggedBody.position = Vector2D(
                    static_cast<float>(mousePixel.x),
                    static_cast<float>(mousePixel.y)
                );

                draggedBody.velocity = Vector2D(0.0f, 0.0f);
            }
        }

        // Cap the timestep so a stalled frame (e.g. window being dragged)
        // cannot move bodies far enough to pass through each other
        float deltaTime = std::min(clock.restart().asSeconds(), 1.0f / 30.0f);

        // Temporary gravity integration for this demo;
        // replace with the full force-generator system.
        if (!settings.paused) {
            for (Body& body : bodies) {
                if (body.invMass != 0.0f) {
                    body.velocity.y += settings.gravityStrength * deltaTime;
                    body.position += body.velocity * deltaTime;
                }
            }

            collisionSystem.step(bodies);
        }

        // Rendering
        window.clear(sf::Color(20, 22, 30));

        for (const Body& body : bodies) {
            drawBody(window, body);

            if (settings.showVelocityVectors) {
                drawVelocityVector(window, body);
            }

            if (settings.showBodyInfo && body.invMass != 0.0f) {
                drawBodyInfo(window, body, font);
            }
        }

        drawStatus(window, font, settings, bodies);

        window.display();
    }

    return 0;
}

} // namespace sandbox
