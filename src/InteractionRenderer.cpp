#include "InteractionRenderer.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>

namespace sandbox {

// Conversions, shape creation and launch values

sf::Vector2f toSFML(const Vector2D& v) {
  return sf::Vector2f(v.x, v.y);
}

namespace {

Vector2D toWorld(const sf::RenderWindow& window, const sf::Vector2i& pixel) {
  sf::Vector2f p = window.mapPixelToCoords(pixel);
  return { p.x, p.y };
}

std::string fixed(float value, int decimals) {
  std::ostringstream s;
  s << std::fixed << std::setprecision(decimals) << value;
  return s.str();
}

} // namespace

Body createShape(const Vector2D& position, SpawnShape shape, float size) {
  switch (shape) {
    case SpawnShape::Circle: return Body::makeCircle(position, size);
    case SpawnShape::Triangle: return Body::makeRegularPolygon(position, 3, size);
    case SpawnShape::Square: return Body::makeRegularPolygon(position, 4, size);
  }
  return Body::makeCircle(position, size);
}

std::string shapeName(SpawnShape shape) {
  switch (shape) {
    case SpawnShape::Circle: return "Circle";
    case SpawnShape::Triangle: return "Triangle";
    case SpawnShape::Square: return "Square";
  }
  return "Unknown";
}

void clampLaunchSettings(AppSettings& settings) {
  settings.launchAngleDegrees = std::fmod(settings.launchAngleDegrees, 360.0f);
  if (settings.launchAngleDegrees < 0.0f) settings.launchAngleDegrees += 360.0f;

  settings.launchSpeed = std::clamp(settings.launchSpeed, MIN_LAUNCH_SPEED, MAX_LAUNCH_SPEED);
  settings.launchMass = std::clamp(settings.launchMass, MIN_BODY_MASS, MAX_BODY_MASS);
}

Body createLaunchedBody(const Vector2D& position, const AppSettings& settings) {
  Body body = createShape(position, settings.currentShape, settings.spawnSize);
  body.setMass(settings.launchMass);
  body.velocity = launchVelocity(settings.launchAngleDegrees, settings.launchSpeed);
  return body;
}


// Drawing

namespace {

sf::Vertex makeVertex(const Vector2D& position, const sf::Color& color) {
  sf::Vertex vertex;
  vertex.position = toSFML(position);
  vertex.color = color;
  return vertex;
}

} // namespace

sf::Text makeText(const sf::Font& font, const std::string& content, unsigned int size) {
#if SFML_VERSION_MAJOR >= 3
  sf::Text text(font, content, size);
#else
  sf::Text text(content, font, size);
#endif
  text.setFillColor(sf::Color::White);
  return text;
}

void drawBody(sf::RenderWindow& window, const Body& body) {
  const sf::Color staticColor(90, 95, 110);

  if (body.shapeType == ShapeType::Circle) {
    sf::CircleShape circle(body.radius);
    circle.setOrigin(sf::Vector2f(body.radius, body.radius));
    circle.setPosition(toSFML(body.position));
    circle.setFillColor(body.isStatic() ? staticColor : sf::Color(90, 160, 255));
    circle.setOutlineColor(sf::Color::White);
    circle.setOutlineThickness(1.5f);
    window.draw(circle);
  }
  else if (body.shapeType == ShapeType::Box) {
    sf::RectangleShape box(sf::Vector2f(body.halfExtents.x * 2.0f, body.halfExtents.y * 2.0f));
    box.setOrigin(toSFML(body.halfExtents));
    box.setPosition(toSFML(body.position));
    box.setFillColor(body.isStatic() ? staticColor : sf::Color(255, 150, 70));
    box.setOutlineColor(sf::Color::White);
    box.setOutlineThickness(body.isStatic() ? 0.0f : 1.5f);
    window.draw(box);
  }
  else if (body.shapeType == ShapeType::Polygon) {
    sf::ConvexShape polygon;
    polygon.setPointCount(body.localVertices.size());
    for (std::size_t i = 0; i < body.localVertices.size(); ++i) {
      polygon.setPoint(i, toSFML(body.localVertices[i]));
    }
    polygon.setPosition(toSFML(body.position));
    polygon.setFillColor(body.isStatic() ? staticColor : sf::Color(170, 110, 255));
    polygon.setOutlineColor(sf::Color::White);
    polygon.setOutlineThickness(1.5f);
    window.draw(polygon);
  }
}

void drawSpring(sf::RenderWindow& window, const SpringForce& spring, const std::vector<Body>& bodies) {
  // Springs are no longer part of the user interface, but the function signature
  // remains for compatibility with the saved run() code and tests
  (void)window; (void)spring; (void)bodies;
}

void drawVelocityVector(sf::RenderWindow& window, const Body& body) {
  const Vector2D velocity = body.velocity;
  if (velocity.length() < 1.0f) return;

  const float scale = 0.15f;
  const sf::Color color = sf::Color::Green;

  Vector2D tip = body.position + velocity * scale;

  sf::Vertex line[] = { makeVertex(body.position, color), makeVertex(tip, color) };
  window.draw(line, 2, sf::PrimitiveType::Lines);

  Vector2D direction = velocity.normalized();
  Vector2D perpendicular(-direction.y, direction.x);

  sf::ConvexShape head;
  head.setPointCount(3);
  head.setPoint(0, toSFML(tip));
  head.setPoint(1, toSFML(tip - direction * 10.0f + perpendicular * 5.0f));
  head.setPoint(2, toSFML(tip - direction * 10.0f - perpendicular * 5.0f));
  head.setFillColor(color);
  window.draw(head);
}

void drawBodyInfo(sf::RenderWindow& window, const Body& body, const sf::Font& font) {
  std::ostringstream info;
  info << std::fixed << std::setprecision(1);
  info << "m: " << body.mass << "\n";
  info << "v: " << body.velocity.length() << " px/s";

  sf::Text text = makeText(font, info.str(), 12);
  text.setPosition(sf::Vector2f(body.position.x + body.boundingRadius() + 8.0f,
    body.position.y - 10.0f));
  window.draw(text);
}

void drawSelection(sf::RenderWindow& window, const Body& body) {
  const float r = body.boundingRadius() + 6.0f;
  sf::CircleShape ring(r);
  ring.setOrigin(sf::Vector2f(r, r));
  ring.setPosition(toSFML(body.position));
  ring.setFillColor(sf::Color::Transparent);
  ring.setOutlineColor(sf::Color(255, 220, 60));
  ring.setOutlineThickness(2.0f);
  window.draw(ring);
}

void drawLauncher(sf::RenderWindow& window, const AppSettings& settings,
  const World& world, float gravity) {
  const Vector2D start = launchPoint(world.getHeight(), settings.spawnSize);
  const Vector2D velocity = launchVelocity(settings.launchAngleDegrees, settings.launchSpeed);

  sf::CircleShape base(8.0f);
  base.setOrigin(sf::Vector2f(8.0f, 8.0f));
  base.setPosition(toSFML(start));
  base.setFillColor(sf::Color(200, 200, 200));
  window.draw(base);

  const Vector2D direction = launchVelocity(settings.launchAngleDegrees, 1.0f);
  const Vector2D tip = start + direction * (30.0f + settings.launchSpeed * 0.06f);
  const sf::Color aimColor(255, 220, 60);
  sf::Vertex aim[] = { makeVertex(start, aimColor), makeVertex(tip, aimColor) };
  window.draw(aim, 2, sf::PrimitiveType::Lines);

  const float ground = floorTop(world.getHeight());
  const float left = WALL_THICKNESS;
  const float right = world.getWidth() - WALL_THICKNESS;
  sf::CircleShape dot(2.0f);
  dot.setOrigin(sf::Vector2f(2.0f, 2.0f));
  dot.setFillColor(sf::Color(220, 220, 220, 140));

  for (int i = 1; i <= 60; ++i) {
    const float t = 0.05f * static_cast<float>(i);
    const Vector2D p = start + velocity * t + Vector2D(0.0f, 0.5f * gravity * t * t);
    if (p.y > ground || p.x < left || p.x > right) break;
    dot.setPosition(toSFML(p));
    window.draw(dot);
  }
}

void drawStatus(sf::RenderWindow& window, const sf::Font& font,
  const AppSettings& settings, const World& world) {
  const PhysicsSettings& physics = world.getSettings();

  std::ostringstream status;
  status << "ForceForge\n";
  status << "Moving bodies: " << countDynamicBodies(world.getState())
    << " / " << MAX_DYNAMIC_BODIES << "\n";
  status << "State: " << (settings.paused ? "PAUSED" : "RUNNING") << "\n";
  status << "Shape: " << shapeName(settings.currentShape)
    << "   Mass: " << fixed(settings.launchMass, 1) << "\n";
  status << "Angle: " << fixed(settings.launchAngleDegrees, 0) << " deg"
    << "   Speed: " << fixed(settings.launchSpeed, 0) << " px/s\n";
  status << "Selected: "
    << (settings.selectedBodyIndex >= 0 ? "body " + std::to_string(settings.selectedBodyIndex)
    : std::string("none")) << "\n";
  status << "Restitution " << fixed(physics.restitution, 2)
    << "   Friction " << fixed(physics.friction, 2)
    << "   Drag " << (physics.dragEnabled ? "on" : "off") << "\n";
  status << "Undo: " << world.getHistory().undoCount()
    << "   Redo: " << world.getHistory().redoCount() << "\n";
  if (!settings.message.empty()) status << settings.message << "\n";

  // SHOW INSTRUCTIONS ONLY IF TOGGLED ON
  if (settings.showInstructions) {
    status << "\n=== FIRE & SPAWN ===\n";
    status << "  Space: Fire from launcher    Click empty space: Spawn\n";

    status << "\n=== ADJUST VALUES ===\n";
    status << "  Up/Down: angle    Left/Right: speed    +/-: mass\n";
    status << "  Hold Shift for fine control\n";

    status << "\n=== SELECT & EDIT ===\n";
    status << "  Click body: select (loads values)    Enter: apply    Del: remove\n";

    status << "\n=== SHAPES (1-3) ===\n";
    status << "  1: Circle    2: Triangle    3: Square\n";

    status << "\n=== PRESETS (F1-F4) ===\n";
    status << "  F1 Projectile  F2 Elastic  F3 Inelastic  F4 Free fall\n";

    status << "\n=== CONTROL ===\n";
    status << "  P: Pause    R: Reset    V: Vectors    I: Info    H: Help (toggle)\n";
    status << "  Ctrl+Z/Y: Undo/Redo    Esc: Exit\n";
  } else {
    status << "\nPress H for instructions";
  }

  sf::Text text = makeText(font, status.str(), 14);
  text.setPosition(sf::Vector2f(WALL_THICKNESS + 12.0f, 12.0f));
  window.draw(text);
}

// Input

#if SFML_VERSION_MAJOR >= 3

std::vector<RawInput> pollInput(sf::RenderWindow& window) {
  std::vector<RawInput> inputs;

  while (const std::optional<sf::Event> event = window.pollEvent()) {
    RawInput input;

    if (event->is<sf::Event::Closed>()) {
      input.kind = RawInput::Kind::Close;
      inputs.push_back(input);
    }
    else if (const auto* pressed = event->getIf<sf::Event::MouseButtonPressed>()) {
      if (pressed->button == sf::Mouse::Button::Left) {
        input.kind = RawInput::Kind::MouseDown;
        input.mouse = toWorld(window, pressed->position);
        inputs.push_back(input);
      }
    }
    else if (const auto* released = event->getIf<sf::Event::MouseButtonReleased>()) {
      if (released->button == sf::Mouse::Button::Left) {
        input.kind = RawInput::Kind::MouseUp;
        input.mouse = toWorld(window, released->position);
        inputs.push_back(input);
      }
    }
    else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
      input.kind = RawInput::Kind::Key;
      input.key = keyPressed->code;
      input.control = keyPressed->control;
      input.shift = keyPressed->shift;
      inputs.push_back(input);
    }
  }

  return inputs;
}

#else

std::vector<RawInput> pollInput(sf::RenderWindow& window) {
  std::vector<RawInput> inputs;
  sf::Event event;

  while (window.pollEvent(event)) {
    RawInput input;

    switch (event.type) {
      case sf::Event::Closed:
        input.kind = RawInput::Kind::Close;
        inputs.push_back(input);
        break;

      case sf::Event::MouseButtonPressed:
        if (event.mouseButton.button == sf::Mouse::Button::Left) {
          input.kind = RawInput::Kind::MouseDown;
          input.mouse = toWorld(window, sf::Vector2i(event.mouseButton.x, event.mouseButton.y));
          inputs.push_back(input);
        }
        break;

      case sf::Event::MouseButtonReleased:
        if (event.mouseButton.button == sf::Mouse::Button::Left) {
          input.kind = RawInput::Kind::MouseUp;
          input.mouse = toWorld(window, sf::Vector2i(event.mouseButton.x, event.mouseButton.y));
          inputs.push_back(input);
        }
        break;

      case sf::Event::KeyPressed:
        input.kind = RawInput::Kind::Key;
        input.key = event.key.code;
        input.control = event.key.control;
        input.shift = event.key.shift;
        inputs.push_back(input);
        break;

      default:
        break;
    }
  }

  return inputs;
}

#endif

namespace {

void stopDragging(AppSettings& settings) {
  settings.dragging = false;
  settings.draggedBodyIndex = -1;
  settings.pressedBodyIndex = -1;
}

void clearSelection(AppSettings& settings) {
  stopDragging(settings);
  settings.selectedBodyIndex = -1;
}

bool roomForAnotherBody(const World& world, AppSettings& settings) {
  if (countDynamicBodies(world.getState()) < MAX_DYNAMIC_BODIES) return true;
  settings.message = "Limit of " + std::to_string(MAX_DYNAMIC_BODIES) +
    " moving bodies reached: press R or remove some with Del";
  return false;
}

bool hasValidSelection(const World& world, const AppSettings& settings) {
  const int i = settings.selectedBodyIndex;
  return i >= 0 && i < static_cast<int>(world.getBodies().size()) &&
    !world.getBodies()[i].isStatic();
}

void loadPresetKey(int number, EventQueue& queue, AppSettings& settings) {
  queue.push(SandboxEvent::preset(number));
  clearSelection(settings);

  if (number == static_cast<int>(Preset::Projectile)) {
    settings.launchAngleDegrees = PROJECTILE_ANGLE_DEGREES;
    settings.launchSpeed = PROJECTILE_SPEED;
    settings.launchMass = 1.0f;
    settings.paused = false;
    settings.message = "Projectile: set angle, speed, mass, then Space to fire";
  } else {
    settings.paused = true;
    settings.message = "Paused: click a body, change its values, Enter to apply, P to start";
  }
}

void handleKey(const RawInput& input, World& world, EventQueue& queue,
  AppSettings& settings, sf::RenderWindow& window) {
  using Key = sf::Keyboard::Key;

  if (input.control) {
    if (input.key == Key::Z) { queue.push(SandboxEvent::undo()); clearSelection(settings); }
    if (input.key == Key::Y) { queue.push(SandboxEvent::redo()); clearSelection(settings); }
    return;
  }

  const float angleStep = input.shift ? 1.0f : 5.0f;
  const float speedStep = input.shift ? 10.0f : 50.0f;
  const float massStep = input.shift ? 0.1f : 0.5f;

  switch (input.key) {
    case Key::Num1: settings.currentShape = SpawnShape::Circle; break;
    case Key::Num2: settings.currentShape = SpawnShape::Triangle; break;
    case Key::Num3: settings.currentShape = SpawnShape::Square; break;

    case Key::Up: settings.launchAngleDegrees += angleStep; break;
    case Key::Down: settings.launchAngleDegrees -= angleStep; break;
    case Key::Right: settings.launchSpeed += speedStep; break;
    case Key::Left: settings.launchSpeed -= speedStep; break;
    case Key::Equal:
    case Key::Add: settings.launchMass += massStep; break;
    case Key::Hyphen:
    case Key::Subtract: settings.launchMass -= massStep; break;

    case Key::Space: {
      Vector2D start = launchPoint(world.getHeight(), settings.spawnSize);
      if (!isAreaClear(world.getState(), start, settings.spawnSize)) {
        settings.message = "Launcher blocked: wait for the last shot to move away";
      } else if (roomForAnotherBody(world, settings)) {
        queue.push(SandboxEvent::spawn(createLaunchedBody(start, settings)));
        settings.message.clear();
      }
      break;
    }

    case Key::Enter:
      if (!hasValidSelection(world, settings)) {
        settings.message = "Click a body first, then press Enter";
        break;
      } else {
        WorldState edited = world.getState();
        Body& body = edited.bodies[settings.selectedBodyIndex];
        body.setMass(settings.launchMass);
        body.velocity = launchVelocity(settings.launchAngleDegrees, settings.launchSpeed);
        queue.push(SandboxEvent::replaceScene(edited));
        stopDragging(settings);
        settings.message = "Applied to body " + std::to_string(settings.selectedBodyIndex);
      }
      break;

    case Key::Delete:
    case Key::Backspace:
      if (hasValidSelection(world, settings)) {
        WorldState edited = world.getState();
        edited.bodies.erase(edited.bodies.begin() + settings.selectedBodyIndex);
        queue.push(SandboxEvent::replaceScene(edited));
        clearSelection(settings);
        settings.message = "Body removed";
      }
      break;

    case Key::F1: loadPresetKey(1, queue, settings); break;
    case Key::F2: loadPresetKey(2, queue, settings); break;
    case Key::F3: loadPresetKey(3, queue, settings); break;
    case Key::F4: loadPresetKey(4, queue, settings); break;

    case Key::P: settings.paused = !settings.paused; break;
    case Key::H: settings.showInstructions = !settings.showInstructions; break;
    case Key::V: settings.showVelocityVectors = !settings.showVelocityVectors; break;
    case Key::I: settings.showBodyInfo = !settings.showBodyInfo; break;

    case Key::R:
      queue.push(SandboxEvent::reset());
      clearSelection(settings);
      settings.message.clear();
      break;

    case Key::Escape: window.close(); break;

    default: break;
  }

  clampLaunchSettings(settings);
}

} // namespace

void handleInput(const RawInput& input, World& world, EventQueue& queue,
  AppSettings& settings, sf::RenderWindow& window) {
  switch (input.kind) {
    case RawInput::Kind::Close:
      window.close();
      break;

    case RawInput::Kind::MouseDown: {
      int index = world.bodyAt(input.mouse);
      if (index < 0) {
        settings.selectedBodyIndex = -1;
        if (roomForAnotherBody(world, settings)) {
          queue.push(SandboxEvent::spawn(createLaunchedBody(input.mouse, settings)));
        }
      } else if (!world.getBodies()[index].isStatic()) {
        settings.selectedBodyIndex = index;
        settings.pressedBodyIndex = index;
        settings.pressPoint = input.mouse;
        readBodyIntoSettings(world.getBodies()[index], settings);
        settings.message = "Body " + std::to_string(index) +
          " selected: change values, Enter to apply";
      } else {
        settings.selectedBodyIndex = -1;
      }
      break;
    }

    case RawInput::Kind::MouseUp:
      stopDragging(settings);
      break;

    case RawInput::Kind::Key:
      handleKey(input, world, queue, settings, window);
      break;
  }
}

// Scene persistence (backend only: no key binding)

bool saveScene(const WorldState& scene, const std::string& filename) {
  std::ofstream file(filename);
  if (!file.is_open()) return false;

  file << scene.bodies.size() << "\n";
  for (const Body& body : scene.bodies) {
    file << static_cast<int>(body.shapeType) << " "
      << body.position.x << " " << body.position.y << " "
      << body.velocity.x << " " << body.velocity.y << " "
      << body.mass << " "
      << body.radius << " "
      << body.halfExtents.x << " " << body.halfExtents.y << " "
      << body.localVertices.size();
    for (const Vector2D& vertex : body.localVertices) {
      file << " " << vertex.x << " " << vertex.y;
    }
    file << "\n";
  }

  file << "springs " << scene.springs.size() << "\n";
  for (const SpringForce& s : scene.springs) {
    file << s.getBodyA() << " " << s.getBodyB() << " " << s.getRestLength() << " "
      << s.getStiffness() << " " << s.getDamping() << "\n";
  }

  file << "settings " << scene.settings.restitution << " " << scene.settings.friction << " "
    << (scene.settings.dragEnabled ? 1 : 0) << "\n";

  return static_cast<bool>(file);
}

bool loadScene(WorldState& scene, const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) return false;

  int count = 0;
  file >> count;
  if (!file || count < 0) return false;

  // Build into a temporary so a damaged file leaves the caller's scene untouched
  WorldState loaded;
  loaded.bodies.reserve(count);

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
      >> body.halfExtents.x >> body.halfExtents.y
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
    if (!file) return false;

    loaded.bodies.push_back(body);
  }

  // Optional sections; files without them still load with defaults
  std::string tag;
  while (file >> tag) {
    if (tag == "springs") {
      int springCount = 0;
      file >> springCount;
      if (!file || springCount < 0) return false;

      for (int i = 0; i < springCount; ++i) {
        int a = 0, b = 0;
        float rest = 0.0f, stiffness = 0.0f, damping = 0.0f;
        file >> a >> b >> rest >> stiffness >> damping;
        if (!file) return false;

        SpringForce spring(a, b, rest, stiffness, damping);
        if (!spring.connects(loaded.bodies.size())) return false;
        loaded.springs.push_back(spring);
      }
    }
    else if (tag == "settings") {
      int drag = 1;
      file >> loaded.settings.restitution >> loaded.settings.friction >> drag;
      if (!file) return false;
      loaded.settings.dragEnabled = (drag != 0);
    }
    else {
      return false; // unknown section: treat the file as damaged
    }
  }

  scene = std::move(loaded);
  return true;
}

// Font and main loop

bool loadFont(sf::Font& font) {
  // Bundled font first, then common system locations on Windows, Linux and macOS
  const char* paths[] = {
    "arial.ttf",
    "C:/Windows/Fonts/arial.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/Library/Fonts/Arial.ttf",
    "/System/Library/Fonts/Supplemental/Arial.ttf"
  };

  for (const char* path : paths) {
#if SFML_VERSION_MAJOR >= 3
  if (font.openFromFile(path)) return true;
#else
  if (font.loadFromFile(path)) return true;
#endif
  }
  return false;
}

int run() {
  const unsigned int windowWidth = 1280;
  const unsigned int windowHeight = 720;
  const std::string title = "ForceForge - Interactive Physics Sandbox";

#if SFML_VERSION_MAJOR >= 3
  sf::RenderWindow window(sf::VideoMode({ windowWidth, windowHeight }), title);
#else
  sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), title);
#endif
  window.setFramerateLimit(60);

  sf::Font font;
  if (!loadFont(font)) {
    std::cerr << "Could not load a font. Place arial.ttf next to the executable.\n";
    return -1;
  }

  World world(static_cast<float>(windowWidth), static_cast<float>(windowHeight));
  EventQueue queue;
  AppSettings settings;
  sf::Clock clock;

  while (window.isOpen()) {
    // 1. Input becomes queued events or frontend setting changes
    for (const RawInput& input : pollInput(window)) {
      handleInput(input, world, queue, settings, window);
    }

    // 2. Scene changes are applied in arrival order, all at once
    world.processEvents(queue);

    // 3. A dragged body follows the mouse
    if (settings.dragging) {
      sf::Vector2i mouse = sf::Mouse::getPosition(window);
      world.dragBody(settings.draggedBodyIndex,
        { static_cast<float>(mouse.x), static_cast<float>(mouse.y) });
    }

    // 4. Physics, in fixed steps (restart() is called even when paused,
    // so un-pausing does not produce one huge step)
    float frameTime = clock.restart().asSeconds();
    if (!settings.paused) world.update(frameTime);

  // 5. Drawing: springs underneath, then bodies, then the overlay text
    window.clear(sf::Color(20, 22, 30));

    for (const SpringForce& spring : world.getSprings()) {
      drawSpring(window, spring, world.getBodies());
    }

    for (const Body& body : world.getBodies()) {
      drawBody(window, body);
      if (body.isStatic()) continue;
      if (settings.showVelocityVectors) drawVelocityVector(window, body);
      if (settings.showBodyInfo) drawBodyInfo(window, body, font);
    }

    drawStatus(window, font, settings, world);
    window.display();
  }

  return 0;
}

} // namespace sandbox 
