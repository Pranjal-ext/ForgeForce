**ForceForge: An Interactive 2D Newtonian Physics Simulation Sandbox**

ForceForge is an interactive 2D Newtonian physics simulation sandbox built in C++17 with SFML. Students create particles, circles and boxes, adjust mass, velocity, friction, bounciness and gravity, and watch collisions, momentum and projectile motion play out in real time, with on-screen vectors and live values. It is designed as a visual teaching tool, so learners see what the formulas actually do.

**Problem Statement**

Physics is usually taught through formulas and static diagrams, so students rarely see how mass, speed and friction change the outcome of a collision. Existing tools are either limited to one concept (PhET), distracting (Algodoo), or built for developers rather than learners (Box2D, Matter.js). There is a need for a focused 2D sandbox where students can experiment freely and see the forces and values behind every motion.

**Objectives**

1. Build a 2D physics engine in C++17 that simulates gravity, friction, drag, springs, momentum and projectile motion.
2. Implement broad-phase and narrow-phase collision detection with impulse-based response.
3. Let users create particles, circles and boxes and adjust mass, velocity, friction, bounciness and gravity.
4. Display force vectors and live values on screen to link visuals with formulas.
5. Support experimentation through undo/redo.

**Team Members**

| S.No. | Name | Student ID | Email | GitHub |

| 1 | Pranjal Negi (Team Lead) | 2510370602 | [2510370602@geu.ac.in](mailto:2510370602@geu.ac.in) | [@Pranjal-ext](https://github.com/Pranjal-ext) |
| 2 | Dilasha Rawat | 2510012530 | [2510012530@geu.ac.in](mailto:2510012530@geu.ac.in) | [@dilasharawat](https://github.com/dilasharawat) |
| 3 | Rishabh Jakhmola | 25101210555 | [25101210555@geu.ac.in](mailto:25101210555@geu.ac.in) | [@rishabhjakhmola121-max](https://github.com/rishabhjakhmola121-max) |
| 4 | Akshit Chhabra | 2510011905 | [2510011905@geu.ac.in](mailto:2510011905@geu.ac.in) | [@chhabraakshit52-glitch](https://github.com/chhabraakshit52-glitch) |

**Major Features/Modules**

Users can fire projectiles from a launcher, spawn circles, triangles and squares, drag bodies with the mouse, and set the mass, speed and launch angle of the launcher or of any selected body. Friction can be changed live with the [ and ] keys. On screen, ForceForge shows velocity vectors and each body's mass and speed. Four presets (F1–F4) demonstrate projectile motion, elastic and inelastic collisions, and free fall.

The engine is built around a Body data type whose ShapeType is Circle, Box or Polygon. Forces come from an abstract ForceGenerator class with GravityForce, DragForce and SpringForce subclasses, used through base-class pointers. Bodies move by semi-implicit Euler integration in fixed 1/60 s steps. Collisions use a spatial hash grid for the broad phase, exact shape tests (SAT for polygons) for the narrow phase, and impulse-based response with restitution, Coulomb friction and position correction. Every scene change passes through a FIFO event queue. Undo/redo keeps two stacks of scene snapshots. SFML handles rendering and input.

**Current Project Status**

**Phase**: Phase III complete. The project is ready for the final demonstration.

Status: All planned modules are finished and integrated: the object model, the fixed-step simulation loop, gravity, drag and spring forces, semi-implicit Euler integration, collision detection (spatial hash grid and SAT) and impulse-based response, presets F1–F4, the launcher, undo/redo, the event queue, the SFML interface, and the test suite. Phase III added drag-and-drop with wall and floor clamping, and live friction control with the [ and ] keys. The quadtree broad phase and per-body restitution planned in Phase II were dropped in favour of these features.

Tests: All 123 automated checks pass.

**Completed**

| Member | Completed work |

| Pranjal Negi | Narrow-phase detection (circle-circle, circle-box, box-box, SAT polygon); spatial hash-grid broad phase; `CollisionSystem::step`; presets F1 to F4 (projectile, elastic, inelastic, free fall); launcher helpers; live friction control (`[` / `]` keys) |
| Dilasha Rawat | `ForceGenerator` hierarchy (gravity, drag, spring); semi-implicit and explicit Euler integration; impulse response (restitution, Coulomb friction, position correction, velocity sleep); undo/redo stacks; event queue ,deque; drag boundary handling (`World::dragBody`) |
| Akshit Chhabra | `Vector2D` math (length, normalise, dot, cross); `Body` data type supporting circle, box and polygon shapes; `World` class with fixed 1/60 s timestep, body picking and dragging, and the simulation loop |
| Rishabh Jakhmola | SFML window, rendering and input handling; launcher (mass, speed, angle); body selection with on-screen velocity vectors and body info; drag input (mouse-move tracking, 4-pixel threshold); controls list;  (not bound to a key); `main.cpp`; support for SFML 2.6 and 3 |
| Whole team | Automated test suite ([N] checks in 15 sections, all passing); README |

**Tools/Technologies Used**

- Programming: C++17
- Tools/Framework: SFML
- IDE:VS Code
- Version Control: Git/GitHub

**Installation & Setup**

Prerequisites: A C++17 compiler and SFML 2.6 or 3.0 should be installed in the system. Then, follow the below instructions to proceed :

1. Download the ZIP from the ForceForge GitHub repo.
2. Extract it into a folder and open that folder in VS Code.
3. Run the tests to verify your setup, using the command : g++ -std=c++17 -Iinclude tests/test_main.cpp src/Body.cpp src/CollisionSystem.cpp src/ForceGenerator.cpp src/Integrator.cpp src/SceneHistory.cpp src/EventQueue.cpp src/Scenarios.cpp src/World.cpp src/InteractionRenderer.cpp -lsfml-graphics -lsfml-window -lsfml-system -o run_tests; ./run_tests
4. Run the simulator, using the command : g++ -std=c++17 -Iinclude src/main.cpp src/Body.cpp src/CollisionSystem.cpp src/ForceGenerator.cpp src/Integrator.cpp src/SceneHistory.cpp src/EventQueue.cpp src/Scenarios.cpp src/World.cpp src/InteractionRenderer.cpp -lsfml-graphics -lsfml-window -lsfml-system -o forceforge; ./forceforge
