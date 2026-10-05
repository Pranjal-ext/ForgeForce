ForceForge

- What it is: a 2D physics sandbox where you create shapes and watch gravity, friction, momentum, collisions and projectile motion happen over time.

- Problem it solves: physics is taught with formulas. Still pictures so students never see what really happens when two objects of mass and speed collide.

- What users can do: create particles, circles and boxes; change mass, speed, friction, bounce and gravity; and watch vectors and real‑time values on the screen.

- Built with: C++17 for the engine and SFML for graphics and user input.

- OOP design: a Body class with Particle, Circle and Box derived from it showing inheritance, encapsulation, abstraction and polymorphism.

- Data structures used:

Arrays and linked lists hold the objects.

Stacks manage undo and redo.

Queues manage events.

A grid or quadtree finds objects.

- Physics engine: force generators such as gravity, friction, air drag and springs feed into integration.

- Collision system: a broad phase quickly finds pairs; a narrow phase checks for real contact and impulse‑based response makes objects bounce realistically.

- Different from tools:

PhET covers one idea at a time.

Algodoo has features that can distract from the main goal.

Box2D and Matter.js are for developers, not for teaching.

- Limits it accepts: physics, one user, at a time and usually than 100 objects.
