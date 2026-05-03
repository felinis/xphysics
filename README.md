# xphysics
_xphysics_ is an experimental rigid body physics library for use in 3D scientific simulations and games.
It uses the **convex hull** as the default collision shape.

## Cubes Demo
[0001-0250.webm](https://github.com/user-attachments/assets/4a84e947-2feb-4987-b9a0-13e62ce94046)

## Project
The goal of this project is to understand how data-oriented program design principles can be applied to a project where multiple data processing in bulk is crucial. I chose to use C++ for this project as a sort of middle ground between C and other high-level languages.

## Status
Work-in-progress, __not production ready__.
### Done
- Implemented stable API
- Implemented broadphase collision detection using sweep-and-prune
- Implemented narrowphase collision detection using convex hulls as the primary shape type
### In progress
- Write physics unit tests
### Nice-to-have
- Island generation
- Multi-threading
- Various optimizations like caching

## Unit tests
There are currently 2 unit tests:
- **Math test** to check the validity of all math calculations.
```ps
.\build\Debug\xp_test_math.exe ; echo $LastExitCode
```
If this command returns 0, it means all tests passed. Otherwise, it returns the index of the test that failed.
- **Main test** which spawns 2 cubes: one is dynamic, it falls on a fixed one and stays on it.
```ps
.\build\Debug\xp_test_main.exe
```
