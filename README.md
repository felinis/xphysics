# xphysics
_xphysics_ is an experimental rigid body physics library for use in scientific simulations and games.

The goal of this project is to understand how data-oriented program design principles can be applied to a project where multiple data processing in bulk is crucial. I chose to use C++ for this project as a sort of middle ground between C and other high-level languages.

## Project status
Work-in-progress, __not production ready__.
### In progress
- Implement unit tests for math primitives
- Implement stable API
### To do
- Use the convex hull as the primary collision detection primitive
- Implement broadphase collision detection
- Write physics unit tests
- Implement island generation

## Unit tests
Currently only math unit tests are present.
### PowerShell
```ps
.\build\Debug\xp_test_math.exe ; echo $LastExitCode
```
If this command returns 0, it means all tests passed. Otherwise, it returns the index of the test that failed.
