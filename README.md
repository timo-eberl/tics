# Tics Physics & Playground

## Development Build

```sh
cmake -S . -B build/
cmake --build build/

# Run demos
cd build/demos/playground
./playground
```

## Release Build

```sh
cmake -S . -B build_release/ -DCMAKE_BUILD_TYPE=Release
cmake --build build_release/ --config Release

# Run demos
cd build_release/demos/playground
./playground
```

## Library-Only Build

To build the `tics` static library without the demos (and graphics dependencies):

```sh
cmake -S . -B build_lib/ -DTICS_BUILD_DEMOS=OFF
cmake --build build_lib/
```

## Creating a Shippable Package

To distribute a demo, the executable requires the `assets/` folder to be located in the same directory:

```text
dist/
├── playground      # Executable
└── assets/         # Directory containing models
```

## Testing

```sh
# Build tests
cmake -S . -B build/ -DTICS_BUILD_TESTS=ON -DTICS_BUILD_DEMOS=OFF
cmake --build build/

# Run all tests and get a nicely formatted output
./tests/run_tests.sh

# or get a list of available test suites
./build/tests/test_runner list
# and run them individually (in this case the 'core' test suite)
./build/tests/test_runner core
```

```
tics/
├── include/             # Public API
├── src/                 # Private headers and implementation
└── tests/
    ├── test.h           # Macros and function declarations
    ├── test_main.c      # Test entry point
    ├── test_api.c       # Black-box tests (Public API)
    └── test_core.c      # White-box tests (Internal API, not static functions)
```

## To-Do

- [x] Remove TICS_GA (Rip)
- [x] Port to C
  - [x] Create wrapper tics_math.h
  - [x] Replace Terathon math with own implementation
  - [x] Update the public interface to a C API and create wrapper around the C++ implementation
  - [x] Rewrite (gradually rot out the C++ internals from the inside)
    - [x] Data-Oriented `tics_world`
      - [x] Replace std::vector with `stb_ds.h` or manual malloc
      - [x] "Swap and Pop"
    - [x] world step
      - [x] Port dynamics
      - [x] Port collision detection
      - [x] Port collision response
        - [x] Impulse solver
        - [x] Position solver
- [ ] Improve collision response to be more stable (no sudden jumping objects, no spinning)
- [ ] Testing
  - [x] Setup
  - [ ] Test public API
  - [ ] Test dynamics
  - [ ] Test collision detection
  - [ ] Test collision response
- [ ] Performance Optimization
  - [x] Separate list for static and rigid bodies (beneficial for broadphase integration, only update AABBs for rigid bodies)
  - [ ] tightly packed vertex data for shapes
  - [ ] SoA instead of AoS for bodies
  - [ ] Broadphase
    - [ ] structure of array for AABBs (cache-locality)
    - [ ] SIMD
      - [ ] individual arrays for min_x, min_y, min_z, max_x, max_y, max_z
      - [ ] check 1 obj against 8 in 1 cycle
    - [ ] list of dynamic indices -> outer loop: dynamic indices, inner loop: all indices
      - [ ] first: dynamic = rigid bodies
      - [ ] then: dynamic = objects that actually moved
  - [ ] Multi-Threading
- [ ] Features
  - [ ] Bodies that are moved externally, but can push rigid bodies
  - [ ] setters and getters for contents of tics_rigid_desc and tics_static_desc
    - [ ] also apply_impulse function that applies impulse at specific location
  - [ ] velocity_iterations + position_iterations
  - [ ] Re-add Areas
  - [ ] Teleporting and swapping shapes -> check for intersection and reposition if required
  - [ ] on_collision_enter + on_collision_exit
  - [ ] On convex hull import, remove duplicate vertices. Enables importing flat shaded geometry without performance penalty.
  - [ ] physics materials
  - [ ] collision shapes
    - [ ] sphere
    - [ ] concave
