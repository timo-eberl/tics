# Tics Physics

Tics is a Rigid Body Physics Library.

If you came here from my [thesis](https://hdms.bsz-bw.de/files/7127/Bachelor_Thesis_Timo_unsigned.pdf) and want to look at the Geometric Algebra implementation, you should go back to commit 2bb376a0e6475f3ed2ead2e03fb818270a6aa55d .

## Development Build

```sh
cmake -S . -B build/
cmake --build build/

# Run demos
cd build/bin/
./playground
```

## Release Build

**Compiler choice:** clang is sometimes faster with OpenMP. gcc is faster for SIMD. gcc seems to be the better choice.

```sh
cmake -S . -B build_release/ -DCMAKE_BUILD_TYPE=Release -DTICS_ENABLE_DEBUG_VIEW=OFF -DCMAKE_C_COMPILER=gcc
cmake --build build_release/ --config Release

# Run demos
cd build_release/bin
./playground
```

### OpenMP Performance Settings

Unless this simulation is the sole active task on your system, OpenMP's default busy-waiting will starve other threads (like rendering or other applications), and degrade the simulation's own performance. On Linux, run `export OMP_WAIT_POLICY=PASSIVE` before executing to disable it. *(Note: On Windows, OpenMP performance is poor regardless).*

## Library-Only Build

To build the `tics` static library without any extra stuff:

```sh
cmake -S . -B build_lib/ -DTICS_BUILD_DEMOS=OFF -DTICS_BUILD_TESTS=OFF -DTICS_ENABLE_DEBUG_VIEW=OFF -DTICS_ENABLE_PROFILER=OFF -DTICS_BUILD_TOOL_GLTF2C=OFF
cmake --build build_lib/
```

## Web Build (Emscripten)

> Web builds don't use Multi-Threading, SIMD or GPUs.

Requirements: [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html)

```sh
# Generate build files with the Emscripten toolchain
emcmake cmake -S . -B build_wasm/ -DCMAKE_BUILD_TYPE=Release -DTICS_ENABLE_DEBUG_VIEW=OFF -DTICS_ENABLE_PROFILER=OFF -DTICS_ENABLE_CUDA=OFF
# Compile
cmake --build build_wasm/
# To run it use Emscripten's built-in `emrun` tool
emrun build_wasm/bin/playground.html
```

## Testing

see [tests/README.md](tests/README.md)

## Debug Visualization (Blick)

Blick is Tics' debug visualization tool. It is enabled by default and automatically spawns a separate window with its own debug view into the physics world. It can draw many debug primitives (like points, arrows, AABBs) that can be used to visualize positions, velocities, intersections. Unlike traditional debug drawing, Blick runs as a separate process, communicating via shared memory. This solves debugging frustrations:

- You can pause the simulation in a conventional debugger and the Blick window remains responsive and you can move the camera around and inspect the frozen scene. You can even step through your application and, for example, watch the bounding volume hierarchy grow.
- If the physics engine crashes, Blick stays open, preserving the final state. You can inspect the "crime scene" to understand exactly what caused the crash.
- You can visualize simulations that don't include a graphics context themself, such as tests.
- Rendering debug primitives happens asynchronously. The simulation is not slowed down by the rendering overhead of drawing thousands of contact points or AABBs.

> It's created for Tics, but could be used to debug other 3D applications too.

> Unfortunately, Blick is currently Unix-only

## gltf2c

`gltf2c` is a tool that converts a gltf/glb file into a bunch of arrays in C syntax that can be pasted into your source code. Allows quickly embedding models without loading models at runtime.

```sh
cmake -S . -B build/ -DTICS_BUILD_TOOL_GLTF2C=ON
cmake --build build/

# Run. You can also specify multiple glb files
./build/bin/gltf2c models/cube.glb
```

## To-Do

- [ ] Fix problem in sparse islands demo (normalize 0-vector assertion)
- [x] Remove Geometric Algebra
- [x] Port to C
- [ ] GPU Broad phase
  - [ ] Auto Detect sizes
  - [x] Unify AABB generation
- [ ] Collision shapes
  - [ ] Add capsule collider
  - [ ] sphere vs convex: use gjk for point (sphere center) vs convex, then use the result (distance) to find collision points (instead of EPA). or if center is inside convex shape, use EPA.
  - [ ] combine shapes (enables concave shapes)
- [ ] GPU Narrow Phase
- [ ] Blick
  - [x] Increase command limit (fix segfault)
  - [ ] Configuration if it should auto-close on crash of main app
  - [ ] Port rendering to sokol
  - [ ] Make it platform independent
- [ ] Improve collision response (esp. resting contacts)
  - [x] Add Iteration loop (sequential impulses)
  - [x] Add Warm Starting
  - [ ] On certain collision cases report multiple collisions (improves resting collisions). All cases:
    - [ ] edge vs edge (parallel): 2 points. project one edge onto the other and find overlapping segment endpoints.
    - [ ] edge vs face: 2 points. clip edge against boundaries of face (Sutherland-Hodgman?)
    - [ ] face vs face: 3+ points (polygon, maybe limit to some number?). clip one face against other (Sutherland-Hodgman)
  - [ ] Baumgarte Stabilization? (teleporting doesn't do the trick anymore. for a lot of stacked objects they just jitter back and forth)
  - [ ] Add restitution threshold: collisions with a low relative velocity are treated as resting -> velocity = 0 (not sure if this is actually a good idea)
- [ ] Portability
  - [ ] Use Function Multiversioning for autovectorized functions `__attribute__((target_clones("avx2","sse2","default")))`
  - [ ] compile with `-march=x86-64` to be explicit about compatibility (guarantees SSE, but function multiversioning also provides AVX versions)
- [ ] Performance Optimization
  - [x] Separate list for static and rigid bodies (beneficial for broadphase integration, only update AABBs for rigid bodies)
  - [x] On convex hull import, remove duplicate vertices. Enables importing flat shaded geometry without performance penalty.
  - [ ] tightly packed vertex data for shapes
  - [ ] SoA instead of AoS for bodies
  - [ ] list of dynamic indices -> outer loop: dynamic indices, inner loop: all indices
    - [ ] first: dynamic = rigid bodies
    - [ ] then: dynamic = objects that actually moved
  - [x] Multi-Threading Narrow Phase
  - [ ] Improve GJK
    - [ ] https://dl.acm.org/doi/10.1145/3072959.3083724
    - [ ] "GJK algorithms are often used incrementally in simulation systems and video games. In this mode, the final simplex from a previous solution is used as the initial guess in the next iteration"
- [ ] Testing
  - [x] Setup
  - [ ] Test core: world creation, adding and removing objects and shapes
  - [x] Test dynamics
  - [ ] More collision detection tests
    - [x] edge vs edge
    - [ ] parallel face vs face (resting contact, should this return multiple contact points?)
    - [ ] touching: objects only touch - should consistently report a hit (implemented with skin width)
    - [ ] shape completely inside another
    - [ ] needle-plate: a very tiny object against another very big object
  - [ ] Test collision response
- [ ] Improve public API (apply_impulse, setters and getters, on_collision_enter, on_collision_exit...)
- [ ] Areas that only detect collisions
- [ ] Bodies that are moved externally, but can push rigid bodies
