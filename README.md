# Tics Physics & Playground

## Development Build

```bash
cmake -S . -B build/
cmake --build build/

# Run demos
cd build/demos/playground
./playground
```

## Release Build

```bash
cmake -S . -B build_release/ -DCMAKE_BUILD_TYPE=Release
cmake --build build_release/ --config Release

# Run demos
cd build_release/demos/playground
./playground
```

## Library-Only Build

To build the `tics` static library without the demos (and graphics dependencies):

```bash
cmake -S . -B build_lib/ -DTICS_BUILD_DEMOS=OFF
cmake --build build_lib/
```

## Creating a Shippable Package

To distribute a demo, the executable requires the `assets/` folder to be located in the same directory:

```text
/dist
├── playground      # Executable
└── assets/         # Directory containing models
```

## To-Do

- [x] Remove TICS_GA (Rip)
- [ ] Port to C
  - [x] Create wrapper tics_math.h
  - [x] Replace Terathon math with own implementation
  - [ ] Update the public interface according to the API specified below
  - [ ] Replace std::vector
    - [ ] Use `stb_ds.h` or manual malloc in the private implementation
  - [ ] Separate list for static and rigid bodies (beneficial for broadphase integration, only update AABBs for rigid bodies)
- [ ] Testing
- [ ] Bug Fixes
- [ ] Performance Optimization
  - [ ] Broadphase
    - [ ] structure of array for AABBs (cache-locality)
    - [ ] SIMD
      - [ ] individual arrays for min_x, min_y, min_z, max_x, max_y, max_z
      - [ ] check 1 obj against 8 in 1 cycle
    - [ ] list of dynamic indices -> outer loop: dynamic indices, inner loop: all indices
      - [ ] first: dynamic = rigid bodies
      - [ ] then: dynamic = objects that actually moved
  - [ ] Multi-Threading
- [ ] Usability features
  - [ ] tics_body_get_transforms_batch
  - [ ] Bodies that are moved externally, but can push rigid bodies
  - [ ] setters and getters for contents of tics_rigid_desc and tics_static_desc
    - [ ] also apply_impulse function that applies impulse at specific location
  - [ ] velocity_iterations + position_iterations
  - [ ] Re-add Areas
  - [ ] Teleporting and swapping shapes -> check for intersection and reposition if required
  - [ ] on_collision_enter + on_collision_exit

## C API

The public interface `tracy.h` when porting is done:

```C
#ifndef TICS_H
#define TICS_H

#include "tics_math.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to the simulation world
typedef struct tics_world tics_world;
// Handle for all types of bodies. 0 is always invalid.
typedef uint32_t tics_body_id;
// 32-bit Handle for collision shapes. 0 is always invalid.
// Using handles allows sharing one mesh data buffer among many bodies.
typedef uint32_t tics_shape_id;

// Transform consisting of position and rotation (quaternion). Scaling is unsupported as the scale
// of a rigid body can per definition not change.
typedef struct tics_transform {
	tics_vec3 position;
	tics_quat rotation;
} tics_transform;
typedef enum tics_shape_type {
	TICS_SHAPE_SPHERE,
	TICS_SHAPE_PLANE,
	TICS_SHAPE_MESH
} tics_shape_type;

// Configuration used to initialize the world
typedef struct tics_world_desc {
	tics_vec3 gravity;
} tics_world_desc;
// Configuration used to create a shape resource
typedef struct tics_shape_desc {
	tics_shape_type type;
	union {
		struct {
			tics_vec3 center;
			float radius;
		} sphere;

		struct {
			tics_vec3 normal;
			float distance;
		} plane;

		struct {
			// Will be copied on creation
			const tics_vec3* vertices;
			size_t vertex_count;
		} mesh;
	} data;
} tics_shape_desc;
// Configuration for creating a Static Body (Ground, Walls)
typedef struct tics_static_desc {
	tics_transform transform;
	tics_shape_id shape; // Reference to a pre-created shape
	float elasticity;	 // [0.0 - 1.0]
} tics_static_desc;
// Configuration for creating a Rigid Body (Moving objects)
typedef struct tics_rigid_desc {
	tics_transform transform;
	tics_shape_id shape; // Reference to a pre-created shape

	tics_vec3 linear_velocity;
	tics_quat angular_velocity;

	float mass;
	float elasticity; // [0.0 - 1.0]
	float gravity_scale;
} tics_rigid_desc;

// Create a new physics world. Returns NULL on failure.
tics_world* tics_world_create(tics_world_desc desc);
// Destroy the world and free all internal resources/bodies.
void tics_world_destroy(tics_world* world);

// Steps the simulation forward by delta (in seconds).
void tics_world_step(tics_world* world, float delta);

// Creates a shape resource. Returns 0 on failure.
tics_shape_id tics_world_create_shape(tics_world* world, tics_shape_desc desc);
// Destroys a shape. Note: Do not destroy a shape while it is in use by a body.
void tics_world_destroy_shape(tics_world* world, tics_shape_id shape);

// Adds a static body. Returns 0 on failure.
tics_body_id tics_world_add_static_body(tics_world* world, tics_static_desc desc);
// Adds a rigid body. Returns 0 on failure.
tics_body_id tics_world_add_rigid_body(tics_world* world, tics_rigid_desc desc);
// Removes and destroys a body. The ID becomes invalid.
void tics_world_remove_body(tics_world* world, tics_body_id id);

// Get the transform. Useful for rendering synchronization.
// Returns identity if ID is invalid.
tics_transform tics_body_get_transform(const tics_world* world, tics_body_id id);

#ifdef __cplusplus
}
#endif

#endif // TICS_H
```
