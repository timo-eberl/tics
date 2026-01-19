#ifndef TICS_INTERNAL_H
#define TICS_INTERNAL_H

#include "tics.h"

// Internal runtime data for shapes and bodies differ from the descriptors that are used to
// initialize them.

// clang-format off

// Internal runtime storage for a shape
typedef struct {
	tics_shape_type type;
	union {
		struct { tics_vec3 center; float radius; } sphere;
		struct { tics_vec3 normal; float distance; } plane;
		struct { tics_vec3* vertices; size_t count; } convex;
	} data;
	tics_shape_id id; // Back-reference to ID, needed for debug drawing
} shape_data;

typedef struct {
	// store shape data directly, because the shape data is small and looking up the shape in a map
	// is slow. the mesh data (which might be big) will still be shared.
	shape_data shape;
	tics_transform transform;
	tics_body_id id; // Back-reference to ID, needed for swap-and-pop updates
	float elasticity;
} static_body_data;

typedef struct {
	// store shape data directly, because the shape data is small and looking up the shape in a map
	// is slow. the mesh data (which might be big) will still be shared.
	shape_data shape;
	tics_transform transform;

	tics_vec3 linear_velocity;
	tics_quat angular_velocity;

	tics_body_id id; // Back-reference to ID, needed for swap-and-pop updates

	float mass;
	float inv_mass; // Pre-calculate 1.0f/mass for solvers
	float elasticity;
	float gravity_scale;
} rigid_body_data;

typedef enum { STATIC_BODY, RIGID_BODY } body_type;
// Holds type and index into either rigid_bodies or static_bodies array
typedef struct { body_type type; size_t index; } body_ref;
typedef struct { tics_body_id key; body_ref value; } body_map_entry;
// Holds index into shapes array
typedef struct { tics_shape_id key; size_t value; } shape_map_entry;

// clang-format on

struct tics_world {
	// Config
	tics_vec3 gravity;

	// --- Dense Data Arrays (stb_ds arrays) ---

	rigid_body_data* rigid_bodies;
	static_body_data* static_bodies;
	shape_data* shapes;

	// --- Lookups (stb_ds hash maps) ---

	// We use a unified map for all body types.
	// Here's a table outlining when a lookup is required:
	// | Operation          | Input          | Map Read?                        |
	// | :----------------- | :------------- | :------------------------------- |
	// | API Call           | `tics_body_id` | YES                              |
	// | Constraints Solver | `tics_body_id` | NO (can be cached in constraint) |
	// | Dynamics           | body arrays    | NO                               |
	// | Collision Det.     | body arrays    | NO                               |
	// | Event Callback     | rb             | NO (can be cached in rb)         |

	// ID -> {Type, Index}
	body_map_entry* body_map;
	// map for shapes: ID -> Index
	shape_map_entry* shape_map;

	// --- ID Generation ---
	// Strictly Increasing IDs: This effectively eliminates "ABA problems" (where you access a
	// reused slot thinking it's the old object) without needing generation counters in the index.
	// Initialized to 1 (0 is invalid)

	uint32_t body_id_counter;
	uint32_t shape_id_counter;
};

typedef struct {
	// a and b are the points where each shape penetrates the other most
	tics_vec3 point_a;
	tics_vec3 point_b;
	tics_vec3 normal; // penetration vector direction
	float depth;	  // penetration vector length
	bool has_collision;
} collision_result;

typedef struct {
	body_ref body_a_ref;
	body_ref body_b_ref;

	collision_result result;
} collision;

collision* collision_narrow_phase(tics_world* world);
collision_result collision_test(const shape_data* a, tics_transform at, const shape_data* b,
								tics_transform bt);

#endif // TICS_INTERNAL_H
