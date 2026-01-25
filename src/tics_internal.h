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

// Axis-aligned bounding box
typedef struct { tics_vec3 min; tics_vec3 max; } aabb;

typedef struct {
	// store shape data directly, because the shape data is small and looking up the shape in a map
	// is slow. the mesh data (which might be big) will still be shared.
	shape_data shape;
	tics_transform transform;
	aabb aabb;		 // Computed once at initialization
	tics_body_id id; // Back-reference to ID, needed for swap-and-pop updates
	float elasticity;
} static_body_data;

typedef struct {
	// store shape data directly, because the shape data is small and looking up the shape in a map
	// is slow. the mesh data (which might be big) will still be shared.
	shape_data shape;
	tics_transform transform;

	tics_vec3 linear_velocity;
	tics_vec3 angular_velocity;

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
	float air_fric_lin;
	float air_fric_ang;

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

// Broadphase Proxy stripped of all physics properties (velocity, mass, etc).
typedef struct {
	aabb aabb;
	// id into rigid_bodies or static_bodies. Type is implied by which array this proxy resides in.
	uint32_t index;
} broad_phase_proxy;

// The output of the broadphase. Represents a potential collision.
// We output body_ref here so the Narrowphase knows exactly which arrays to look into to find the
// shape data.
typedef struct {
	body_ref a;
	body_ref b;
} broad_phase_pair;

aabb tics_calculate_aabb(const shape_data* shape, tics_transform t);

// Proxy Builders
// These functions iterate over the world bodies, compute/fetch the AABB, and return a new dynamic
// array (stb_ds) of proxies. Separation allows us to treat Static bodies as passive in the
// broadphase.
broad_phase_proxy* build_rigid_proxies(const tics_world* world);
broad_phase_proxy* build_static_proxies(const tics_world* world);

// Broad phase collision detection
// Takes two lists to enable optimizations (we do not need to check static vs static).
broad_phase_pair* collision_broad_phase(const broad_phase_proxy* rigids, size_t rigid_count,
										const broad_phase_proxy* statics, size_t static_count);

// Narrow phase collision detection
// Takes the list of pairs found by the broadphase. Requires pointers to the body arrays to resolve
// the indices in 'broad_phase_pair' to actual shape data for the geometric checks.
collision* collision_narrow_phase(const broad_phase_pair* pairs, size_t pair_count,
								  const rigid_body_data* r_bodies,
								  const static_body_data* s_bodies);

collision_result collision_test(const shape_data* a, tics_transform at, const shape_data* b,
								tics_transform bt);

void apply_gravity_and_air_friction(tics_world* world, float delta);

// Calculates the instantaneous linear velocity of a specific point on the rigid body.
// The result accounts for both the body's linear velocity and the tangential velocity.
// Input and output are in world space.
tics_vec3 get_velocity_at_point(rigid_body_data* rb, tics_vec3 point);

// Calculates and applies instantaneous impulses to handle momentum transfer, restitution, and
// contact friction. This function modifies the bodies linear and angular velocities to prevent
// them from moving deeper into an intersection during the following integration step.
void resolve_velocities(tics_world* world, collision* collisions);

// Directly translates (teleports) bodies to correct geometric overlaps. It modifies the positions
// directly to enforce non-penetration without adding energy.
void resolve_penetrations(tics_world* world, collision* collisions);

// Applies velocities (linear and angular) to position and rotation.
void apply_velocities(tics_world* world, float delta);

#endif // TICS_INTERNAL_H
