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

	// accumulated, applied and reset every frame
	// an impulse is an instantaneous change in momentum
	tics_vec3 impulse;
	// angular impulse (instantaneous change in angular momentum) divided by square distance to the
	// application pos
	tics_quat an_imp_div_sq_dst;

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

	// Unified map for all body types: ID -> {Type, Index}
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

collision_result collision_test(const shape_data* a, tics_transform at, const shape_data* b,
								tics_transform bt);

#ifdef TICS_ENABLE_DEBUG_VIEW

#include "tics_debug_view_shm.h"

void tics_view_init(void);
void tics_view_shutdown(void);
void tics_view_start_frame(void);
void tics_view_update_frame(void);
void tics_view_end_frame(void);

void tics_view_record_line(tics_vec3 start, tics_vec3 end, uint32_t color);
void tics_view_record_point(tics_vec3 pos, float radius, uint32_t color);

#define TICS_VIEW_INIT() tics_view_init()
#define TICS_VIEW_SHUTDOWN() tics_view_shutdown()
#define TICS_VIEW_FRAME_START() tics_view_start_frame()
#define TICS_VIEW_FRAME_UPDATE() tics_view_update_frame()
#define TICS_VIEW_FRAME_END() tics_view_end_frame()

#define TICS_VIEW_LINE(s, e, c) tics_view_record_line(s, e, c)
#define TICS_VIEW_POINT(p, r, c) tics_view_record_point(p, r, c)

#else

#define TICS_VIEW_INIT() ((void)0)
#define TICS_VIEW_SHUTDOWN() ((void)0)
#define TICS_VIEW_FRAME_START() ((void)0)
#define TICS_VIEW_FRAME_UPDATE() tics_view_update_frame()
#define TICS_VIEW_FRAME_END() ((void)0)

#define TICS_VIEW_LINE(...) ((void)0)
#define TICS_VIEW_POINT(...) ((void)0)

#endif

#endif // TICS_INTERNAL_H
