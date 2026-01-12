#include "test.h"

#include <blick_adapter.h>
#include <tics.h>
#include <tics_internal.h>

static void pyramid_test() {
	tics_world_desc world_desc = {.gravity = {0.0f, -9.81f, 0.0f}};
	tics_world* world = tics_world_create(world_desc);

	// Pyramid. Geometry: Flat Base at Y=1, Sharp Tip at Y=-1.
	tics_vec3 verts[] = {{-1, 1, -1}, {1, 1, -1}, {1, 1, 1}, {-1, 1, 1}, {0, -1, 0}};
	tics_shape_desc shape_desc = {
		.type = TICS_SHAPE_CONVEX,
		.data.convex = {.vertices = verts, .vertex_count = sizeof(verts) / sizeof(verts[0])}};

	tics_shape_id shape = tics_create_shape(world, shape_desc);
	// Access internal shape data
	const shape_data* shape_ptr = &world->shapes[0];

	// Setup:
	// Position B at Y = 1.9.
	// B's Tip World Y = 1.9 + (-1.0) = 0.9.
	// A's Top World Y = 1.0.
	// Expected Depth = 1.0 - 0.9 = 0.1.
	tics_transform tA = {.position = {0, 0, 0}, .rotation = {0, 0, 0, 1}};
	tics_transform tB = {.position = {0, 1.9f, 0}, .rotation = {0, 0, 0, 1}};
	{
		collision_result result = collision_test(shape_ptr, tA, shape_ptr, tB);

		ASSERT_TRUE(result.has_collision);
		ASSERT_FLOAT_APPROX(result.depth, 0.1f);
		// Normal points A -> B (Down)
		ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){0, -1, 0}));
		// Point A is on the surface of A (Y=1.0)
		ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){0, 1.0f, 0}));
		// Point B is the tip of B (Y=0.9)
		ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){0, 0.9f, 0}));
	}
	{
		// swapped transforms -> A and B are swapped and normal is flipped
		collision_result result = collision_test(shape_ptr, tB, shape_ptr, tA);

		ASSERT_TRUE(result.has_collision);
		ASSERT_FLOAT_APPROX(result.depth, 0.1f);
		ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){0, 1, 0}));
		ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){0, 0.9f, 0}));
		ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){0, 1.0f, 0}));
	}
	{
		// rotate B by 180° -> now the tips are intersecting, result should be unchanged
		tB.rotation = (tics_quat){1, 0, 0, 0};
		collision_result result = collision_test(shape_ptr, tA, shape_ptr, tB);

		ASSERT_TRUE(result.has_collision);
		ASSERT_FLOAT_APPROX(result.depth, 0.1f);
		ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){0, -1, 0}));
		ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){0, 1.0f, 0}));
		ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){0, 0.9f, 0}));
	}

	tics_world_destroy(world);
}

void run_collision_test_tests(void) {
	pyramid_test();
}
