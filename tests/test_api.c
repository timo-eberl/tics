#include "test.h"

void run_api_tests(void) {
	// world creation
	tics_world_desc world_desc = {.gravity = {0, -9.81f, 0}};
	tics_world* world = tics_world_create(world_desc);
	ASSERT_TRUE(world != NULL);

	// shape creation
	tics_shape_id shape_id = tics_create_sphere_shape(world, (tics_vec3){0}, 0.5f);
	ASSERT_INT_NEQ(shape_id, 0);

	// Cleanup
	tics_destroy_shape(world, shape_id);
	tics_world_destroy(world);
}
