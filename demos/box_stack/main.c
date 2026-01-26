#include <tics.h>

#include <stdio.h>
#include <unistd.h> // For usleep

#define BOX_COUNT 5

int main() {
	tics_world* world = tics_world_create((tics_world_desc){.gravity = {0.0f, -9.81f, 0.0f}});

	// Standard cube: 2x2x2 (extends from -1 to 1)
	tics_vec3 cube_verts[] = {{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
							  {-1, -1, 1},	{1, -1, 1},	 {1, 1, 1},	 {-1, 1, 1}};

	// Large ground plate
	tics_vec3 ground_verts[] = {{-15, -1, -15}, {15, -1, -15}, {15, 1, -15}, {-15, 1, -15},
								{-15, -1, 15},	{15, -1, 15},  {15, 1, 15},	 {-15, 1, 15}};

	// Cube indices (same for both shapes)
	uint32_t indices[] = {4, 5, 6, 4, 6, 7, 1, 0, 3, 1, 3, 2, 0, 4, 7, 0, 7, 3,
						  5, 1, 2, 5, 2, 6, 7, 6, 2, 7, 2, 3, 0, 1, 5, 0, 5, 4};

	tics_shape_desc box_desc = {.type = TICS_SHAPE_CONVEX,
								.data.convex = {.vertices = cube_verts, .vertex_count = 8}};
	tics_shape_desc ground_desc = {.type = TICS_SHAPE_CONVEX,
								   .data.convex = {.vertices = ground_verts, .vertex_count = 8}};

	tics_shape_id box_shape = tics_create_shape(world, box_desc);
	tics_shape_id ground_shape = tics_create_shape(world, ground_desc);

	tics_debug_upload_shape_mesh(box_shape, cube_verts, indices,
								 sizeof(indices) / sizeof(uint32_t));
	tics_debug_upload_shape_mesh(ground_shape, ground_verts, indices,
								 sizeof(indices) / sizeof(uint32_t));

	// Static ground
	// Ground vertices are Y -1 to 1. Position 0,0,0 means top surface is at Y=1.
	tics_static_body_desc static_desc = {
		.transform = (tics_transform){.position = {0, 0, 0}, .rotation = {0, 0, 0, 1}},
		.shape = ground_shape,
		.elasticity = 1.0f};
	tics_world_add_static_body(world, static_desc);

	tics_body_id boxes[BOX_COUNT];
	for (int i = 0; i < BOX_COUNT; i++) {
		// Calculate Y position:
		// Ground top is Y=1.
		// Box is 2 units tall (center to edge is 1).
		// First box center should be around Y=2.
		float y = 2.0f + (i * 2.0f);

		tics_rigid_body_desc body_desc = {
			.shape = box_shape,
			.transform = {.position = {0.0f, y, 0.0f}, .rotation = {0,0,0,1}},
			.mass = 3.0f,
			.elasticity = 0.5f, // Low elasticity to prevent the stack from bouncing forever
			.gravity_scale = 1.0f,
		};

		boxes[i] = tics_world_add_rigid_body(world, body_desc);
	}

	const float delta = 1.0f / 60.0f;
	printf("Simulating stack of %d boxes...\n", BOX_COUNT);

	for (int i = 0; true; i++) {
		tics_world_step(world, delta);
		usleep((unsigned int)(delta * 1000000.0f));
	}

	tics_world_destroy(world);
	return 0;
}
