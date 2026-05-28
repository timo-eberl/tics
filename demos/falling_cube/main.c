#include <tics.h>

#include <stdio.h>
#include <unistd.h> // For usleep

// A simple, standalone simulation loop. It creates a world, defines a convex cube, spawns it at
// Y=10, and steps the simulation at 60 FPS.

int main() {
	// Create the Physics World
	tics_world* world = tics_world_create((tics_world_desc){.gravity = {0.0f, -9.81f, 0.0f}});

	// Create Shapes
	tics_vec3 cube_verts[] = {{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
							  {-1, -1, 1},	{1, -1, 1},	 {1, 1, 1},	 {-1, 1, 1}};
	tics_vec3 ground_verts[] = {{-10, -1, -10}, {10, -1, -10}, {10, 1, -10}, {-10, 1, -10},
								{-10, -1, 10},	{10, -1, 10},  {10, 1, 10},	 {-10, 1, 10}};
	uint32_t indices[] = {4, 5, 6, 4, 6, 7, 1, 0, 3, 1, 3, 2, 0, 4, 7, 0, 7, 3,
						  5, 1, 2, 5, 2, 6, 7, 6, 2, 7, 2, 3, 0, 1, 5, 0, 5, 4};

	tics_shape_id shape = tics_create_convex_shape(
		world, cube_verts, 8, indices, sizeof(indices) / sizeof(uint32_t));
	tics_shape_id ground_shape = tics_create_convex_shape(
		world, ground_verts, 8, indices, sizeof(indices) / sizeof(uint32_t));

	tics_quat rotated_5 = {0.044, -0.002, 0.044, 0.998};			  // rotated 5° around x and z
	tics_quat rotated_10 = {0.086824, -0.007596, 0.086824, 0.992404}; // rotated 10° around x and z
	// Add a Rigid Body (Falling Cube)
	tics_rigid_body_desc body_desc = {
		.shape = shape,
		.transform = {.position = {0, 10, 0}, .rotation = rotated_10},
		.mass = 3.0f,
		.elasticity = 1.0f,
		.gravity_scale = 1.0f,
		//.angular_velocity = {0.6283185f, 0.0f, 0.0f}, // 1/10 full rotation per second
	};
	tics_body_id body = tics_world_add_rigid_body(world, body_desc);
	// Add the Static Bodies
	tics_static_body_desc static_desc = {
		.transform = (tics_transform){.position = {0}, .rotation = {0, 0, 0, 1}},
		.shape = ground_shape,
		.elasticity = 0.5};
	tics_body_id static_body = tics_world_add_static_body(world, static_desc);

	// Simulation Loop
	const float delta = 1.0f / 60.0f;

	for (int i = 0; true; i++) {
		tics_world_step(world, delta);

		// Retrieve transform for display
		tics_transform t = tics_body_get_transform(world, body);
		printf("Frame %3d | Pos: %.4f, %.4f, %.4f\n", i, t.position.x, t.position.y, t.position.z);

		// Sleep based on delta to simulate real-time simulation (seconds -> microseconds)
		usleep((unsigned int)(delta * 1000000.0f));
	}

	// Cleanup
	tics_world_destroy(world);
	return 0;
}
