#include <tics.h>

#include <stdio.h>
#include <unistd.h> // For usleep

// A simple, standalone simulation loop. It creates a world, defines a convex cube, spawns it at
// Y=10, and steps the simulation at 60 FPS.

int main() {
	printf("[HOST] Simulation starting...\n");

	// 1. Create the Physics World
	tics_world_desc world_desc = {.gravity = {0.0f, -9.81f, 0.0f}};
	tics_world* world = tics_world_create(world_desc);
	if (!world) {
		fprintf(stderr, "Failed to create world\n");
		return 1;
	}

	// 2. Create a Cube Shape (Convex Hull of 8 points)
	tics_vec3 cube_verts[] = {{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
							  {-1, -1, 1},	{1, -1, 1},	 {1, 1, 1},	 {-1, 1, 1}};
	tics_shape_desc shape_desc = {.type = TICS_SHAPE_CONVEX,
								  .data.convex = {.vertices = cube_verts, .vertex_count = 8}};
	tics_shape_id shape = tics_create_shape(world, shape_desc);

	// 3. Add a Rigid Body (Falling Cube)
	tics_rigid_body_desc body_desc = {0};
	body_desc.shape = shape;
	body_desc.transform.position = (tics_vec3){0.0f, 10.0f, 0.0f};		// Start 10m up
	body_desc.transform.rotation = (tics_quat){0.0f, 0.0f, 0.0f, 1.0f}; // Identity
	body_desc.mass = 1.0f;
	body_desc.gravity_scale = 1.0f;

	tics_body_id body = tics_world_add_rigid_body(world, body_desc);

	// 4. Simulation Loop
	const float dt = 1.0f / 60.0f;

	for (int i = 0; i < 600; i++) {
		// Step the physics
		tics_world_step(world, dt);

		// Retrieve new position for display
		tics_transform t = tics_body_get_transform(world, body);

		printf("[HOST] Frame %3d | Pos: %.4f, %.4f, %.4f\n", i, t.position.x, t.position.y, t.position.z);

		// Sleep ~16ms to simulate real-time 60 FPS
		usleep(16666);
	}

	// 5. Cleanup
	tics_world_destroy(world);

	printf("[HOST] Simulation finished. Shutting down.\n");
	return 0;
}
