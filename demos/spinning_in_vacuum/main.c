#include <tics.h>

#include <stdio.h>
#include <unistd.h>

// A simple, standalone simulation loop. It creates a world without gravity or air friction, spawns
// a convex cube, and sets its radial velocity to a full rotation per second around x

int main() {
	tics_world_desc world_desc = {.gravity = {0}}; // no gravity or air friction
	tics_world* world = tics_world_create(world_desc);

	// Create a Cube Shape (Convex Hull of 8 points)
	tics_vec3 cube_verts[] = {{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
							  {-1, -1, 1},	{1, -1, 1},	 {1, 1, 1},	 {-1, 1, 1}};
	uint32_t indices[] = {4, 5, 6, 4, 6, 7, 1, 0, 3, 1, 3, 2, 0, 4, 7, 0, 7, 3,
						  5, 1, 2, 5, 2, 6, 7, 6, 2, 7, 2, 3, 0, 1, 5, 0, 5, 4};
	tics_shape_desc shape_desc = {.type = TICS_SHAPE_CONVEX,
								  .data.convex = {.vertices = cube_verts, .vertex_count = 8}};
	tics_shape_id shape = tics_create_shape(world, shape_desc);

	// Only for debugging - not required
	tics_debug_upload_shape_mesh(shape, cube_verts, indices, sizeof(indices) / sizeof(uint32_t));

	tics_rigid_body_desc body_desc = {
		.shape = shape,
		.transform = {.position = {0, 0, 0}, .rotation = {0, 0, 0, 1}},
		.mass = 1.0f,
		.gravity_scale = 1.0f,
		// Full rotation (360 deg) per second around X.
		// 360 deg = 2 * PI radians (~6.283185)
		.angular_velocity = {6.283185f, 0.0f, 0.0f}};
	tics_body_id body = tics_world_add_rigid_body(world, body_desc);

	// Simulation Loop
	const float delta = 1.0f / 60.0f;
	tics_world_step(world, 0.0f); // run once so debug visualiuation updates
	for (int i = 0; true; i++) {
		tics_world_step(world, delta);
		// Sleep based on delta to simulate real-time simulation (seconds -> microseconds)
		usleep((unsigned int)(delta * 1000000.0f));
	}

	tics_world_destroy(world);
	return 0;
}
