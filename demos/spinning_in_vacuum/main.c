#include <tics.h>

#include <stdio.h>
#include <unistd.h>

// A simple, standalone simulation loop. It creates a world without gravity or air friction, spawns
// a sphere, and sets its radial velocity to a full rotation per second around x

int main() {
	tics_world_desc world_desc = {.gravity = {0}}; // no gravity or air friction
	tics_world* world = tics_world_create(world_desc);

	tics_shape_desc shape_desc = {.type = TICS_SHAPE_SPHERE,
								  .data.sphere = {.center = {0}, .radius = 1.0}};
	tics_shape_id shape = tics_create_shape(world, shape_desc);

	tics_rigid_body_desc body_desc = {
		.shape = shape,
		// rotated by 10° around x
		.transform = {.position = {0, 0, 0}, .rotation = {0.087156, 0, 0, 0.996195}},
		.mass = 1.0f,
		.gravity_scale = 1.0f,
		// 6.283185 (2 Pi): Full rotation per second
		.angular_velocity = {0, 0.1 * 6.283185f, 0}};
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
