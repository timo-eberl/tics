#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <tics.h>
#include <unistd.h> // For usleep

// Configuration: Change this to spawn more or fewer spheres
#define SPHERE_COUNT 300

int main() {
	// Create the physics world with standard gravity
	tics_world* world = tics_world_create((tics_world_desc){.gravity = {0.0f, -9.81f, 0.0f},
															.air_friction_linear = 0.01f,
															.air_friction_angular = 0.01f});

	// --- Define Shapes ---

	// Large sphere shape for the environment (Ground and Walls)
	float large_radius = 50.0f;
	tics_shape_desc large_sphere_desc = {
		.type = TICS_SHAPE_SPHERE, .data.sphere = {.center = {0, 0, 0}, .radius = large_radius}};
	tics_shape_id large_shape = tics_create_shape(world, large_sphere_desc);

	// Small sphere shape for the dynamic objects
	float small_radius = 1.0f;
	tics_shape_desc small_sphere_desc = {
		.type = TICS_SHAPE_SPHERE, .data.sphere = {.center = {0, 0, 0}, .radius = small_radius}};
	tics_shape_id small_shape = tics_create_shape(world, small_sphere_desc);

	// --- Create Static Environment ---

	// Ground: Place center at -large_radius so the "top" of the sphere is at Y=0
	tics_static_body_desc ground_desc = {
		.transform = {.position = {0, -large_radius, 0}, .rotation = {0, 0, 0, 1}},
		.shape = large_shape,
		.elasticity = 1.0f};
	tics_world_add_static_body(world, ground_desc);

	// Walls: We position 4 large spheres around the center.
	// wall_offset determines how far from the center the "inner surface" of the wall begins.
	float wall_offset = 2.0f;
	float wall_pos = wall_offset + large_radius;

	tics_vec3 wall_positions[] = {
		{wall_pos, wall_offset, 0},	 // Right (+X)
		{-wall_pos, wall_offset, 0}, // Left  (-X)
		{0, wall_offset, wall_pos},	 // Back  (+Z)
		{0, wall_offset, -wall_pos}	 // Front (-Z)
	};

	for (int i = 0; i < 4; i++) {
		tics_static_body_desc wall_desc = {
			.transform = {.position = wall_positions[i], .rotation = {0, 0, 0, 1}},
			.shape = large_shape,
			.elasticity = 1.0f};
		tics_world_add_static_body(world, wall_desc);
	}

	// --- Spawn Dynamic Spheres ---

	for (int i = 0; i < SPHERE_COUNT; i++) {
		// Spawn spheres in a spiral
		// Calculate spiral position in the XZ plane
		float angle = i * 0.5f;
		float radius = 2.0f;
		float x_pos = cosf(angle) * radius;
		float z_pos = sinf(angle) * radius;
		float y_pos = 10.0f + (i * (small_radius * 2.1f));

		tics_rigid_body_desc body_desc = {
			.shape = small_shape,
			.transform = {.position = {x_pos, y_pos, z_pos}, .rotation = {0, 0, 0, 1}},
			.mass = 1.0f,
			.elasticity = 0.87f,
			.gravity_scale = 1.0f,
			.linear_velocity = {0}};
		tics_world_add_rigid_body(world, body_desc);
	}

	const float delta = 1.0f / 60.0f;
	while (true) {
		tics_world_step(world, delta);

		usleep((unsigned int)(delta * 1000000.0f));
	}

	tics_world_destroy(world);
	return 0;
}
