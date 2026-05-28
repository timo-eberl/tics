#include <stdbool.h>
#include <stdio.h>
#include <tics.h>
#include <unistd.h> // For usleep

#define SPHERE_COUNT 2

int main() {
	tics_world* world = tics_world_create((tics_world_desc){.gravity = {0.0f, -10.0f, 0.0f},
															.air_friction_linear = 0.01f,
															.air_friction_angular = 0.01f});

	// Large sphere shape for the environment (Ground and Walls)
	float large_radius = 50.0f;
	tics_shape_id large_shape = tics_create_sphere_shape(world, (tics_vec3){0}, large_radius);
	// Small sphere shape for the dynamic objects
	float small_radius = 1.0f;
	tics_shape_id small_shape = tics_create_sphere_shape(world, (tics_vec3){0}, small_radius);

	// Ground: Place center at -large_radius so the "top" of the sphere is at Y=0
	tics_static_body_desc ground_desc = {
		.transform = {.position = {0, -large_radius, 0}, .rotation = {0, 0, 0, 1}},
		.shape = large_shape,
		.elasticity = 0.5f};
	tics_world_add_static_body(world, ground_desc);

	// The simulation behaves a bit differently when objects are added in a different order
	// for (int i = SPHERE_COUNT-1; i >= 0; i--) {
	for (int i = 0; i < SPHERE_COUNT; i++) {
		// Stack them vertically in the center
		// They are slightly penetrating, because that's realistic in a physics engine
		float y_pos = 0.999f + (i * (small_radius * 1.999f));

		tics_rigid_body_desc body_desc = {
			.shape = small_shape,
			.transform = {.position = {0.0f, y_pos, 0.0f}, .rotation = {0, 0, 0, 1}},
			.mass = 1.0f,
			.elasticity = 0.0f,
			.gravity_scale = 1.0f,
			.linear_velocity = {0} // Perfect symmetry - spheres will stack on top of each other
		};
		tics_world_add_rigid_body(world, body_desc);
	}

	const float delta = 1.0f / 5.0f;
	while (true) {
		tics_world_step(world, delta);

		usleep((unsigned int)(delta * 1000000.0f));
	}

	tics_world_destroy(world);
	return 0;
}
