#include <stdbool.h>
#include <stdio.h>
#include <tics.h>
#include <unistd.h> // For usleep

const tics_vec3 velocity = {-1.5, 0, 0};

int main() {
	tics_world* world = tics_world_create((tics_world_desc){
		.gravity = {0}, .air_friction_linear = 0.01f, .air_friction_angular = 0.01f});
	float small_radius = 1.0f;
	tics_shape_desc small_sphere_desc = {
		.type = TICS_SHAPE_SPHERE, .data.sphere = {.center = {0, 0, 0}, .radius = small_radius}};
	tics_shape_id small_shape = tics_create_shape(world, small_sphere_desc);

	tics_static_body_desc ground_desc = {.transform = {.position = {0}, .rotation = {0, 0, 0, 1}},
										 .shape = small_shape,
										 .elasticity = 0.0f};
	tics_world_add_static_body(world, ground_desc);

	tics_rigid_body_desc dyn_desc = {
		.transform = {.position = {10, 0, 0}, .rotation = {0, 0, 0, 1}},
		.shape = small_shape,
		.elasticity = 0.0f,
		.mass = 1.0f,
		.linear_velocity = velocity};
	tics_world_add_rigid_body(world, dyn_desc);
	dyn_desc.transform.position.x = 20;
	dyn_desc.linear_velocity.x = 0;
	tics_world_add_rigid_body(world, dyn_desc);

	const float delta = 1.0f / 60.0f;
	while (true) {
		tics_world_step(world, delta);

		usleep((unsigned int)(delta * 1000000.0f));
	}

	tics_world_destroy(world);
	return 0;
}
