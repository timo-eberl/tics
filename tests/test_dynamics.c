#include "test.h"

#include <tics.h>
#include <tics_math.h>

#define PI 3.14159265359f

void run_dynamics_tests(void) {
	// Setup world with no gravity or friction
	tics_world_desc world_desc = {.gravity = {0.0f, 0.0f, 0.0f}};
	tics_world* world = tics_world_create(world_desc);
	ASSERT_TRUE(world != NULL);

	// Create Rigid Body
	tics_shape_desc shape_desc = {.type = TICS_SHAPE_SPHERE, .data.sphere.radius = 1};
	tics_shape_id shape = tics_create_shape(world, shape_desc);
	// 2*PI rad/s around X axis = 1 full rotation per second
	tics_vec3 angular_vel = {2.0f * PI, 0.0f, 0.0f};
	tics_rigid_body_desc body_desc = {
		.shape = shape,
		.transform = {.position = {0, 0, 0}, .rotation = {0, 0, 0, 1}},
		.angular_velocity = angular_vel,
		.mass = 1.0f};
	tics_body_id body = tics_world_add_rigid_body(world, body_desc);

	// Test Step(0) - Ensure it is not rotated
	tics_world_step(world, 0.0f);
	tics_transform t = tics_body_get_transform(world, body);
	// Expect strict Identity
	tics_quat q_identity = {0, 0, 0, 1};
	ASSERT_QUAT_APPROX(t.rotation, q_identity);

	// Simulation Loop: Run 20 steps of 1/10s = 2.0s total
	const int steps = 10 * 2;
	const float delta = 1.0f / 10.0f;
	float current_time = 0.0f;
	tics_vec3 axis = {1.0f, 0.0f, 0.0f};

	for (int i = 0; i < steps; i++) {
		tics_world_step(world, delta);
		current_time += delta;

		// Calculate expected rotation
		float expected_angle = current_time * (2.0f * PI);
		tics_quat q_expected = quat_from_axis_angle(axis, expected_angle);

		t = tics_body_get_transform(world, body);

		// Verify quaternion components
		ASSERT_QUAT_APPROX(t.rotation, q_expected);
	}

	// Expect Identity after full rotation
	ASSERT_QUAT_APPROX(t.rotation, q_identity);

	tics_world_destroy(world);
}
