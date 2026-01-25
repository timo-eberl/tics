#include "test.h"

#include "tics_internal.h"
#include <tics_math.h>

#define PI 3.14159265359f

/*
 * Case A: Gravity Scaling
 * Goal: Verify per-body gravity scales work, including negative and zero scales.
 *
 * Case B & C: Air Friction
 * Goal: Verify resistance is applied to translation and rotation.
 * Logic: Semi-implicit euler with friction applied after gravity.
 */
static void test_gravity_and_friction_internals(void) {
	// Setup: Gravity -10 Y, Linear Friction 0.1, Angular Friction 0.5
	tics_world_desc world_desc = {
		.gravity = {0.0f, -10.0f, 0.0f},
		.air_friction_linear = 0.1f,
		.air_friction_angular = 0.5f,
	};
	tics_world* world = tics_world_create(world_desc);
	ASSERT_TRUE(world != NULL);

	// Create a dummy shape (required to create bodies)
	tics_shape_desc shape_desc = {.type = TICS_SHAPE_SPHERE, .data.sphere.radius = 1.0f};
	tics_shape_id shape = tics_create_shape(world, shape_desc);

	// --- Populate World with Test Bodies ---

	// Body 0: Case A - Normal Gravity (Scale 1.0)
	// Expect: v = (0 + -10 * 1 * 1) * (1 - 0.1) = -9.0
	tics_world_add_rigid_body(
		world, (tics_rigid_body_desc){.shape = shape, .mass = 1.0f, .gravity_scale = 1.0f});

	// Body 1: Case A - Half Gravity (Scale 0.5)
	// Expect: v = (0 + -10 * 0.5 * 1) * (1 - 0.1) = -4.5
	tics_world_add_rigid_body(
		world, (tics_rigid_body_desc){.shape = shape, .mass = 1.0f, .gravity_scale = 0.5f});

	// Body 2: Case A - Zero Gravity (Scale 0.0)
	// Expect: v = (0 + 0) * (1 - 0.1) = 0.0
	tics_world_add_rigid_body(
		world, (tics_rigid_body_desc){.shape = shape, .mass = 1.0f, .gravity_scale = 0.0f});

	// Body 3: Case A - Inverted Gravity (Scale -1.0)
	// Expect: v = (0 + -10 * -1 * 1) * (1 - 0.1) = 9.0
	tics_world_add_rigid_body(
		world, (tics_rigid_body_desc){.shape = shape, .mass = 1.0f, .gravity_scale = -1.0f});

	// Body 4: Case B - Linear Friction only (Gravity disabled via scale)
	// Setup: Initial Vel {100, 0, 0}, Gravity Scale 0.
	// Expect: v = 100 * (1.0 - 0.1 * 1.0) = 90.0
	tics_world_add_rigid_body(world,
							  (tics_rigid_body_desc){.shape = shape,
													 .mass = 1.0f,
													 .gravity_scale = 0.0f,
													 .linear_velocity = {100.0f, 0.0f, 0.0f}});

	// Body 5: Case C - Angular Friction
	// Setup: Initial AngVel {0, 10, 0}.
	// Expect: v = 10 * (1.0 - 0.5 * 1.0) = 5.0
	tics_world_add_rigid_body(world,
							  (tics_rigid_body_desc){.shape = shape,
													 .mass = 1.0f,
													 .gravity_scale = 0.0f,
													 .angular_velocity = {0.0f, 10.0f, 0.0f}});

	// --- Execute Internal Function ---
	float delta = 1.0f;
	apply_gravity_and_air_friction(world, delta);

	// --- Verify Internal State (White-Box) ---

	// The internal array is `rigid_bodies`. We assume insertion order matches array index
	// for a freshly created world.

	// Case A: Normal Gravity (Scale 1.0) -> (-10 * 0.9) = -9.0
	ASSERT_FLOAT_APPROX(world->rigid_bodies[0].linear_velocity.y, -9.0f);

	// Case A: Half Gravity (Scale 0.5) -> (-5 * 0.9) = -4.5
	ASSERT_FLOAT_APPROX(world->rigid_bodies[1].linear_velocity.y, -4.5f);

	// Case A: Zero Gravity (Scale 0.0) -> 0.0
	ASSERT_FLOAT_APPROX(world->rigid_bodies[2].linear_velocity.y, 0.0f);

	// Case A: Inverted Gravity (Scale -1.0) -> (10 * 0.9) = 9.0
	ASSERT_FLOAT_APPROX(world->rigid_bodies[3].linear_velocity.y, 9.0f);

	// Case B: Linear Friction
	// 100 * 0.9 = 90.0
	ASSERT_FLOAT_APPROX(world->rigid_bodies[4].linear_velocity.x, 90.0f);
	// Ensure gravity didn't leak in (gravity scale was 0)
	ASSERT_FLOAT_APPROX(world->rigid_bodies[4].linear_velocity.y, 0.0f);

	// Case C: Angular Friction
	// 10 * 0.5 = 5.0
	ASSERT_FLOAT_APPROX(world->rigid_bodies[5].angular_velocity.y, 5.0f);

	tics_world_destroy(world);
}

/*
 * Angular Velocity Integration
 * Goal: Verify that tics_world_step correctly integrates angular velocity into the body's transform
 * rotation quaternion over time.
 */
static void test_integration_rotation(void) {
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

void run_dynamics_tests(void) {
	test_gravity_and_friction_internals();
	test_integration_rotation();
}
