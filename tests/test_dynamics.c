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

	// The internal array is `rigid_bodies`. We assume insertion order matches array index for a
	// freshly created world.

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
 * Test: Gravity Integration Loop
 * Goal: Verify that gravity accumulates correctly over many small simulation steps and moves the
 * body according to the laws of physics.
 */
static void test_gravity_integration_loop(void) {
	// Setup: Pure gravity, no friction to ensure linear accumulation
	tics_world_desc world_desc = {
		.gravity = {0.0f, -10.0f, 0.0f}, .air_friction_linear = 0.0f, .air_friction_angular = 0.0f};
	tics_world* world = tics_world_create(world_desc);
	ASSERT_TRUE(world != NULL);

	// Create dummy body
	tics_shape_desc shape_desc = {.type = TICS_SHAPE_SPHERE, .data.sphere.radius = 1.0f};
	tics_shape_id shape = tics_create_shape(world, shape_desc);

	tics_world_add_rigid_body(world, (tics_rigid_body_desc){.shape = shape,
															.mass = 1.0f,
															.gravity_scale = 1.0f,
															.transform = {.position = {0, 0, 0},
																		  .rotation = {0, 0, 0, 1}},
															.linear_velocity = {0.0f, 0.0f, 0.0f}});

	// Simulation parameters
	int steps_per_second = 60;
	float delta = 1.0f / (float)steps_per_second;

	// Run integration loop
	// Note: We use tics_world_step here instead of just apply_gravity_and_air_friction
	// because we need the position to update (via apply_velocities) to test the trajectory.
	for (int i = 0; i < steps_per_second; i++) {
		tics_world_step(world, delta);
	}

	// Velocity Verification
	// Formula: v = v0 + a * t
	// v_y = 0 + (-10.0 * 1.0) = -10.0
	const float expected_vel_y = world_desc.gravity.y * 1.0f;
	ASSERT_FLOAT_APPROX(world->rigid_bodies[0].linear_velocity.y, expected_vel_y);

	// Position Verification (Calculus vs Euler)
	// Formula: p = p0 + v0 * t + 0.5 * a * t^2
	// p_y = 0 + 0 + 0.5 * -10.0 * (1.0)^2 = -5.0
	// Note: Semi-implicit Euler is inaccurate.
	// Actual result will be slightly larger in magnitude, meaning the distance it has fallen is
	// greater than in reality (meaning it has lost energy, but did not gain it).
	// The assertion is written against the perfect calculus solution, though it may fail if the
	// integrator error exceeds the epsilon.
	const float expected_pos_y = 0.5f * world_desc.gravity.y * (1.0f * 1.0f);

	// We use a big epsilon because we tolerate some integration error.
	float epsilon = 0.1f;
	ASSERT_FLOAT_WITHIN(world->rigid_bodies[0].transform.position.y, expected_pos_y, epsilon);

	// Verify that we did not gain energy: While the kinetic energy is correct (kinetic energy), the
	// body has fallen further than expected (potential energy). It has lost more potential energy
	// than it has gained kinetic energy.
	ASSERT_TRUE(world->rigid_bodies[0].transform.position.y <= expected_pos_y);

	// run it again with a more steps and less fault tolerance
	{
		// reset
		world->rigid_bodies[0].transform.position = (tics_vec3){0, 0, 0};
		world->rigid_bodies[0].linear_velocity = (tics_vec3){0, 0, 0};

		steps_per_second = 6000;
		delta = 1.0f / (float)steps_per_second;
		for (int i = 0; i < steps_per_second; i++) {
			tics_world_step(world, delta);
		}

		// We use a smaller epsilon because with increasing steps, the integration error shrinks.
		epsilon = 0.001f;
		ASSERT_FLOAT_WITHIN(world->rigid_bodies[0].transform.position.y, expected_pos_y, epsilon);

		// Verify that we did not gain energy.
		ASSERT_TRUE(world->rigid_bodies[0].transform.position.y <= expected_pos_y);
	}

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
	test_gravity_integration_loop();
	test_integration_rotation();
}
