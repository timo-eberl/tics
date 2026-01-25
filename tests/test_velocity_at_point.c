#include "test.h"
#include "tics.h"
#include "tics_internal.h"

// Helper to quickly initialize a rigid body with relevant kinematic data.
// We zero-init the struct first to ensure irrelevant fields (mass, shape, etc.) are clean.
static rigid_body_data make_test_body(tics_vec3 pos, tics_vec3 lin_vel, tics_vec3 ang_vel) {
	rigid_body_data rb = {0};
	rb.transform.position = pos;
	// Rotation quaternion doesn't affect velocity calculation (which uses angular velocity vector),
	// but we set it to identity for sanity.
	rb.transform.rotation = (tics_quat){0, 0, 0, 1};
	rb.linear_velocity = lin_vel;
	rb.angular_velocity = ang_vel;
	return rb;
}

void run_velocity_at_point_tests(void) {
	// -------------------------------------------------------------------------
	// A. The "Statue" Test (Zero State)
	// -------------------------------------------------------------------------
	// Explanation: An object with no linear or angular velocity must return
	// zero velocity for any point in space.
	{
		rigid_body_data rb =
			make_test_body((tics_vec3){0, 0, 0}, (tics_vec3){0, 0, 0}, (tics_vec3){0, 0, 0});
		tics_vec3 point = {10.0f, 20.0f, 30.0f};
		tics_vec3 result = get_velocity_at_point(&rb, point);

		ASSERT_VEC3_APPROX(result, ((tics_vec3){0, 0, 0}));
	}

	// -------------------------------------------------------------------------
	// B. Pure Linear Translation
	// -------------------------------------------------------------------------
	// Explanation: If angular velocity is zero, every point on the body (even
	// far away from center) must move with exactly the linear velocity.
	{
		tics_vec3 lin_vel = {10.0f, -5.0f, 0.0f};
		rigid_body_data rb = make_test_body((tics_vec3){0, 0, 0}, lin_vel, (tics_vec3){0, 0, 0});

		// Test at center
		ASSERT_VEC3_APPROX(get_velocity_at_point(&rb, ((tics_vec3){0, 0, 0})), lin_vel);
		// Test far away
		ASSERT_VEC3_APPROX(get_velocity_at_point(&rb, ((tics_vec3){100, 100, 100})), lin_vel);
	}

	// -------------------------------------------------------------------------
	// C. The "Eye of the Storm" (Center of Mass)
	// -------------------------------------------------------------------------
	// Explanation: At the center of mass, the cross product (w x r) becomes
	// (w x 0) = 0. Therefore, even with high rotation, the velocity at the COM
	// equals the linear velocity.
	{
		tics_vec3 center = {5.0f, 5.0f, 5.0f};
		tics_vec3 lin_vel = {1.0f, 0.0f, 0.0f};
		rigid_body_data rb =
			make_test_body(center, lin_vel, (tics_vec3){100, 100, 100} // High rotation
			);

		tics_vec3 result = get_velocity_at_point(&rb, center);
		ASSERT_VEC3_APPROX(result, lin_vel);
	}

	// -------------------------------------------------------------------------
	// D. Pure Rotation (Tangential Velocity)
	// -------------------------------------------------------------------------
	// Explanation: Test v = w x r.
	// Body is at (0,0,0). Spinning around Z axis (CCW).
	// Point at (1,0,0). Tangential velocity should be up Y axis.
	{
		rigid_body_data rb = make_test_body((tics_vec3){0, 0, 0}, (tics_vec3){0, 0, 0},
											(tics_vec3){0, 0, 1} // 1 rad/s around Z
		);

		// r = {1, 0, 0}
		// v = {0, 0, 1} x {1, 0, 0} = {0, 1, 0}
		ASSERT_VEC3_APPROX(get_velocity_at_point(&rb, ((tics_vec3){1, 0, 0})),
						   ((tics_vec3){0, 1, 0}));

		// Test Distance scaling (Lever Arm)
		// r = {2, 0, 0} -> v = {0, 2, 0}
		ASSERT_VEC3_APPROX(get_velocity_at_point(&rb, ((tics_vec3){2, 0, 0})),
						   ((tics_vec3){0, 2, 0}));
	}

	// -------------------------------------------------------------------------
	// E. World Space Offset Logic
	// -------------------------------------------------------------------------
	// Explanation: Critical test. We move the body away from the world origin.
	// If the function fails to subtract the body position from the input point
	// before the cross product, the result will be massively wrong.
	{
		rigid_body_data rb =
			make_test_body((tics_vec3){100, 0, 0},					  // Body shifted 100 units X
						   (tics_vec3){0, 0, 0}, (tics_vec3){0, 0, 1} // Spin around Z
			);

		// Point is at 101 X. Relative vector r is (1, 0, 0).
		// Result should be identical to Test D (velocity {0, 1, 0}).
		// Failure case: If it uses world pos, r={101,0,0}, result={0,101,0}.
		tics_vec3 point = {101.0f, 0.0f, 0.0f};
		ASSERT_VEC3_APPROX(get_velocity_at_point(&rb, point), ((tics_vec3){0, 1, 0}));
	}

	// -------------------------------------------------------------------------
	// F. Superposition (Translation + Rotation)
	// -------------------------------------------------------------------------
	// Explanation: Verify that Linear and Tangential velocities are summed correctly.
	// Body moves UP at 10 m/s. Spines around Z.
	// Point 1 unit Right should move UP at 10 (linear) + 1 (tangential) = 11.
	{
		rigid_body_data rb =
			make_test_body((tics_vec3){0, 0, 0}, (tics_vec3){0, 10, 0}, (tics_vec3){0, 0, 1});

		tics_vec3 point = {1.0f, 0.0f, 0.0f};
		ASSERT_VEC3_APPROX(get_velocity_at_point(&rb, point), ((tics_vec3){0, 11, 0}));
	}

	// -------------------------------------------------------------------------
	// G. Rolling Wheel (Opposing Velocities)
	// -------------------------------------------------------------------------
	// Explanation: Simulates the contact point of a wheel rolling to the right.
	// The bottom of the wheel should be momentarily stationary (velocity zero)
	// because linear velocity cancels out tangential rotation.
	{
		// Wheel center at Y=1. Radius 1.
		rigid_body_data rb =
			make_test_body((tics_vec3){0, 1, 0}, (tics_vec3){1, 0, 0}, // Moving Right at 1 m/s
						   (tics_vec3){0, 0, -1} // Spinning CW (Negative Z) at 1 rad/s
			);

		// Contact point at World Origin (0,0,0)
		// r = {0,0,0} - {0,1,0} = {0, -1, 0} (Down)
		// w = {0, 0, -1}
		// Tangential = w x r = (-1 Z) x (-1 Y) = -1 (Z x Y) = -1 (-X) = +X?
		// Wait: Z x Y = -X. So (-1)*(-1)*(-1) = -1. Tangential is {-1, 0, 0}.
		// Linear {1, 0, 0} + Tangential {-1, 0, 0} = {0, 0, 0}.
		tics_vec3 contact_point = {0.0f, 0.0f, 0.0f};

		ASSERT_VEC3_APPROX(get_velocity_at_point(&rb, contact_point), ((tics_vec3){0, 0, 0}));
	}
}
