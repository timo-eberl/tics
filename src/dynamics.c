#include "tics_internal.h"
#include "tics_math.h"

#include <stb_ds.h>

static tics_quat to_legacy_angular_velocity(tics_vec3 av) {
	float speed = vec3_length(av);
	tics_vec3 axis = vec3_normalize(av);
	return quat_from_axis_angle(axis, speed * 0.1f);
}

static tics_vec3 from_legacy_angular_velocity(tics_quat q) {
	// q.w = cos(angle / 2), so angle = 2 * acos(q.w)
	// The stored angle represents rotation over 0.1s, so multiply by 10 for rad/s
	float speed = 2.0f * acosf(q.w) * 10.0f;

	// The vector part (x,y,z) is axis * sin(angle/2). Normalizing it recovers the axis.
	tics_vec3 axis = vec3_normalize((tics_vec3){q.x, q.y, q.z});

	return vec3_mul_f(axis, speed);
}

void apply_gravity_and_air_friction(tics_world* world, float delta) {
	// iterate directly over the flat array of rigid bodies for cache efficiency
	size_t count = arrlen(world->rigid_bodies);
	for (size_t i = 0; i < count; ++i) {
		rigid_body_data* rb = &world->rigid_bodies[i];

		// Gravity: Apply directly to linear velocity: v += gravity * delta * scale
		tics_vec3 gravity_impulse =
			vec3_mul_f(world->gravity, rb->mass * delta * rb->gravity_scale);
		tics_vec3 delta_v_grav = vec3_mul_f(gravity_impulse, rb->inv_mass);
		rb->linear_velocity = vec3_add(rb->linear_velocity, delta_v_grav);

		// linear air friction: v = (1 - (friction * delta))
		rb->linear_velocity =
			vec3_mul_f(rb->linear_velocity, (1.0f - (world->air_fric_lin * delta)));

		// angular air friction: simply scale the velocity vector
		rb->angular_velocity =
			vec3_mul_f(rb->angular_velocity, (1.0f - (world->air_fric_ang * delta)));
	}
}

tics_vec3 get_velocity_at_point(rigid_body_data* rb, tics_vec3 point) {
	tics_vec3 rotation_center = rb->transform.position;
	// Vector from center of mass to the point
	tics_vec3 r = vec3_sub(point, rotation_center);
	// Tangential velocity caused by rotation = angular_velocity CROSS r
	tics_vec3 v_tangential = vec3_cross(rb->angular_velocity, r);
	// Total velocity = linear + tangential
	return vec3_add(rb->linear_velocity, v_tangential);
}

void resolve_velocities(tics_world* world, collision* collisions) {
	size_t count = arrlen(collisions);
	for (size_t i = 0; i < count; ++i) {
		collision* col = &collisions[i];

		rigid_body_data* rb_a = NULL;
		static_body_data* sb_a = NULL;
		rigid_body_data* rb_b = NULL;
		static_body_data* sb_b = NULL;

		if (col->body_a_ref.type == RIGID_BODY) rb_a = &world->rigid_bodies[col->body_a_ref.index];
		else sb_a = &world->static_bodies[col->body_a_ref.index];

		if (col->body_b_ref.type == RIGID_BODY) rb_b = &world->rigid_bodies[col->body_b_ref.index];
		else sb_b = &world->static_bodies[col->body_b_ref.index];

		// continue if the objects are no valid object combination
		if (!((rb_a && rb_b) || (rb_a && sb_b) || (sb_a && rb_b))) { continue; }

		tics_vec3 velocity_a =
			rb_a ? get_velocity_at_point(rb_a, col->result.point_a) : (tics_vec3){0};
		tics_vec3 velocity_b =
			rb_b ? get_velocity_at_point(rb_b, col->result.point_b) : (tics_vec3){0};

		tics_vec3 pos_a = rb_a ? rb_a->transform.position : sb_a->transform.position;
		tics_vec3 pos_b = rb_b ? rb_b->transform.position : sb_b->transform.position;

		tics_vec3 r_a = vec3_sub(col->result.point_a, pos_a);
		tics_vec3 r_b = vec3_sub(col->result.point_b, pos_b);

		float r_a_dist_squared = vec3_length_sq(r_a);
		float r_b_dist_squared = vec3_length_sq(r_b);

		tics_vec3 n = col->result.normal;

		tics_vec3 v_r = vec3_sub(velocity_a, velocity_b);
		// relative velocity in the collision normal direction
		float n_dot_vr = vec3_dot(v_r, n);

		// n_dot_v is > 0 if the bodies are moving away from each other
		if (n_dot_vr >= 0) { continue; }

		// coefficient of restitution (cor) is the ratio of the relative velocity of separation
		// after collision to the relative velocity of approach before collision. it is a property
		// of BOTH collision objects (their "bounciness").
		float elas_a = rb_a ? rb_a->elasticity : sb_a->elasticity;
		float elas_b = rb_b ? rb_b->elasticity : sb_b->elasticity;
		float cor = elas_a * elas_b;

		float inv_mass_a = rb_a ? rb_a->inv_mass : 0.0f;
		float inv_mass_b = rb_b ? rb_b->inv_mass : 0.0f;

		// This is a rough approximation at best and completely wrong at worst
		// It is assumed that r_a_distance is equal to the radius
		float inv_moment_of_inertia_a = rb_a ? 1.0f / (rb_a->mass * r_a_dist_squared) : 0.0f;
		float inv_moment_of_inertia_b = rb_b ? 1.0f / (rb_b->mass * r_b_dist_squared) : 0.0f;

		// https://en.wikipedia.org/wiki/Collision_response (Impulse-based reaction model)
		// denom calculation: inv_mass_a + inv_mass_b + dot(n, ...)
		tics_vec3 term1 = vec3_cross(vec3_cross(r_a, n), r_a);
		term1 = vec3_mul_f(term1, inv_moment_of_inertia_a);
		tics_vec3 term2 = vec3_cross(vec3_cross(r_b, n), r_b);
		term2 = vec3_mul_f(term2, inv_moment_of_inertia_b);
		float denom = inv_mass_a + inv_mass_b + vec3_dot(n, vec3_add(term1, term2));

		float impulse_magnitude = (-(1.0f + cor) * n_dot_vr) / denom;

		// add impulse-based friction
		const float dynamic_friction_coefficient = 0.07f;
		// collision_tangent = Normalize( v_r - (Dot(v_r, n) * n) )
		tics_vec3 normal_comp = vec3_mul_f(n, vec3_dot(v_r, n));
		tics_vec3 collision_tangent = vec3_normalize(vec3_sub(v_r, normal_comp));

		tics_vec3 friction_impulse =
			vec3_mul_f(collision_tangent, impulse_magnitude * dynamic_friction_coefficient);

		// impulse = (magnitude * n) - friction
		tics_vec3 impulse = vec3_sub(vec3_mul_f(n, impulse_magnitude), friction_impulse);

		// apply impulses only to rigid bodies
		if (rb_a) {
			// Apply Linear Impulse immediately: v += impulse / mass
			tics_vec3 delta_v = vec3_mul_f(impulse, rb_a->inv_mass);
			rb_a->linear_velocity = vec3_add(rb_a->linear_velocity, delta_v);

			// Calculate Angular Impulse (Intermediate Variable)
			tics_vec3 angular_impulse = vec3_cross(r_a, impulse);
			if (angular_impulse.x != 0 || angular_impulse.y != 0 || angular_impulse.z != 0) {
				float str = vec3_length(angular_impulse) * 0.1f / r_a_dist_squared;
				tics_vec3 axis = vec3_normalize(angular_impulse);

				// angular impulse (instantaneous change in angular momentum) divided by square
				// distance to the application pos
				tics_quat an_imp_div_sq_dst = quat_from_axis_angle(axis, str);

				// Apply Angular Impulse immediately
				tics_quat legacy_av = to_legacy_angular_velocity(rb_a->angular_velocity);
				tics_quat angular_vel_change = quat_scale(an_imp_div_sq_dst, rb_a->inv_mass);
				legacy_av = quat_mul(angular_vel_change, legacy_av);
				rb_a->angular_velocity = from_legacy_angular_velocity(legacy_av);
			}
		}
		if (rb_b) {
			tics_vec3 impulse_neg = vec3_negate(impulse); // apply impulse in opposite direction

			// Apply Linear Impulse immediately
			tics_vec3 delta_v = vec3_mul_f(impulse_neg, rb_b->inv_mass);
			rb_b->linear_velocity = vec3_add(rb_b->linear_velocity, delta_v);

			// Calculate Angular Impulse (Intermediate Variable)
			tics_vec3 angular_impulse = vec3_cross(r_b, impulse_neg);
			if (angular_impulse.x != 0 || angular_impulse.y != 0 || angular_impulse.z != 0) {
				float str = vec3_length(angular_impulse) * 0.1f / r_b_dist_squared;
				tics_vec3 axis = vec3_normalize(angular_impulse);

				tics_quat an_imp_div_sq_dst = quat_from_axis_angle(axis, str);

				// Apply Angular Impulse immediately
				// CONVERSION: Vec3 -> Quat -> Math -> Vec3
				tics_quat legacy_av = to_legacy_angular_velocity(rb_b->angular_velocity);
				tics_quat angular_vel_change = quat_scale(an_imp_div_sq_dst, rb_b->inv_mass);
				legacy_av = quat_mul(angular_vel_change, legacy_av);
				rb_b->angular_velocity = from_legacy_angular_velocity(legacy_av);
			}
		}
	}
}

void resolve_penetrations(tics_world* world, collision* collisions) {
	size_t count = arrlen(collisions);
	for (size_t i = 0; i < count; ++i) {
		collision* col = &collisions[i];

		rigid_body_data* rb_a = NULL;
		static_body_data* sb_a = NULL;
		rigid_body_data* rb_b = NULL;
		static_body_data* sb_b = NULL;

		if (col->body_a_ref.type == RIGID_BODY) rb_a = &world->rigid_bodies[col->body_a_ref.index];
		else sb_a = &world->static_bodies[col->body_a_ref.index];

		if (col->body_b_ref.type == RIGID_BODY) rb_b = &world->rigid_bodies[col->body_b_ref.index];
		else sb_b = &world->static_bodies[col->body_b_ref.index];

		const float percent = 0.8f;
		const float depth_tolerance = 0.01f; // how much they are allowed to glitch into another

		float depth_with_tolerance = fmaxf(col->result.depth - depth_tolerance, 0.0f);
		// distance that the objects are moved away from each other
		tics_vec3 correction = vec3_mul_f(col->result.normal, percent * depth_with_tolerance);

		if (rb_a && rb_b) {
			// rigid body vs rigid body
			// Apply proportional offset based on mass (heavier object moves less)
			float b_share = rb_b->mass / (rb_a->mass + rb_b->mass);

			tics_vec3 offset_a = vec3_mul_f(correction, b_share);
			tics_vec3 offset_b = vec3_mul_f(correction, -(1.0f - b_share));

			rb_a->transform.position = vec3_add(rb_a->transform.position, offset_a);
			rb_b->transform.position = vec3_add(rb_b->transform.position, offset_b);
		}
		else if (rb_a && sb_b) {
			// rigid body vs static body (Only move rigid body A)
			rb_a->transform.position = vec3_add(rb_a->transform.position, correction);
		}
		else if (sb_a && rb_b) {
			// static body vs RigidBody (Only move rigid body B)
			tics_vec3 neg_correction = vec3_negate(correction);
			rb_b->transform.position = vec3_add(rb_b->transform.position, neg_correction);
		}
	}
}

void apply_velocities(tics_world* world, float delta) {
	size_t count = arrlen(world->rigid_bodies);
	for (size_t i = 0; i < count; ++i) {
		rigid_body_data* rb = &world->rigid_bodies[i];

		// apply linear velocity to transform
		// position += v * delta
		tics_vec3 pos_change = vec3_mul_f(rb->linear_velocity, delta);
		rb->transform.position = vec3_add(rb->transform.position, pos_change);

		// apply angular velocity to transform
		// angle = speed (rad/s) * delta
		float angle = vec3_length(rb->angular_velocity) * delta;
		tics_vec3 axis = vec3_normalize(rb->angular_velocity);
		tics_quat rotation_change = quat_from_axis_angle(axis, angle);

		rb->transform.rotation = quat_mul(rb->transform.rotation, rotation_change);
		rb->transform.rotation = quat_normalize(rb->transform.rotation);
	}
}
