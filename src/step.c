#define _POSIX_C_SOURCE 199309L // Required for clock_gettime

#include "tics_debug_view_shm_internal.h"
#include "tics_internal.h"
#include "tics_math.h"

#include <stb_ds.h>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// high-resolution timing
static uint64_t get_time_ns() {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

static tics_vec3 get_velocity(rigid_body_data* rb, tics_vec3 point) {
	tics_vec3 q_vec = {rb->angular_velocity.x, rb->angular_velocity.y, rb->angular_velocity.z};
	bool no_rotation = vec3_length(q_vec) < 0.01f;

	tics_vec3 axis = no_rotation ? (tics_vec3){1, 0, 0} : vec3_negate(vec3_normalize(q_vec));

	float half_angle = no_rotation ? 0.0f : acosf(rb->angular_velocity.w);
	if (isnan(half_angle)) { // NaN check
		axis = (tics_vec3){1, 0, 0};
		half_angle = 0.0f;
	}

	tics_vec3 lin_vel = rb->linear_velocity;
	tics_vec3 rotation_center = rb->transform.position;

	// move to local space of rigid body
	tics_vec3 local = vec3_sub(point, rotation_center);
	// "move" the point according to the angular velocity
	tics_vec3 rotated = quat_rotate_vec3(local, quat_from_axis_angle(axis, half_angle * 2.0f));
	tics_vec3 rotated_world_space = vec3_add(rotated, rotation_center);

	// ws_premoved_point = rotated_world_space + lin_vel * 0.1f
	tics_vec3 ws_premoved_point = vec3_add(rotated_world_space, vec3_mul_f(lin_vel, 0.1f));

	// total_v = (ws_premoved_point - point) * 10.0f
	tics_vec3 total_v = vec3_mul_f(vec3_sub(ws_premoved_point, point), 10.0f);
	return total_v;
}

static void solve_impulses(tics_world* world, collision* collisions, float delta) {
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

		tics_vec3 velocity_a = rb_a ? get_velocity(rb_a, col->result.point_a) : (tics_vec3){0};
		tics_vec3 velocity_b = rb_b ? get_velocity(rb_b, col->result.point_b) : (tics_vec3){0};

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

		float inv_moment_of_inertia_a = rb_a ? 1.0f / (rb_a->mass * r_a_dist_squared) : 0.0f;
		float inv_moment_of_inertia_b = rb_b ? 1.0f / (rb_b->mass * r_b_dist_squared) : 0.0f;

		// https://en.wikipedia.org/wiki/Collision_response
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
			rb_a->impulse = vec3_add(rb_a->impulse, impulse);

			tics_vec3 angular_impulse = vec3_cross(r_a, impulse);
			if (angular_impulse.x != 0 || angular_impulse.y != 0 || angular_impulse.z != 0) {
				float str = vec3_length(angular_impulse) * 0.1f / r_a_dist_squared;
				tics_vec3 axis = vec3_normalize(angular_impulse);
				rb_a->an_imp_div_sq_dst = quat_from_axis_angle(vec3_negate(axis), str);
			}
		}
		if (rb_b) {
			impulse = vec3_negate(impulse); // apply impulse in opposite direction
			rb_b->impulse = vec3_add(rb_b->impulse, impulse);

			tics_vec3 angular_impulse = vec3_cross(r_b, impulse);
			if (angular_impulse.x != 0 || angular_impulse.y != 0 || angular_impulse.z != 0) {
				float str = vec3_length(angular_impulse) * 0.1f / r_b_dist_squared;
				tics_vec3 axis = vec3_normalize(angular_impulse);
				rb_b->an_imp_div_sq_dst = quat_from_axis_angle(vec3_negate(axis), str);
			}
		}
	}
}

static void solve_positions(tics_world* world, collision* collisions, float delta) {
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

void tics_world_step(tics_world* world, float delta) {
	assert(world);

	TICS_VIEW_FRAME_START();

	static uint64_t dynamics_total = 0;
	static uint64_t collision_total = 0;
	static uint64_t solver_total = 0;
	static int steps = 0;

	// --- Dynamics ---

	uint64_t start_dynamics = get_time_ns();

	// iterate directly over the flat array of rigid bodies for cache efficiency
	size_t count = arrlen(world->rigid_bodies);
	for (size_t i = 0; i < count; ++i) {
		rigid_body_data* rb = &world->rigid_bodies[i];

		// Gravity: impulse += gravity * mass * delta * scale
		tics_vec3 gravity_impulse =
			vec3_mul_f(world->gravity, rb->mass * delta * rb->gravity_scale);
		rb->impulse = vec3_add(rb->impulse, gravity_impulse);

		// apply linear impulse to linear velocity
		// v += impulse / mass
		tics_vec3 delta_v = vec3_mul_f(rb->impulse, rb->inv_mass);
		rb->linear_velocity = vec3_add(rb->linear_velocity, delta_v);

		// apply angular impulse to angular velocity
		// NOTE: angular velocity is stored in rad / 0.1s
		tics_quat angular_vel_change = quat_scale(rb->an_imp_div_sq_dst, rb->inv_mass);
		rb->angular_velocity = quat_mul(angular_vel_change, rb->angular_velocity);

		// apply linear velocity to transform
		// position += v * delta
		tics_vec3 pos_change = vec3_mul_f(rb->linear_velocity, delta);
		rb->transform.position = vec3_add(rb->transform.position, pos_change);

		// apply angular velocity to transform
		// rotation *= ang_vel * delta * 10.0f
		tics_quat rotation_change = quat_scale(rb->angular_velocity, delta * 10.0f);
		rb->transform.rotation = quat_mul(rb->transform.rotation, rotation_change);

		// linear air friction
		const float lin_fric = 0.2f;
		tics_vec3 fric_loss = vec3_mul_f(rb->linear_velocity, lin_fric * delta);
		rb->linear_velocity = vec3_sub(rb->linear_velocity, fric_loss);

		// angular air friction
		const float ang_fric = 0.5f;
		tics_quat identity = {0, 0, 0, 1};
		// Lerp towards identity
		rb->angular_velocity = quat_lerp(rb->angular_velocity, identity, ang_fric * delta);

		// reset impulses
		rb->impulse = (tics_vec3){0, 0, 0};
		rb->an_imp_div_sq_dst = (tics_quat){0, 0, 0, 1};
	}

	uint64_t end_dynamics = get_time_ns();
	dynamics_total += (end_dynamics - start_dynamics);

	// --- Collision Detection ---

	uint64_t start_cd = get_time_ns();

	// Temporary array to store collisions for collision response
	collision* collisions = NULL;

	size_t rb_count = arrlen(world->rigid_bodies);
	size_t sb_count = arrlen(world->static_bodies);

	// RigidBody vs RigidBody
	// Checks unique pairs: i vs j where j > i
	for (size_t i = 0; i < rb_count; ++i) {
		for (size_t j = i + 1; j < rb_count; ++j) {
			rigid_body_data* rb_a = &world->rigid_bodies[i];
			rigid_body_data* rb_b = &world->rigid_bodies[j];

			collision_result res =
				collision_test(&rb_a->shape, rb_a->transform, &rb_b->shape, rb_b->transform);

			if (res.has_collision) {
				collision col;
				col.body_a_ref = (body_ref){RIGID_BODY, i};
				col.body_b_ref = (body_ref){RIGID_BODY, j};
				col.result = res;
				arrput(collisions, col);
			}
		}
	}

	// RigidBody vs StaticBody
	for (size_t i = 0; i < rb_count; ++i) {
		for (size_t j = 0; j < sb_count; ++j) {
			rigid_body_data* rb = &world->rigid_bodies[i];
			static_body_data* sb = &world->static_bodies[j];

			collision_result res =
				collision_test(&rb->shape, rb->transform, &sb->shape, sb->transform);

			if (res.has_collision) {
				collision col;
				col.body_a_ref = (body_ref){RIGID_BODY, i};
				col.body_b_ref = (body_ref){STATIC_BODY, j};
				col.result = res;
				arrput(collisions, col);
			}
		}
	}

	uint64_t end_cd = get_time_ns();
	collision_total += (end_cd - start_cd);

	// --- Collision Response ---

	uint64_t start_cr = get_time_ns();

	solve_impulses(world, collisions, delta);
	solve_positions(world, collisions, delta);

	uint64_t end_cr = get_time_ns();
	solver_total += (end_cr - start_cr);

	steps++;
	if (steps % 10 == 0) {
		double d_avg = (double)dynamics_total / steps;
		double cd_avg = (double)collision_total / steps;
		double cr_avg = (double)solver_total / steps;
		// printf("d: %.0fns, cd: %.0fns, cr: %.0fns\n", d_avg, cd_avg, cr_avg);
	}

	arrfree(collisions);

	if (arrlen(world->rigid_bodies) > 0) {
		TICS_VIEW_POINT(world->rigid_bodies[0].transform.position, 0.5f, 0xFFFF0000);
	}

	TICS_VIEW_FRAME_END();
}
