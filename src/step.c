#define _POSIX_C_SOURCE 199309L // Required for clock_gettime

#include "tics_internal.h"

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

void tics_world_step(tics_world* world, float delta) {
	assert(world);

	static uint64_t dynamics_total = 0;
	static uint64_t collision_total = 0;
	static int steps = 0;

	// --- Dynamics ---

	uint64_t start_dynamics = get_time_ns();

	// iterate directly over the flat array of rigid bodies for cache efficiency
	size_t count = arrlen(world->rigid_bodies);
	for (size_t i = 0; i < count; ++i) {
		rigid_body_data* rb = &world->rigid_bodies[i];

		// Gravity: impulse += gravity * mass * delta * scale
		tics_vec3 gravity_impulse =
			tics_vec3_mul_f(world->gravity, rb->mass * delta * rb->gravity_scale);
		rb->impulse = tics_vec3_add(rb->impulse, gravity_impulse);

		// apply linear impulse to linear velocity
		// v += impulse / mass
		tics_vec3 delta_v = tics_vec3_mul_f(rb->impulse, rb->inv_mass);
		rb->linear_velocity = tics_vec3_add(rb->linear_velocity, delta_v);

		// apply angular impulse to angular velocity
		// NOTE: angular velocity is stored in rad / 0.1s
		tics_quat angular_vel_change = tics_quat_scale(rb->an_imp_div_sq_dst, rb->inv_mass);
		rb->angular_velocity = tics_quat_mul(angular_vel_change, rb->angular_velocity);

		// apply linear velocity to transform
		// position += v * delta
		tics_vec3 pos_change = tics_vec3_mul_f(rb->linear_velocity, delta);
		rb->transform.position = tics_vec3_add(rb->transform.position, pos_change);

		// apply angular velocity to transform
		// rotation *= ang_vel * delta * 10.0f
		tics_quat rotation_change = tics_quat_scale(rb->angular_velocity, delta * 10.0f);
		rb->transform.rotation = tics_quat_mul(rb->transform.rotation, rotation_change);

		// linear air friction
		const float lin_fric = 0.2f;
		tics_vec3 fric_loss = tics_vec3_mul_f(rb->linear_velocity, lin_fric * delta);
		rb->linear_velocity = tics_vec3_sub(rb->linear_velocity, fric_loss);

		// angular air friction
		const float ang_fric = 0.5f;
		tics_quat identity = {0, 0, 0, 1};
		// Lerp towards identity
		rb->angular_velocity = tics_quat_lerp(rb->angular_velocity, identity, ang_fric * delta);

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

			collision_result res = collision_test(&rb_a->shape, rb_a->transform, &rb_b->shape,
													   rb_b->transform);

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
	
	steps++;
	if (steps % 10 == 0) {
		double d_avg = (double)dynamics_total / steps;
		double cd_avg = (double)collision_total / steps;
		printf("d: %.0fns, cd: %.0fns, cr: 0ns\n", d_avg, cd_avg);
	}

	// --- Collision Response ---
	// TODO

	arrfree(collisions);
}
