#include "blick_adapter.h"
#include "profiler.h"
#include "tics_internal.h"
#include "tics_math.h"

#include <stb_ds.h>

#include <assert.h>
#include <stdio.h>

const bool USE_WARM_STARTING = true;
const bool USE_POSITION_SOLVER = false;
const int SOLVER_ITERATIONS = 10;

void tics_world_step(tics_world* world, float delta) {
	assert(world);

	PROFILE("Apply Forces") {
		apply_gravity_and_air_friction(world, delta);
	}

	for (size_t i = 0; i < arrlen(world->rigid_bodies); ++i) {
		rigid_body_data rb = world->rigid_bodies[i];
		// shapes
		BLICK_DRAW_SHAPE(1, rb.shape, rb.transform, 0x11DDFFDD, false);
		BLICK_DRAW_SHAPE(1, rb.shape, rb.transform, 0xFF99AA44, true);
		// velocities
		tics_vec3 to = vec3_add(rb.transform.position, vec3_mul_f(rb.linear_velocity, 0.2f));
		BLICK_ARROW(2, rb.transform.position, to, 0xFFFF44FF);
		// transforms
		BLICK_TRANSFORM(2, rb.transform, 0.4);
		// IDs
		// BLICK_TEXT_INT(3, rb.transform.position, rb.id, 0xFFFFFFFF);
	}

	broad_phase_proxy* proxies_r = NULL;
	broad_phase_proxy* proxies_s = NULL;
	broad_phase_proxy_typed* proxies_typed = NULL;
	broad_phase_proxies_soa proxies_r_soa;
	broad_phase_proxies_soa proxies_s_soa;
	broad_phase_pair* potential_collision_pairs = NULL;
	collision* collisions = NULL;

	PROFILE("Collision Detection") {
		PROFILE("Proxy Collection") {
			proxies_r = build_rigid_proxies(world);
			proxies_s = build_static_proxies(world);
		}
		PROFILE("Proxy Collection Typed") {
			proxies_typed = build_typed_proxies(world);
		}
		PROFILE("Proxy Collection SoA") {
			proxies_r_soa = build_rigid_proxies_soa(world);
			proxies_s_soa = build_static_proxies_soa(world);
		}
		PROFILE("Broad Phase") {
			// clang-format off

			// potential_collision_pairs = broad_phase_naive(
			// 	proxies_r, arrlen(proxies_r), proxies_s, arrlen(proxies_s));

			// potential_collision_pairs = broad_phase_naive_parallel(
			// 	proxies_r, arrlen(proxies_r), proxies_s, arrlen(proxies_s));

			// potential_collision_pairs = broad_phase_naive_simd(proxies_r_soa, proxies_s_soa);

			// potential_collision_pairs = broad_phase_naive_autovec(proxies_r_soa, proxies_s_soa);

			// potential_collision_pairs = broad_phase_naive_simd_speculative(
			// 	proxies_r_soa, proxies_s_soa);

			// potential_collision_pairs = broad_phase_naive_autovec_parallel(
			// 	proxies_r_soa, proxies_s_soa);

			potential_collision_pairs = broad_phase_sap(proxies_typed, arrlen(proxies_typed));

			// clang-format on

			// Verify correctness
			// broad_phase_pair* reference_pairs =
			// 	broad_phase_naive(proxies_r, arrlen(proxies_r), proxies_s, arrlen(proxies_s));
			// int len = arrlen(potential_collision_pairs);
			// int reflen = arrlen(reference_pairs);
			// assert(len == reflen);
			// arrfree(reference_pairs);
		}
		PROFILE("Narrow Phase") {
			collisions = narrow_phase(potential_collision_pairs, arrlen(potential_collision_pairs),
									  world->rigid_bodies, world->static_bodies);
		}
	}

	BLICK_CLEAR(0b10000);
	// broadphase AABBs
	// for (size_t i = 0; i < arrlen(proxies_r); ++i) {
	// 	broad_phase_proxy* p = &proxies_r[i];
	// 	BLICK_AABB(4, p->aabb.min, p->aabb.max, 0xFFFF0000);
	// }
	// broadphase SoA AABBs
	// for (size_t i = 0; i < proxies_r_soa.count; ++i) {
	// 	tics_vec3 aabb_min = {proxies_r_soa.min_x[i], proxies_r_soa.min_y[i],
	// 						  proxies_r_soa.min_z[i]};
	// 	tics_vec3 aabb_max = {proxies_r_soa.max_x[i], proxies_r_soa.max_y[i],
	// 						  proxies_r_soa.max_z[i]};
	// 	BLICK_AABB(4, aabb_min, aabb_max, 0xFFFF0000);
	// }
	// collisions
	for (size_t i = 0; i < arrlen(collisions); ++i) {
		collision_result result = collisions[i].result;
		// draw a red arrow between collision points (might be very small)
		BLICK_ARROW(2, result.point_a, result.point_b, 0xFF0000FF);
		// draw two yellow lines with a fixed length extending in both directions of the arrow
		tics_vec3 target_a = vec3_add(result.point_a, vec3_mul_f(result.normal, -0.2f));
		tics_vec3 target_b = vec3_add(result.point_b, vec3_mul_f(result.normal, 0.2f));
		BLICK_LINE(2, result.point_a, target_a, 0xFF00FFFF);
		BLICK_LINE(2, result.point_b, target_b, 0xFF00FFFF);
	}
	BLICK_REFRESH();

	arrfree(proxies_r);
	arrfree(proxies_s);
	arrfree(proxies_typed);
	free_proxies_soa(&proxies_r_soa);
	free_proxies_soa(&proxies_s_soa);
	arrfree(potential_collision_pairs);

	PROFILE("Collision Response") {
		size_t col_count = arrlen(collisions);

		prepare_velocity_solver(world, collisions);

		// Warm Starting: Apply cached impulses from previous step
		for (size_t i = 0; i < col_count; ++i) {
			collision* c = &collisions[i];
			c->accumulated_impulse = 0.0f; // Default state
			if (!USE_WARM_STARTING) continue;

			// Get body IDs and transform
			rigid_body_data* rb_a = (c->body_a_ref.type == RIGID_BODY)
										? &world->rigid_bodies[c->body_a_ref.index]
										: NULL;
			static_body_data* sb_a = (c->body_a_ref.type == STATIC_BODY)
										 ? &world->static_bodies[c->body_a_ref.index]
										 : NULL;
			rigid_body_data* rb_b = (c->body_b_ref.type == RIGID_BODY)
										? &world->rigid_bodies[c->body_b_ref.index]
										: NULL;
			static_body_data* sb_b = (c->body_b_ref.type == STATIC_BODY)
										 ? &world->static_bodies[c->body_b_ref.index]
										 : NULL;
			tics_body_id id_a = rb_a ? rb_a->id : sb_a->id;
			tics_body_id id_b = rb_b ? rb_b->id : sb_b->id;
			tics_transform t_a = rb_a ? rb_a->transform : sb_a->transform;

			// Generate sorted key for hash map lookup
			manifold_key key =
				(id_a < id_b) ? (manifold_key){id_a, id_b} : (manifold_key){id_b, id_a};

			// Lookup in persistent map
			ptrdiff_t index = hmgeti(world->manifold_map, key);
			if (index >= 0) {
				manifold_cache_entry* entry = &world->manifold_map[index];

				// SPATIAL CHECK: Calculate where the current world contact point is relative to
				// Body A. Compare it to where it was last frame.
				tics_vec3 current_local_a = world_to_local(t_a, c->result.point_a);
				float dist_sq = vec3_length_sq(vec3_sub(current_local_a, entry->local_point_a));

				// Threshold: 5cm squared (0.0025).
				// If the contact jumped to a different location, discard it.
				if (dist_sq < 0.0025f) {
					c->accumulated_impulse = entry->accumulated_impulse;

					// Apply warm start impulse
					tics_vec3 impulse_vec = vec3_mul_f(c->result.normal, c->accumulated_impulse);
					// Newton's 3rd law: equal and opposite force
					if (rb_a) rigid_body_apply_impulse(rb_a, impulse_vec, c->result.point_a);
					if (rb_b) {
						rigid_body_apply_impulse(rb_b, vec3_negate(impulse_vec), c->result.point_b);
					}
				}
			}
		}

		// Iterative Solver: More iterations yield more stable resting contacts (impulses propagate
		// through stacks)
		for (size_t i = 0; i < SOLVER_ITERATIONS; i++) {
			resolve_velocities(world, collisions);
		}

		// Build the map for the next frame
		manifold_cache_entry* new_map = NULL;

		for (size_t i = 0; i < col_count; ++i) {
			collision* c = &collisions[i];

			rigid_body_data* rb_a = (c->body_a_ref.type == RIGID_BODY)
										? &world->rigid_bodies[c->body_a_ref.index]
										: NULL;
			static_body_data* sb_a = (c->body_a_ref.type == STATIC_BODY)
										 ? &world->static_bodies[c->body_a_ref.index]
										 : NULL;
			rigid_body_data* rb_b = (c->body_b_ref.type == RIGID_BODY)
										? &world->rigid_bodies[c->body_b_ref.index]
										: NULL;
			static_body_data* sb_b = (c->body_b_ref.type == STATIC_BODY)
										 ? &world->static_bodies[c->body_b_ref.index]
										 : NULL;

			tics_body_id id_a = rb_a ? rb_a->id : sb_a->id;
			tics_body_id id_b = rb_b ? rb_b->id : sb_b->id;
			tics_transform t_a = rb_a ? rb_a->transform : sb_a->transform;

			manifold_cache_entry entry;
			entry.key = (id_a < id_b) ? (manifold_key){id_a, id_b} : (manifold_key){id_b, id_a};
			entry.accumulated_impulse = c->accumulated_impulse;

			entry.local_point_a = world_to_local(t_a, c->result.point_a);

			hmputs(new_map, entry);
		}

		// Swap maps
		hmfree(world->manifold_map);
		world->manifold_map = new_map;

		if (USE_POSITION_SOLVER) resolve_penetrations(world, collisions);
	}

	arrfree(collisions);

	// Apply velocities to transform
	PROFILE("Apply Velocities") {
		apply_velocities(world, delta);
	}

	static int steps = 0;
	steps++;
	if (steps % 10 == 0) profile_print();

	BLICK_REFRESH();
	BLICK_CLEAR(0b11110);
}
