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
		BLICK_DRAW_SHAPE(1, rb.shape, rb.transform, 0xFFDDFFDD, false);
		BLICK_DRAW_SHAPE(1, rb.shape, rb.transform, 0xFF99AA44, true);
		tics_vec3 to = vec3_add(rb.transform.position, vec3_mul_f(rb.linear_velocity, 0.2f));
		// BLICK_ARROW(2, rb.transform.position, to, 0xFFFF44FF);
		// BLICK_TEXT_INT(2, rb.transform.position, rb.id, 0xFFFFFFFF);
		BLICK_TRANSFORM(1, rb.transform, 0.4);
		// add transform trail to the first object
		// if (i == 0) {
		// 	BLICK_TRANSFORM(5, rb.transform, 0.1);
		// 	BLICK_TRIM_LAYER(5, 200);
		// }
	}

	broad_phase_proxy* proxies_r = NULL;
	broad_phase_proxy* proxies_s = NULL;
	broad_phase_pair* potential_collision_pairs = NULL;
	collision* collisions = NULL;

	PROFILE("Collision Detection") {
		PROFILE("Proxy Collection") {
			proxies_r = build_rigid_proxies(world);
			proxies_s = build_static_proxies(world);
		}
		PROFILE("Broad Phase") {
			potential_collision_pairs =
				collision_broad_phase(proxies_r, arrlen(proxies_r), proxies_s, arrlen(proxies_s));
		}
		PROFILE("Narrow Phase") {
			collisions =
				collision_narrow_phase(potential_collision_pairs, arrlen(potential_collision_pairs),
									   world->rigid_bodies, world->static_bodies);
		}
	}

	BLICK_CLEAR(0b10000);
	// for (size_t i = 0; i < arrlen(proxies_r); ++i) {
	// 	broad_phase_proxy* p = &proxies_r[i];
	// 	BLICK_AABB(3, p->aabb.min, p->aabb.max, 0xFFFF0000);
	// }
	// for (size_t i = 0; i < arrlen(proxies_s); ++i) {
	// 	broad_phase_proxy* p = &proxies_s[i];
	// 	BLICK_AABB(3, p->aabb.min, p->aabb.max, 0xFF000000);
	// }
	// for (size_t i = 0; i < arrlen(potential_collision_pairs); ++i) {
	// 	broad_phase_pair* p = &potential_collision_pairs[i];
	// 	aabb a_box =
	// 		(p->a.type == RIGID_BODY) ? proxies_r[p->a.index].aabb : proxies_s[p->a.index].aabb;
	// 	tics_vec3 a_pos = (p->a.type == RIGID_BODY)
	// 						  ? world->rigid_bodies[p->a.index].transform.position
	// 						  : world->static_bodies[p->a.index].transform.position;
	// 	aabb b_box =
	// 		(p->b.type == RIGID_BODY) ? proxies_r[p->b.index].aabb : proxies_s[p->b.index].aabb;
	// 	tics_vec3 b_pos = (p->b.type == RIGID_BODY)
	// 						  ? world->rigid_bodies[p->b.index].transform.position
	// 						  : world->static_bodies[p->b.index].transform.position;
	// 	BLICK_AABB(5, a_box.min, a_box.max, 0xFF00FF00);
	// 	BLICK_AABB(5, b_box.min, b_box.max, 0xFF00FF00);
	// 	BLICK_LINE(4, a_pos, b_pos, 0xFFFF0000);
	// 	BLICK_REFRESH();
	// 	BLICK_CLEAR(0b100000);
	// }
	for (size_t i = 0; i < arrlen(collisions); ++i) {
		collision* c = &collisions[i];
		BLICK_POINT(1, c->result.point_a, 0.2f, 0xFF0000FF);
		BLICK_POINT(1, c->result.point_b, 0.2f, 0xFF00FFFF);
		BLICK_ARROW(1, c->result.point_a, c->result.point_b, 0xFFFF0000);
	}

	BLICK_REFRESH();

	arrfree(proxies_r);
	arrfree(proxies_s);
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

		// Iterative Solver: More interations yield more stable resting contacts (impulses propagate
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
	BLICK_CLEAR(0b1110);
}
