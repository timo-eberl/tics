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

#ifndef GRID_CELL_SIZE
#define GRID_CELL_SIZE 10.0f
#endif

void tics_world_step(tics_world* world, float delta) {
	assert(world);

	PROFILE("Apply Forces") {
		apply_gravity_and_air_friction(world, delta);
	}

	for (size_t i = 0; i < arrlen(world->rigid_bodies); ++i) {
		rigid_body_data rb = world->rigid_bodies[i];
		// shapes
		if (rb.mass == 0.0f) { BLICK_DRAW_SHAPE(1, rb.shape, rb.transform, 0x22DDFFFF, false); }
		else {
			float color_height = 25.0f; // change color based on object position
			float R = color_height, t = fmaxf(0, fminf(1, (rb.transform.position.y + R) / (2 * R)));
			BLICK_DRAW_SHAPE(1, rb.shape, rb.transform,
							 0xFF000000 | (int)(255 * (1 - t)) << 16 | (int)(255 * t) << 8, 0);
		}
		// velocities
		tics_vec3 to = vec3_add(rb.transform.position, vec3_mul_f(rb.linear_velocity, 0.2f));
		BLICK_ARROW(2, rb.transform.position, to, 0x22FF44FF);
	}

	broad_phase_pair* potential_collision_pairs = NULL;
	collision* collisions = NULL;

	PROFILE("Collision Detection") {
		PROFILE("Proxy Collection Typed") {
			update_typed_proxies(world, GRID_CELL_SIZE);
		}
		PROFILE("Proxy Collection Packed") {
			update_packed_proxies(world, GRID_CELL_SIZE);
		}
		PROFILE("Broad Phase") {
			potential_collision_pairs = broad_phase_sap(
				world->typed_proxies, arrlen(world->typed_proxies), world->typed_proxy_map);
		}

#ifdef TICS_HAS_GPU_BROAD_PHASE
		broad_phase_pair* gpu_result = NULL;

		gpu_grid_config config;
		config.res_x = GRID_RES_X;
		config.res_y = GRID_RES_Y;
		config.res_z = GRID_RES_Z;
		config.origin_x = -50.0f;
		config.origin_y = -50.0f;
		config.origin_z = -50.0f;
		config.cell_size = GRID_CELL_SIZE;

		// Verify that no small object has escaped the grid boundaries
		float g_max_x = config.origin_x + config.res_x * config.cell_size;
		float g_max_y = config.origin_y + config.res_y * config.cell_size;
		float g_max_z = config.origin_z + config.res_z * config.cell_size;

		for (size_t i = 0; i < arrlen(world->typed_proxies); ++i) {
			aabb box = world->typed_proxies[i].aabb;
			if (box.min.x < config.origin_x || box.max.x > g_max_x ||
				box.min.y < config.origin_y || box.max.y > g_max_y ||
				box.min.z < config.origin_z || box.max.z > g_max_z) {
				const char* msg = "[Error] Small object out of grid bounds!\n";
				fprintf(stdout, "%s", msg);
				fprintf(stderr, "%s", msg);
				assert(false && "Small object is out of grid bounds");
			}
		}

#ifdef GPU_STRATEGY_A
		PROFILE("Broad Phase GPU Grid A") {
			gpu_result = gpu_broad_phase_run_grid_a(
				world->gpu_state, config, world->packed_rigid_proxies,
				arrlen(world->packed_rigid_proxies), world->packed_static_proxies,
				arrlen(world->packed_static_proxies), world->static_bodies_dirty);
		}
#endif

#ifdef GPU_STRATEGY_B_HALF_SHELL
		PROFILE("Broad Phase GPU Grid B Half Shell") {
			gpu_result = gpu_broad_phase_run_grid_b_half_shell(
				world->gpu_state, config, world->packed_rigid_proxies,
				arrlen(world->packed_rigid_proxies), world->packed_static_proxies,
				arrlen(world->packed_static_proxies), world->static_bodies_dirty);
		}
#endif

#ifdef GPU_STRATEGY_B_NAIVE
		PROFILE("Broad Phase GPU Grid B Naive") {
			gpu_result = gpu_broad_phase_run_grid_b_naive(
				world->gpu_state, config, world->packed_rigid_proxies,
				arrlen(world->packed_rigid_proxies), world->packed_static_proxies,
				arrlen(world->packed_static_proxies), world->static_bodies_dirty);
		}
#endif

		if (gpu_result) {
			// Translate local subset indices back to actual global physics indices.
			// The GPU only processes small objects and knows nothing about the world's global arrays.
			size_t gpu_pair_count = arrlen(gpu_result);
			for (size_t i = 0; i < gpu_pair_count; ++i) {
				gpu_result[i].a.index = world->packed_rigid_map[gpu_result[i].a.index];
				if (gpu_result[i].b.type == RIGID_BODY) {
					gpu_result[i].b.index = world->packed_rigid_map[gpu_result[i].b.index];
				} else {
					gpu_result[i].b.index = world->packed_static_map[gpu_result[i].b.index];
				}
			}
		}
#endif

		// Perform brute-force intersections for bodies excluded from standard broad phases.
		// We check large vs large (using n*(n-1)/2 iterations) and large vs small.
		// Static vs static checks are skipped to match culling rules.
		broad_phase_pair* large_pairs = NULL;
		size_t large_count = arrlen(world->large_bodies);
		size_t small_count = arrlen(world->typed_proxies);

		for (size_t i = 0; i < large_count; ++i) {
			body_ref ref_a = world->large_bodies[i];
			bool a_is_static = (ref_a.type == STATIC_BODY);
			aabb box_a = a_is_static ? world->static_bodies[ref_a.index].aabb
				: calculate_aabb(&world->rigid_bodies[ref_a.index].shape,
								 world->rigid_bodies[ref_a.index].transform);

			// Large vs Large intersections
			for (size_t j = i + 1; j < large_count; ++j) {
				body_ref ref_b = world->large_bodies[j];
				if (a_is_static && ref_b.type == STATIC_BODY) {
					continue;
				}

				aabb box_b = (ref_b.type == STATIC_BODY)
					? world->static_bodies[ref_b.index].aabb
					: calculate_aabb(&world->rigid_bodies[ref_b.index].shape,
									 world->rigid_bodies[ref_b.index].transform);

				bool intersect = !((box_a.max.x < box_b.min.x) || (box_a.min.x > box_b.max.x) ||
								   (box_a.max.y < box_b.min.y) || (box_a.min.y > box_b.max.y) ||
								   (box_a.max.z < box_b.min.z) || (box_a.min.z > box_b.max.z));
				if (intersect) {
					broad_phase_pair p = {ref_a, ref_b};
					arrput(large_pairs, p);
				}
			}

			// Large vs Small intersections
			for (size_t j = 0; j < small_count; ++j) {
				broad_phase_proxy_typed* proxy_b = &world->typed_proxies[j];
				if (a_is_static && proxy_b->type == STATIC_BODY) {
					continue;
				}

				aabb box_b = proxy_b->aabb;
				bool intersect = !((box_a.max.x < box_b.min.x) || (box_a.min.x > box_b.max.x) ||
								   (box_a.max.y < box_b.min.y) || (box_a.min.y > box_b.max.y) ||
								   (box_a.max.z < box_b.min.z) || (box_a.min.z > box_b.max.z));
				if (intersect) {
					body_ref ref_b = {proxy_b->type, proxy_b->index};
					broad_phase_pair p = {ref_a, ref_b};
					arrput(large_pairs, p);
				}
			}
		}

		// Append calculated fallback results uniformly to both outputs to ensure identical lists
		size_t large_pair_count = arrlen(large_pairs);
		for (size_t i = 0; i < large_pair_count; ++i) {
			arrput(potential_collision_pairs, large_pairs[i]);
		}

#ifdef TICS_HAS_GPU_BROAD_PHASE
		for (size_t i = 0; i < large_pair_count; ++i) {
			arrput(gpu_result, large_pairs[i]);
		}

		{
			// Verify correctness
			int gpu_len = arrlen(gpu_result);
			int reflen = arrlen(potential_collision_pairs);
			if (gpu_len != reflen) { printf("[Error] GPU broad phase incorrect result\n"); }
			assert(gpu_len == reflen);
		}

		// V1: delete gpu result, use cpu result
		// arrfree(gpu_result);

		// V2: delete cpu result, use gpu result
		arrfree(potential_collision_pairs);
		potential_collision_pairs = gpu_result;
#endif

		arrfree(large_pairs);

		PROFILE("Narrow Phase") {
			collisions = narrow_phase(potential_collision_pairs, arrlen(potential_collision_pairs),
									  world->rigid_bodies, world->static_bodies);
		}
	}

	// draw broad phase debug info
	BLICK_CLEAR(0b110000);
	// Draw small objects (included) in green
	for (size_t i = 0; i < arrlen(world->typed_proxies); ++i) {
		aabb box = world->typed_proxies[i].aabb;
		BLICK_AABB(5, box.min, box.max, 0xFF00FF00);
	}
	// Draw large objects (excluded) in red
	for (size_t i = 0; i < arrlen(world->large_bodies); ++i) {
		body_ref ref = world->large_bodies[i];
		aabb box = (ref.type == STATIC_BODY)
			? world->static_bodies[ref.index].aabb
			: calculate_aabb(&world->rigid_bodies[ref.index].shape,
							 world->rigid_bodies[ref.index].transform);
		BLICK_AABB(5, box.min, box.max, 0xFF0000FF);
	}
	// draw grid
#ifdef TICS_HAS_GPU_BROAD_PHASE
	float origin_x = -50.0f;
	float origin_y = -50.0f;
	float origin_z = -50.0f;
	float w = GRID_RES_X * GRID_CELL_SIZE;
	float h = GRID_RES_Y * GRID_CELL_SIZE;
	float d = GRID_RES_Z * GRID_CELL_SIZE;
	uint32_t grid_color = 0xFF888888;
	for (int i = 0; i <= GRID_RES_X; ++i) {
		float x = origin_x + i * GRID_CELL_SIZE;
		for (int j = 0; j <= GRID_RES_Y; ++j) {
			float y = origin_y + j * GRID_CELL_SIZE;
			BLICK_LINE(4, ((tics_vec3){x, y, origin_z}),
					   ((tics_vec3){x, y, origin_z + d}), grid_color);
		}
		for (int k = 0; k <= GRID_RES_Z; ++k) {
			float z = origin_z + k * GRID_CELL_SIZE;
			BLICK_LINE(4, ((tics_vec3){x, origin_y, z}),
					   ((tics_vec3){x, origin_y + h, z}), grid_color);
		}
	}
	for (int j = 0; j <= GRID_RES_Y; ++j) {
		float y = origin_y + j * GRID_CELL_SIZE;
		for (int k = 0; k <= GRID_RES_Z; ++k) {
			float z = origin_z + k * GRID_CELL_SIZE;
			BLICK_LINE(4, ((tics_vec3){origin_x, y, z}),
					   ((tics_vec3){origin_x + w, y, z}), grid_color);
		}
	}
#endif

	// collisions
	// for (size_t i = 0; i < arrlen(collisions); ++i) {
	// 	collision_result result = collisions[i].result;
	// 	// draw a red arrow between collision points (might be very small)
	// 	BLICK_ARROW(3, result.point_a, result.point_b, 0xFF0000FF);
	// 	// draw two yellow lines with a fixed length extending in both directions of the arrow
	// 	tics_vec3 target_a = vec3_add(result.point_a, vec3_mul_f(result.normal, -0.2f));
	// 	tics_vec3 target_b = vec3_add(result.point_b, vec3_mul_f(result.normal, 0.2f));
	// 	BLICK_LINE(3, result.point_a, target_a, 0xFF00FFFF);
	// 	BLICK_LINE(3, result.point_b, target_b, 0xFF00FFFF);
	// }
	BLICK_REFRESH();

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

	// Reset dirty flags after all step logic is done
	world->rigid_bodies_dirty = false;
	world->static_bodies_dirty = false;

	static int steps = 0;
	steps++;
	if (steps % 10 == 0) profile_print();

	BLICK_REFRESH();
	BLICK_CLEAR(0b11110);
}
