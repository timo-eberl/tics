#include "tics_internal.h"

#include <broad_phase_dx.h>
#include <stb_ds.h>

#include <assert.h>
#include <stddef.h>

// This file is the only place that knows about DirectX. It translates between tics' GPU-agnostic
// API and the DX implementation.

typedef struct dx_state_all {
	dx_shared_state* shared_state;

	dx_state_brute_force* state_brute_force;
	// dx_state_grid_a* state_grid_a;
	// dx_state_grid_b* state_grid_b;
} dx_state_all;

void* gpu_broad_phase_create(void) {
	dx_state_all* s = (dx_state_all*)calloc(1, sizeof(dx_state_all));
	s->shared_state = dx_shared_state_create();
	s->state_brute_force = dx_state_brute_force_create(s->shared_state);
	// s->state_grid_a = dx_state_grid_a_create();
	// s->state_grid_b = dx_state_grid_b_create();
	return s;
}

void gpu_broad_phase_destroy(void* state) {
	dx_state_all* s = (dx_state_all*)state;
	dx_shared_state_destroy(s->shared_state);
	dx_state_brute_force_destroy(s->state_brute_force);
	// dx_state_grid_a_destroy(s->state_grid_a);
	// dx_state_grid_b_destroy(s->state_grid_b);
	free(s);
}

static broad_phase_pair* convert_pairs(dx_pair* gpu_pairs, size_t count) {
	broad_phase_pair* pairs = NULL;
	if (count > 0) {
		arrsetlen(pairs, count);
		for (size_t i = 0; i < count; ++i) {
			pairs[i].a.type = RIGID_BODY;
			pairs[i].a.index = gpu_pairs[i].a_index;
			pairs[i].b.type = gpu_pairs[i].b_type;
			pairs[i].b.index = gpu_pairs[i].b_index;
		}
		free(gpu_pairs);
	}
	return pairs;
}

// Compile-time assertions: Ensure packed_aabb and dx_aabb have the same layout
_Static_assert(sizeof(packed_aabb) == sizeof(dx_aabb),
			   "packed_aabb and dx_aabb must have identical size");
_Static_assert(offsetof(packed_aabb, min_x) == offsetof(dx_aabb, min_x),
			   "packed_aabb and dx_aabb layout mismatch");
_Static_assert(offsetof(packed_aabb, max_z) == offsetof(dx_aabb, max_z),
			   "packed_aabb and dx_aabb layout mismatch");

broad_phase_pair* gpu_broad_phase_run_grid_a(void* gpu_state, gpu_grid_config config,
											 packed_aabb* packed_rigid_proxies,
											 size_t rigid_count, packed_aabb* packed_static_proxies,
											 size_t static_count, bool statics_changed) {
	dx_grid_config gpu_config;
	gpu_config.res_x = config.res_x;
	gpu_config.res_y = config.res_y;
	gpu_config.res_z = config.res_z;
	gpu_config.origin_x = config.origin_x;
	gpu_config.origin_y = config.origin_y;
	gpu_config.origin_z = config.origin_z;
	gpu_config.cell_size = config.cell_size;

	// size_t count = 0;
	// dx_pair* gpu_pairs = dx_broad_phase_grid_a(
	// 	((dx_state_all*)gpu_state)->shared_state, ((dx_state_all*)gpu_state)->state_grid_a,
	// 	&gpu_config, (const dx_aabb*)packed_rigid_proxies, (int)rigid_count,
	// 	(const dx_aabb*)packed_static_proxies, (int)static_count, statics_changed, &count);
	// return convert_pairs(gpu_pairs, count);
	return NULL;
}

broad_phase_pair* gpu_broad_phase_run_grid_b_half_shell(void* gpu_state, gpu_grid_config config,
														packed_aabb* packed_rigid_proxies,
														size_t rigid_count,
														packed_aabb* packed_static_proxies,
														size_t static_count, bool statics_changed) {
	dx_grid_config gpu_config;
	gpu_config.res_x = config.res_x;
	gpu_config.res_y = config.res_y;
	gpu_config.res_z = config.res_z;
	gpu_config.origin_x = config.origin_x;
	gpu_config.origin_y = config.origin_y;
	gpu_config.origin_z = config.origin_z;
	gpu_config.cell_size = config.cell_size;

	// size_t count = 0;
	// dx_pair* gpu_pairs = dx_broad_phase_grid_b(
	// 	((dx_state_all*)gpu_state)->shared_state, ((dx_state_all*)gpu_state)->state_grid_b,
	// 	&gpu_config, (const dx_aabb*)packed_rigid_proxies, (int)rigid_count,
	// 	(const dx_aabb*)packed_static_proxies, (int)static_count, statics_changed, &count,
	// 	true);
	// return convert_pairs(gpu_pairs, count);
	return NULL;
}

broad_phase_pair* gpu_broad_phase_run_grid_b_naive(void* gpu_state, gpu_grid_config config,
												   packed_aabb* packed_rigid_proxies,
												   size_t rigid_count,
												   packed_aabb* packed_static_proxies,
												   size_t static_count, bool statics_changed) {
	dx_grid_config gpu_config;
	gpu_config.res_x = config.res_x;
	gpu_config.res_y = config.res_y;
	gpu_config.res_z = config.res_z;
	gpu_config.origin_x = config.origin_x;
	gpu_config.origin_y = config.origin_y;
	gpu_config.origin_z = config.origin_z;
	gpu_config.cell_size = config.cell_size;

	// size_t count = 0;
	// dx_pair* gpu_pairs = dx_broad_phase_grid_b(
	// 	((dx_state_all*)gpu_state)->shared_state, ((dx_state_all*)gpu_state)->state_grid_b,
	// 	&gpu_config, (const dx_aabb*)packed_rigid_proxies, (int)rigid_count,
	// 	(const dx_aabb*)packed_static_proxies, (int)static_count, statics_changed, &count,
	// 	false);
	// return convert_pairs(gpu_pairs, count);
	return NULL;
}

broad_phase_pair* gpu_broad_phase_run_brute_force(void* gpu_state,
												  packed_aabb* packed_rigid_proxies,
												  size_t rigid_count,
												  packed_aabb* packed_static_proxies,
												  size_t static_count, bool statics_changed) {
	size_t count = 0;
	dx_pair* gpu_pairs = dx_broad_phase_brute_force(
		((dx_state_all*)gpu_state)->shared_state, ((dx_state_all*)gpu_state)->state_brute_force,
		(const dx_aabb*)packed_rigid_proxies, (int)rigid_count,
		(const dx_aabb*)packed_static_proxies, (int)static_count, statics_changed, &count);
	return convert_pairs(gpu_pairs, count);
}
