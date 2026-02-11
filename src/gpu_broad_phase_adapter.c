#ifdef TICS_HAS_GPU_BROAD_PHASE

#include "tics_internal.h"

#include "broad_phase_cuda.h"

#include <stb_ds.h>

#include <assert.h>
#include <stddef.h>

// ================================================================================================
// GPU Broad Phase Adapter
// ================================================================================================
// This file is the only place that knows about CUDA. It translates between tics' GPU-agnostic API
// (gpu_*) and the CUDA-specific implementation (cuda_*).

void* gpu_broad_phase_create(void) {
	return cuda_broad_phase_create();
}

void gpu_broad_phase_destroy(void* state) {
	cuda_broad_phase_destroy((cuda_broad_phase_state*)state);
}

broad_phase_pair* gpu_broad_phase_run(tics_world* world) {
	// Compile-time layout assertions — ensures zero-cost cast is valid
	_Static_assert(sizeof(packed_aabb) == sizeof(cuda_aabb),
				   "packed_aabb and cuda_aabb must have identical size");
	_Static_assert(offsetof(packed_aabb, min_x) == offsetof(cuda_aabb, min_x),
				   "packed_aabb and cuda_aabb layout mismatch");
	_Static_assert(offsetof(packed_aabb, max_z) == offsetof(cuda_aabb, max_z),
				   "packed_aabb and cuda_aabb layout mismatch");

	size_t rigid_count = arrlen(world->gpu_rigid_aabbs);
	size_t static_count = arrlen(world->gpu_static_aabbs);

	size_t count = 0;
	cuda_broad_phase_pair* cu_pairs = cuda_broad_phase_run(
		(cuda_broad_phase_state*)world->gpu_state, (const cuda_aabb*)world->gpu_rigid_aabbs,
		rigid_count, (const cuda_aabb*)world->gpu_static_aabbs, static_count,
		world->gpu_statics_dirty, &count);

	// Flag consumed — GPU side has the data now
	world->gpu_statics_dirty = false;

	// Convert to stb_ds array
	broad_phase_pair* pairs = NULL;
	if (count > 0) {
		arrsetlen(pairs, count);
		for (size_t i = 0; i < count; ++i) {
			pairs[i].a.type = RIGID_BODY;
			pairs[i].a.index = cu_pairs[i].a_index;
			pairs[i].b.type = cu_pairs[i].b_type;
			pairs[i].b.index = cu_pairs[i].b_index;
		}
		free(cu_pairs);
	}

	return pairs;
}

#endif
