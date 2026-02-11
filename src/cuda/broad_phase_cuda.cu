#include "broad_phase_cuda.h"

#include <stdlib.h>

struct cuda_broad_phase_state {
	// Device-side buffers
	cuda_aabb* d_rigids;
	cuda_aabb* d_statics;
	size_t d_rigids_capacity; // allocated count (not bytes)
	size_t d_statics_capacity;
};

extern "C" cuda_broad_phase_state* cuda_broad_phase_create(void) {
	cuda_broad_phase_state* state =
		(cuda_broad_phase_state*)calloc(1, sizeof(cuda_broad_phase_state));
	return state;
}

extern "C" void cuda_broad_phase_destroy(cuda_broad_phase_state* state) {
	if (!state) return;
	if (state->d_rigids) cudaFree(state->d_rigids);
	if (state->d_statics) cudaFree(state->d_statics);
	free(state);
}

extern "C" cuda_broad_phase_pair* cuda_broad_phase_run(cuda_broad_phase_state* state,
													   const cuda_aabb* rigids, size_t rigid_count,
													   const cuda_aabb* statics,
													   size_t static_count, bool statics_changed,
													   size_t* out_count) {
	(void)state;
	(void)rigids;
	(void)rigid_count;
	(void)statics;
	(void)static_count;
	(void)statics_changed;
	*out_count = 0;
	return NULL;
}
