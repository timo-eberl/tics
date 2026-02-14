#include "broad_phase_cuda.h"
#include "cuda_common.h"

#include <cuda_runtime.h>

extern "C" cuda_shared_state* cuda_shared_state_create(void) {
	cuda_shared_state* s = (cuda_shared_state*)calloc(1, sizeof(cuda_shared_state));
	CUDA_CHECK(cudaMalloc(&s->d_pair_count, sizeof(unsigned int)));
	return s;
}

extern "C" void cuda_shared_state_destroy(cuda_shared_state* s) {
	if (!s) return;
	// Clean up input and output buffers
	if (s->d_rigids) cudaFree(s->d_rigids);
	if (s->d_statics) cudaFree(s->d_statics);
	if (s->d_pairs) cudaFree(s->d_pairs);
	if (s->d_pair_count) cudaFree(s->d_pair_count);
	// Clean up host registration
	if (s->h_last_rigids_ptr) cudaHostUnregister((void*)s->h_last_rigids_ptr);
	if (s->h_last_statics_ptr) cudaHostUnregister((void*)s->h_last_statics_ptr);
	free(s);
}
