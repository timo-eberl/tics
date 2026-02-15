#include "broad_phase_cuda.h"
#include "cuda_common.h"
#include "cuda_profile.h"

struct cuda_state_brute_force {};

extern "C" cuda_state_brute_force* cuda_state_brute_force_create(void) {
	cuda_state_brute_force* s = (cuda_state_brute_force*)calloc(1, sizeof(cuda_state_brute_force));
	return s;
}

extern "C" void cuda_state_brute_force_destroy(cuda_state_brute_force* s) {
	if (!s) return;
	free(s);
}

__device__ static bool aabb_overlap(const cuda_aabb* a, const cuda_aabb* b) {
	return a->max_x >= b->min_x && a->min_x <= b->max_x && a->max_y >= b->min_y &&
		   a->min_y <= b->max_y && a->max_z >= b->min_z && a->min_z <= b->max_z;
}

__global__ void brute_force_kernel(const cuda_aabb* rigids, int rigid_count,
								   const cuda_aabb* statics, int static_count, cuda_pair* pairs,
								   unsigned int* pair_count, unsigned int max_pairs) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= rigid_count) return;

	cuda_aabb ri = rigids[i];

	for (int j = i + 1; j < rigid_count; ++j) {
		if (aabb_overlap(&ri, &rigids[j])) {
			unsigned int idx = atomicAdd(pair_count, 1);
			if (idx < max_pairs) pairs[idx] = {i, (uint32_t)j, 1};
		}
	}

	for (int j = 0; j < static_count; ++j) {
		if (aabb_overlap(&ri, &statics[j])) {
			unsigned int idx = atomicAdd(pair_count, 1);
			if (idx < max_pairs) pairs[idx] = {i, (uint32_t)j, 0};
		}
	}
}

extern "C" cuda_pair* cuda_broad_phase_brute_force(cuda_shared_state* sh, cuda_state_brute_force* s,
												   const cuda_aabb* rigids, int rigid_count,
												   const cuda_aabb* statics, int static_count,
												   bool statics_changed, size_t* out_count) {
	*out_count = 0;
	if (rigid_count == 0) return NULL;

	// Page lock the host memory, so uploading data becomes faster
	ensure_host_memory_registered(rigids, rigid_count * sizeof(cuda_aabb), &sh->h_last_rigids_ptr,
								  &sh->h_last_rigids_size);
	ensure_host_memory_registered(statics, static_count * sizeof(cuda_aabb), &sh->h_last_statics_ptr,
								  &sh->h_last_statics_size);

	cuda_profile prof = {0};
	cuda_profile_begin(&prof);

	// Upload AABBs
	ensure_device_buffer((void**)&sh->d_rigids, &sh->d_rigids_size, rigid_count, sizeof(cuda_aabb));
	CUDA_CHECK(
		cudaMemcpy(sh->d_rigids, rigids, rigid_count * sizeof(cuda_aabb), cudaMemcpyHostToDevice));

	if (statics_changed) {
		ensure_device_buffer((void**)&sh->d_statics, &sh->d_statics_size, static_count,
							 sizeof(cuda_aabb));
		if (static_count > 0 && sh->d_statics)
			CUDA_CHECK(cudaMemcpy(sh->d_statics, statics, static_count * sizeof(cuda_aabb),
								  cudaMemcpyHostToDevice));
	}

	cuda_profile_step(&prof, "upload");

	// Heuristic output size: We probably won't have more than 8 potential collisions per body.
	// If we actually have more than that, the first run fails and will output the actual required
	// size (count). We then run a second time with the actually required size.
	size_t pairs_needed = sh->d_pairs_size;
	if (pairs_needed < 8 * (rigid_count + static_count))
		pairs_needed = 8 * (rigid_count + static_count);
	if (pairs_needed < 1024) pairs_needed = 1024;

	const int block_size = 256;
	int grid_size = (rigid_count + block_size - 1) / block_size;
	unsigned int count = 0;

	for (int attempt = 0; attempt < 2; ++attempt) {
		ensure_device_buffer((void**)&sh->d_pairs, &sh->d_pairs_size, pairs_needed,
							 sizeof(cuda_pair));

		unsigned int kernel_max =
			(sh->d_pairs_size > (size_t)UINT32_MAX) ? UINT32_MAX : (unsigned int)sh->d_pairs_size;

		CUDA_CHECK(cudaMemset(sh->d_pair_count, 0, sizeof(unsigned int)));
		brute_force_kernel<<<grid_size, block_size>>>(sh->d_rigids, rigid_count, sh->d_statics,
													  static_count, sh->d_pairs, sh->d_pair_count,
													  kernel_max);
		CUDA_CHECK(cudaDeviceSynchronize());
		CUDA_CHECK(
			cudaMemcpy(&count, sh->d_pair_count, sizeof(unsigned int), cudaMemcpyDeviceToHost));

		if (count <= kernel_max) break; // Everything fit.
		pairs_needed = (size_t)count;	// Rerun with the actual count
	}

	cuda_profile_step(&prof, "kernel");

	cuda_pair* h_pairs = NULL;
	if (count > 0) {
		h_pairs = (cuda_pair*)malloc(count * sizeof(cuda_pair));
		CUDA_CHECK(
			cudaMemcpy(h_pairs, sh->d_pairs, count * sizeof(cuda_pair), cudaMemcpyDeviceToHost));
		*out_count = count;
	}

	cuda_profile_step(&prof, "readback");

	static cuda_profile_acc prof_acc;
	static bool prof_init = false;
	if (!prof_init) {
		cuda_profile_acc_init(&prof_acc);
		prof_init = true;
	}

	cuda_profile_end(&prof);
	cuda_profile_log(&prof, &prof_acc, "naive", 10);

	return h_pairs;
}
