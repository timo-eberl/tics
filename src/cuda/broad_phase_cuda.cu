#include "broad_phase_cuda.h"

#include <stdio.h>
#include <stdlib.h>

struct cuda_broad_phase_state {
	cuda_aabb* d_rigids;
	cuda_aabb* d_statics;
	size_t d_rigids_capacity;
	size_t d_statics_capacity;

	cuda_broad_phase_pair* d_pairs;
	size_t d_pairs_capacity;
	unsigned int* d_pair_count;

	cudaEvent_t ev_start, ev_after_upload, ev_after_kernel, ev_after_readback;
};

#define CUDA_CHECK(call)                                                                           \
	do {                                                                                           \
		cudaError_t err_ = (call);                                                                 \
		if (err_ != cudaSuccess)                                                                   \
			fprintf(stderr, "[cuda] %s:%d %s\n", __FILE__, __LINE__, cudaGetErrorString(err_));    \
	} while (0)

static void ensure_device_buffer(void** d_buf, size_t* capacity, size_t needed, size_t elem_size) {
	if (*capacity >= needed) return;
	if (*d_buf) cudaFree(*d_buf);
	*d_buf = NULL;
	*capacity = 0;
	CUDA_CHECK(cudaMalloc(d_buf, needed * elem_size));
	if (*d_buf) *capacity = needed;
}

__device__ static bool aabb_overlap(const cuda_aabb* a, const cuda_aabb* b) {
	return a->max_x >= b->min_x && a->min_x <= b->max_x && a->max_y >= b->min_y &&
		   a->min_y <= b->max_y && a->max_z >= b->min_z && a->min_z <= b->max_z;
}

__global__ void brute_force_kernel(const cuda_aabb* rigids, size_t rigid_count,
								   const cuda_aabb* statics, size_t static_count,
								   cuda_broad_phase_pair* pairs, unsigned int* pair_count,
								   unsigned int max_pairs) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= rigid_count) return;

	cuda_aabb ri = rigids[i];

	for (size_t j = i + 1; j < rigid_count; ++j) {
		if (aabb_overlap(&ri, &rigids[j])) {
			unsigned int idx = atomicAdd(pair_count, 1);
			if (idx < max_pairs) pairs[idx] = {i, (uint32_t)j, 1};
		}
	}

	for (size_t j = 0; j < static_count; ++j) {
		if (aabb_overlap(&ri, &statics[j])) {
			unsigned int idx = atomicAdd(pair_count, 1);
			if (idx < max_pairs) pairs[idx] = {i, (uint32_t)j, 0};
		}
	}
}

extern "C" cuda_broad_phase_state* cuda_broad_phase_state_create(void) {
	cuda_broad_phase_state* s = (cuda_broad_phase_state*)calloc(1, sizeof(cuda_broad_phase_state));
	cudaMalloc(&s->d_pair_count, sizeof(unsigned int));
	cudaEventCreate(&s->ev_start);
	cudaEventCreate(&s->ev_after_upload);
	cudaEventCreate(&s->ev_after_kernel);
	cudaEventCreate(&s->ev_after_readback);
	return s;
}

extern "C" void cuda_broad_phase_state_destroy(cuda_broad_phase_state* s) {
	if (!s) return;
	if (s->d_rigids) cudaFree(s->d_rigids);
	if (s->d_statics) cudaFree(s->d_statics);
	if (s->d_pairs) cudaFree(s->d_pairs);
	if (s->d_pair_count) cudaFree(s->d_pair_count);
	cudaEventDestroy(s->ev_start);
	cudaEventDestroy(s->ev_after_upload);
	cudaEventDestroy(s->ev_after_kernel);
	cudaEventDestroy(s->ev_after_readback);
	free(s);
}

extern "C" cuda_broad_phase_pair* cuda_broad_phase_naive(cuda_broad_phase_state* s,
													   const cuda_aabb* rigids, size_t rigid_count,
													   const cuda_aabb* statics,
													   size_t static_count, bool statics_changed,
													   size_t* out_count) {
	*out_count = 0;
	if (rigid_count == 0) return NULL;

	cudaEventRecord(s->ev_start);

	// Upload AABBs
	ensure_device_buffer((void**)&s->d_rigids, &s->d_rigids_capacity, rigid_count,
						 sizeof(cuda_aabb));
	CUDA_CHECK(
		cudaMemcpy(s->d_rigids, rigids, rigid_count * sizeof(cuda_aabb), cudaMemcpyHostToDevice));

	if (statics_changed) {
		ensure_device_buffer((void**)&s->d_statics, &s->d_statics_capacity, static_count,
							 sizeof(cuda_aabb));
		if (static_count > 0 && s->d_statics)
			CUDA_CHECK(cudaMemcpy(s->d_statics, statics, static_count * sizeof(cuda_aabb),
								  cudaMemcpyHostToDevice));
	}

	cudaEventRecord(s->ev_after_upload);

	// Heuristic output size: We probably won't have more than 8 potential collisions per body.
	// If we actually have more than that, the first run fails and will output the actual required
	// size (count). We then run a second time with the actually required size.
	size_t pairs_needed = s->d_pairs_capacity;
	if (pairs_needed < 8 * (rigid_count + static_count))
		pairs_needed = 8 * (rigid_count + static_count);
	if (pairs_needed < 1024) pairs_needed = 1024;

	const int block_size = 256;
	int grid_size = ((int)rigid_count + block_size - 1) / block_size;
	unsigned int count = 0;

	for (int attempt = 0; attempt < 2; ++attempt) {
		ensure_device_buffer((void**)&s->d_pairs, &s->d_pairs_capacity, pairs_needed,
							 sizeof(cuda_broad_phase_pair));

		unsigned int kernel_max = (s->d_pairs_capacity > (size_t)UINT32_MAX)
									  ? UINT32_MAX
									  : (unsigned int)s->d_pairs_capacity;

		CUDA_CHECK(cudaMemset(s->d_pair_count, 0, sizeof(unsigned int)));
		brute_force_kernel<<<grid_size, block_size>>>(s->d_rigids, rigid_count, s->d_statics,
													  static_count, s->d_pairs, s->d_pair_count,
													  kernel_max);
		CUDA_CHECK(cudaDeviceSynchronize());
		CUDA_CHECK(
			cudaMemcpy(&count, s->d_pair_count, sizeof(unsigned int), cudaMemcpyDeviceToHost));

		if (count <= kernel_max) break; // Everything fit.
		pairs_needed = (size_t)count;	// Rerun with the actual count
	}

	cudaEventRecord(s->ev_after_kernel);

	cuda_broad_phase_pair* h_pairs = NULL;
	if (count > 0) {
		h_pairs = (cuda_broad_phase_pair*)malloc(count * sizeof(cuda_broad_phase_pair));
		CUDA_CHECK(cudaMemcpy(h_pairs, s->d_pairs, count * sizeof(cuda_broad_phase_pair),
							  cudaMemcpyDeviceToHost));
		*out_count = count;
	}

	cudaEventRecord(s->ev_after_readback);
	cudaEventSynchronize(s->ev_after_readback);

	float t_upload, t_kernel, t_readback;
	cudaEventElapsedTime(&t_upload, s->ev_start, s->ev_after_upload);
	cudaEventElapsedTime(&t_kernel, s->ev_after_upload, s->ev_after_kernel);
	cudaEventElapsedTime(&t_readback, s->ev_after_kernel, s->ev_after_readback);

	static float t_upload_acc = 0, t_kernel_acc = 0, t_readback_acc = 0; // accumulators
	static int steps = 0;
	t_upload_acc += t_upload;
	t_kernel_acc += t_kernel;
	t_readback_acc += t_readback;
	if (++steps % 10 == 0)
		fprintf(stderr,
				"[cuda] (avg over %d) upload=%.3fms kernel=%.3fms readback=%.3fms total=%.3fms\n",
				steps, t_upload_acc / steps, t_kernel_acc / steps, t_readback_acc / steps,
				(t_upload_acc + t_kernel_acc + t_readback_acc) / steps);

	return h_pairs;
}
