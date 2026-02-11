#include "broad_phase_cuda.h"

#include <stdlib.h>
#include <stdio.h>

struct cuda_broad_phase_state {
	// Device-side buffers
	cuda_aabb* d_rigids;
	cuda_aabb* d_statics;
	size_t d_rigids_capacity; // allocated count (not bytes)
	size_t d_statics_capacity;

	// Output buffers
	cuda_broad_phase_pair* d_pairs;
	size_t d_pairs_capacity;
	unsigned int* d_pair_count; // atomic counter on device
};

// ---------- helpers ----------

// Grow a device buffer if the current capacity is too small.
// Returns the (possibly new) device pointer. Updates *capacity.
static void ensure_device_buffer(void** d_buf, size_t* capacity, size_t needed, size_t elem_size) {
	if (*capacity >= needed) return;
	if (*d_buf) cudaFree(*d_buf);
	*capacity = needed;
	cudaMalloc(d_buf, needed * elem_size);
}

// ---------- AABB overlap test ----------

__device__ static bool aabb_overlap(const cuda_aabb* a, const cuda_aabb* b) {
	return a->max_x >= b->min_x && a->min_x <= b->max_x &&
	       a->max_y >= b->min_y && a->min_y <= b->max_y &&
	       a->max_z >= b->min_z && a->min_z <= b->max_z;
}

// ---------- brute-force kernel ----------
// One thread per rigid body. Each thread tests against:
//   - all rigid bodies with index > its own (rigid vs rigid, avoids duplicates)
//   - all static bodies (rigid vs static)

__global__ void brute_force_kernel(const cuda_aabb* rigids, size_t rigid_count,
                                   const cuda_aabb* statics, size_t static_count,
                                   cuda_broad_phase_pair* pairs, unsigned int* pair_count,
                                   unsigned int max_pairs) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= rigid_count) return;

	cuda_aabb ri = rigids[i];

	// Rigid vs Rigid (j > i to avoid duplicate pairs)
	for (size_t j = i + 1; j < rigid_count; ++j) {
		if (aabb_overlap(&ri, &rigids[j])) {
			unsigned int idx = atomicAdd(pair_count, 1);
			if (idx < max_pairs) {
				pairs[idx].a_index = i;
				pairs[idx].b_index = (uint32_t)j;
				pairs[idx].b_type = 1; // RIGID_BODY
			}
		}
	}

	// Rigid vs Static
	for (size_t j = 0; j < static_count; ++j) {
		if (aabb_overlap(&ri, &statics[j])) {
			unsigned int idx = atomicAdd(pair_count, 1);
			if (idx < max_pairs) {
				pairs[idx].a_index = i;
				pairs[idx].b_index = (uint32_t)j;
				pairs[idx].b_type = 0; // STATIC_BODY
			}
		}
	}
}

// ---------- public API ----------

extern "C" cuda_broad_phase_state* cuda_broad_phase_create(void) {
	cuda_broad_phase_state* state =
		(cuda_broad_phase_state*)calloc(1, sizeof(cuda_broad_phase_state));
	// Allocate the atomic counter once (single unsigned int on the device)
	cudaMalloc(&state->d_pair_count, sizeof(unsigned int));
	return state;
}

extern "C" void cuda_broad_phase_destroy(cuda_broad_phase_state* state) {
	if (!state) return;
	if (state->d_rigids) cudaFree(state->d_rigids);
	if (state->d_statics) cudaFree(state->d_statics);
	if (state->d_pairs) cudaFree(state->d_pairs);
	if (state->d_pair_count) cudaFree(state->d_pair_count);
	free(state);
}

extern "C" cuda_broad_phase_pair* cuda_broad_phase_run(cuda_broad_phase_state* state,
                                                       const cuda_aabb* rigids, size_t rigid_count,
                                                       const cuda_aabb* statics,
                                                       size_t static_count, bool statics_changed,
                                                       size_t* out_count) {
	*out_count = 0;
	if (rigid_count == 0) return NULL;

	// --- Upload AABBs ---

	ensure_device_buffer((void**)&state->d_rigids, &state->d_rigids_capacity,
	                     rigid_count, sizeof(cuda_aabb));
	cudaMemcpy(state->d_rigids, rigids, rigid_count * sizeof(cuda_aabb), cudaMemcpyHostToDevice);

	if (statics_changed) {
		ensure_device_buffer((void**)&state->d_statics, &state->d_statics_capacity,
		                     static_count, sizeof(cuda_aabb));
		if (static_count > 0) {
			cudaMemcpy(state->d_statics, statics, static_count * sizeof(cuda_aabb),
			           cudaMemcpyHostToDevice);
		}
	}

	// --- Prepare output buffer ---
	// Worst case: every rigid pairs with every other rigid + every static
	// rigid_count*(rigid_count-1)/2 + rigid_count*static_count
	size_t max_pairs = rigid_count * (rigid_count - 1) / 2 + rigid_count * static_count;
	if (max_pairs == 0) return NULL;

	ensure_device_buffer((void**)&state->d_pairs, &state->d_pairs_capacity,
	                     max_pairs, sizeof(cuda_broad_phase_pair));

	// Reset atomic counter
	cudaMemset(state->d_pair_count, 0, sizeof(unsigned int));

	// --- Launch kernel ---
	const int block_size = 256;
	int grid_size = ((int)rigid_count + block_size - 1) / block_size;

	brute_force_kernel<<<grid_size, block_size>>>(
		state->d_rigids, rigid_count,
		state->d_statics, static_count,
		state->d_pairs, state->d_pair_count, (unsigned int)max_pairs);

	// --- Read back results ---
	unsigned int count = 0;
	cudaMemcpy(&count, state->d_pair_count, sizeof(unsigned int), cudaMemcpyDeviceToHost);

	// Clamp to max_pairs (should not happen, but be safe)
	if (count > max_pairs) count = (unsigned int)max_pairs;
	if (count == 0) return NULL;

	cuda_broad_phase_pair* h_pairs =
		(cuda_broad_phase_pair*)malloc(count * sizeof(cuda_broad_phase_pair));
	cudaMemcpy(h_pairs, state->d_pairs, count * sizeof(cuda_broad_phase_pair),
	           cudaMemcpyDeviceToHost);

	*out_count = count;
	return h_pairs;
}
