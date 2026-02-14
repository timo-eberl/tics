#ifndef CUDA_COMMON_H
#define CUDA_COMMON_H

#include <cuda_runtime.h>

#include <stdio.h>

struct cuda_shared_state {
	// Input
	cuda_aabb* d_rigids;
	size_t d_rigids_size;
	cuda_aabb* d_statics;
	size_t d_statics_size;
	// Output
	cuda_pair* d_pairs;
	size_t d_pairs_size;
	unsigned int* d_pair_count;
	// Host Memory Registration Tracking
	const void* h_last_rigids_ptr;
	size_t h_last_rigids_size;
	const void* h_last_statics_ptr;
	size_t h_last_statics_size;
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

static void ensure_host_memory_registered(const void* current_ptr, size_t current_size,
										  const void** tracked_ptr, size_t* tracked_size) {
	if (!current_ptr || current_size == 0) return;

	// Check if pointer changed or size grew
	if (*tracked_ptr != current_ptr || current_size > *tracked_size) {

		// Unregister old if it exists
		if (*tracked_ptr) {
			cudaHostUnregister((void*)*tracked_ptr);
			*tracked_ptr = NULL;
			*tracked_size = 0;
		}

		// Register new
		cudaError_t err =
			cudaHostRegister((void*)current_ptr, current_size, cudaHostRegisterDefault);

		if (err == cudaSuccess) {
			*tracked_ptr = current_ptr;
			*tracked_size = current_size;
		}
		else {
			// Fallback: Proceed without pinning, but log warning once
			// (In production you might want to suppress this after one failure)
			fprintf(stderr, "[Warning] Failed to register host memory at %p: %s\n", current_ptr,
					cudaGetErrorString(err));
			*tracked_ptr = NULL; // Ensure we don't try to unregister invalid ptr later
			*tracked_size = 0;
		}
	}
}

#endif
