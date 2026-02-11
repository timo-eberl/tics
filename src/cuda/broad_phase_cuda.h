#ifndef BROAD_PHASE_CUDA_H
#define BROAD_PHASE_CUDA_H

#include <stddef.h>
#include <stdint.h>

// Mirror the plain-C types from tics_internal.h so this header has no
// dependency on tics internals. These are layout-compatible.
typedef struct {
	const float* min_x;
	const float* max_x;
	const float* min_y;
	const float* max_y;
	const float* min_z;
	const float* max_z;
	const uint32_t* indices;
	size_t count;
} cuda_broad_phase_proxies_soa;

typedef struct {
	uint8_t a_type;
	uint32_t a_index;
	uint8_t b_type;
	uint32_t b_index;
} cuda_broad_phase_pair;

#ifdef __cplusplus
extern "C" {
#endif

// Returns a malloc'd array of pairs and writes the count to *out_count.
// The caller is responsible for freeing the returned pointer.
// Returns NULL and sets *out_count = 0 if no pairs found.
cuda_broad_phase_pair* broad_phase_cuda(const cuda_broad_phase_proxies_soa* rigids,
										const cuda_broad_phase_proxies_soa* statics,
										size_t* out_count);

#ifdef __cplusplus
}
#endif

#endif
