#ifndef BROAD_PHASE_CUDA_H
#define BROAD_PHASE_CUDA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Per-body AABB in Array-of-Structs layout. 24 bytes, no padding.
// LAYOUT MUST MATCH tics_internal.h::packed_aabb
typedef struct {
	float min_x, max_x, min_y, max_y, min_z, max_z;
} cuda_aabb;

// Output pair. Body A is always rigid (implicit in the broadphase contract).
// Reordered so the uint8_t is last to minimize padding: 4 + 4 + 1 + 3 pad = 12 bytes.
typedef struct {
	uint32_t a_index;
	uint32_t b_index;
	uint8_t b_type; // 0 = STATIC_BODY, 1 = RIGID_BODY (matches body_type enum)
} cuda_broad_phase_pair;

// Opaque handle to persistent GPU-side state (device buffers, grid, etc.)
typedef struct cuda_broad_phase_state cuda_broad_phase_state;

#ifdef __cplusplus
extern "C" {
#endif

cuda_broad_phase_state* cuda_broad_phase_create(void);
void cuda_broad_phase_destroy(cuda_broad_phase_state* state);

// Run the broad phase. The CUDA side keeps internal device buffers and only
// reallocates when counts change. Static AABBs are only re-uploaded when
// statics_changed is true.
//
// Returns a malloc'd array of pairs; caller frees. Writes count to *out_count.
// Returns NULL and sets *out_count = 0 if no pairs found.
cuda_broad_phase_pair* cuda_broad_phase_run(cuda_broad_phase_state* state, const cuda_aabb* rigids,
											size_t rigid_count, const cuda_aabb* statics,
											size_t static_count, bool statics_changed,
											size_t* out_count);

#ifdef __cplusplus
}
#endif

#endif
