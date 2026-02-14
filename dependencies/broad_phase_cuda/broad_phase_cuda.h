#ifndef BROAD_PHASE_CUDA_H
#define BROAD_PHASE_CUDA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Per-body AABB in Array-of-Structs layout. 24 bytes, no padding.
typedef struct {
	float min_x, max_x, min_y, max_y, min_z, max_z;
} cuda_aabb;

// Output pair. Body A is always rigid (implicit in the broadphase contract).
// Reordered so the uint8_t is last to minimize padding: 4 + 4 + 1 + 3 pad = 12 bytes.
typedef struct {
	uint32_t a_index;
	uint32_t b_index;
	uint8_t b_type; // 0 = STATIC_BODY, 1 = RIGID_BODY
} cuda_pair;

// Opaque handles to persistent GPU-side state (device buffers, grid, etc.)
// Shared state between all algorithms
typedef struct cuda_shared_state cuda_shared_state;
// Each algorithm has its own state
typedef struct cuda_state_brute_force cuda_state_brute_force;
typedef struct cuda_state_grid_a cuda_state_grid_a;
typedef struct cuda_state_grid_b cuda_state_grid_b;

#ifdef __cplusplus
extern "C" {
#endif

cuda_shared_state* cuda_shared_state_create(void);
void cuda_shared_state_destroy(cuda_shared_state* state);

cuda_state_brute_force* cuda_state_brute_force_create(void);
void cuda_state_brute_force_destroy(cuda_state_brute_force* state);
// Brute-force O(n^2) broad phase.
cuda_pair* cuda_broad_phase_brute_force(cuda_shared_state* shared_state,
										cuda_state_brute_force* state, const cuda_aabb* rigids,
										int rigid_count, const cuda_aabb* statics, int static_count,
										bool statics_changed, size_t* out_count);

cuda_state_grid_a* cuda_state_grid_a_create(void);
void cuda_state_grid_a_destroy(cuda_state_grid_a* state);
// Uniform-grid broad phase (Strategy A: multi-cell insert, same-cell test).
cuda_pair* cuda_broad_phase_grid_a(cuda_shared_state* shared_state, cuda_state_grid_a* state,
								   const cuda_aabb* rigids, int rigid_count,
								   const cuda_aabb* statics, int static_count, bool statics_changed,
								   size_t* out_count);

cuda_state_grid_b* cuda_state_grid_b_create(void);
void cuda_state_grid_b_destroy(cuda_state_grid_b* state);
// Uniform-grid broad phase (Strategy B: single-cell insert, multi-cell test).
// World size is limited and objects diameters are limited.
cuda_pair* cuda_broad_phase_grid_b(cuda_shared_state* shared_state, cuda_state_grid_b* state,
								   const cuda_aabb* rigids, int rigid_count,
								   const cuda_aabb* statics, int static_count, bool statics_changed,
								   size_t* out_count);

#ifdef __cplusplus
}
#endif

#endif
