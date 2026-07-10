#ifndef BROAD_PHASE_DX_H
#define BROAD_PHASE_DX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
	float min_x, max_x, min_y, max_y, min_z, max_z;
} dx_aabb;

typedef struct {
	uint32_t a_index;
	uint32_t b_index;
	uint8_t b_type;
} dx_pair;

typedef struct {
	int res_x, res_y, res_z;
	float origin_x, origin_y, origin_z;
	float cell_size;
} dx_grid_config;

typedef struct dx_shared_state dx_shared_state;
typedef struct dx_state_brute_force dx_state_brute_force;

#ifdef __cplusplus
extern "C" {
#endif

dx_shared_state* dx_shared_state_create(void);
void dx_shared_state_destroy(dx_shared_state* state);

dx_state_brute_force* dx_state_brute_force_create(dx_shared_state* shared_state);
void dx_state_brute_force_destroy(dx_state_brute_force* state);

dx_pair* dx_broad_phase_brute_force(dx_shared_state* shared_state,
									dx_state_brute_force* state, const dx_aabb* rigids,
									int rigid_count, const dx_aabb* statics, int static_count,
									bool statics_changed, size_t* out_count);

#ifdef __cplusplus
}
#endif

#endif
