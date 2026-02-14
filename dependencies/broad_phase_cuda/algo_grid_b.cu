#include "broad_phase_cuda.h"
#include "cuda_common.h"
#include "cuda_profile.h"

#include <cub/device/device_radix_sort.cuh>

#define GRID_CELL_SIZE 1.0f
#define GRID_RES_X 100
#define GRID_RES_Y 100
#define GRID_RES_Z 100
#define GRID_NUM_CELLS (GRID_RES_X * GRID_RES_Y * GRID_RES_Z) // 1,000,000
#define GRID_ORIGIN_X (-50.0f)
#define GRID_ORIGIN_Y (-50.0f)
#define GRID_ORIGIN_Z (-50.0f)

struct cuda_state_grid_b {
	uint32_t* d_keys_in;
	size_t d_keys_in_size;
	uint32_t* d_keys_out;
	size_t d_keys_out_size;

	uint32_t* d_vals_in;
	size_t d_vals_in_size;
	uint32_t* d_vals_out;
	size_t d_vals_out_size;

	cuda_aabb* d_sorted_aabbs;
	size_t d_sorted_aabbs_size;

	uint32_t* d_cell_start; // size = GRID_NUM_CELLS
	uint32_t* d_cell_end;	// size = GRID_NUM_CELLS

	void* d_sort_tmp;
	size_t d_sort_tmp_size;
};

extern "C" cuda_state_grid_b* cuda_state_grid_b_create(void) {
	cuda_state_grid_b* s = (cuda_state_grid_b*)calloc(1, sizeof(cuda_state_grid_b));
	CUDA_CHECK(cudaMalloc(&s->d_cell_start, GRID_NUM_CELLS * sizeof(uint32_t)));
	CUDA_CHECK(cudaMalloc(&s->d_cell_end, GRID_NUM_CELLS * sizeof(uint32_t)));
	return s;
}

extern "C" void cuda_state_grid_b_destroy(cuda_state_grid_b* s) {
	if (!s) return;
	if (s->d_keys_in) cudaFree(s->d_keys_in);
	if (s->d_keys_out) cudaFree(s->d_keys_out);
	if (s->d_vals_in) cudaFree(s->d_vals_in);
	if (s->d_vals_out) cudaFree(s->d_vals_out);
	if (s->d_sorted_aabbs) cudaFree(s->d_sorted_aabbs);
	if (s->d_cell_start) cudaFree(s->d_cell_start);
	if (s->d_cell_end) cudaFree(s->d_cell_end);
	if (s->d_sort_tmp) cudaFree(s->d_sort_tmp);
	free(s);
}

__device__ static bool aabb_overlap(const cuda_aabb* a, const cuda_aabb* b) {
	return a->max_x >= b->min_x && a->min_x <= b->max_x && a->max_y >= b->min_y &&
		   a->min_y <= b->max_y && a->max_z >= b->min_z && a->min_z <= b->max_z;
}
__device__ static int clamp_int(int v, int lo, int hi) {
	return v < lo ? lo : (v > hi ? hi : v);
}
__device__ static int cell_coord(float pos, float origin, int grid_resolution) {
	return clamp_int((int)floorf((pos - origin) / GRID_CELL_SIZE), 0, grid_resolution - 1);
}
__device__ static uint32_t cell_index(int cx, int cy, int cz) {
	return (uint32_t)cx + (uint32_t)cy * GRID_RES_X + (uint32_t)cz * GRID_RES_X * GRID_RES_Y;
}

// Phase 1: Assign each body to its min-corner cell
__global__ void grid_b_assign_kernel(const cuda_aabb* bodies, int body_count, uint32_t* keys_out,
									 uint32_t* values_out, int val_offset) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= body_count) return;

	cuda_aabb b = bodies[i];
	int cx = cell_coord(b.min_x, GRID_ORIGIN_X, GRID_RES_X);
	int cy = cell_coord(b.min_y, GRID_ORIGIN_Y, GRID_RES_Y);
	int cz = cell_coord(b.min_z, GRID_ORIGIN_Z, GRID_RES_Z);

	// Output Key (Cell Index) and Value (Original Body Index)
	keys_out[val_offset + i] = cell_index(cx, cy, cz);
	values_out[val_offset + i] = (uint32_t)(val_offset + i);
}

// Phase 2b: Permute AABBs into linear sorted order
__global__ void grid_b_permute_aabbs_kernel(const uint32_t* sorted_indices, const cuda_aabb* rigids,
											int rigid_count, const cuda_aabb* statics,
											int static_count, cuda_aabb* sorted_aabbs_out) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	int total = rigid_count + static_count;
	if (i >= total) return;

	uint32_t original_idx = sorted_indices[i];

	// This is a random memory read. We can't avoid doing it entirely, so we do it once here instead
	// of multiple times in Phase 4
	cuda_aabb aabb;
	if (original_idx < rigid_count) { aabb = rigids[original_idx]; }
	else { aabb = statics[original_idx - rigid_count]; }

	sorted_aabbs_out[i] = aabb;
}

// Phase 3: Find cell boundaries
__global__ void grid_b_find_boundaries_kernel(const uint32_t* keys, uint32_t num_keys,
											  uint32_t* cell_start, uint32_t* cell_end) {
	uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= num_keys) return;

	uint32_t cell_i = keys[i];
	if (i == 0) { cell_start[cell_i] = 0; }
	else {
		uint32_t cell_prev = keys[i - 1];
		if (cell_i != cell_prev) {
			cell_start[cell_i] = i;
			cell_end[cell_prev] = i;
		}
	}

	if (i == num_keys - 1) { cell_end[cell_i] = (uint32_t)(num_keys); }
}

// Phase 4: Multi-cell pair test
__global__ void grid_b_test_pairs_kernel(const cuda_aabb* sorted_aabbs,
										 const uint32_t* sorted_indices, const uint32_t* cell_start,
										 const uint32_t* cell_end, uint32_t rigid_count,
										 uint32_t total_bodies, cuda_pair* pairs,
										 unsigned int* pair_count, unsigned int max_pairs) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= total_bodies) return;

	uint32_t my_original_idx = sorted_indices[i];

	// Statics do not search. They are only searched against.
	if (my_original_idx >= rigid_count) return;

	// Coalesced Load
	cuda_aabb ri = sorted_aabbs[i];

	// Cell range this rigid's AABB min corner maps to
	int cx = cell_coord(ri.min_x, GRID_ORIGIN_X, GRID_RES_X);
	int cy = cell_coord(ri.min_y, GRID_ORIGIN_Y, GRID_RES_Y);
	int cz = cell_coord(ri.min_z, GRID_ORIGIN_Z, GRID_RES_Z);

	// Search 3x3x3 neighborhood
	int cx_lo = clamp_int(cx - 1, 0, GRID_RES_X - 1);
	int cx_hi = clamp_int(cx + 1, 0, GRID_RES_X - 1);
	int cy_lo = clamp_int(cy - 1, 0, GRID_RES_Y - 1);
	int cy_hi = clamp_int(cy + 1, 0, GRID_RES_Y - 1);
	int cz_lo = clamp_int(cz - 1, 0, GRID_RES_Z - 1);
	int cz_hi = clamp_int(cz + 1, 0, GRID_RES_Z - 1);

	// Because sorted_aabbs is sorted spatially, we iterate through cells in a coalesced way here
	for (int nz = cz_lo; nz <= cz_hi; ++nz) {
		for (int ny = cy_lo; ny <= cy_hi; ++ny) {
			for (int nx = cx_lo; nx <= cx_hi; ++nx) {
				uint32_t ci = cell_index(nx, ny, nz);

				uint32_t start = cell_start[ci];
				uint32_t end = cell_end[ci];
				if (start == 0xFFFFFFFFu) continue; // Skip empty cells

				// Iterate over all bodies in this cell
				for (uint32_t k = start; k < end; ++k) {
					// Coalesced Load of neighbor original index
					uint32_t other_idx = sorted_indices[k];
					// Coalesced Load of neighbor AABB
					cuda_aabb r_neigh = sorted_aabbs[k];

					// Check Type based on Index Range
					bool is_rigid = (other_idx < rigid_count);
					// Rigid vs Rigid: Index Deduplication
					if (is_rigid && my_original_idx >= other_idx) continue;
					if (!is_rigid) other_idx -= rigid_count; // static body: restore original index

					if (aabb_overlap(&ri, &r_neigh)) {
						unsigned int idx = atomicAdd(pair_count, 1);
						if (idx < max_pairs) pairs[idx] = {my_original_idx, other_idx, is_rigid};
					}
				}
			}
		}
	}
}

extern "C" cuda_pair* cuda_broad_phase_grid_b(cuda_shared_state* sh, cuda_state_grid_b* s,
											  const cuda_aabb* rigids, int rigid_count,
											  const cuda_aabb* statics, int static_count,
											  bool statics_changed, size_t* out_count) {
	*out_count = 0;
	if (rigid_count == 0) return NULL;

	const int block_size = 1024;

	cuda_profile prof = {0};
	cuda_profile_begin(&prof);

	// Page lock the host memory, so uploading data becomes faster
	ensure_host_memory_registered(rigids, rigid_count * sizeof(cuda_aabb), &sh->h_last_rigids_ptr,
								  &sh->h_last_rigids_size);
	ensure_host_memory_registered(statics, static_count * sizeof(cuda_aabb),
								  &sh->h_last_statics_ptr, &sh->h_last_statics_size);
	cuda_profile_step(&prof, "pagelock");

	// ---- Upload AABBs ----
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

	// ---- Phase 1: Assign cells (one key per body) ----
	// key = cell index, value = original index
	int total_bodies = rigid_count + static_count;
	ensure_device_buffer((void**)&s->d_keys_in, &s->d_keys_in_size, total_bodies, sizeof(uint32_t));
	ensure_device_buffer((void**)&s->d_keys_out, &s->d_keys_out_size, total_bodies, sizeof(uint32_t));
	ensure_device_buffer((void**)&s->d_vals_in, &s->d_vals_in_size, total_bodies, sizeof(uint32_t));
	ensure_device_buffer((void**)&s->d_vals_out, &s->d_vals_out_size, total_bodies, sizeof(uint32_t));
	// write keys and values for rigids
	int grid_size_rigids = (rigid_count + block_size - 1) / block_size;
	grid_b_assign_kernel<<<grid_size_rigids, block_size>>>(sh->d_rigids, rigid_count, s->d_keys_in,
														   s->d_vals_in, 0);
	// same for statics in the same buffers after rigids
	if (static_count > 0) {
		int grid_size_statics = (static_count + block_size - 1) / block_size;
		grid_b_assign_kernel<<<grid_size_statics, block_size>>>(
			sh->d_statics, static_count, s->d_keys_in, s->d_vals_in, rigid_count);
	}
	cuda_profile_step(&prof, "1-assign");

	// ---- Phase 2: Radix sort by key (2a) and gather (2b) ----
	size_t temp_bytes_needed = 0;
	cub::DeviceRadixSort::SortPairs(NULL, temp_bytes_needed, s->d_keys_in, s->d_keys_out,
									s->d_vals_in, s->d_vals_out, total_bodies);
	ensure_device_buffer(&s->d_sort_tmp, &s->d_sort_tmp_size, temp_bytes_needed, 1);
	CUDA_CHECK(cub::DeviceRadixSort::SortPairs(s->d_sort_tmp, s->d_sort_tmp_size, s->d_keys_in,
											   s->d_keys_out, s->d_vals_in, s->d_vals_out,
											   total_bodies));
	cuda_profile_step(&prof, "2a-sort");

	ensure_device_buffer((void**)&s->d_sorted_aabbs, &s->d_sorted_aabbs_size, total_bodies,
						 sizeof(cuda_aabb));
	int grid_size = (total_bodies + block_size - 1) / block_size;
	grid_b_permute_aabbs_kernel<<<grid_size, block_size>>>(s->d_vals_out, // The sorted indices
														   sh->d_rigids, rigid_count, sh->d_statics,
														   static_count, s->d_sorted_aabbs);
	cuda_profile_step(&prof, "2b-permute");

	// ---- Phase 3: Find cell boundaries (using sorted keys) ----
	CUDA_CHECK(cudaMemset(s->d_cell_start, 0xFF, GRID_NUM_CELLS * sizeof(uint32_t)));
	CUDA_CHECK(cudaMemset(s->d_cell_end, 0xFF, GRID_NUM_CELLS * sizeof(uint32_t)));
	grid_b_find_boundaries_kernel<<<grid_size, block_size>>>(s->d_keys_out, total_bodies,
															 s->d_cell_start, s->d_cell_end);
	cuda_profile_step(&prof, "3-bounds");

	// ---- Phase 4: Test pairs (multi-cell) ----
	size_t pairs_needed = sh->d_pairs_size;
	if (pairs_needed < 8 * (rigid_count + static_count))
		pairs_needed = 8 * (rigid_count + static_count);
	if (pairs_needed < 1024) pairs_needed = 1024;
	unsigned int count = 0;
	for (int attempt = 0; attempt < 2; ++attempt) {
		ensure_device_buffer((void**)&sh->d_pairs, &sh->d_pairs_size, pairs_needed,
							 sizeof(cuda_pair));
		unsigned int kernel_max =
			(sh->d_pairs_size > (size_t)UINT32_MAX) ? UINT32_MAX : (unsigned int)sh->d_pairs_size;
		CUDA_CHECK(cudaMemset(sh->d_pair_count, 0, sizeof(unsigned int)));
		grid_b_test_pairs_kernel<<<grid_size, block_size>>>(
			s->d_sorted_aabbs, // Linear AABBs
			s->d_vals_out,	   // Linear Indices
			s->d_cell_start, s->d_cell_end, (uint32_t)rigid_count, (uint32_t)total_bodies,
			sh->d_pairs, sh->d_pair_count, kernel_max);
		CUDA_CHECK(cudaDeviceSynchronize());
		CUDA_CHECK(
			cudaMemcpy(&count, sh->d_pair_count, sizeof(unsigned int), cudaMemcpyDeviceToHost));
		if (count <= kernel_max) break;
		pairs_needed = (size_t)count;
	}
	cuda_profile_step(&prof, "4-tests");

	// ---- Readback ----
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
	cuda_profile_log(&prof, &prof_acc, "grid_b", 10);

	return h_pairs;
}
