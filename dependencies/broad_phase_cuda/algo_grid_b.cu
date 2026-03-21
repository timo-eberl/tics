#include "broad_phase_cuda.h"
#include "cuda_common.h"
#include "cuda_profile.h"

#include <cub/device/device_radix_sort.cuh>
#include <cub/device/device_scan.cuh>
#include <cuda/std/functional>

// Can optionally be configured when building
#ifndef GRID_CELL_SIZE
#define GRID_CELL_SIZE 1.0f
#endif
#ifndef GRID_RES_X
#define GRID_RES_X 100
#endif
#ifndef GRID_RES_Y
#define GRID_RES_Y 100
#endif
#ifndef GRID_RES_Z
#define GRID_RES_Z 100
#endif

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

	uint32_t* d_cell_ends; // size = GRID_NUM_CELLS

	void* d_sort_tmp;
	size_t d_sort_tmp_size;
	void* d_scan_tmp;
	size_t d_scan_tmp_size;
};

extern "C" cuda_state_grid_b* cuda_state_grid_b_create(void) {
	cuda_state_grid_b* s = (cuda_state_grid_b*)calloc(1, sizeof(cuda_state_grid_b));
	CUDA_CHECK(cudaMalloc(&s->d_cell_ends, GRID_NUM_CELLS * sizeof(uint32_t)));
	return s;
}

extern "C" void cuda_state_grid_b_destroy(cuda_state_grid_b* s) {
	if (!s) return;
	if (s->d_keys_in) cudaFree(s->d_keys_in);
	if (s->d_keys_out) cudaFree(s->d_keys_out);
	if (s->d_vals_in) cudaFree(s->d_vals_in);
	if (s->d_vals_out) cudaFree(s->d_vals_out);
	if (s->d_sorted_aabbs) cudaFree(s->d_sorted_aabbs);
	if (s->d_cell_ends) cudaFree(s->d_cell_ends);
	if (s->d_sort_tmp) cudaFree(s->d_sort_tmp);
	if (s->d_scan_tmp) cudaFree(s->d_scan_tmp);
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
											  uint32_t* cell_ends) {
	uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= num_keys) return;

	uint32_t key = keys[i];

	// Check if this is the LAST body in this cell
	// Either it's the absolute last body, or the next body has a different key
	bool is_last = (i == num_keys - 1) || (keys[i + 1] != key);

	// Write exclusive end index
	if (is_last) { cell_ends[key] = i + 1; }
}

// 14 Neighbors (Half-Shell) for 3D
// Z=0 Plane: Self(1) + Right(1) + Top Row(3) = 5 cells
// Z=1 Plane: All 9 neighbors = 9 cells
// Total: 14 cells
__constant__ int8_t offs_x[] = {0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1};
__constant__ int8_t offs_y[] = {0, 0, 1, 1, 1, -1, -1, -1, 0, 0, 0, 1, 1, 1};
__constant__ int8_t offs_z[] = {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1};

// Phase 4: Multi-cell pair test
__global__ void grid_b_test_pairs_half_shell_kernel(const cuda_aabb* __restrict__ sorted_aabbs,
													const uint32_t* __restrict__ sorted_indices,
													const uint32_t* __restrict__ cell_ends,
													uint32_t rigid_count, uint32_t total_bodies,
													cuda_pair* pairs, unsigned int* pair_count,
													unsigned int max_pairs) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= total_bodies) return;

	// Coalesced Loading
	uint32_t my_idx = sorted_indices[i];
	cuda_aabb ri = sorted_aabbs[i];

	// Statics also search - minimizes divergence
	bool i_am_static = (my_idx >= rigid_count);

	// Cell range this rigid's AABB min corner maps to
	int cx = cell_coord(ri.min_x, GRID_ORIGIN_X, GRID_RES_X);
	int cy = cell_coord(ri.min_y, GRID_ORIGIN_Y, GRID_RES_Y);
	int cz = cell_coord(ri.min_z, GRID_ORIGIN_Z, GRID_RES_Z);
	uint32_t my_ci = cell_index(cx, cy, cz);

	// Iterate specific half-shell offsets instead of nested loops
	for (int n = 0; n < 14; ++n) {
		int nx = cx + offs_x[n];
		int ny = cy + offs_y[n];
		int nz = cz + offs_z[n];
		// Boundary Check (World limits)
		if (nx < 0 || nx >= GRID_RES_X || ny < 0 || ny >= GRID_RES_Y || nz < 0 || nz >= GRID_RES_Z)
			continue;

		uint32_t other_ci = cell_index(nx, ny, nz);

		uint32_t start = (other_ci == 0) ? 0 : cell_ends[other_ci - 1];
		uint32_t end = cell_ends[other_ci];
		// if other_ci is empty, cell_end_buffer[other_ci] equals cell_end_buffer[other_ci-1]

		// Iterate over all bodies in this cell
		for (uint32_t k = start; k < end; ++k) {
			// Coalesced Load of neighbor data
			uint32_t other_idx = sorted_indices[k];
			cuda_aabb r_neigh = sorted_aabbs[k];

			// If both are in the same cell (n == 0), do index deduplication.
			// This will also skip self-collisions.
			// Otherwise, the half-shell structure guarantees uniqueness.
			if (n == 0 && my_idx >= other_idx) continue;

			bool other_is_static = (other_idx >= rigid_count);
			if (i_am_static && other_is_static) continue;

			if (aabb_overlap(&ri, &r_neigh)) {
				unsigned int idx = atomicAdd(pair_count, 1);
				if (idx < max_pairs) {
					cuda_pair p;
					if (i_am_static) {
						// Static vs Rigid: Swap
						uint32_t my_original_idx = my_idx - rigid_count;
						p = {other_idx, my_original_idx, 0};
					}
					else {
						// Rigid vs either
						uint32_t other_original_idx =
							other_is_static ? other_idx - rigid_count : other_idx;
						p = {my_idx, other_original_idx, !other_is_static};
					}
					pairs[idx] = p;
				}
			}
		}
	}
}

// Phase 4: Multi-cell pair test
__global__ void grid_b_test_pairs_3x3x3_kernel(const cuda_aabb* __restrict__ sorted_aabbs,
											   const uint32_t* __restrict__ sorted_indices,
											   const uint32_t* __restrict__ cell_ends,
											   uint32_t rigid_count, uint32_t total_bodies,
											   cuda_pair* pairs, unsigned int* pair_count,
											   unsigned int max_pairs) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= total_bodies) return;

	uint32_t my_idx = sorted_indices[i];

	// Statics do not search. They are only searched against.
	if (my_idx >= rigid_count) return;

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
				uint32_t other_ci = cell_index(nx, ny, nz);

				uint32_t start = (other_ci == 0) ? 0 : cell_ends[other_ci - 1];
				uint32_t end = cell_ends[other_ci];

				// Iterate over all bodies in this cell
				for (uint32_t k = start; k < end; ++k) {
					// Coalesced Load of neighbor original index
					uint32_t other_idx = sorted_indices[k];
					// Coalesced Load of neighbor AABB
					cuda_aabb r_neigh = sorted_aabbs[k];

					// Check Type based on Index Range
					bool is_rigid = (other_idx < rigid_count);
					// Rigid vs Rigid: Index Deduplication
					if (is_rigid && my_idx >= other_idx) continue;
					if (!is_rigid) other_idx -= rigid_count; // static body: restore original index

					if (aabb_overlap(&ri, &r_neigh)) {
						unsigned int idx = atomicAdd(pair_count, 1);
						if (idx < max_pairs) pairs[idx] = {my_idx, other_idx, is_rigid};
					}
				}
			}
		}
	}
}

extern "C" cuda_pair* cuda_broad_phase_grid_b(cuda_shared_state* sh, cuda_state_grid_b* s,
											  const cuda_aabb* rigids, int rigid_count,
											  const cuda_aabb* statics, int static_count,
											  bool statics_changed, size_t* out_count,
											  bool use_half_shell) {
	*out_count = 0;
	if (rigid_count == 0) return NULL;

	const int block_size = 1024;

	// Page lock the host memory, so uploading data becomes faster
	ensure_host_memory_registered(rigids, rigid_count * sizeof(cuda_aabb), &sh->h_last_rigids_ptr,
								  &sh->h_last_rigids_size);
	ensure_host_memory_registered(statics, static_count * sizeof(cuda_aabb),
								  &sh->h_last_statics_ptr, &sh->h_last_statics_size);

	cuda_profile prof_naive = {0};
	cuda_profile prof_half_shell = {0};
	cuda_profile prof = use_half_shell ? prof_half_shell : prof_naive;
	cuda_profile_begin(&prof);

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
	ensure_device_buffer((void**)&s->d_keys_out, &s->d_keys_out_size, total_bodies,
						 sizeof(uint32_t));
	ensure_device_buffer((void**)&s->d_vals_in, &s->d_vals_in_size, total_bodies, sizeof(uint32_t));
	ensure_device_buffer((void**)&s->d_vals_out, &s->d_vals_out_size, total_bodies,
						 sizeof(uint32_t));
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
	// cuda_profile_step(&prof, "1-assign");

	// ---- Phase 2: Radix sort by key (2a) and gather (2b) ----
	size_t temp_bytes_needed = 0;
	cub::DeviceRadixSort::SortPairs(NULL, temp_bytes_needed, s->d_keys_in, s->d_keys_out,
									s->d_vals_in, s->d_vals_out, total_bodies);
	ensure_device_buffer(&s->d_sort_tmp, &s->d_sort_tmp_size, temp_bytes_needed, 1);
	CUDA_CHECK(cub::DeviceRadixSort::SortPairs(s->d_sort_tmp, s->d_sort_tmp_size, s->d_keys_in,
											   s->d_keys_out, s->d_vals_in, s->d_vals_out,
											   total_bodies));
	// cuda_profile_step(&prof, "2a-sort");

	ensure_device_buffer((void**)&s->d_sorted_aabbs, &s->d_sorted_aabbs_size, total_bodies,
						 sizeof(cuda_aabb));
	int grid_size = (total_bodies + block_size - 1) / block_size;
	grid_b_permute_aabbs_kernel<<<grid_size, block_size>>>(s->d_vals_out, // The sorted indices
														   sh->d_rigids, rigid_count, sh->d_statics,
														   static_count, s->d_sorted_aabbs);
	// cuda_profile_step(&prof, "2b-permute");

	// ---- Phase 3: Find cell boundaries (using sorted keys) ----
	// Reset Single List to 0 (Crucial for the scan to work on empty cells)
	CUDA_CHECK(cudaMemset(s->d_cell_ends, 0, GRID_NUM_CELLS * sizeof(uint32_t)));
	// Run Boundary Kernel (Writes sparse values)
	grid_b_find_boundaries_kernel<<<grid_size, block_size>>>(s->d_keys_out, total_bodies,
															 s->d_cell_ends);

	size_t scan_temp = 0;
	cub::DeviceScan::InclusiveScan(NULL, scan_temp, s->d_cell_ends, s->d_cell_ends,
								   cuda::maximum<uint32_t>(), GRID_NUM_CELLS);
	ensure_device_buffer(&s->d_scan_tmp, &s->d_scan_tmp_size, scan_temp, 1);
	cub::DeviceScan::InclusiveScan(s->d_scan_tmp, scan_temp, s->d_cell_ends, s->d_cell_ends,
								   cuda::maximum<uint32_t>(), GRID_NUM_CELLS);

	// cuda_profile_step(&prof, "3-bounds");
	cuda_profile_step(&prof, "build");

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
		if (use_half_shell) {
			grid_b_test_pairs_half_shell_kernel<<<grid_size, block_size>>>(
				s->d_sorted_aabbs, // Linear AABBs
				s->d_vals_out,	   // Linear Indices
				s->d_cell_ends, (uint32_t)rigid_count, (uint32_t)total_bodies, sh->d_pairs,
				sh->d_pair_count, kernel_max);
		}
		else {
			grid_b_test_pairs_3x3x3_kernel<<<grid_size, block_size>>>(
				s->d_sorted_aabbs, // Linear AABBs
				s->d_vals_out,	   // Linear Indices
				s->d_cell_ends, (uint32_t)rigid_count, (uint32_t)total_bodies, sh->d_pairs,
				sh->d_pair_count, kernel_max);
		}
		CUDA_CHECK(cudaDeviceSynchronize());
		CUDA_CHECK(
			cudaMemcpy(&count, sh->d_pair_count, sizeof(unsigned int), cudaMemcpyDeviceToHost));
		if (count <= kernel_max) break;
		pairs_needed = (size_t)count;
	}
	cuda_profile_step(&prof, "query");

	// ---- Readback ----
	cuda_pair* h_pairs = NULL;
	if (count > 0) {
		h_pairs = (cuda_pair*)malloc(count * sizeof(cuda_pair));
		CUDA_CHECK(
			cudaMemcpy(h_pairs, sh->d_pairs, count * sizeof(cuda_pair), cudaMemcpyDeviceToHost));
		*out_count = count;
	}
	cuda_profile_step(&prof, "readback");

	static cuda_profile_acc prof_acc_naive;
	static cuda_profile_acc prof_acc_half_shell;
	cuda_profile_acc* prof_acc = use_half_shell ? &prof_acc_half_shell : &prof_acc_naive;
	static bool prof_init = false;
	if (!prof_init) {
		cuda_profile_acc_init(prof_acc);
		prof_init = true;
	}
	cuda_profile_end(&prof);
	cuda_profile_log(&prof, prof_acc, use_half_shell ? "grid_b_half_shell" : "grid_b_naive", 10);

	return h_pairs;
}
