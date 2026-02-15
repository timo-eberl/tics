#include "broad_phase_cuda.h"
#include "cuda_common.h"
#include "cuda_profile.h"

#include <cub/device/device_radix_sort.cuh>
#include <cub/device/device_scan.cuh>
#include <cuda/std/functional>

#define GRID_CELL_SIZE 1.0f
#define GRID_RES_X 100
#define GRID_RES_Y 100
#define GRID_RES_Z 100
#define GRID_NUM_CELLS (GRID_RES_X * GRID_RES_Y * GRID_RES_Z) // 1,000,000
#define GRID_ORIGIN_X (-50.0f)
#define GRID_ORIGIN_Y (-50.0f)
#define GRID_ORIGIN_Z (-50.0f)

struct cuda_state_grid_a {
	uint32_t* d_pre_keys_in;
	size_t d_pre_keys_in_size;
	uint32_t* d_pre_keys_out;
	size_t d_pre_keys_out_size;

	uint32_t* d_pre_vals_in;
	size_t d_pre_vals_in_size;
	uint32_t* d_pre_vals_out;
	size_t d_pre_vals_out_size;

	cuda_aabb* d_sorted_aabbs;
	size_t d_sorted_aabbs_size;

	unsigned int* d_counts;
	size_t d_counts_size;
	unsigned int* d_offsets;
	size_t d_offsets_size;

	uint32_t* d_keys_in;
	size_t d_keys_in_size;
	uint32_t* d_keys_out;
	size_t d_keys_out_size;
	uint32_t* d_vals_in;
	size_t d_vals_in_size;
	uint32_t* d_vals_out;
	size_t d_vals_out_size;

	uint32_t* d_cell_ends; // size = GRID_NUM_CELLS

	void* d_sort_tmp;
	size_t d_sort_tmp_size;
	void* d_scan_tmp;
	size_t d_scan_tmp_size;
};

extern "C" cuda_state_grid_a* cuda_state_grid_a_create(void) {
	cuda_state_grid_a* s = (cuda_state_grid_a*)calloc(1, sizeof(cuda_state_grid_a));
	CUDA_CHECK(cudaMalloc(&s->d_cell_ends, GRID_NUM_CELLS * sizeof(uint32_t)));
	return s;
}

extern "C" void cuda_state_grid_a_destroy(cuda_state_grid_a* s) {
	if (!s) return;
	if (s->d_pre_keys_in) cudaFree(s->d_pre_keys_in);
	if (s->d_pre_keys_out) cudaFree(s->d_pre_keys_out);
	if (s->d_pre_vals_in) cudaFree(s->d_pre_vals_in);
	if (s->d_pre_vals_out) cudaFree(s->d_pre_vals_out);
	if (s->d_sorted_aabbs) cudaFree(s->d_sorted_aabbs);
	if (s->d_counts) cudaFree(s->d_counts);
	if (s->d_offsets) cudaFree(s->d_offsets);
	if (s->d_keys_in) cudaFree(s->d_keys_in);
	if (s->d_keys_out) cudaFree(s->d_keys_out);
	if (s->d_vals_in) cudaFree(s->d_vals_in);
	if (s->d_vals_out) cudaFree(s->d_vals_out);
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

// Phase 0a: Load keys and values into buffers before sorting them
__global__ void grid_a_load_key_value_kernel(const cuda_aabb* bodies, int body_count,
											 uint32_t* keys_out, uint32_t* values_out,
											 int val_offset) {
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

// Phase 0c: Write sorted AABBs after sorting
__global__ void grid_a_permute_aabbs_kernel(const uint32_t* sorted_indices, const cuda_aabb* rigids,
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

// Phase 1a: Count number of cells per body for prefix sum
__global__ void grid_a_count_cells_kernel(const cuda_aabb* bodies, int body_count,
										  unsigned int* counts) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= body_count) return;

	// Compute how many cells an AABB overlaps
	cuda_aabb b = bodies[i];
	int cx0 = cell_coord(b.min_x, GRID_ORIGIN_X, GRID_RES_X);
	int cy0 = cell_coord(b.min_y, GRID_ORIGIN_Y, GRID_RES_Y);
	int cz0 = cell_coord(b.min_z, GRID_ORIGIN_Z, GRID_RES_Z);
	int cx1 = cell_coord(b.max_x, GRID_ORIGIN_X, GRID_RES_X);
	int cy1 = cell_coord(b.max_y, GRID_ORIGIN_Y, GRID_RES_Y);
	int cz1 = cell_coord(b.max_z, GRID_ORIGIN_Z, GRID_RES_Z);
	unsigned int count = (cx1 - cx0 + 1) * (cy1 - cy0 + 1) * (cz1 - cz0 + 1);

	counts[i] = count;
}

// Phase 1c: Write keys (cell index) and values (sorted body index) using precomputed offsets
__global__ void grid_a_assign_kernel(const cuda_aabb* bodies, int body_count,
									 const unsigned int* offsets, uint32_t* keys_out,
									 uint32_t* values_out) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= body_count) return;

	cuda_aabb b = bodies[i];
	int cx0 = cell_coord(b.min_x, GRID_ORIGIN_X, GRID_RES_X);
	int cy0 = cell_coord(b.min_y, GRID_ORIGIN_Y, GRID_RES_Y);
	int cz0 = cell_coord(b.min_z, GRID_ORIGIN_Z, GRID_RES_Z);
	int cx1 = cell_coord(b.max_x, GRID_ORIGIN_X, GRID_RES_X);
	int cy1 = cell_coord(b.max_y, GRID_ORIGIN_Y, GRID_RES_Y);
	int cz1 = cell_coord(b.max_z, GRID_ORIGIN_Z, GRID_RES_Z);

	unsigned int off = offsets[i];
	for (int cz = cz0; cz <= cz1; ++cz) {
		for (int cy = cy0; cy <= cy1; ++cy) {
			for (int cx = cx0; cx <= cx1; ++cx) {
				keys_out[off] = cell_index(cx, cy, cz);
				values_out[off] = (uint32_t)i; // Store sorted index
				off++;
			}
		}
	}
}

// Phase 3: Find cell boundaries
__global__ void grid_a_find_boundaries_kernel(const uint32_t* keys, uint32_t num_keys,
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

// Compute the lowest cell index shared by two AABBs (for deduplication).
// Returns 0xFFFFFFFF if no overlap in cell ranges (should not happen for overlapping AABBs).
__device__ static uint32_t lowest_common_cell(const cuda_aabb* a, const cuda_aabb* b) {
	int ax0 = cell_coord(a->min_x, GRID_ORIGIN_X, GRID_RES_X);
	int ay0 = cell_coord(a->min_y, GRID_ORIGIN_Y, GRID_RES_Y);
	int az0 = cell_coord(a->min_z, GRID_ORIGIN_Z, GRID_RES_Z);
	int ax1 = cell_coord(a->max_x, GRID_ORIGIN_X, GRID_RES_X);
	int ay1 = cell_coord(a->max_y, GRID_ORIGIN_Y, GRID_RES_Y);
	int az1 = cell_coord(a->max_z, GRID_ORIGIN_Z, GRID_RES_Z);

	int bx0 = cell_coord(b->min_x, GRID_ORIGIN_X, GRID_RES_X);
	int by0 = cell_coord(b->min_y, GRID_ORIGIN_Y, GRID_RES_Y);
	int bz0 = cell_coord(b->min_z, GRID_ORIGIN_Z, GRID_RES_Z);
	int bx1 = cell_coord(b->max_x, GRID_ORIGIN_X, GRID_RES_X);
	int by1 = cell_coord(b->max_y, GRID_ORIGIN_Y, GRID_RES_Y);
	int bz1 = cell_coord(b->max_z, GRID_ORIGIN_Z, GRID_RES_Z);

	// Lowest shared cell = max of the two min corners per axis
	int sx = ax0 > bx0 ? ax0 : bx0;
	int sy = ay0 > by0 ? ay0 : by0;
	int sz = az0 > bz0 ? az0 : bz0;

	// Verify overlap in cell ranges
	if (sx > ax1 || sx > bx1) return 0xFFFFFFFFu;
	if (sy > ay1 || sy > by1) return 0xFFFFFFFFu;
	if (sz > az1 || sz > bz1) return 0xFFFFFFFFu;

	return cell_index(sx, sy, sz);
}

// Phase 4: Same-cell pair test with lowest-common-cell dedup
__global__ void
grid_a_test_pairs_kernel(const cuda_aabb* sorted_aabbs, const uint32_t* sorted_indices,
						 const uint32_t* sorted_body_indices, // Multi-cell sorted values
						 const uint32_t* cell_ends, int rigid_count, int total_bodies,
						 cuda_pair* pairs, unsigned int* pair_count, unsigned int max_pairs) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= total_bodies) return;

	uint32_t my_original_idx = sorted_indices[i];

	// Statics do not search
	if (my_original_idx >= rigid_count) return;

	cuda_aabb ri = sorted_aabbs[i];

	// Iterate over all cells this rigid occupies
	int cx0 = cell_coord(ri.min_x, GRID_ORIGIN_X, GRID_RES_X);
	int cy0 = cell_coord(ri.min_y, GRID_ORIGIN_Y, GRID_RES_Y);
	int cz0 = cell_coord(ri.min_z, GRID_ORIGIN_Z, GRID_RES_Z);
	int cx1 = cell_coord(ri.max_x, GRID_ORIGIN_X, GRID_RES_X);
	int cy1 = cell_coord(ri.max_y, GRID_ORIGIN_Y, GRID_RES_Y);
	int cz1 = cell_coord(ri.max_z, GRID_ORIGIN_Z, GRID_RES_Z);

	for (int cz = cz0; cz <= cz1; ++cz) {
		for (int cy = cy0; cy <= cy1; ++cy) {
			for (int cx = cx0; cx <= cx1; ++cx) {
				uint32_t ci = cell_index(cx, cy, cz);

				uint32_t start = (ci == 0) ? 0 : cell_ends[ci - 1];
				uint32_t end = cell_ends[ci];

				for (uint32_t k = start; k < end; ++k) {
					// Lookup neighbor from the sorted multi-key array
					uint32_t neighbor_sorted_idx = sorted_body_indices[k];
					// Get original ID for output/dedup
					uint32_t other_idx = sorted_indices[neighbor_sorted_idx];

					bool is_rigid = (other_idx < rigid_count);

					// Index deduplication
					if (is_rigid && other_idx <= my_original_idx) continue;

					cuda_aabb r_neigh = sorted_aabbs[neighbor_sorted_idx];

					if (!aabb_overlap(&ri, &r_neigh)) continue;
					// Dedup: only emit from the lowest common cell
					if (ci != lowest_common_cell(&ri, &r_neigh)) continue;

					unsigned int idx = atomicAdd(pair_count, 1);
					if (idx < max_pairs) {
						if (!is_rigid) other_idx -= rigid_count; // restore original index
						pairs[idx] = {my_original_idx, other_idx, is_rigid};
					}
				}
			}
		}
	}
}

extern "C" cuda_pair* cuda_broad_phase_grid_a(cuda_shared_state* sh, cuda_state_grid_a* s,
											  const cuda_aabb* rigids, int rigid_count,
											  const cuda_aabb* statics, int static_count,
											  bool statics_changed, size_t* out_count) {
	*out_count = 0;
	if (rigid_count == 0) return NULL;

	const int block_size = 1024;

	// Page lock the host memory, so uploading data becomes faster
	ensure_host_memory_registered(rigids, rigid_count * sizeof(cuda_aabb), &sh->h_last_rigids_ptr,
								  &sh->h_last_rigids_size);
	ensure_host_memory_registered(statics, static_count * sizeof(cuda_aabb),
								  &sh->h_last_statics_ptr, &sh->h_last_statics_size);

	cuda_profile prof = {0};
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

	// ---- Phase 0: Pre-Sort Bodies by Min-Corner Cell ----
	int total_bodies = rigid_count + static_count;
	ensure_device_buffer((void**)&s->d_pre_keys_in, &s->d_pre_keys_in_size, total_bodies,
						 sizeof(uint32_t));
	ensure_device_buffer((void**)&s->d_pre_keys_out, &s->d_pre_keys_out_size, total_bodies,
						 sizeof(uint32_t));
	ensure_device_buffer((void**)&s->d_pre_vals_in, &s->d_pre_vals_in_size, total_bodies,
						 sizeof(uint32_t));
	ensure_device_buffer((void**)&s->d_pre_vals_out, &s->d_pre_vals_out_size, total_bodies,
						 sizeof(uint32_t));
	int grid_size_rigids = (rigid_count + block_size - 1) / block_size;
	grid_a_load_key_value_kernel<<<grid_size_rigids, block_size>>>(
		sh->d_rigids, rigid_count, s->d_pre_keys_in, s->d_pre_vals_in, 0);
	if (static_count > 0) {
		int grid_size_statics = (static_count + block_size - 1) / block_size;
		grid_a_load_key_value_kernel<<<grid_size_statics, block_size>>>(
			sh->d_statics, static_count, s->d_pre_keys_in, s->d_pre_vals_in, rigid_count);
	}
	cuda_profile_step(&prof, "0a-load");

	size_t temp_bytes_needed = 0;
	cub::DeviceRadixSort::SortPairs(NULL, temp_bytes_needed, s->d_pre_keys_in, s->d_pre_keys_out,
									s->d_pre_vals_in, s->d_pre_vals_out, total_bodies);
	ensure_device_buffer(&s->d_sort_tmp, &s->d_sort_tmp_size, temp_bytes_needed, 1);
	CUDA_CHECK(cub::DeviceRadixSort::SortPairs(s->d_sort_tmp, s->d_sort_tmp_size, s->d_pre_keys_in,
											   s->d_pre_keys_out, s->d_pre_vals_in,
											   s->d_pre_vals_out, total_bodies));
	cuda_profile_step(&prof, "0b-sort");

	ensure_device_buffer((void**)&s->d_sorted_aabbs, &s->d_sorted_aabbs_size, total_bodies,
						 sizeof(cuda_aabb));
	int grid_size_total = (total_bodies + block_size - 1) / block_size;
	grid_a_permute_aabbs_kernel<<<grid_size_total, block_size>>>(s->d_pre_vals_out, sh->d_rigids,
																 rigid_count, sh->d_statics,
																 static_count, s->d_sorted_aabbs);
	cuda_profile_step(&prof, "0c-permute");

	// ---- Phase 1: Multi-cell assignment (Using Sorted Bodies) ----
	// Phase 1a: Count keys per body
	ensure_device_buffer((void**)&s->d_counts, &s->d_counts_size, total_bodies,
						 sizeof(unsigned int));
	ensure_device_buffer((void**)&s->d_offsets, &s->d_offsets_size, total_bodies + 1,
						 sizeof(unsigned int));
	// One launch for all bodies since they are in a single sorted array
	grid_a_count_cells_kernel<<<grid_size_total, block_size>>>(s->d_sorted_aabbs, total_bodies,
															   s->d_counts);
	cuda_profile_step(&prof, "1a-count");

	// Phase 1b: prefix sum to get offsets
	size_t scan_temp = 0;
	cub::DeviceScan::ExclusiveSum(NULL, scan_temp, s->d_counts, s->d_offsets, total_bodies);
	ensure_device_buffer(&s->d_scan_tmp, &s->d_scan_tmp_size, scan_temp, 1);
	CUDA_CHECK(cub::DeviceScan::ExclusiveSum(s->d_scan_tmp, s->d_scan_tmp_size, s->d_counts,
											 s->d_offsets, total_bodies));
	unsigned int last_offset = 0, last_count = 0;
	CUDA_CHECK(cudaMemcpy(&last_offset, s->d_offsets + total_bodies - 1, sizeof(unsigned int),
						  cudaMemcpyDeviceToHost));
	CUDA_CHECK(cudaMemcpy(&last_count, s->d_counts + total_bodies - 1, sizeof(unsigned int),
						  cudaMemcpyDeviceToHost));
	unsigned int total_keys = last_offset + last_count;
	cuda_profile_step(&prof, "1b-scan");

	// Phase 1c: Allocate keys and write them
	ensure_device_buffer((void**)&s->d_keys_in, &s->d_keys_in_size, total_keys, sizeof(uint32_t));
	ensure_device_buffer((void**)&s->d_keys_out, &s->d_keys_out_size, total_keys, sizeof(uint32_t));
	ensure_device_buffer((void**)&s->d_vals_in, &s->d_vals_in_size, total_keys, sizeof(uint32_t));
	ensure_device_buffer((void**)&s->d_vals_out, &s->d_vals_out_size, total_keys, sizeof(uint32_t));
	grid_a_assign_kernel<<<grid_size_total, block_size>>>(s->d_sorted_aabbs, total_bodies,
														  s->d_offsets, s->d_keys_in, s->d_vals_in);
	cuda_profile_step(&prof, "1c-assign");

	// ---- Phase 2: Radix sort keys (and move values) ----
	temp_bytes_needed = 0;
	cub::DeviceRadixSort::SortPairs(NULL, temp_bytes_needed, s->d_keys_in, s->d_keys_out,
									s->d_vals_in, s->d_vals_out, (int)total_keys);
	ensure_device_buffer(&s->d_sort_tmp, &s->d_sort_tmp_size, temp_bytes_needed, 1);
	CUDA_CHECK(cub::DeviceRadixSort::SortPairs(s->d_sort_tmp, s->d_sort_tmp_size, s->d_keys_in,
											   s->d_keys_out, s->d_vals_in, s->d_vals_out,
											   (int)total_keys));
	cuda_profile_step(&prof, "2-sort");

	// ---- Phase 3: Find cell boundaries ----
	// Reset Single List to 0 (Crucial for the scan to work on empty cells)
	CUDA_CHECK(cudaMemset(s->d_cell_ends, 0, GRID_NUM_CELLS * sizeof(uint32_t)));
	// Run Boundary Kernel (Writes sparse values)
	int grid_size_keys = (total_keys + block_size - 1) / block_size;
	grid_a_find_boundaries_kernel<<<grid_size_keys, block_size>>>(s->d_keys_out, total_keys,
																  s->d_cell_ends);
	size_t scan_temp_2 = 0;
	cub::DeviceScan::InclusiveScan(NULL, scan_temp_2, s->d_cell_ends, s->d_cell_ends,
								   cuda::maximum<int>(), GRID_NUM_CELLS);
	ensure_device_buffer(&s->d_scan_tmp, &s->d_scan_tmp_size, scan_temp_2, 1);
	cub::DeviceScan::InclusiveScan(s->d_scan_tmp, scan_temp_2, s->d_cell_ends, s->d_cell_ends,
								   cuda::maximum<int>(), GRID_NUM_CELLS);
	cuda_profile_step(&prof, "3-bounds");

	// ---- Phase 4: Test pairs (same-cell only, with lowest-common-cell dedup) ----
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
		grid_a_test_pairs_kernel<<<grid_size_total, block_size>>>(
			s->d_sorted_aabbs, s->d_pre_vals_out, s->d_vals_out, s->d_cell_ends, rigid_count,
			total_bodies, sh->d_pairs, sh->d_pair_count, kernel_max);
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
	cuda_profile_log(&prof, &prof_acc, "grid_a", 10);

	return h_pairs;
}
