#include "broad_phase_cuda.h"
#include "cuda_profile.h"

#include <cub/device/device_radix_sort.cuh>
#include <cub/device/device_scan.cuh>

#include <stdio.h>
#include <stdlib.h>

// ---------------------------------------------------------------------------
// Grid constants
// ---------------------------------------------------------------------------
#define GRID_CELL_SIZE 1.0f
#define GRID_RES_X 100
#define GRID_RES_Y 100
#define GRID_RES_Z 100
#define GRID_NUM_CELLS (GRID_RES_X * GRID_RES_Y * GRID_RES_Z) // 1,000,000
#define GRID_ORIGIN_X (-50.0f)
#define GRID_ORIGIN_Y (-50.0f)
#define GRID_ORIGIN_Z (-50.0f)

// Packed into a single 64-bit key for radix sort:
//   bits [63..32] = cell_index  (uint32_t)
//   bits [31..31] = body_type   (1 = rigid, 0 = static)
//   bits [30.. 0] = body_index  (up to 2^31 bodies)
typedef unsigned long long grid_key_t;

static __host__ __device__ grid_key_t make_grid_key(uint32_t cell, uint8_t type, uint32_t index) {
	return ((grid_key_t)cell << 32) | ((grid_key_t)type << 31) | (grid_key_t)index;
}
static __host__ __device__ void unpack_grid_key(grid_key_t key, uint32_t* cell, uint8_t* type,
												uint32_t* index) {
	*cell = (uint32_t)(key >> 32);
	*type = (uint8_t)((key >> 31) & 1u);
	*index = (uint32_t)(key & 0x7FFFFFFFu);
}
static __host__ __device__ uint32_t unpack_grid_key_cell(grid_key_t key) {
	return (uint32_t)(key >> 32);
}

// ---------------------------------------------------------------------------
// Persistent state
// ---------------------------------------------------------------------------
struct cuda_broad_phase_state {
	cuda_aabb* d_rigids;
	cuda_aabb* d_statics;
	size_t d_rigids_capacity;
	size_t d_statics_capacity;

	cuda_broad_phase_pair* d_pairs;
	size_t d_pairs_capacity;
	unsigned int* d_pair_count;

	// Grid buffers (Strategy A)
	grid_key_t* d_keys_in;
	grid_key_t* d_keys_out;
	size_t d_keys_capacity;
	size_t d_keys_out_capacity;

	// Optimization for Strategy B: Separate Key (Cell) and Value (Original Index)
	// plus sorted AABBs for linear memory access.
	uint32_t* d_b_keys_in;
	uint32_t* d_b_keys_out;
	size_t d_b_keys_capacity;
	size_t d_b_keys_out_capacity;

	uint32_t* d_b_values_in;
	uint32_t* d_b_values_out;
	size_t d_b_values_capacity;
	size_t d_b_values_out_capacity;

	cuda_aabb* d_b_sorted_aabbs;
	size_t d_b_sorted_aabbs_capacity;

	uint32_t* d_cell_start; // size = GRID_NUM_CELLS
	uint32_t* d_cell_end;	// size = GRID_NUM_CELLS

	void* d_sort_temp;
	size_t d_sort_temp_size;

	unsigned int* d_counts;
	size_t d_counts_capacity;
	unsigned int* d_offsets;
	size_t d_offsets_capacity;
	void* d_scan_temp;
	size_t d_scan_temp_size;

	// --- Host Memory Registration Tracking ---
	const void* h_last_rigids_ptr;
	size_t h_last_rigids_size;
	const void* h_last_statics_ptr;
	size_t h_last_statics_size;
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
extern "C" cuda_broad_phase_state* cuda_broad_phase_state_create(void) {
	cuda_broad_phase_state* s = (cuda_broad_phase_state*)calloc(1, sizeof(cuda_broad_phase_state));
	CUDA_CHECK(cudaMalloc(&s->d_pair_count, sizeof(unsigned int)));
	CUDA_CHECK(cudaMalloc(&s->d_cell_start, GRID_NUM_CELLS * sizeof(uint32_t)));
	CUDA_CHECK(cudaMalloc(&s->d_cell_end, GRID_NUM_CELLS * sizeof(uint32_t)));
	return s;
}

extern "C" void cuda_broad_phase_state_destroy(cuda_broad_phase_state* s) {
	if (!s) return;
	if (s->d_rigids) cudaFree(s->d_rigids);
	if (s->d_statics) cudaFree(s->d_statics);
	if (s->d_pairs) cudaFree(s->d_pairs);
	if (s->d_pair_count) cudaFree(s->d_pair_count);

	if (s->d_keys_in) cudaFree(s->d_keys_in);
	if (s->d_keys_out) cudaFree(s->d_keys_out);

	if (s->d_b_keys_in) cudaFree(s->d_b_keys_in);
	if (s->d_b_keys_out) cudaFree(s->d_b_keys_out);
	if (s->d_b_values_in) cudaFree(s->d_b_values_in);
	if (s->d_b_values_out) cudaFree(s->d_b_values_out);
	if (s->d_b_sorted_aabbs) cudaFree(s->d_b_sorted_aabbs);

	if (s->d_cell_start) cudaFree(s->d_cell_start);
	if (s->d_cell_end) cudaFree(s->d_cell_end);
	if (s->d_sort_temp) cudaFree(s->d_sort_temp);
	if (s->d_counts) cudaFree(s->d_counts);
	if (s->d_offsets) cudaFree(s->d_offsets);
	if (s->d_scan_temp) cudaFree(s->d_scan_temp);

	// Clean up host registration
	if (s->h_last_rigids_ptr) cudaHostUnregister((void*)s->h_last_rigids_ptr);
	if (s->h_last_statics_ptr) cudaHostUnregister((void*)s->h_last_statics_ptr);

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

// ---------------------------------------------------------------------------
// Brute-force kernel
// ---------------------------------------------------------------------------
__global__ void brute_force_kernel(const cuda_aabb* rigids, int rigid_count,
								   const cuda_aabb* statics, int static_count,
								   cuda_broad_phase_pair* pairs, unsigned int* pair_count,
								   unsigned int max_pairs) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= rigid_count) return;

	cuda_aabb ri = rigids[i];

	for (int j = i + 1; j < rigid_count; ++j) {
		if (aabb_overlap(&ri, &rigids[j])) {
			unsigned int idx = atomicAdd(pair_count, 1);
			if (idx < max_pairs) pairs[idx] = {i, (uint32_t)j, 1};
		}
	}

	for (int j = 0; j < static_count; ++j) {
		if (aabb_overlap(&ri, &statics[j])) {
			unsigned int idx = atomicAdd(pair_count, 1);
			if (idx < max_pairs) pairs[idx] = {i, (uint32_t)j, 0};
		}
	}
}

// ---------------------------------------------------------------------------
// Naive broad phase
// ---------------------------------------------------------------------------
extern "C" cuda_broad_phase_pair* cuda_broad_phase_naive(cuda_broad_phase_state* s,
														 const cuda_aabb* rigids, int rigid_count,
														 const cuda_aabb* statics, int static_count,
														 bool statics_changed, size_t* out_count) {
	*out_count = 0;
	if (rigid_count == 0) return NULL;

	cuda_profile prof;
	cuda_profile_begin(&prof);

	// Page lock the host memory, so uploading data becomes faster
	ensure_host_memory_registered(rigids, rigid_count * sizeof(cuda_aabb), &s->h_last_rigids_ptr,
								  &s->h_last_rigids_size);
	ensure_host_memory_registered(statics, static_count * sizeof(cuda_aabb), &s->h_last_statics_ptr,
								  &s->h_last_statics_size);

	cuda_profile_step(&prof, "pagelock");

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

	cuda_profile_step(&prof, "upload");

	// Heuristic output size: We probably won't have more than 8 potential collisions per body.
	// If we actually have more than that, the first run fails and will output the actual required
	// size (count). We then run a second time with the actually required size.
	size_t pairs_needed = s->d_pairs_capacity;
	if (pairs_needed < 8 * (rigid_count + static_count))
		pairs_needed = 8 * (rigid_count + static_count);
	if (pairs_needed < 1024) pairs_needed = 1024;

	const int block_size = 256;
	int grid_size = (rigid_count + block_size - 1) / block_size;
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

	cuda_profile_step(&prof, "kernel");

	cuda_broad_phase_pair* h_pairs = NULL;
	if (count > 0) {
		h_pairs = (cuda_broad_phase_pair*)malloc(count * sizeof(cuda_broad_phase_pair));
		CUDA_CHECK(cudaMemcpy(h_pairs, s->d_pairs, count * sizeof(cuda_broad_phase_pair),
							  cudaMemcpyDeviceToHost));
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
	cuda_profile_log(&prof, &prof_acc, "naive", 10);

	return h_pairs;
}

// ---------------------------------------------------------------------------
// Phase 3: Find cell boundaries from sorted key array
// ---------------------------------------------------------------------------
__global__ void grid_find_boundaries_kernel(const grid_key_t* keys, uint32_t num_keys,
											uint32_t* cell_start, uint32_t* cell_end) {
	uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= num_keys) return;

	uint32_t cell_i = unpack_grid_key_cell(keys[i]);
	if (i == 0) { cell_start[cell_i] = 0; }
	else {
		uint32_t cell_prev = unpack_grid_key_cell(keys[i - 1]);
		if (cell_i != cell_prev) {
			cell_start[cell_i] = i;
			cell_end[cell_prev] = i;
		}
	}

	if (i == num_keys - 1) { cell_end[cell_i] = (uint32_t)(num_keys); }
}

// ---------------------------------------------------------------------------
// Strategy B Phase 1: Assign each body to its min-corner cell
// Outputs 32-bit Key (Cell) and 32-bit Value (Original Index)
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Strategy B Phase 2.5: Permute AABBs into linear sorted order
// ---------------------------------------------------------------------------
__global__ void permute_aabbs_kernel(const uint32_t* sorted_indices, const cuda_aabb* rigids,
									 int rigid_count, const cuda_aabb* statics, int static_count,
									 cuda_aabb* sorted_aabbs_out) {
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

// ---------------------------------------------------------------------------
// Strategy B Phase 3: Find cell boundaries
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Strategy B Phase 4: Each rigid searches its
// ---------------------------------------------------------------------------
__global__ void grid_b_test_pairs_kernel(const cuda_aabb* sorted_aabbs,
										 const uint32_t* sorted_indices, const uint32_t* cell_start,
										 const uint32_t* cell_end, uint32_t rigid_count,
										 uint32_t total_bodies, cuda_broad_phase_pair* pairs,
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

// ---------------------------------------------------------------------------
// Strategy B: Uniform grid broad phase (single-cell insert, multi-cell test)
// ---------------------------------------------------------------------------
extern "C" cuda_broad_phase_pair* cuda_broad_phase_grid_b(cuda_broad_phase_state* s,
														  const cuda_aabb* rigids, int rigid_count,
														  const cuda_aabb* statics,
														  int static_count, bool statics_changed,
														  size_t* out_count) {

	*out_count = 0;
	if (rigid_count == 0) return NULL;

	const int block_size = 1024;

	cuda_profile prof;
	cuda_profile_begin(&prof);

	// Page lock the host memory, so uploading data becomes faster
	ensure_host_memory_registered(rigids, rigid_count * sizeof(cuda_aabb), &s->h_last_rigids_ptr,
								  &s->h_last_rigids_size);
	ensure_host_memory_registered(statics, static_count * sizeof(cuda_aabb), &s->h_last_statics_ptr,
								  &s->h_last_statics_size);

	cuda_profile_step(&prof, "pagelock");

	// ---- Upload AABBs ----
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

	cuda_profile_step(&prof, "upload");

	// ---- Phase 1: Assign cells (one key per body) ----
	// key = cell index, value = original index
	int total_bodies = rigid_count + static_count;

	ensure_device_buffer((void**)&s->d_b_keys_in, &s->d_b_keys_capacity, total_bodies,
						 sizeof(uint32_t));
	ensure_device_buffer((void**)&s->d_b_keys_out, &s->d_b_keys_out_capacity, total_bodies,
						 sizeof(uint32_t));
	ensure_device_buffer((void**)&s->d_b_values_in, &s->d_b_values_capacity, total_bodies,
						 sizeof(uint32_t));
	ensure_device_buffer((void**)&s->d_b_values_out, &s->d_b_values_out_capacity, total_bodies,
						 sizeof(uint32_t));

	// write keys and values for rigids
	int grid_size_rigids = (rigid_count + block_size - 1) / block_size;
	grid_b_assign_kernel<<<grid_size_rigids, block_size>>>(s->d_rigids, rigid_count, s->d_b_keys_in,
														   s->d_b_values_in, 0);

	// same for statics in the same buffers after rigids
	if (static_count > 0) {
		int grid_size_statics = (static_count + block_size - 1) / block_size;
		grid_b_assign_kernel<<<grid_size_statics, block_size>>>(
			s->d_statics, static_count, s->d_b_keys_in, s->d_b_values_in, rigid_count);
	}

	cuda_profile_step(&prof, "assign");

	// ---- Phase 2: Radix Sort PAIRS: Sort by key (cell index), Move Values (body indices) ----
	size_t temp_bytes_needed = 0;
	// Query temp storage
	cub::DeviceRadixSort::SortPairs(NULL, temp_bytes_needed, s->d_b_keys_in, s->d_b_keys_out,
									s->d_b_values_in, s->d_b_values_out, total_bodies);

	ensure_device_buffer(&s->d_sort_temp, &s->d_sort_temp_size, temp_bytes_needed, 1);

	// Perform Sort
	CUDA_CHECK(cub::DeviceRadixSort::SortPairs(s->d_sort_temp, s->d_sort_temp_size, s->d_b_keys_in,
											   s->d_b_keys_out, s->d_b_values_in, s->d_b_values_out,
											   total_bodies));

	cuda_profile_step(&prof, "sort");

	// ---- Phase 2.5: Permute AABBs (Gather) ----
	ensure_device_buffer((void**)&s->d_b_sorted_aabbs, &s->d_b_sorted_aabbs_capacity, total_bodies,
						 sizeof(cuda_aabb));

	int grid_size = (total_bodies + block_size - 1) / block_size;
	permute_aabbs_kernel<<<grid_size, block_size>>>(s->d_b_values_out, // The sorted indices
													s->d_rigids, rigid_count, s->d_statics,
													static_count, s->d_b_sorted_aabbs);

	cuda_profile_step(&prof, "permute");

	// ---- Phase 3: Find cell boundaries (using sorted keys) ----
	CUDA_CHECK(cudaMemset(s->d_cell_start, 0xFF, GRID_NUM_CELLS * sizeof(uint32_t)));
	CUDA_CHECK(cudaMemset(s->d_cell_end, 0xFF, GRID_NUM_CELLS * sizeof(uint32_t)));

	grid_b_find_boundaries_kernel<<<grid_size, block_size>>>(s->d_b_keys_out, total_bodies,
															 s->d_cell_start, s->d_cell_end);

	cuda_profile_step(&prof, "bounds");

	// ---- Phase 4: Test pairs (Optimized Streaming) ----
	size_t pairs_needed = s->d_pairs_capacity;
	if (pairs_needed < 8 * (rigid_count + static_count))
		pairs_needed = 8 * (rigid_count + static_count);
	if (pairs_needed < 1024) pairs_needed = 1024;

	unsigned int count = 0;

	for (int attempt = 0; attempt < 2; ++attempt) {
		ensure_device_buffer((void**)&s->d_pairs, &s->d_pairs_capacity, pairs_needed,
							 sizeof(cuda_broad_phase_pair));

		unsigned int kernel_max = (s->d_pairs_capacity > (size_t)UINT32_MAX)
									  ? UINT32_MAX
									  : (unsigned int)s->d_pairs_capacity;

		CUDA_CHECK(cudaMemset(s->d_pair_count, 0, sizeof(unsigned int)));

		grid_b_test_pairs_kernel<<<grid_size, block_size>>>(
			s->d_b_sorted_aabbs, // Linear Geometry
			s->d_b_values_out,	 // Linear Indices (ID + Type info)
			s->d_cell_start, s->d_cell_end, (uint32_t)rigid_count, (uint32_t)total_bodies,
			s->d_pairs, s->d_pair_count, kernel_max);

		CUDA_CHECK(cudaDeviceSynchronize());
		CUDA_CHECK(
			cudaMemcpy(&count, s->d_pair_count, sizeof(unsigned int), cudaMemcpyDeviceToHost));

		if (count <= kernel_max) break;
		pairs_needed = (size_t)count;
	}

	cuda_profile_step(&prof, "tests");

	// ---- Readback ----
	cuda_broad_phase_pair* h_pairs = NULL;
	if (count > 0) {
		h_pairs = (cuda_broad_phase_pair*)malloc(count * sizeof(cuda_broad_phase_pair));
		CUDA_CHECK(cudaMemcpy(h_pairs, s->d_pairs, count * sizeof(cuda_broad_phase_pair),
							  cudaMemcpyDeviceToHost));
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

// ---------------------------------------------------------------------------
// Strategy A helpers (Unchanged except comments)
// ---------------------------------------------------------------------------

// Compute how many cells an AABB overlaps (1–8 for max 1m diameter, 1m cells)
__device__ static int grid_a_cell_count(const cuda_aabb* b) {
	int cx0 = cell_coord(b->min_x, GRID_ORIGIN_X, GRID_RES_X);
	int cy0 = cell_coord(b->min_y, GRID_ORIGIN_Y, GRID_RES_Y);
	int cz0 = cell_coord(b->min_z, GRID_ORIGIN_Z, GRID_RES_Z);
	int cx1 = cell_coord(b->max_x, GRID_ORIGIN_X, GRID_RES_X);
	int cy1 = cell_coord(b->max_y, GRID_ORIGIN_Y, GRID_RES_Y);
	int cz1 = cell_coord(b->max_z, GRID_ORIGIN_Z, GRID_RES_Z);
	return (cx1 - cx0 + 1) * (cy1 - cy0 + 1) * (cz1 - cz0 + 1);
}

// Compute the lowest cell index shared by two AABBs (for deduplication).
// Returns 0xFFFFFFFF if no overlap in cell ranges (should not happen for overlapping AABBs).
__device__ static uint32_t grid_a_lowest_common_cell(const cuda_aabb* a, const cuda_aabb* b) {
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

// ---------------------------------------------------------------------------
// Strategy A Phase 1a: Count keys per body (for prefix sum / offset computation)
// ---------------------------------------------------------------------------
__global__ void grid_a_count_kernel(const cuda_aabb* bodies, int body_count, unsigned int* counts) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= body_count) return;
	counts[i] = (unsigned int)grid_a_cell_count(&bodies[i]);
}

// ---------------------------------------------------------------------------
// Strategy A Phase 1c: Write keys using precomputed offsets
// ---------------------------------------------------------------------------
__global__ void grid_a_assign_kernel(const cuda_aabb* bodies, int body_count, uint8_t body_type,
									 const unsigned int* offsets, grid_key_t* keys_out) {
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
				keys_out[off++] = make_grid_key(cell_index(cx, cy, cz), body_type, (uint32_t)i);
			}
		}
	}
}

// ---------------------------------------------------------------------------
// Strategy A Phase 4: Same-cell pair test with lowest-common-cell dedup
// ---------------------------------------------------------------------------
__global__ void grid_a_test_pairs_kernel(const cuda_aabb* rigids, int rigid_count,
										 const cuda_aabb* statics, const grid_key_t* sorted_keys,
										 const uint32_t* cell_start, const uint32_t* cell_end,
										 cuda_broad_phase_pair* pairs, unsigned int* pair_count,
										 unsigned int max_pairs) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= rigid_count) return;

	cuda_aabb ri = rigids[i];

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
				uint32_t start = cell_start[ci];
				if (start == 0xFFFFFFFFu) continue;
				uint32_t end = cell_end[ci];

				for (uint32_t k = start; k < end; ++k) {
					uint32_t b_cell, b_index;
					uint8_t b_type;
					unpack_grid_key(sorted_keys[k], &b_cell, &b_type, &b_index);

					if (b_type == 1) {
						// Rigid vs rigid
						if (b_index <= i) continue;
						if (!aabb_overlap(&ri, &rigids[b_index])) continue;
						// Dedup: only emit from the lowest common cell
						if (ci != grid_a_lowest_common_cell(&ri, &rigids[b_index])) continue;
						unsigned int idx = atomicAdd(pair_count, 1);
						if (idx < max_pairs) pairs[idx] = {i, b_index, 1};
					}
					else {
						// Rigid vs static
						if (!aabb_overlap(&ri, &statics[b_index])) continue;
						// Dedup: only emit from the lowest common cell
						if (ci != grid_a_lowest_common_cell(&ri, &statics[b_index])) continue;
						unsigned int idx = atomicAdd(pair_count, 1);
						if (idx < max_pairs) pairs[idx] = {i, b_index, 0};
					}
				}
			}
		}
	}
}

// ---------------------------------------------------------------------------
// Strategy A: Uniform grid broad phase (multi-cell insert, same-cell test)
// ---------------------------------------------------------------------------
extern "C" cuda_broad_phase_pair* cuda_broad_phase_grid_a(cuda_broad_phase_state* s,
														  const cuda_aabb* rigids, int rigid_count,
														  const cuda_aabb* statics,
														  int static_count, bool statics_changed,
														  size_t* out_count) {

	*out_count = 0;
	if (rigid_count == 0) return NULL;

	const int block_size = 1024;

	cuda_profile prof;
	cuda_profile_begin(&prof);

	// Page lock the host memory, so uploading data becomes faster
	ensure_host_memory_registered(rigids, rigid_count * sizeof(cuda_aabb), &s->h_last_rigids_ptr,
								  &s->h_last_rigids_size);
	ensure_host_memory_registered(statics, static_count * sizeof(cuda_aabb), &s->h_last_statics_ptr,
								  &s->h_last_statics_size);

	cuda_profile_step(&prof, "pagelock");

	// ---- Upload AABBs ----
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

	cuda_profile_step(&prof, "upload");

	// ---- Phase 1: Multi-cell assignment ----
	// We need a prefix sum to compute offsets for each body's keys.
	// Step 1a: Count keys per body
	int total_bodies = rigid_count + static_count;
	ensure_device_buffer((void**)&s->d_counts, &s->d_counts_capacity, total_bodies,
						 sizeof(unsigned int));
	ensure_device_buffer((void**)&s->d_offsets, &s->d_offsets_capacity, total_bodies + 1,
						 sizeof(unsigned int));

	int grid_size_rigids = (rigid_count + block_size - 1) / block_size;
	grid_a_count_kernel<<<grid_size_rigids, block_size>>>(s->d_rigids, rigid_count, s->d_counts);
	if (static_count > 0) {
		int grid_size_statics = (static_count + block_size - 1) / block_size;
		grid_a_count_kernel<<<grid_size_statics, block_size>>>(s->d_statics, static_count,
															   s->d_counts + rigid_count);
	}

	cuda_profile_step(&prof, "1a: count");

	// Step 1b: Exclusive prefix sum to get offsets
	size_t scan_temp_size = 0;
	// because we pass NULL, the required allocation size is written to `scan_temp_size` and no
	// work is done.
	cub::DeviceScan::ExclusiveSum(NULL, scan_temp_size, s->d_counts, s->d_offsets, total_bodies);

	// Now that we know the temp size, we allocate it and run the scan
	ensure_device_buffer(&s->d_scan_temp, &s->d_scan_temp_size, scan_temp_size, 1);
	CUDA_CHECK(cub::DeviceScan::ExclusiveSum(s->d_scan_temp, scan_temp_size, s->d_counts,
											 s->d_offsets, total_bodies));

	// Compute total number of keys: offsets[last] + counts[last]
	unsigned int last_offset = 0, last_count = 0;
	CUDA_CHECK(cudaMemcpy(&last_offset, s->d_offsets + total_bodies - 1, sizeof(unsigned int),
						  cudaMemcpyDeviceToHost));
	CUDA_CHECK(cudaMemcpy(&last_count, s->d_counts + total_bodies - 1, sizeof(unsigned int),
						  cudaMemcpyDeviceToHost));
	unsigned int total_keys = last_offset + last_count;

	cuda_profile_step(&prof, "1b: scan");

	// Step 1c: Allocate keys and write them
	ensure_device_buffer((void**)&s->d_keys_in, &s->d_keys_capacity, total_keys,
						 sizeof(grid_key_t));
	ensure_device_buffer((void**)&s->d_keys_out, &s->d_keys_out_capacity, total_keys,
						 sizeof(grid_key_t));

	grid_a_assign_kernel<<<grid_size_rigids, block_size>>>(s->d_rigids, rigid_count, 1 /* rigid */,
														   s->d_offsets, s->d_keys_in);
	if (static_count > 0) {
		int grid_size_statics = (static_count + block_size - 1) / block_size;
		grid_a_assign_kernel<<<grid_size_statics, block_size>>>(
			s->d_statics, static_count, 0 /* static */, s->d_offsets + rigid_count, s->d_keys_in);
	}

	cuda_profile_step(&prof, "1c: assign");

	// ---- Phase 2: Radix sort keys by cell index ----
	size_t temp_bytes_needed = 0;
	cub::DeviceRadixSort::SortKeys(NULL, temp_bytes_needed, s->d_keys_in, s->d_keys_out,
								   (int)total_keys);
	ensure_device_buffer(&s->d_sort_temp, &s->d_sort_temp_size, temp_bytes_needed, 1);
	CUDA_CHECK(cub::DeviceRadixSort::SortKeys(s->d_sort_temp, s->d_sort_temp_size, s->d_keys_in,
											  s->d_keys_out, (int)total_keys));

	cuda_profile_step(&prof, "sort");

	// ---- Phase 3: Find cell boundaries ----
	CUDA_CHECK(cudaMemset(s->d_cell_start, 0xFF, GRID_NUM_CELLS * sizeof(uint32_t)));
	CUDA_CHECK(cudaMemset(s->d_cell_end, 0xFF, GRID_NUM_CELLS * sizeof(uint32_t)));

	int grid_size_keys = (total_keys + block_size - 1) / block_size;
	grid_find_boundaries_kernel<<<grid_size_keys, block_size>>>(s->d_keys_out, total_keys,
																s->d_cell_start, s->d_cell_end);

	cuda_profile_step(&prof, "bounds");

	// ---- Phase 4: Test pairs (same-cell only, with lowest-common-cell dedup) ----
	size_t pairs_needed = s->d_pairs_capacity;
	if (pairs_needed < 8 * (rigid_count + static_count))
		pairs_needed = 8 * (rigid_count + static_count);
	if (pairs_needed < 1024) pairs_needed = 1024;

	unsigned int count = 0;

	for (int attempt = 0; attempt < 2; ++attempt) {
		ensure_device_buffer((void**)&s->d_pairs, &s->d_pairs_capacity, pairs_needed,
							 sizeof(cuda_broad_phase_pair));

		unsigned int kernel_max = (s->d_pairs_capacity > (size_t)UINT32_MAX)
									  ? UINT32_MAX
									  : (unsigned int)s->d_pairs_capacity;

		CUDA_CHECK(cudaMemset(s->d_pair_count, 0, sizeof(unsigned int)));
		grid_a_test_pairs_kernel<<<grid_size_rigids, block_size>>>(
			s->d_rigids, rigid_count, s->d_statics, s->d_keys_out, s->d_cell_start, s->d_cell_end,
			s->d_pairs, s->d_pair_count, kernel_max);
		CUDA_CHECK(cudaDeviceSynchronize());
		CUDA_CHECK(
			cudaMemcpy(&count, s->d_pair_count, sizeof(unsigned int), cudaMemcpyDeviceToHost));

		if (count <= kernel_max) break;
		pairs_needed = (size_t)count;
	}

	cuda_profile_step(&prof, "tests");

	// ---- Readback ----
	cuda_broad_phase_pair* h_pairs = NULL;
	if (count > 0) {
		h_pairs = (cuda_broad_phase_pair*)malloc(count * sizeof(cuda_broad_phase_pair));
		CUDA_CHECK(cudaMemcpy(h_pairs, s->d_pairs, count * sizeof(cuda_broad_phase_pair),
							  cudaMemcpyDeviceToHost));
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
