#include "broad_phase_cuda.h"
#include "cuda_profile.h"

#include <cub/device/device_radix_sort.cuh>

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

	// Grid buffers
	grid_key_t* d_keys_in;
	grid_key_t* d_keys_out;
	size_t d_keys_capacity;
	size_t d_keys_out_capacity;

	uint32_t* d_cell_start; // size = GRID_NUM_CELLS
	uint32_t* d_cell_end;	// size = GRID_NUM_CELLS

	void* d_sort_temp;
	size_t d_sort_temp_size;
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
// Phase 1: Assign each body to its min-corner cell -> produce one key per body
// ---------------------------------------------------------------------------
__global__ void grid_assign_kernel(const cuda_aabb* bodies, int body_count, uint8_t body_type,
								   grid_key_t* keys_out, int key_offset) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= body_count) return;

	cuda_aabb b = bodies[i];
	int cx = cell_coord(b.min_x, GRID_ORIGIN_X, GRID_RES_X);
	int cy = cell_coord(b.min_y, GRID_ORIGIN_Y, GRID_RES_Y);
	int cz = cell_coord(b.min_z, GRID_ORIGIN_Z, GRID_RES_Z);

	keys_out[key_offset + i] = make_grid_key(cell_index(cx, cy, cz), body_type, (uint32_t)i);
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
// Phase 4: Each rigid searches its 3x3x3 neighborhood for collisions
// ---------------------------------------------------------------------------
__global__ void grid_test_pairs_kernel(const cuda_aabb* rigids, int rigid_count,
									   const cuda_aabb* statics, const grid_key_t* sorted_keys,
									   const uint32_t* cell_start, const uint32_t* cell_end,
									   cuda_broad_phase_pair* pairs, unsigned int* pair_count,
									   unsigned int max_pairs) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= rigid_count) return;

	cuda_aabb ri = rigids[i];

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

	for (int nz = cz_lo; nz <= cz_hi; ++nz) {
		for (int ny = cy_lo; ny <= cy_hi; ++ny) {
			for (int nx = cx_lo; nx <= cx_hi; ++nx) {
				uint32_t ci = cell_index(nx, ny, nz);
				uint32_t start = cell_start[ci];	// major bottleneck - L2 misses
				if (start == 0xFFFFFFFFu) continue; // empty cell
				uint32_t end = cell_end[ci];		// major bottleneck - L2 misses

				for (uint32_t k = start; k < end; ++k) {
					uint32_t b_cell, b_index;
					uint8_t b_type;
					unpack_grid_key(sorted_keys[k], &b_cell, &b_type, &b_index);

					if (b_type == 1) {
						// Rigid vs rigid: only emit if i < b_index
						if (b_index <= i) continue;
						if (aabb_overlap(&ri, &rigids[b_index])) {
							unsigned int idx = atomicAdd(pair_count, 1);
							if (idx < max_pairs) pairs[idx] = {i, b_index, 1};
						}
					}
					else {
						// Rigid vs static
						if (aabb_overlap(&ri, &statics[b_index])) {
							unsigned int idx = atomicAdd(pair_count, 1);
							if (idx < max_pairs) pairs[idx] = {i, b_index, 0};
						}
					}
				}
			}
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
	if (s->d_cell_start) cudaFree(s->d_cell_start);
	if (s->d_cell_end) cudaFree(s->d_cell_end);
	if (s->d_sort_temp) cudaFree(s->d_sort_temp);
	free(s);
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
// Uniform grid broad phase
// ---------------------------------------------------------------------------
extern "C" cuda_broad_phase_pair* cuda_broad_phase_grid(cuda_broad_phase_state* s,
														const cuda_aabb* rigids, int rigid_count,
														const cuda_aabb* statics, int static_count,
														bool statics_changed, size_t* out_count) {

	assert((size_t)GRID_RES_X * (size_t)GRID_RES_Y * (size_t)GRID_RES_Z <= (size_t)UINT32_MAX &&
		   "Grid is too large.");

	*out_count = 0;
	if (rigid_count == 0) return NULL;

	const int block_size = 1024;

	cuda_profile prof;
	cuda_profile_begin(&prof);

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
	int total_bodies = rigid_count + static_count;
	ensure_device_buffer((void**)&s->d_keys_in, &s->d_keys_capacity, total_bodies,
						 sizeof(grid_key_t));
	ensure_device_buffer((void**)&s->d_keys_out, &s->d_keys_out_capacity, total_bodies,
						 sizeof(grid_key_t));
	// rigids
	int grid_size_rigids = (rigid_count + block_size - 1) / block_size;
	grid_assign_kernel<<<grid_size_rigids, block_size>>>(s->d_rigids, rigid_count, 1 /* rigid */,
														 s->d_keys_in, 0);
	// statics
	if (static_count > 0) {
		int grid_size_statics = (static_count + block_size - 1) / block_size;
		grid_assign_kernel<<<grid_size_statics, block_size>>>(
			s->d_statics, static_count, 0 /* static */, s->d_keys_in, rigid_count);
	}

	cuda_profile_step(&prof, "assign");

	// ---- Phase 2: Radix sort keys by cell index ----
	size_t temp_bytes_needed = 0;
	cub::DeviceRadixSort::SortKeys(NULL, temp_bytes_needed, s->d_keys_in, s->d_keys_out,
								   total_bodies);
	ensure_device_buffer(&s->d_sort_temp, &s->d_sort_temp_size, temp_bytes_needed, 1);
	CUDA_CHECK(cub::DeviceRadixSort::SortKeys(s->d_sort_temp, s->d_sort_temp_size, s->d_keys_in,
											  s->d_keys_out, total_bodies));

	cuda_profile_step(&prof, "sort");

	// ---- Phase 3: Find cell boundaries ----
	CUDA_CHECK(cudaMemset(s->d_cell_start, 0xFF, GRID_NUM_CELLS * sizeof(uint32_t)));
	CUDA_CHECK(cudaMemset(s->d_cell_end, 0xFF, GRID_NUM_CELLS * sizeof(uint32_t)));

	int grid_size = (total_bodies + block_size - 1) / block_size; // for phase 3 & 4
	grid_find_boundaries_kernel<<<grid_size, block_size>>>(s->d_keys_out, total_bodies,
														   s->d_cell_start, s->d_cell_end);

	cuda_profile_step(&prof, "bounds");

	// ---- Phase 4: Test pairs in 3x3x3 neighborhood ----
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
		grid_test_pairs_kernel<<<grid_size, block_size>>>(
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
	cuda_profile_log(&prof, &prof_acc, "grid", 10);

	return h_pairs;
}
