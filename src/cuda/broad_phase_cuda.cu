#include "broad_phase_cuda.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Device helpers
// ---------------------------------------------------------------------------

// Body type constants matching the C enum (STATIC_BODY = 0, RIGID_BODY = 1)
#define TYPE_STATIC 0
#define TYPE_RIGID 1

// Returns 1 if the two AABBs overlap on all three axes, 0 otherwise.
__device__ static int aabb_overlap(float a_min_x, float a_max_x, float a_min_y, float a_max_y,
								   float a_min_z, float a_max_z, float b_min_x, float b_max_x,
								   float b_min_y, float b_max_y, float b_min_z, float b_max_z) {
	return (a_max_x >= b_min_x) && (a_min_x <= b_max_x) && (a_max_y >= b_min_y) &&
		   (a_min_y <= b_max_y) && (a_max_z >= b_min_z) && (a_min_z <= b_max_z);
}

// ---------------------------------------------------------------------------
// Kernel 1: Rigid vs Rigid  (upper-triangle, no self-test)
//   Total work items = rigid_count * (rigid_count - 1) / 2
//   We map a linear index to (i, j) with i < j.
// ---------------------------------------------------------------------------
__global__ void
kernel_rigid_rigid(const float* __restrict__ r_min_x, const float* __restrict__ r_max_x,
				   const float* __restrict__ r_min_y, const float* __restrict__ r_max_y,
				   const float* __restrict__ r_min_z, const float* __restrict__ r_max_z,
				   const uint32_t* __restrict__ r_indices, size_t rigid_count,
				   uint32_t* __restrict__ flags) // 1 flag per work item
{
	size_t tid = blockIdx.x * (size_t)blockDim.x + threadIdx.x;
	size_t total = rigid_count * (rigid_count - 1) / 2;
	if (tid >= total) return;

	// Map linear index -> upper-triangle (i, j) with i < j.
	// row i starts at cumulative offset i*rigid_count - i*(i+1)/2
	// We solve for i using the quadratic formula.
	// i = floor( (2*N - 1 - sqrt((2*N-1)^2 - 8*tid)) / 2 )
	double N = (double)rigid_count;
	double discriminant = (2.0 * N - 1.0) * (2.0 * N - 1.0) - 8.0 * (double)tid;
	size_t i = (size_t)((2.0 * N - 1.0 - sqrt(discriminant)) / 2.0);
	// j = tid - offset_for_row_i + (i + 1)
	size_t offset = i * rigid_count - i * (i + 1) / 2;
	size_t j = tid - offset + (i + 1);
	// Clamp in case of floating-point rounding
	if (j <= i) j = i + 1;
	if (i >= rigid_count || j >= rigid_count) return;

	flags[tid] = (uint32_t)aabb_overlap(r_min_x[i], r_max_x[i], r_min_y[i], r_max_y[i], r_min_z[i],
										r_max_z[i], r_min_x[j], r_max_x[j], r_min_y[j], r_max_y[j],
										r_min_z[j], r_max_z[j]);
}

// ---------------------------------------------------------------------------
// Kernel 2: Rigid vs Static  (full NxM grid)
//   Total work items = rigid_count * static_count
// ---------------------------------------------------------------------------
__global__ void
kernel_rigid_static(const float* __restrict__ r_min_x, const float* __restrict__ r_max_x,
					const float* __restrict__ r_min_y, const float* __restrict__ r_max_y,
					const float* __restrict__ r_min_z, const float* __restrict__ r_max_z,
					const float* __restrict__ s_min_x, const float* __restrict__ s_max_x,
					const float* __restrict__ s_min_y, const float* __restrict__ s_max_y,
					const float* __restrict__ s_min_z, const float* __restrict__ s_max_z,
					size_t rigid_count, size_t static_count,
					uint32_t* __restrict__ flags) // 1 flag per work item
{
	size_t tid = blockIdx.x * (size_t)blockDim.x + threadIdx.x;
	size_t total = rigid_count * static_count;
	if (tid >= total) return;

	size_t i = tid / static_count; // rigid index
	size_t j = tid % static_count; // static index

	flags[tid] = (uint32_t)aabb_overlap(r_min_x[i], r_max_x[i], r_min_y[i], r_max_y[i], r_min_z[i],
										r_max_z[i], s_min_x[j], s_max_x[j], s_min_y[j], s_max_y[j],
										s_min_z[j], s_max_z[j]);
}

// ---------------------------------------------------------------------------
// Host helper: upload one SoA proxy set to the device.
// ---------------------------------------------------------------------------
typedef struct {
	float *min_x, *max_x, *min_y, *max_y, *min_z, *max_z;
	uint32_t* indices;
	size_t count;
} device_proxies;

static device_proxies upload_proxies(const cuda_broad_phase_proxies_soa* src) {
	device_proxies d;
	d.count = src->count;
	size_t fb = src->count * sizeof(float);
	size_t ib = src->count * sizeof(uint32_t);

	cudaMalloc(&d.min_x, fb);
	cudaMemcpy(d.min_x, src->min_x, fb, cudaMemcpyHostToDevice);
	cudaMalloc(&d.max_x, fb);
	cudaMemcpy(d.max_x, src->max_x, fb, cudaMemcpyHostToDevice);
	cudaMalloc(&d.min_y, fb);
	cudaMemcpy(d.min_y, src->min_y, fb, cudaMemcpyHostToDevice);
	cudaMalloc(&d.max_y, fb);
	cudaMemcpy(d.max_y, src->max_y, fb, cudaMemcpyHostToDevice);
	cudaMalloc(&d.min_z, fb);
	cudaMemcpy(d.min_z, src->min_z, fb, cudaMemcpyHostToDevice);
	cudaMalloc(&d.max_z, fb);
	cudaMemcpy(d.max_z, src->max_z, fb, cudaMemcpyHostToDevice);
	cudaMalloc(&d.indices, ib);
	cudaMemcpy(d.indices, src->indices, ib, cudaMemcpyHostToDevice);

	return d;
}

static void free_device_proxies(device_proxies* d) {
	cudaFree(d->min_x);
	cudaFree(d->max_x);
	cudaFree(d->min_y);
	cudaFree(d->max_y);
	cudaFree(d->min_z);
	cudaFree(d->max_z);
	cudaFree(d->indices);
}

// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------
extern "C" cuda_broad_phase_pair* broad_phase_cuda(const cuda_broad_phase_proxies_soa* rigids,
												   const cuda_broad_phase_proxies_soa* statics,
												   size_t* out_count) {
	*out_count = 0;

	size_t rc = rigids->count;
	size_t sc = statics->count;

	// Nothing to do if there are no rigid bodies
	if (rc == 0) return NULL;

	size_t rr_total = rc * (rc - 1) / 2; // rigid-rigid pairs to test
	size_t rs_total = rc * sc;			 // rigid-static pairs to test
	size_t grand_total = rr_total + rs_total;
	if (grand_total == 0) return NULL;

	// Upload proxy data to device
	device_proxies d_rigids = upload_proxies(rigids);
	device_proxies d_statics = {0};
	if (sc > 0) d_statics = upload_proxies(statics);

	const int block_size = 256;

	// --- Rigid vs Rigid ---
	uint32_t* d_rr_flags = NULL;
	uint32_t* h_rr_flags = NULL;
	if (rr_total > 0) {
		cudaMalloc(&d_rr_flags, rr_total * sizeof(uint32_t));
		cudaMemset(d_rr_flags, 0, rr_total * sizeof(uint32_t));

		int grid = (int)((rr_total + block_size - 1) / block_size);
		kernel_rigid_rigid<<<grid, block_size>>>(d_rigids.min_x, d_rigids.max_x, d_rigids.min_y,
												 d_rigids.max_y, d_rigids.min_z, d_rigids.max_z,
												 d_rigids.indices, rc, d_rr_flags);

		h_rr_flags = (uint32_t*)malloc(rr_total * sizeof(uint32_t));
		cudaMemcpy(h_rr_flags, d_rr_flags, rr_total * sizeof(uint32_t), cudaMemcpyDeviceToHost);
		cudaFree(d_rr_flags);
	}

	// --- Rigid vs Static ---
	uint32_t* d_rs_flags = NULL;
	uint32_t* h_rs_flags = NULL;
	if (rs_total > 0) {
		cudaMalloc(&d_rs_flags, rs_total * sizeof(uint32_t));
		cudaMemset(d_rs_flags, 0, rs_total * sizeof(uint32_t));

		int grid = (int)((rs_total + block_size - 1) / block_size);
		kernel_rigid_static<<<grid, block_size>>>(
			d_rigids.min_x, d_rigids.max_x, d_rigids.min_y, d_rigids.max_y, d_rigids.min_z,
			d_rigids.max_z, d_statics.min_x, d_statics.max_x, d_statics.min_y, d_statics.max_y,
			d_statics.min_z, d_statics.max_z, rc, sc, d_rs_flags);

		h_rs_flags = (uint32_t*)malloc(rs_total * sizeof(uint32_t));
		cudaMemcpy(h_rs_flags, d_rs_flags, rs_total * sizeof(uint32_t), cudaMemcpyDeviceToHost);
		cudaFree(d_rs_flags);
	}

	// Free device proxy memory
	free_device_proxies(&d_rigids);
	if (sc > 0) free_device_proxies(&d_statics);

	// --- Count hits and build output on host ---
	// We also need the host-side index arrays for pair construction.
	// (They are still in the original caller-owned SoA, so just read from there.)

	size_t hit_count = 0;
	for (size_t t = 0; t < rr_total; ++t)
		hit_count += h_rr_flags[t];
	for (size_t t = 0; t < rs_total; ++t)
		hit_count += h_rs_flags[t];

	if (hit_count == 0) {
		free(h_rr_flags);
		free(h_rs_flags);
		return NULL;
	}

	cuda_broad_phase_pair* pairs =
		(cuda_broad_phase_pair*)malloc(hit_count * sizeof(cuda_broad_phase_pair));
	size_t idx = 0;

	// Reconstruct rigid-rigid pairs (same upper-triangle mapping as the kernel)
	if (h_rr_flags) {
		size_t t = 0;
		for (size_t i = 0; i < rc; ++i) {
			for (size_t j = i + 1; j < rc; ++j, ++t) {
				if (h_rr_flags[t]) {
					pairs[idx].a_type = TYPE_RIGID;
					pairs[idx].a_index = rigids->indices[i];
					pairs[idx].b_type = TYPE_RIGID;
					pairs[idx].b_index = rigids->indices[j];
					idx++;
				}
			}
		}
		free(h_rr_flags);
	}

	// Reconstruct rigid-static pairs
	if (h_rs_flags) {
		for (size_t t = 0; t < rs_total; ++t) {
			if (h_rs_flags[t]) {
				size_t i = t / sc;
				size_t j = t % sc;
				pairs[idx].a_type = TYPE_RIGID;
				pairs[idx].a_index = rigids->indices[i];
				pairs[idx].b_type = TYPE_STATIC;
				pairs[idx].b_index = statics->indices[j];
				idx++;
			}
		}
		free(h_rs_flags);
	}

	*out_count = hit_count;
	return pairs;
}
