#include "profiler.h"
#include "tics_internal.h"
#include "tics_math.h"

#include <immintrin.h>
#include <omp.h>
#include <stb_ds.h>

#include <float.h>
#include <stdint.h>
#include <stdlib.h>

aabb calculate_aabb(const shape_data* shape, tics_transform t) {
	// Initialize with (inverted) infinity
	aabb box = {.min = {FLT_MAX, FLT_MAX, FLT_MAX}, .max = {-FLT_MAX, -FLT_MAX, -FLT_MAX}};

	switch (shape->type) {
	case TICS_SHAPE_SPHERE: {
		float r = shape->data.sphere.radius;

		// Rotate the local center offset by the object's rotation
		tics_vec3 rotated_center = quat_rotate_vec3(shape->data.sphere.center, t.rotation);

		// Add the rotated offset to the object's world position
		tics_vec3 world_center = vec3_add(t.position, rotated_center);

		// Expand bounding box from the calculated world center
		box.min = (tics_vec3){world_center.x - r, world_center.y - r, world_center.z - r};
		box.max = (tics_vec3){world_center.x + r, world_center.y + r, world_center.z + r};
	} break;

	case TICS_SHAPE_CONVEX: {
		tics_vec3* verts = shape->data.convex.vertices;
		size_t count = shape->data.convex.count;

		for (size_t i = 0; i < count; i++) {
			// Transform vertex: (Rot * v) + Pos
			tics_vec3 world_v = vec3_add(t.position, quat_rotate_vec3(verts[i], t.rotation));
			// Min
			if (world_v.x < box.min.x) box.min.x = world_v.x;
			if (world_v.y < box.min.y) box.min.y = world_v.y;
			if (world_v.z < box.min.z) box.min.z = world_v.z;
			// Max
			if (world_v.x > box.max.x) box.max.x = world_v.x;
			if (world_v.y > box.max.y) box.max.y = world_v.y;
			if (world_v.z > box.max.z) box.max.z = world_v.z;
		}
	} break;
	} // switch

	return box;
}

broad_phase_proxy* build_rigid_proxies(const tics_world* world) {
	broad_phase_proxy* proxies = NULL; // stb_ds array

	size_t count = arrlen(world->rigid_bodies);
	for (size_t i = 0; i < count; i++) {
		rigid_body_data* rb = &world->rigid_bodies[i];
		// Recalculate AABB every frame because rigid bodies move
		aabb box = calculate_aabb(&rb->shape, rb->transform);
		broad_phase_proxy p = {.aabb = box, .index = (uint32_t)i};
		arrput(proxies, p);
	}

	return proxies;
}

broad_phase_proxy* build_static_proxies(const tics_world* world) {
	broad_phase_proxy* proxies = NULL; // stb_ds array

	size_t count = arrlen(world->static_bodies);
	for (size_t i = 0; i < count; i++) {
		static_body_data* sb = &world->static_bodies[i];
		// Optimization: Static bodies do not move. Their AABB is pre-calculated
		broad_phase_proxy p = {.aabb = sb->aabb, .index = (uint32_t)i};
		arrput(proxies, p);
	}

	return proxies;
}

broad_phase_proxy_typed* build_typed_proxies(const tics_world* world) {
	size_t static_count = arrlen(world->static_bodies);
	size_t rigid_count = arrlen(world->rigid_bodies);

	broad_phase_proxy_typed* proxies = NULL;
	// Pre-allocate list to avoid resizing
	arrsetcap(proxies, rigid_count + static_count);

	for (size_t i = 0; i < rigid_count; ++i) {
		rigid_body_data* rb = &world->rigid_bodies[i];
		// Recalculate AABB every frame because rigid bodies move
		aabb box = calculate_aabb(&rb->shape, rb->transform);
		broad_phase_proxy_typed e = {box, (uint32_t)i, RIGID_BODY};
		arrput(proxies, e);
	}

	for (size_t i = 0; i < static_count; ++i) {
		static_body_data* sb = &world->static_bodies[i];
		// Optimization: Static bodies do not move. Their AABB is pre-calculated
		broad_phase_proxy_typed e = {sb->aabb, (uint32_t)i, STATIC_BODY};
		arrput(proxies, e);
	}

	return proxies;
}

void build_packed_rigid_aabbs(tics_world* world) {
	size_t count = arrlen(world->rigid_bodies);

	// Resize persistent buffer to match current body count (no-op if size unchanged)
	arrsetlen(world->gpu_rigid_aabbs, count);

	for (size_t i = 0; i < count; ++i) {
		rigid_body_data* rb = &world->rigid_bodies[i];
		aabb box = calculate_aabb(&rb->shape, rb->transform);
		world->gpu_rigid_aabbs[i] = (packed_aabb){
			.min_x = box.min.x,
			.max_x = box.max.x,
			.min_y = box.min.y,
			.max_y = box.max.y,
			.min_z = box.min.z,
			.max_z = box.max.z,
		};
	}
}

void build_packed_static_aabbs(tics_world* world) {
	size_t count = arrlen(world->static_bodies);

	arrsetlen(world->gpu_static_aabbs, count);

	for (size_t i = 0; i < count; ++i) {
		static_body_data* sb = &world->static_bodies[i];
		// Static AABBs are pre-calculated, no need to call calculate_aabb
		world->gpu_static_aabbs[i] = (packed_aabb){
			.min_x = sb->aabb.min.x,
			.max_x = sb->aabb.max.x,
			.min_y = sb->aabb.min.y,
			.max_y = sb->aabb.max.y,
			.min_z = sb->aabb.min.z,
			.max_z = sb->aabb.max.z,
		};
	}
}

// Sort by AABB min.x ascending
static int compare_sap_entries(const void* a, const void* b) {
	const broad_phase_proxy_typed* ea = (const broad_phase_proxy_typed*)a;
	const broad_phase_proxy_typed* eb = (const broad_phase_proxy_typed*)b;

	if (ea->aabb.min.x < eb->aabb.min.x) return -1;
	if (ea->aabb.min.x > eb->aabb.min.x) return 1;
	return 0;
}

void update_typed_proxies(tics_world* world) {
	size_t rigid_count = arrlen(world->rigid_bodies);
	size_t static_count = arrlen(world->static_bodies);

	// Case 1: Rebuild
	if (world->broadphase_dirty) {
		// Clear the existing list but keep memory capacity if possible
		arrsetlen(world->proxies, 0);
		// Pre-allocate list to avoid resizing
		arrsetcap(world->proxies, rigid_count + static_count);

		// (Re)populate Rigids
		for (size_t i = 0; i < rigid_count; ++i) {
			rigid_body_data* rb = &world->rigid_bodies[i];
			aabb box = calculate_aabb(&rb->shape, rb->transform);
			broad_phase_proxy_typed p = {box, (uint32_t)i, RIGID_BODY};
			arrput(world->proxies, p);
		}

		// (Re)populate Statics
		for (size_t i = 0; i < static_count; ++i) {
			static_body_data* sb = &world->static_bodies[i];
			broad_phase_proxy_typed p = {sb->aabb, (uint32_t)i, STATIC_BODY};
			arrput(world->proxies, p);
		}

		// Sort the combined list along the X-axis
		// qsort works perfectly on stb_ds arrays since they are contiguous blocks of memory.
		qsort(world->proxies, arrlen(world->proxies), sizeof(broad_phase_proxy_typed),
			  compare_sap_entries);

		// (Re)build the Lookup Map
		// Map size matches rigid body count (we don't map statics as they don't update)
		arrsetlen(world->proxy_map, rigid_count);
		size_t count = arrlen(world->proxies);
		for (size_t i = 0; i < count; ++i) {
			if (world->proxies[i].type == RIGID_BODY) {
				// The rigid body with ID [index] is located at [i] in the sorted list
				world->proxy_map[world->proxies[i].index] = (uint32_t)i;
			}
		}

		// Reset flag
		world->broadphase_dirty = false;

		return;
	}

	// Case 2: Movement Only (Update In-Place)
	// The list is sorted by X. We iterate the PROXY list (not the body list).
	// This preserves the sorted order for Insertion Sort.

	// OPTIMIZATION: Linear Read, Random Write
	// We iterate the rigid bodies linearly to ensure perfect cache usage for the heavy physics data
	// reads. We write to the proxy list using the map.
	// Note: We skip statics entirely as they don't move.
	for (size_t i = 0; i < rigid_count; ++i) {
		rigid_body_data* rb = &world->rigid_bodies[i];

		// Calculate AABBs with linear memory access
		aabb box = calculate_aabb(&rb->shape, rb->transform);

		// Update the AABB (Random Write)
		// Random Writes are faster than Random Reads (due to CPU Store Buffers vs Load Stalls).
		uint32_t proxy_index = world->proxy_map[i];
		world->proxies[proxy_index].aabb = box;
	}
}

broad_phase_proxies_soa build_rigid_proxies_soa(const tics_world* world) {
	broad_phase_proxies_soa soa = {0}; // Initialize pointers to NULL
	size_t count = arrlen(world->rigid_bodies);

	if (count == 0) return soa;

	// Optimization: Pre-allocate memory to avoid resizing during the loop
	arrsetcap(soa.min_x, count);
	arrsetcap(soa.max_x, count);
	arrsetcap(soa.min_y, count);
	arrsetcap(soa.max_y, count);
	arrsetcap(soa.min_z, count);
	arrsetcap(soa.max_z, count);
	arrsetcap(soa.indices, count);

	for (size_t i = 0; i < count; i++) {
		rigid_body_data* rb = &world->rigid_bodies[i];

		// Recalculate AABB
		aabb box = calculate_aabb(&rb->shape, rb->transform);

		// Fill separate arrays
		arrput(soa.min_x, box.min.x);
		arrput(soa.max_x, box.max.x);
		arrput(soa.min_y, box.min.y);
		arrput(soa.max_y, box.max.y);
		arrput(soa.min_z, box.min.z);
		arrput(soa.max_z, box.max.z);
		arrput(soa.indices, (uint32_t)i);
	}

	soa.count = count;
	return soa;
}

broad_phase_proxies_soa build_static_proxies_soa(const tics_world* world) {
	broad_phase_proxies_soa soa = {0}; // Initialize pointers to NULL
	size_t count = arrlen(world->static_bodies);

	if (count == 0) return soa;

	// Optimization: Pre-allocate memory
	arrsetcap(soa.min_x, count);
	arrsetcap(soa.max_x, count);
	arrsetcap(soa.min_y, count);
	arrsetcap(soa.max_y, count);
	arrsetcap(soa.min_z, count);
	arrsetcap(soa.max_z, count);
	arrsetcap(soa.indices, count);

	for (size_t i = 0; i < count; i++) {
		static_body_data* sb = &world->static_bodies[i];

		// Static bodies have pre-calculated AABBs
		arrput(soa.min_x, sb->aabb.min.x);
		arrput(soa.max_x, sb->aabb.max.x);
		arrput(soa.min_y, sb->aabb.min.y);
		arrput(soa.max_y, sb->aabb.max.y);
		arrput(soa.min_z, sb->aabb.min.z);
		arrput(soa.max_z, sb->aabb.max.z);
		arrput(soa.indices, (uint32_t)i);
	}

	soa.count = count;
	return soa;
}

void free_proxies_soa(broad_phase_proxies_soa* soa) {
	arrfree(soa->min_x);
	arrfree(soa->max_x);
	arrfree(soa->min_y);
	arrfree(soa->max_y);
	arrfree(soa->min_z);
	arrfree(soa->max_z);
	arrfree(soa->indices);
	memset(soa, 0, sizeof(broad_phase_proxies_soa));
}

broad_phase_pair* broad_phase_naive(const broad_phase_proxy* rigids, size_t rigid_count,
									const broad_phase_proxy* statics, size_t static_count) {
	broad_phase_pair* pairs = NULL;

	// Rigid Body vs Rigid Body
	// We iterate j starting from i + 1 to avoid duplicates (checking A vs B but not B vs A)
	for (size_t i = 0; i < rigid_count; ++i) {
		for (size_t j = i + 1; j < rigid_count; ++j) {

			if (rigids[i].aabb.max.x < rigids[j].aabb.min.x ||
				rigids[i].aabb.min.x > rigids[j].aabb.max.x ||
				rigids[i].aabb.max.y < rigids[j].aabb.min.y ||
				rigids[i].aabb.min.y > rigids[j].aabb.max.y ||
				rigids[i].aabb.max.z < rigids[j].aabb.min.z ||
				rigids[i].aabb.min.z > rigids[j].aabb.max.z) {
				continue;
			}

			broad_phase_pair p;

			p.a.type = RIGID_BODY;
			p.a.index = rigids[i].index;

			p.b.type = RIGID_BODY;
			p.b.index = rigids[j].index;

			arrput(pairs, p);
		}
	}

	// Rigid Body vs Static Body
	// Check every rigid body against every static body
	for (size_t i = 0; i < rigid_count; ++i) {
		for (size_t j = 0; j < static_count; ++j) {

			if (rigids[i].aabb.max.x < statics[j].aabb.min.x ||
				rigids[i].aabb.min.x > statics[j].aabb.max.x ||
				rigids[i].aabb.max.y < statics[j].aabb.min.y ||
				rigids[i].aabb.min.y > statics[j].aabb.max.y ||
				rigids[i].aabb.max.z < statics[j].aabb.min.z ||
				rigids[i].aabb.min.z > statics[j].aabb.max.z) {
				continue;
			}

			broad_phase_pair p;

			p.a.type = RIGID_BODY;
			p.a.index = rigids[i].index;

			p.b.type = STATIC_BODY;
			p.b.index = statics[j].index;

			arrput(pairs, p);
		}
	}

	return pairs;
}

broad_phase_pair* broad_phase_naive_parallel(const broad_phase_proxy* rigids, size_t rigid_count,
											 const broad_phase_proxy* statics,
											 size_t static_count) {
	broad_phase_pair* pairs = NULL;

	// Uncomment this to disable multi-threading
	// omp_set_num_threads(1);

	// Get the maximum number of threads available
	int max_threads = omp_get_max_threads();

	// Allocate an array to hold the thread-local dynamic arrays.
	// We use calloc to ensure pointers are initially NULL for stb_ds.
	broad_phase_pair** thread_buffers =
		(broad_phase_pair**)calloc(max_threads, sizeof(broad_phase_pair*));

	// Threads are spawned here
	// When calling `#pragma omp for` later, it does not spawn new threads; it splits the loop
	// iterations among the existing team of threads created by the outer parallel block.
#pragma omp parallel
	{
		int tid = omp_get_thread_num();
		broad_phase_pair* local_pairs = NULL; // This thread's private list

		// Rigid Body vs Rigid Body
		// We use schedule(dynamic) because the inner loop size (j=i+1) shrinks as i increases.
		// Static scheduling would give the first thread much more work than the last.
#pragma omp for schedule(dynamic)
		for (size_t i = 0; i < rigid_count; ++i) {
			for (size_t j = i + 1; j < rigid_count; ++j) {

				// Inline AABB check
				if (rigids[i].aabb.max.x < rigids[j].aabb.min.x ||
					rigids[i].aabb.min.x > rigids[j].aabb.max.x ||
					rigids[i].aabb.max.y < rigids[j].aabb.min.y ||
					rigids[i].aabb.min.y > rigids[j].aabb.max.y ||
					rigids[i].aabb.max.z < rigids[j].aabb.min.z ||
					rigids[i].aabb.min.z > rigids[j].aabb.max.z) {
					continue;
				}

				broad_phase_pair p;
				p.a.type = RIGID_BODY;
				p.a.index = rigids[i].index;
				p.b.type = RIGID_BODY;
				p.b.index = rigids[j].index;

				arrput(local_pairs, p);
			}
		}

		// Rigid Body vs Static Body
		// The work here is rectangular (N * M), so default static scheduling is fine.
#pragma omp for schedule(static)
		for (size_t i = 0; i < rigid_count; ++i) {
			for (size_t j = 0; j < static_count; ++j) {

				if (rigids[i].aabb.max.x < statics[j].aabb.min.x ||
					rigids[i].aabb.min.x > statics[j].aabb.max.x ||
					rigids[i].aabb.max.y < statics[j].aabb.min.y ||
					rigids[i].aabb.min.y > statics[j].aabb.max.y ||
					rigids[i].aabb.max.z < statics[j].aabb.min.z ||
					rigids[i].aabb.min.z > statics[j].aabb.max.z) {
					continue;
				}

				broad_phase_pair p;
				p.a.type = RIGID_BODY;
				p.a.index = rigids[i].index;
				p.b.type = STATIC_BODY;
				p.b.index = statics[j].index;

				arrput(local_pairs, p);
			}
		}

		// Save the local buffer to the shared array so we can merge later
		thread_buffers[tid] = local_pairs;
	}

	// Merge Step (Serial)
	// Combine all thread-local buffers into the main return array
	for (int i = 0; i < max_threads; ++i) {
		if (thread_buffers[i]) {
			size_t count = arrlen(thread_buffers[i]);
			for (size_t k = 0; k < count; ++k) {
				arrput(pairs, thread_buffers[i][k]);
			}
			arrfree(thread_buffers[i]);
		}
	}

	free(thread_buffers);
	return pairs;
}

// Compare Body A (scalar) against 8 bodies from B (pointers to arrays)
// Force inlining, because clang won't do it otherwise and is slow as a result
static inline __attribute__((always_inline)) void check_8_bodies(
	// Body A properties
	float a_min_x, float a_max_x, float a_min_y, float a_max_y, float a_min_z, float a_max_z,
	body_ref ref_a,
	// Body B arrays (pointers to the current block of 8)
	const float* b_min_x, const float* b_max_x, const float* b_min_y, const float* b_max_y,
	const float* b_min_z, const float* b_max_z, const uint32_t* b_indices, uint8_t type_b,
	// Output list
	broad_phase_pair** pairs) {

	// 1. Load Body A bounds into vectors (Broadcast scalar to all 8 lanes)
	__m256 A_min_x = _mm256_set1_ps(a_min_x);
	__m256 A_max_x = _mm256_set1_ps(a_max_x);
	__m256 A_min_y = _mm256_set1_ps(a_min_y);
	__m256 A_max_y = _mm256_set1_ps(a_max_y);
	__m256 A_min_z = _mm256_set1_ps(a_min_z);
	__m256 A_max_z = _mm256_set1_ps(a_max_z);

	// 2. Load 8 bounds from Body B arrays
	// Use loadu (unaligned) because &b_min_x[j] might not be on a 32-byte boundary
	__m256 B_min_x = _mm256_loadu_ps(b_min_x);
	__m256 B_max_x = _mm256_loadu_ps(b_max_x);
	__m256 B_min_y = _mm256_loadu_ps(b_min_y);
	__m256 B_max_y = _mm256_loadu_ps(b_max_y);
	__m256 B_min_z = _mm256_loadu_ps(b_min_z);
	__m256 B_max_z = _mm256_loadu_ps(b_max_z);

	// 3. Compare AABBs
	// Overlap condition: (A.max >= B.min) AND (A.min <= B.max)
	// Note: _CMP_GE_OQ means "Greater or Equal, Ordered, Quiet"

	__m256 mask_x = _mm256_and_ps(_mm256_cmp_ps(A_max_x, B_min_x, _CMP_GE_OQ),
								  _mm256_cmp_ps(A_min_x, B_max_x, _CMP_LE_OQ));

	__m256 mask_y = _mm256_and_ps(_mm256_cmp_ps(A_max_y, B_min_y, _CMP_GE_OQ),
								  _mm256_cmp_ps(A_min_y, B_max_y, _CMP_LE_OQ));

	__m256 mask_z = _mm256_and_ps(_mm256_cmp_ps(A_max_z, B_min_z, _CMP_GE_OQ),
								  _mm256_cmp_ps(A_min_z, B_max_z, _CMP_LE_OQ));

	// Combine axes: X & Y & Z
	__m256 overlap = _mm256_and_ps(mask_x, _mm256_and_ps(mask_y, mask_z));

	// 4. Create integer mask (1 bit per body)
	int mask = _mm256_movemask_ps(overlap);

	// 5. Extract pairs if any collisions found
	// This is an optimization that avoids writing to the stack and instead uses registers. The
	// resulting speedup is ~3x in the popcorn_machine demo.
	if (mask != 0) {
		for (int k = 0; k < 8; ++k) {
			if (mask & (1 << k)) {
				broad_phase_pair p;
				p.a = ref_a;
				p.b.type = type_b;
				p.b.index = b_indices[k];
				arrput(*pairs, p);
			}
		}
	}
}

broad_phase_pair* broad_phase_naive_simd(const broad_phase_proxies_soa rigids,
										 const broad_phase_proxies_soa statics) {
	broad_phase_pair* pairs = NULL;

	// ---------------------------------------------------------
	// 1. Rigid Body vs Rigid Body
	// ---------------------------------------------------------
	for (size_t i = 0; i < rigids.count; ++i) {
		// Cache Body A properties
		float ax_min = rigids.min_x[i];
		float ax_max = rigids.max_x[i];
		float ay_min = rigids.min_y[i];
		float ay_max = rigids.max_y[i];
		float az_min = rigids.min_z[i];
		float az_max = rigids.max_z[i];

		body_ref ref_a = {.type = RIGID_BODY, .index = rigids.indices[i]};

		// Start checking from i + 1
		size_t j = i + 1;

		// SIMD Loop (Process 8 at a time)
		for (; j + 8 <= rigids.count; j += 8) {
			check_8_bodies(ax_min, ax_max, ay_min, ay_max, az_min, az_max, ref_a, &rigids.min_x[j],
						   &rigids.max_x[j], &rigids.min_y[j], &rigids.max_y[j], &rigids.min_z[j],
						   &rigids.max_z[j], &rigids.indices[j], RIGID_BODY, &pairs);
		}

		// Scalar Loop (Handle remaining items)
		for (; j < rigids.count; ++j) {
			if (ax_max < rigids.min_x[j] || ax_min > rigids.max_x[j] || ay_max < rigids.min_y[j] ||
				ay_min > rigids.max_y[j] || az_max < rigids.min_z[j] || az_min > rigids.max_z[j]) {
				continue;
			}
			broad_phase_pair p;
			p.a = ref_a;
			p.b.type = RIGID_BODY;
			p.b.index = rigids.indices[j];
			arrput(pairs, p);
		}
	}

	// ---------------------------------------------------------
	// 2. Rigid Body vs Static Body
	// ---------------------------------------------------------
	for (size_t i = 0; i < rigids.count; ++i) {
		float ax_min = rigids.min_x[i];
		float ax_max = rigids.max_x[i];
		float ay_min = rigids.min_y[i];
		float ay_max = rigids.max_y[i];
		float az_min = rigids.min_z[i];
		float az_max = rigids.max_z[i];

		body_ref ref_a = {.type = RIGID_BODY, .index = rigids.indices[i]};

		size_t j = 0;

		// SIMD Loop
		for (; j + 8 <= statics.count; j += 8) {
			check_8_bodies(ax_min, ax_max, ay_min, ay_max, az_min, az_max, ref_a, &statics.min_x[j],
						   &statics.max_x[j], &statics.min_y[j], &statics.max_y[j],
						   &statics.min_z[j], &statics.max_z[j], &statics.indices[j], STATIC_BODY,
						   &pairs);
		}

		// Scalar Loop
		for (; j < statics.count; ++j) {
			if (ax_max < statics.min_x[j] || ax_min > statics.max_x[j] ||
				ay_max < statics.min_y[j] || ay_min > statics.max_y[j] ||
				az_max < statics.min_z[j] || az_min > statics.max_z[j]) {
				continue;
			}
			broad_phase_pair p;
			p.a = ref_a;
			p.b.type = STATIC_BODY;
			p.b.index = statics.indices[j];
			arrput(pairs, p);
		}
	}

	return pairs;
}

// 32 is a sweet spot: small enough to fit in registers (AVX/NEON), large enough to amortize loop
// overhead.
#define BATCH_SIZE 32

/**
 * Helper function to process a range of bodies against a single Body A.
 *
 * Algorithm: Speculative Batching
 * 1. Pass 1 (Vectorized): Checks a batch of 32 bodies using bitwise OR reduction.
 *    - No branches inside the loop = CPU pipeline saturation.
 *    - Returns "True" if *any* collision exists in the batch.
 * 2. Pass 2 (Scalar): Only runs if Pass 1 detected a collision.
 *    - Re-checks the batch one by one to find the specific index and extract it.
 *
 * Performance Note:
 * - Best Case (Sparse/No Collisions): extremely fast. The scalar pass is skipped entirely.
 * - Worst Case (Dense/High Collisions): slower. We pay the cost of the vector pass PLUS the cost of
 *   the scalar pass. This happens in "stacking" scenarios.
 */
static inline void check_batch(
	// Body A properties
	float ax_min, float ax_max, float ay_min, float ay_max, float az_min, float az_max,
	body_ref ref_a,
	// Target Array (SoA) and range
	const broad_phase_proxies_soa* targets, size_t start_index, size_t end_index,
	uint8_t target_type,
	// Output
	broad_phase_pair** pairs) {

	// Loop through the target array in chunks of BATCH_SIZE
	for (size_t j = start_index; j < end_index; j += BATCH_SIZE) {
		// Calculate how many items are in this specific batch (usually 32, unless at end of array)
		size_t remaining = end_index - j;
		size_t count = (remaining < BATCH_SIZE) ? remaining : BATCH_SIZE;

		// --- PASS 1: The "Did Anything Happen?" Check (Vectorizable) ---
		// The compiler sees this as a reduction. It will use vector OR instructions.
		// Storing the overlap test results for all objects in an array is magnitudes slower,
		// presumably because any_collision can be stored in a register and stack memory operations
		// are avoided.
		int any_collision = 0;
		for (size_t k = 0; k < count; ++k) {
			size_t idx = j + k;
			// We use BITWISE AND (&) instead of LOGICAL AND (&&).
			// This prevents the compiler from generating branches (jumps).
			// The CPU executes all instructions linearly, keeping the pipeline full.
			int overlap = (ax_max >= targets->min_x[idx]) & (ax_min <= targets->max_x[idx]) &
						  (ay_max >= targets->min_y[idx]) & (ay_min <= targets->max_y[idx]) &
						  (az_max >= targets->min_z[idx]) & (az_min <= targets->max_z[idx]);
			any_collision |= overlap;
		}

		// Optimization: In a broadphase, most checks return false.
		// If nothing collided in this batch, we skip the expensive memory writes entirely.
		if (!any_collision) continue;

		// --- PASS 2: Extract Pairs (Scalar) ---
		// We only pay this cost if there was actually a hit.
		// In the sphere_pool demo with 5000 spheres (lots of resting contacts), this runs
		// frequently, causing the performance dip compared to the explicit SIMD version.
		for (size_t k = 0; k < count; ++k) {
			size_t idx = j + k;

			// Standard scalar check with short-circuiting (&&)
			if ((ax_max >= targets->min_x[idx]) && (ax_min <= targets->max_x[idx]) &&
				(ay_max >= targets->min_y[idx]) && (ay_min <= targets->max_y[idx]) &&
				(az_max >= targets->min_z[idx]) && (az_min <= targets->max_z[idx])) {

				broad_phase_pair p;
				p.a = ref_a;
				p.b.type = target_type;
				p.b.index = targets->indices[idx];
				arrput(*pairs, p);
			}
		}
	}
}

broad_phase_pair* broad_phase_naive_autovec(const broad_phase_proxies_soa rigids,
											const broad_phase_proxies_soa statics) {
	broad_phase_pair* pairs = NULL;

	// Iterate over all active rigid bodies (Body A)
	for (size_t i = 0; i < rigids.count; ++i) {

		// Load Body A properties once
		float ax_min = rigids.min_x[i];
		float ax_max = rigids.max_x[i];
		float ay_min = rigids.min_y[i];
		float ay_max = rigids.max_y[i];
		float az_min = rigids.min_z[i];
		float az_max = rigids.max_z[i];

		body_ref ref_a = {.type = RIGID_BODY, .index = rigids.indices[i]};

		// 1. Check against other Rigid Bodies
		// Start from i + 1 to avoid self-check and duplicates (A vs B, B vs A)
		check_batch(ax_min, ax_max, ay_min, ay_max, az_min, az_max, ref_a,
					&rigids,	  // Target Array
					i + 1,		  // Start Index
					rigids.count, // End Index
					RIGID_BODY,	  // Target Type
					&pairs);

		// 2. Check against Static Bodies
		// Check against all statics
		check_batch(ax_min, ax_max, ay_min, ay_max, az_min, az_max, ref_a,
					&statics,	   // Target Array
					0,			   // Start Index
					statics.count, // End Index
					STATIC_BODY,   // Target Type
					&pairs);
	}

	return pairs;
}

broad_phase_pair* broad_phase_naive_autovec_parallel(const broad_phase_proxies_soa rigids,
													 const broad_phase_proxies_soa statics) {
	broad_phase_pair* pairs = NULL;

	int max_threads = omp_get_max_threads();

	// Allocate pointers for thread-local arrays
	// We use calloc to ensure pointers are initially NULL for stb_ds
	broad_phase_pair** thread_buffers =
		(broad_phase_pair**)calloc(max_threads, sizeof(broad_phase_pair*));

#pragma omp parallel
	{
		int tid = omp_get_thread_num();
		broad_phase_pair* local_pairs = NULL; // Thread-local dynamic array

		// We use dynamic scheduling because the loop workload decreases as 'i' increases.
#pragma omp for schedule(dynamic)
		for (size_t i = 0; i < rigids.count; ++i) {

			// Load Body A properties
			// (These loads are naturally parallel-safe as they are read-only)
			float ax_min = rigids.min_x[i];
			float ax_max = rigids.max_x[i];
			float ay_min = rigids.min_y[i];
			float ay_max = rigids.max_y[i];
			float az_min = rigids.min_z[i];
			float az_max = rigids.max_z[i];

			body_ref ref_a = {.type = RIGID_BODY, .index = rigids.indices[i]};

			// 1. Rigid vs Rigid
			check_batch(ax_min, ax_max, ay_min, ay_max, az_min, az_max, ref_a, &rigids, i + 1,
						rigids.count, RIGID_BODY, &local_pairs);

			// 2. Rigid vs Static
			check_batch(ax_min, ax_max, ay_min, ay_max, az_min, az_max, ref_a, &statics, 0,
						statics.count, STATIC_BODY, &local_pairs);
		}

		// Store this thread's result
		thread_buffers[tid] = local_pairs;
	}

	// Optimization: Pre-calculate total size to resize 'pairs' once
	size_t total_collisions = 0;
	for (int i = 0; i < max_threads; ++i) {
		total_collisions += arrlen(thread_buffers[i]);
	}

	if (total_collisions > 0) {
		arrsetcap(pairs, total_collisions);

		for (int i = 0; i < max_threads; ++i) {
			broad_phase_pair* buf = thread_buffers[i];
			size_t count = arrlen(buf);

			// Append thread buffer to main array
			for (size_t k = 0; k < count; ++k) {
				arrput(pairs, buf[k]);
			}

			// Free the thread-local buffer
			arrfree(buf);
		}
	}

	free(thread_buffers);
	return pairs;
}

// Computes the intersection mask for 1 body (A) vs 8 bodies (B).
// We force inline to ensure the compiler merges this into the main loop registers.
static inline __m256 get_overlap_mask(__m256 A_min_x, __m256 A_max_x, __m256 A_min_y,
									  __m256 A_max_y, __m256 A_min_z, __m256 A_max_z,
									  const float* b_min_x, const float* b_max_x,
									  const float* b_min_y, const float* b_max_y,
									  const float* b_min_z, const float* b_max_z) {
	// Load B data (unaligned load is fine on modern CPUs)
	__m256 B_min_x = _mm256_loadu_ps(b_min_x);
	__m256 B_max_x = _mm256_loadu_ps(b_max_x);

	// X Overlap: (A.max >= B.min) && (A.min <= B.max)
	__m256 mask = _mm256_and_ps(_mm256_cmp_ps(A_max_x, B_min_x, _CMP_GE_OQ),
								_mm256_cmp_ps(A_min_x, B_max_x, _CMP_LE_OQ));

	// Early exit logic is not worth it here (vector pipelines prefer straight lines).
	// We proceed to AND in the Y and Z axes.
	__m256 B_min_y = _mm256_loadu_ps(b_min_y);
	__m256 B_max_y = _mm256_loadu_ps(b_max_y);

	mask = _mm256_and_ps(mask, _mm256_and_ps(_mm256_cmp_ps(A_max_y, B_min_y, _CMP_GE_OQ),
											 _mm256_cmp_ps(A_min_y, B_max_y, _CMP_LE_OQ)));

	__m256 B_min_z = _mm256_loadu_ps(b_min_z);
	__m256 B_max_z = _mm256_loadu_ps(b_max_z);

	mask = _mm256_and_ps(mask, _mm256_and_ps(_mm256_cmp_ps(A_max_z, B_min_z, _CMP_GE_OQ),
											 _mm256_cmp_ps(A_min_z, B_max_z, _CMP_LE_OQ)));

	return mask;
}

// Extracts valid pairs from a mask and adds them to the list.
static inline void extract_pairs(__m256 mask, body_ref ref_a, const uint32_t* b_indices,
								 uint8_t type_b, broad_phase_pair** pairs) {
	int bitmask = _mm256_movemask_ps(mask);
	if (!bitmask) return;

	// This loop usually unrolls or runs very fast for the 0-case
	for (int k = 0; k < 8; ++k) {
		if (bitmask & (1 << k)) {
			broad_phase_pair p = {ref_a, {type_b, b_indices[k]}};
			arrput(*pairs, p);
		}
	}
}

// Checks one rigid body (A) against an entire array of other bodies (B).
// This function handles both the AVX optimized batching and the scalar tail.
static inline void
query_body_against_array(size_t index_a,
						 const broad_phase_proxies_soa* rigids_source, // Where A comes from
						 const broad_phase_proxies_soa* target, // The array to check against (B)
						 size_t start_index_b, // Optimization: Skip indices (e.g. self-check)
						 uint8_t type_b, broad_phase_pair** pairs) {
	// 1. Broadcast Body A's bounds into AVX registers
	__m256 A_min_x = _mm256_set1_ps(rigids_source->min_x[index_a]);
	__m256 A_max_x = _mm256_set1_ps(rigids_source->max_x[index_a]);
	__m256 A_min_y = _mm256_set1_ps(rigids_source->min_y[index_a]);
	__m256 A_max_y = _mm256_set1_ps(rigids_source->max_y[index_a]);
	__m256 A_min_z = _mm256_set1_ps(rigids_source->min_z[index_a]);
	__m256 A_max_z = _mm256_set1_ps(rigids_source->max_z[index_a]);

	body_ref ref_a = {.type = RIGID_BODY, .index = rigids_source->indices[index_a]};

	size_t j = start_index_b;

	// 2. Vector Loop: Process 32 bodies (4 x AVX registers) per iteration
	// We use 32 to fill the pipeline with independent math, hiding instruction latency.
	for (; j + 32 <= target->count; j += 32) {

		// Compute 4 separate masks (8 bodies each)
		__m256 m0 = get_overlap_mask(A_min_x, A_max_x, A_min_y, A_max_y, A_min_z, A_max_z,
									 &target->min_x[j], &target->max_x[j], &target->min_y[j],
									 &target->max_y[j], &target->min_z[j], &target->max_z[j]);

		__m256 m1 =
			get_overlap_mask(A_min_x, A_max_x, A_min_y, A_max_y, A_min_z, A_max_z,
							 &target->min_x[j + 8], &target->max_x[j + 8], &target->min_y[j + 8],
							 &target->max_y[j + 8], &target->min_z[j + 8], &target->max_z[j + 8]);

		__m256 m2 = get_overlap_mask(A_min_x, A_max_x, A_min_y, A_max_y, A_min_z, A_max_z,
									 &target->min_x[j + 16], &target->max_x[j + 16],
									 &target->min_y[j + 16], &target->max_y[j + 16],
									 &target->min_z[j + 16], &target->max_z[j + 16]);

		__m256 m3 = get_overlap_mask(A_min_x, A_max_x, A_min_y, A_max_y, A_min_z, A_max_z,
									 &target->min_x[j + 24], &target->max_x[j + 24],
									 &target->min_y[j + 24], &target->max_y[j + 24],
									 &target->min_z[j + 24], &target->max_z[j + 24]);

		// Accumulate results using OR.
		__m256 accum = _mm256_or_ps(_mm256_or_ps(m0, m1), _mm256_or_ps(m2, m3));

		// OPTIMIZATION: "Skip Instruction"
		// If the accumulator is all zeros, NO collisions happened in this batch of 32.
		// We jump immediately to the next batch, skipping memory writes and bit-scanning.
		if (_mm256_testz_ps(accum, accum)) continue;

		// If we are here, at least one collision happened. Extract from the masks we already have.
		// Note: We do NOT re-compare. The data is already in m0-m3.
		extract_pairs(m0, ref_a, &target->indices[j], type_b, pairs);
		extract_pairs(m1, ref_a, &target->indices[j + 8], type_b, pairs);
		extract_pairs(m2, ref_a, &target->indices[j + 16], type_b, pairs);
		extract_pairs(m3, ref_a, &target->indices[j + 24], type_b, pairs);
	}

	// 3. Scalar Loop: Handle the remaining bodies (0 to 31 items)
	for (; j < target->count; ++j) {
		if (rigids_source->max_x[index_a] >= target->min_x[j] &&
			rigids_source->min_x[index_a] <= target->max_x[j] &&
			rigids_source->max_y[index_a] >= target->min_y[j] &&
			rigids_source->min_y[index_a] <= target->max_y[j] &&
			rigids_source->max_z[index_a] >= target->min_z[j] &&
			rigids_source->min_z[index_a] <= target->max_z[j]) {

			broad_phase_pair p = {ref_a, {type_b, target->indices[j]}};
			arrput(*pairs, p);
		}
	}
}

broad_phase_pair* broad_phase_naive_simd_speculative(const broad_phase_proxies_soa rigids,
													 const broad_phase_proxies_soa statics) {
	broad_phase_pair* pairs = NULL;

	for (size_t i = 0; i < rigids.count; ++i) {
		// 1. Rigid vs Rigid
		// start_index = i + 1 to avoid self-collision and duplicates (A vs B, don't check B vs A)
		query_body_against_array(i, &rigids, &rigids, i + 1, RIGID_BODY, &pairs);

		// 2. Rigid vs Static
		// start_index = 0 because we check against a totally different array
		query_body_against_array(i, &rigids, &statics, 0, STATIC_BODY, &pairs);
	}

	return pairs;
}

void insertion_sort_proxies(broad_phase_proxy_typed* arr, size_t count, uint32_t* proxy_map) {
	if (count < 2) return;

	// Insertion Sort
	// We iterate 1..N. If an element is out of order, we slide it backwards.
	for (size_t i = 1; i < count; ++i) {
		broad_phase_proxy_typed key = arr[i];
		size_t j = i;

		// Slide backwards while the previous element is greater than the current key
		while (j > 0 && arr[j - 1].aabb.min.x > key.aabb.min.x) {
			arr[j] = arr[j - 1]; // Move struct forward
			j--;
		}
		arr[j] = key;
	}
	// Rebuild Map Once
	// Updating the map inside the inner loop destroys performance (cache thrashing).
	// It is faster to rebuild it linearly once the list is sorted.
	// This ensures the map stays valid.
	for (size_t i = 0; i < count; ++i) {
		if (arr[i].type == RIGID_BODY) { proxy_map[arr[i].index] = (uint32_t)i; }
	}
}

broad_phase_pair* broad_phase_sap(broad_phase_proxy_typed* proxies, size_t count,
								  uint32_t* proxy_map) {
	broad_phase_pair* pairs = NULL;

	PROFILE("Sort") {
		// Sort the combined list along the X-axis
		// insertion sort is cheap for nearly sorted lists
		insertion_sort_proxies(proxies, count, proxy_map);
	}

	PROFILE("Sweep") {
		int max_threads = omp_get_max_threads();
		broad_phase_pair** thread_buffers =
			(broad_phase_pair**)calloc(max_threads, sizeof(broad_phase_pair*));

#pragma omp parallel
		{
			int tid = omp_get_thread_num();
			broad_phase_pair* local_pairs = NULL;

			// Schedule dynamic is crucial here.
			// Some areas of the X-axis might be empty (fast), others dense (slow).
#pragma omp for schedule(dynamic)
			for (size_t i = 0; i < count; ++i) {
				broad_phase_proxy_typed* s1 = &proxies[i];

				float s1_max_x = s1->aabb.max.x;
				int s1_is_static = (s1->type == STATIC_BODY);

				// Look ahead
				for (size_t j = i + 1; j < count; ++j) {
					broad_phase_proxy_typed* s2 = &proxies[j];

					// PRUNE: Axis Separation Test
					// Since the list is sorted by min.x, if s2 starts AFTER s1 ends,
					// then s2 (and every body after s2) cannot possibly collide with s1.
					if (s2->aabb.min.x > s1_max_x) { break; }

					// Optimization: Skip Static vs Static checks
					if (s1_is_static && s2->type == STATIC_BODY) { continue; }

					// CHECK: X-overlap is guaranteed by the logic above. Check Y and Z.
					// Use | instead of || to avoid branch misprediction (big performance
					// difference)
					int separated =
						(s1->aabb.max.y < s2->aabb.min.y) | (s1->aabb.min.y > s2->aabb.max.y) |
						(s1->aabb.max.z < s2->aabb.min.z) | (s1->aabb.min.z > s2->aabb.max.z);

					if (!separated) {
						broad_phase_pair p = {.a.type = s1->type,
											  .a.index = s1->index,
											  .b.type = s2->type,
											  .b.index = s2->index};
						arrput(local_pairs, p);
					}
				}
			}
			thread_buffers[tid] = local_pairs;
		}
		// Merge Results
		size_t total_collisions = 0;
		for (int i = 0; i < max_threads; ++i) {
			total_collisions += arrlen(thread_buffers[i]);
		}
		if (total_collisions > 0) {
			arrsetcap(pairs, total_collisions);
			for (int i = 0; i < max_threads; ++i) {
				broad_phase_pair* buf = thread_buffers[i];
				size_t buf_count = arrlen(buf);
				for (size_t k = 0; k < buf_count; ++k)
					arrput(pairs, buf[k]);
				arrfree(buf);
			}
		}
		free(thread_buffers);
	}

	// profile_print();

	return pairs;
}
