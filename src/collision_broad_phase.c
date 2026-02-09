#include "tics_internal.h"
#include "tics_math.h"

#include <immintrin.h>
#include <omp.h>
#include <stb_ds.h>

#include <float.h>

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
