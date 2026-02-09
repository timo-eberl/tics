#include "tics_internal.h"
#include "tics_math.h"

#include <omp.h>
#include <stb_ds.h>

#include <float.h>

aabb tics_calculate_aabb(const shape_data* shape, tics_transform t) {
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
		aabb box = tics_calculate_aabb(&rb->shape, rb->transform);
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
