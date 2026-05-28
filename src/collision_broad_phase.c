#include "profiler.h"
#include "tics_internal.h"
#include "tics_math.h"

#include <stb_ds.h>

#include <float.h>
#include <stdint.h>
#include <stdlib.h>

aabb calculate_aabb(const shape_data* shape, tics_transform t) {
	// Initialize with (inverted) infinity
	aabb box = {.min = {FLT_MAX, FLT_MAX, FLT_MAX}, .max = {-FLT_MAX, -FLT_MAX, -FLT_MAX}};

	switch (shape->type) {
	case SHAPE_SPHERE: {
		float r = shape->data.sphere.radius;

		// Rotate the local center offset by the object's rotation
		tics_vec3 rotated_center = quat_rotate_vec3(shape->data.sphere.center, t.rotation);

		// Add the rotated offset to the object's world position
		tics_vec3 world_center = vec3_add(t.position, rotated_center);

		// Expand bounding box from the calculated world center
		box.min = (tics_vec3){world_center.x - r, world_center.y - r, world_center.z - r};
		box.max = (tics_vec3){world_center.x + r, world_center.y + r, world_center.z + r};
	} break;

	case SHAPE_CONVEX: {
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

void update_packed_proxies(tics_world* world) {
	size_t rigid_count = arrlen(world->rigid_bodies);
	// Resize persistent buffer to match current body count (no-op if size unchanged)
	arrsetlen(world->packed_rigid_proxies, rigid_count);
	for (size_t i = 0; i < rigid_count; ++i) {
		rigid_body_data* rb = &world->rigid_bodies[i];
		aabb box = calculate_aabb(&rb->shape, rb->transform);
		world->packed_rigid_proxies[i] = (packed_aabb){
			.min_x = box.min.x,
			.max_x = box.max.x,
			.min_y = box.min.y,
			.max_y = box.max.y,
			.min_z = box.min.z,
			.max_z = box.max.z,
		};
	}

	if (world->static_bodies_dirty) {
		size_t static_count = arrlen(world->static_bodies);
		arrsetlen(world->packed_static_proxies, static_count);
		for (size_t i = 0; i < static_count; ++i) {
			static_body_data* sb = &world->static_bodies[i];
			// Static AABBs are pre-calculated, no need to call calculate_aabb
			world->packed_static_proxies[i] = (packed_aabb){
				.min_x = sb->aabb.min.x,
				.max_x = sb->aabb.max.x,
				.min_y = sb->aabb.min.y,
				.max_y = sb->aabb.max.y,
				.min_z = sb->aabb.min.z,
				.max_z = sb->aabb.max.z,
			};
		}
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

	// Case 1: Objects were added or removed (Rebuild and qsort)
	if (world->rigid_bodies_dirty || world->static_bodies_dirty) {
		// Clear the existing list but keep memory capacity if possible
		arrsetlen(world->typed_proxies, 0);
		// Pre-allocate list to avoid resizing
		arrsetcap(world->typed_proxies, rigid_count + static_count);

		// (Re)populate Rigids
		for (size_t i = 0; i < rigid_count; ++i) {
			rigid_body_data* rb = &world->rigid_bodies[i];
			aabb box = calculate_aabb(&rb->shape, rb->transform);
			broad_phase_proxy_typed p = {box, (uint32_t)i, RIGID_BODY};
			arrput(world->typed_proxies, p);
		}

		// (Re)populate Statics
		for (size_t i = 0; i < static_count; ++i) {
			static_body_data* sb = &world->static_bodies[i];
			broad_phase_proxy_typed p = {sb->aabb, (uint32_t)i, STATIC_BODY};
			arrput(world->typed_proxies, p);
		}

		// Sort the combined list along the X-axis
		// qsort works perfectly on stb_ds arrays since they are contiguous blocks of memory.
		qsort(world->typed_proxies, arrlen(world->typed_proxies), sizeof(broad_phase_proxy_typed),
			  compare_sap_entries);

		// (Re)build the Lookup Map
		// Map size matches rigid body count (we don't map statics as they don't update)
		arrsetlen(world->typed_proxy_map, rigid_count);
		size_t count = arrlen(world->typed_proxies);
		for (size_t i = 0; i < count; ++i) {
			if (world->typed_proxies[i].type == RIGID_BODY) {
				// The rigid body with ID [index] is located at [i] in the sorted list
				world->typed_proxy_map[world->typed_proxies[i].index] = (uint32_t)i;
			}
		}

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
		uint32_t proxy_index = world->typed_proxy_map[i];
		world->typed_proxies[proxy_index].aabb = box;
	}
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

	static int steps = 0;
	steps++;
	if (steps % 10 == 0) profile_print();

	return pairs;
}
