#include "tics_debug_view_shm_internal.h"
#include "tics_internal.h"

#include <stb_ds.h>

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// Handles world creation and configuration, memory management, and the administration of bodies and
// shapes (creation, destruction, storage).

tics_world* tics_world_create(tics_world_desc desc) {
	// calloc to zero-initialize the memory, ensuring stb_ds pointers are NULL
	tics_world* world = (tics_world*)calloc(1, sizeof(tics_world));
	if (!world) return NULL;

	// config
	world->gravity = desc.gravity;

	// stb_ds arrays and maps start as NULL, which is valid.

	// Initialize counters to 1 (0 is reserved for invalid handles)
	world->body_id_counter = 1;
	world->shape_id_counter = 1;

	TICS_VIEW_INIT();

	return world;
}

void tics_world_destroy(tics_world* world) {
	assert(world);
	if (!world) return;

	TICS_VIEW_SHUTDOWN();

	// Free convex collision data
	if (world->shapes) {
		size_t count = arrlen(world->shapes);
		for (size_t i = 0; i < count; ++i) {
			if (world->shapes[i].type == TICS_SHAPE_CONVEX) {
				if (world->shapes[i].data.convex.vertices) {
					free(world->shapes[i].data.convex.vertices);
				}
			}
		}
	}

	// Free stb_ds structures
	arrfree(world->rigid_bodies);
	arrfree(world->static_bodies);
	arrfree(world->shapes);
	hmfree(world->body_map);
	hmfree(world->shape_map);

	free(world);
}

tics_shape_id tics_create_shape(tics_world* world, tics_shape_desc desc) {
	assert(world);

	shape_data sd;
	sd.type = desc.type;

	switch (desc.type) {
	case TICS_SHAPE_SPHERE:
		sd.data.sphere.center = desc.data.sphere.center;
		sd.data.sphere.radius = desc.data.sphere.radius;
		break;
	case TICS_SHAPE_PLANE:
		sd.data.plane.normal = desc.data.plane.normal;
		sd.data.plane.distance = desc.data.plane.distance;
		break;
	case TICS_SHAPE_CONVEX:
		// We copy the vertex data and remove duplicate vertices
		if (desc.data.convex.vertices && desc.data.convex.vertex_count > 0) {
			// Allocate worst-case size first (assuming no duplicates)
			size_t max_size = sizeof(tics_vec3) * desc.data.convex.vertex_count;
			sd.data.convex.vertices = (tics_vec3*)malloc(max_size);

			if (sd.data.convex.vertices) {
				int unique_count = 0;
				for (int i = 0; i < desc.data.convex.vertex_count; ++i) {
					tics_vec3 v = desc.data.convex.vertices[i];
					bool is_duplicate = false;

					// Check if exact vertex already exists in our new list
					for (int j = 0; j < unique_count; ++j) {
						if (memcmp(&sd.data.convex.vertices[j], &v, sizeof(tics_vec3)) == 0) {
							is_duplicate = true;
							break;
						}
					}

					if (!is_duplicate) { sd.data.convex.vertices[unique_count++] = v; }
				}

				// Resize to fit actual count to save memory
				if (unique_count < desc.data.convex.vertex_count) {
					tics_vec3* shrunk = (tics_vec3*)realloc(sd.data.convex.vertices,
															sizeof(tics_vec3) * unique_count);
					if (shrunk) sd.data.convex.vertices = shrunk;
				}
				sd.data.convex.count = unique_count;
			}
			else { sd.data.convex.count = 0; }
		}
		else {
			sd.data.convex.vertices = NULL;
			sd.data.convex.count = 0;
		}
		break;
	default:
		assert(false);
		return 0;
	}

	tics_shape_id id = world->shape_id_counter;
	world->shape_id_counter++;

	// Add to array
	arrput(world->shapes, sd);
	// Add ID -> Index mapping
	size_t index = arrlen(world->shapes) - 1;
	hmput(world->shape_map, id, index);

	return id;
}

void tics_destroy_shape(tics_world* world, tics_shape_id shape) {
	assert(world);

	ptrdiff_t map_idx = hmgeti(world->shape_map, shape);
	if (map_idx == -1) return;

	size_t index_to_remove = world->shape_map[map_idx].value;

	// Handle resource cleanup for convex shape
	if (world->shapes[index_to_remove].type == TICS_SHAPE_CONVEX) {
		if (world->shapes[index_to_remove].data.convex.vertices) {
			free(world->shapes[index_to_remove].data.convex.vertices);
		}
	}

	// Swap and Pop Logic for Shapes
	size_t last_index = arrlen(world->shapes) - 1;
	if (index_to_remove != last_index) {
		// Move last element to hole
		world->shapes[index_to_remove] = world->shapes[last_index];

		// Update the map for the moved shape.
		for (size_t i = 0; i < hmlen(world->shape_map); ++i) {
			if (world->shape_map[i].value == last_index) {
				world->shape_map[i].value = index_to_remove;
				break;
			}
		}
	}
	arrsetlen(world->shapes, last_index);

	// Remove from map
	hmdel(world->shape_map, shape);
}

tics_body_id tics_world_add_static_body(tics_world* world, tics_static_body_desc desc) {
	assert(world);

	// Look up shape
	ptrdiff_t shape_map_idx = hmgeti(world->shape_map, desc.shape);
	if (shape_map_idx == -1) return 0;

	size_t shape_index = world->shape_map[shape_map_idx].value;

	tics_body_id id = world->body_id_counter;
	world->body_id_counter++;

	static_body_data sb;
	sb.id = id;
	// Copy shape data for cache locality (except mesh pointer which is shared)
	sb.shape = world->shapes[shape_index];
	sb.transform = desc.transform;
	sb.elasticity = desc.elasticity;

	arrput(world->static_bodies, sb);
	size_t index = arrlen(world->static_bodies) - 1;

	body_ref ref = {STATIC_BODY, index};
	hmput(world->body_map, id, ref);

	return id;
}

tics_body_id tics_world_add_rigid_body(tics_world* world, tics_rigid_body_desc desc) {
	assert(world);

	ptrdiff_t shape_map_idx = hmgeti(world->shape_map, desc.shape);
	if (shape_map_idx == -1) return 0;

	size_t shape_index = world->shape_map[shape_map_idx].value;

	tics_body_id id = world->body_id_counter;
	world->body_id_counter++;

	rigid_body_data rb;
	rb.id = id;
	rb.shape = world->shapes[shape_index];
	rb.transform = desc.transform;
	rb.linear_velocity = desc.linear_velocity;
	rb.angular_velocity = desc.angular_velocity;
	assert(desc.mass > 0.0f);
	rb.mass = desc.mass;
	rb.inv_mass = 1.0f / desc.mass;
	rb.elasticity = desc.elasticity;
	rb.gravity_scale = desc.gravity_scale;

	// Reset runtime accumulators
	rb.impulse = (tics_vec3){0, 0, 0};
	rb.an_imp_div_sq_dst = (tics_quat){0, 0, 0, 1};

	arrput(world->rigid_bodies, rb);
	size_t index = arrlen(world->rigid_bodies) - 1;

	body_ref ref = {RIGID_BODY, index};
	hmput(world->body_map, id, ref);

	return id;
}

void tics_world_remove_body(tics_world* world, tics_body_id id) {
	assert(world);

	ptrdiff_t idx = hmgeti(world->body_map, id);
	if (idx == -1) return; // Not found

	body_ref ref = world->body_map[idx].value;

	if (ref.type == RIGID_BODY) {
		size_t remove_idx = ref.index;
		size_t last_idx = arrlen(world->rigid_bodies) - 1;

		if (remove_idx != last_idx) {
			// Swap with last
			rigid_body_data* last_body = &world->rigid_bodies[last_idx];
			rigid_body_data* target = &world->rigid_bodies[remove_idx];

			// Update the map for the swapped body
			tics_body_id moved_id = last_body->id;
			ptrdiff_t moved_map_idx = hmgeti(world->body_map, moved_id);
			if (moved_map_idx != -1) { world->body_map[moved_map_idx].value.index = remove_idx; }

			*target = *last_body; // Move data
		}
		arrsetlen(world->rigid_bodies, last_idx);
	}
	else if (ref.type == STATIC_BODY) {
		size_t remove_idx = ref.index;
		size_t last_idx = arrlen(world->static_bodies) - 1;

		if (remove_idx != last_idx) {
			static_body_data* last_body = &world->static_bodies[last_idx];
			static_body_data* target = &world->static_bodies[remove_idx];

			tics_body_id moved_id = last_body->id;
			ptrdiff_t moved_map_idx = hmgeti(world->body_map, moved_id);
			if (moved_map_idx != -1) { world->body_map[moved_map_idx].value.index = remove_idx; }

			*target = *last_body;
		}
		arrsetlen(world->static_bodies, last_idx);
	}
	else {
		assert(false); // not implemented
	}

	hmdel(world->body_map, id);
}

tics_transform tics_body_get_transform(const tics_world* world, tics_body_id id) {
	assert(world);

	tics_transform t = {{0, 0, 0}, {0, 0, 0, 1}};

	// we get a compiler error because of hmgeti, so we cast to non const and trust
	tics_world* non_const_world = (tics_world*)world;
	ptrdiff_t idx = hmgeti(non_const_world->body_map, id);
	if (idx == -1) {
		assert(false); // body doesn't exist
		return t;
	}

	body_ref ref = world->body_map[idx].value;

	if (ref.type == RIGID_BODY) {
		if (ref.index < (size_t)arrlen(world->rigid_bodies)) {
			t = world->rigid_bodies[ref.index].transform;
		}
	}
	else if (ref.type == STATIC_BODY) {
		if (ref.index < (size_t)arrlen(world->static_bodies)) {
			t = world->static_bodies[ref.index].transform;
		}
	}
	else {
		assert(false); // not implemented
	}

	return t;
}
