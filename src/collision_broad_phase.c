#include "tics_internal.h"
#include "tics_math.h"

#include <stb_ds.h>

#include <float.h>

aabb tics_calculate_aabb(const shape_data* shape, tics_transform t) {
	// Initialize with (inverted) infinity
	aabb box = {.min = {FLT_MAX, FLT_MAX, FLT_MAX}, .max = {-FLT_MAX, -FLT_MAX, -FLT_MAX}};

	switch (shape->type) {
	case TICS_SHAPE_SPHERE: {
		float r = shape->data.sphere.radius;
		box.min = (tics_vec3){t.position.x - r, t.position.y - r, t.position.z - r};
		box.max = (tics_vec3){t.position.x + r, t.position.y + r, t.position.z + r};
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
