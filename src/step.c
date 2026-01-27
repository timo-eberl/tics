#include "blick_adapter.h"
#include "profiler.h"
#include "tics_internal.h"
#include "tics_math.h"

#include <stb_ds.h>

#include <assert.h>
#include <stdio.h>

void tics_world_step(tics_world* world, float delta) {
	assert(world);

	PROFILE("Apply Forces") {
		apply_gravity_and_air_friction(world, delta);
	}

	for (size_t i = 0; i < arrlen(world->rigid_bodies); ++i) {
		rigid_body_data rb = world->rigid_bodies[i];
		BLICK_DRAW_SHAPE(1, rb.shape, rb.transform, 0x11DDFFDD, false);
		BLICK_DRAW_SHAPE(1, rb.shape, rb.transform, 0xFF99AA44, true);
		tics_vec3 to = vec3_add(rb.transform.position, vec3_mul_f(rb.linear_velocity, 0.2f));
		BLICK_ARROW(1, rb.transform.position, to, 0xFFFF44FF);
		BLICK_TEXT_INT(2, rb.transform.position, rb.id, 0xFFFFFFFF);
		if (i == 0) {
			BLICK_TRANSFORM(5, rb.transform, 0.1);
			BLICK_TRIM_LAYER(5, 200);
		}
	}

	broad_phase_proxy* proxies_r = NULL;
	broad_phase_proxy* proxies_s = NULL;
	broad_phase_pair* potential_collision_pairs = NULL;
	collision* collisions = NULL;

	PROFILE("Collision Detection") {
		PROFILE("Proxy Collection") {
			proxies_r = build_rigid_proxies(world);
			proxies_s = build_static_proxies(world);
		}
		PROFILE("Broad Phase") {
			potential_collision_pairs =
				collision_broad_phase(proxies_r, arrlen(proxies_r), proxies_s, arrlen(proxies_s));
		}
		PROFILE("Narrow Phase") {
			collisions =
				collision_narrow_phase(potential_collision_pairs, arrlen(potential_collision_pairs),
									   world->rigid_bodies, world->static_bodies);
		}
	}

	BLICK_CLEAR(0b10000);
	// for (size_t i = 0; i < arrlen(proxies_r); ++i) {
	// 	broad_phase_proxy* p = &proxies_r[i];
	// 	BLICK_AABB(3, p->aabb.min, p->aabb.max, 0xFFFF0000);
	// }
	// for (size_t i = 0; i < arrlen(proxies_s); ++i) {
	// 	broad_phase_proxy* p = &proxies_s[i];
	// 	BLICK_AABB(3, p->aabb.min, p->aabb.max, 0xFF000000);
	// }
	// for (size_t i = 0; i < arrlen(potential_collision_pairs); ++i) {
	// 	broad_phase_pair* p = &potential_collision_pairs[i];
	// 	aabb a_box =
	// 		(p->a.type == RIGID_BODY) ? proxies_r[p->a.index].aabb : proxies_s[p->a.index].aabb;
	// 	tics_vec3 a_pos = (p->a.type == RIGID_BODY)
	// 						  ? world->rigid_bodies[p->a.index].transform.position
	// 						  : world->static_bodies[p->a.index].transform.position;
	// 	aabb b_box =
	// 		(p->b.type == RIGID_BODY) ? proxies_r[p->b.index].aabb : proxies_s[p->b.index].aabb;
	// 	tics_vec3 b_pos = (p->b.type == RIGID_BODY)
	// 						  ? world->rigid_bodies[p->b.index].transform.position
	// 						  : world->static_bodies[p->b.index].transform.position;
	// 	BLICK_AABB(5, a_box.min, a_box.max, 0xFF00FF00);
	// 	BLICK_AABB(5, b_box.min, b_box.max, 0xFF00FF00);
	// 	BLICK_LINE(4, a_pos, b_pos, 0xFFFF0000);
	// 	BLICK_REFRESH();
	// 	BLICK_CLEAR(0b100000);
	// }
	for (size_t i = 0; i < arrlen(collisions); ++i) {
		collision* c = &collisions[i];
		BLICK_POINT(1, c->result.point_a, 0.2f, 0xFF0000FF);
		BLICK_POINT(1, c->result.point_b, 0.2f, 0xFF00FFFF);
		BLICK_ARROW(1, c->result.point_a, c->result.point_b, 0xFFFF0000);
	}

	arrfree(proxies_r);
	arrfree(proxies_s);
	arrfree(potential_collision_pairs);

	PROFILE("Collision Response") {
		// More velocity solver iterations improve resting contact stability
		for (size_t i = 0; i < 10; i++) {
			resolve_velocities(world, collisions);
		}

		// resolve_penetrations(world, collisions);
	}

	arrfree(collisions);

	// Apply velocities to transform
	PROFILE("Apply Velocities") {
		apply_velocities(world, delta);
	}

	static int steps = 0;
	steps++;
	if (steps % 10 == 0) profile_print();

	size_t rb_count = arrlen(world->rigid_bodies);
	for (size_t i = 0; i < rb_count; ++i) {
		rigid_body_data* rb = &world->rigid_bodies[i];
		BLICK_TRANSFORM(1, rb->transform, 0.4);
	}

	BLICK_REFRESH();
	BLICK_CLEAR(0b1110);
}
