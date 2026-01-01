#define _POSIX_C_SOURCE 199309L // Required for clock_gettime

#include "tics_internal.h"

#include <stb_ds.h>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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

	return world;
}

void tics_world_destroy(tics_world* world) {
	assert(world);
	if (!world) return;

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

// Helper for high-resolution timing
static uint64_t get_time_ns() {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

void tics_world_step(tics_world* world, float delta) {
	assert(world);

	static uint64_t dynamics_total = 0;
	static uint64_t collision_total = 0;
	static int steps = 0;

	// --- Dynamics ---

	uint64_t start_dynamics = get_time_ns();

	// iterate directly over the flat array of rigid bodies for cache efficiency
	size_t count = arrlen(world->rigid_bodies);
	for (size_t i = 0; i < count; ++i) {
		rigid_body_data* rb = &world->rigid_bodies[i];

		// Gravity: impulse += gravity * mass * delta * scale
		tics_vec3 gravity_impulse =
			tics_vec3_mul_f(world->gravity, rb->mass * delta * rb->gravity_scale);
		rb->impulse = tics_vec3_add(rb->impulse, gravity_impulse);

		// apply linear impulse to linear velocity
		// v += impulse / mass
		tics_vec3 delta_v = tics_vec3_mul_f(rb->impulse, rb->inv_mass);
		rb->linear_velocity = tics_vec3_add(rb->linear_velocity, delta_v);

		// apply angular impulse to angular velocity
		// NOTE: angular velocity is stored in rad / 0.1s
		tics_quat angular_vel_change = tics_quat_scale(rb->an_imp_div_sq_dst, rb->inv_mass);
		rb->angular_velocity = tics_quat_mul(angular_vel_change, rb->angular_velocity);

		// apply linear velocity to transform
		// position += v * delta
		tics_vec3 pos_change = tics_vec3_mul_f(rb->linear_velocity, delta);
		rb->transform.position = tics_vec3_add(rb->transform.position, pos_change);

		// apply angular velocity to transform
		// rotation *= ang_vel * delta * 10.0f
		tics_quat rotation_change = tics_quat_scale(rb->angular_velocity, delta * 10.0f);
		rb->transform.rotation = tics_quat_mul(rb->transform.rotation, rotation_change);

		// linear air friction
		const float lin_fric = 0.2f;
		tics_vec3 fric_loss = tics_vec3_mul_f(rb->linear_velocity, lin_fric * delta);
		rb->linear_velocity = tics_vec3_sub(rb->linear_velocity, fric_loss);

		// angular air friction
		const float ang_fric = 0.5f;
		tics_quat identity = {0, 0, 0, 1};
		// Lerp towards identity
		rb->angular_velocity = tics_quat_lerp(rb->angular_velocity, identity, ang_fric * delta);

		// reset impulses
		rb->impulse = (tics_vec3){0, 0, 0};
		rb->an_imp_div_sq_dst = (tics_quat){0, 0, 0, 1};
	}

	uint64_t end_dynamics = get_time_ns();
	dynamics_total += (end_dynamics - start_dynamics);

	// --- Collision Detection ---
	
	uint64_t start_cd = get_time_ns();

	// Temporary array to store collisions for collision response
	collision* collisions = NULL;

	size_t rb_count = arrlen(world->rigid_bodies);
	size_t sb_count = arrlen(world->static_bodies);

	// RigidBody vs RigidBody
	// Checks unique pairs: i vs j where j > i
	for (size_t i = 0; i < rb_count; ++i) {
		for (size_t j = i + 1; j < rb_count; ++j) {
			rigid_body_data* rb_a = &world->rigid_bodies[i];
			rigid_body_data* rb_b = &world->rigid_bodies[j];

			collision_result res = collision_test(&rb_a->shape, rb_a->transform, &rb_b->shape,
													   rb_b->transform);

			if (res.has_collision) {
				collision col;
				col.body_a_ref = (body_ref){RIGID_BODY, i};
				col.body_b_ref = (body_ref){RIGID_BODY, j};
				col.result = res;
				arrput(collisions, col);
			}
		}
	}

	// RigidBody vs StaticBody
	for (size_t i = 0; i < rb_count; ++i) {
		for (size_t j = 0; j < sb_count; ++j) {
			rigid_body_data* rb = &world->rigid_bodies[i];
			static_body_data* sb = &world->static_bodies[j];

			collision_result res =
				collision_test(&rb->shape, rb->transform, &sb->shape, sb->transform);

			if (res.has_collision) {
				collision col;
				col.body_a_ref = (body_ref){RIGID_BODY, i};
				col.body_b_ref = (body_ref){STATIC_BODY, j};
				col.result = res;
				arrput(collisions, col);
			}
		}
	}

	uint64_t end_cd = get_time_ns();
	collision_total += (end_cd - start_cd);
	
	steps++;
	if (steps % 10 == 0) {
		double d_avg = (double)dynamics_total / steps;
		double cd_avg = (double)collision_total / steps;
		printf("d: %.0fns, cd: %.0fns, cr: 0ns\n", d_avg, cd_avg);
	}

	// --- Collision Response ---
	// TODO

	arrfree(collisions);
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
		// We must allocate and own the vertex data
		if (desc.data.convex.vertices && desc.data.convex.vertex_count > 0) {
			size_t size = sizeof(tics_vec3) * desc.data.convex.vertex_count;
			sd.data.convex.vertices = (tics_vec3*)malloc(size);
			if (sd.data.convex.vertices) {
				memcpy(sd.data.convex.vertices, desc.data.convex.vertices, size);
				sd.data.convex.count = desc.data.convex.vertex_count;
			} else {
				sd.data.convex.count = 0;
			}
		} else {
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
	} else if (ref.type == STATIC_BODY) {
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
	} else {
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
	} else if (ref.type == STATIC_BODY) {
		if (ref.index < (size_t)arrlen(world->static_bodies)) {
			t = world->static_bodies[ref.index].transform;
		}
	} else {
		assert(false); // not implemented
	}

	return t;
}
