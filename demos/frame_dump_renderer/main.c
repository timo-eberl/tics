#include <blick.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#ifndef PROJECT_ROOT_DIR
#define PROJECT_ROOT_DIR "."
#endif

typedef struct {
	uint32_t a_index;
	uint32_t b_index;
	uint32_t b_type; // 0 = Static, 1 = Rigid
	float depth;
	float point_a[3];
	float point_b[3];
	float normal[3];
	uint32_t pad[3];
} dx_collision_full; // 64 bytes

typedef struct {
	float position[3];
	uint32_t shape_type; // 0 = Sphere, 1 = Capsule, 2 = OBB
	float rotation[4];
	uint32_t shape_index;
	uint32_t pad[3];
} dx_entity; // 48 bytes

typedef union {
	struct { float radius; uint32_t pad[3]; } sphere;
	struct { float half_height; float radius; uint32_t pad[2]; } capsule;
	struct { float half_extents[3]; uint32_t pad; } obb;
	float data[4];
} dx_shape; // 16 bytes

static inline blick_vec3 vec3_add(blick_vec3 a, blick_vec3 b) {
	return (blick_vec3){a.x + b.x, a.y + b.y, a.z + b.z};
}

static inline blick_vec3 vec3_mul_f(blick_vec3 v, float s) {
	return (blick_vec3){v.x * s, v.y * s, v.z * s};
}

static inline blick_vec3 vec3_cross(blick_vec3 a, blick_vec3 b) {
	return (blick_vec3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// Optimized Grassman rotation for vector by quaternion
static inline blick_vec3 quat_rotate_vec3(blick_vec3 v, blick_quat q) {
	blick_vec3 q_xyz = {q.x, q.y, q.z};
	blick_vec3 t = vec3_cross(q_xyz, v);
	t = vec3_add(t, t);
	
	blick_vec3 term1 = vec3_mul_f(t, q.w);
	blick_vec3 term2 = vec3_cross(q_xyz, t);
	return vec3_add(v, vec3_add(term1, term2));
}

static void render_entities(dx_entity* entities, uint32_t count, dx_shape* shapes, bool is_static) {
	for (uint32_t i = 0; i < count; i++) {
		dx_entity* ent = &entities[i];
		dx_shape* shape = &shapes[ent->shape_index];

		blick_vec3 pos = {ent->position[0], ent->position[1], ent->position[2]};
		blick_quat rot = {
			ent->rotation[0], ent->rotation[1], ent->rotation[2], ent->rotation[3]
		};

		uint8_t layer = is_static ? 0 : 1;
		uint32_t color = 0xFFFFFFFF;

		switch (ent->shape_type) {
			case 0: { // Sphere: Reddish variants (ABGR)
				color = is_static ? 0xFF404090 : 0xFF6060FF;
				blick_record_sphere(layer, pos, rot, shape->sphere.radius, color, false);
				break;
			}
			case 1: { // Capsule: Greenish variants (ABGR)
				color = is_static ? 0xFF409040 : 0xFF60FF60;
				blick_vec3 local_offset = {0.0f, shape->capsule.half_height, 0.0f};
				blick_vec3 rotated_offset = quat_rotate_vec3(local_offset, rot);

				blick_vec3 p_a = vec3_add(pos, rotated_offset);
				blick_vec3 neg_offset = vec3_mul_f(rotated_offset, -1.0f);
				blick_vec3 p_b = vec3_add(pos, neg_offset);

				blick_record_capsule(layer, p_a, p_b, shape->capsule.radius, color, false);
				break;
			}
			case 2: { // OBB: Blueish variants (ABGR)
				color = is_static ? 0xFF904020 : 0xFFFF9040;
				
				// Fix sizes: convert half_extents back to full extents for the renderer
				blick_vec3 ext = {
					shape->obb.half_extents[0] * 2.0f,
					shape->obb.half_extents[1] * 2.0f,
					shape->obb.half_extents[2] * 2.0f
				};

				// Check if this box is considered 'very big'
				if (ext.x > 10.0f || ext.y > 10.0f || ext.z > 10.0f) {
					// Swap alpha channel to ~25% (0x40) for the solid fill
					uint32_t trans_color = (color & 0x00FFFFFF) | 0x10000000;
					blick_record_obb(layer, pos, rot, ext, trans_color, false);
					
					// Draw an opaque wireframe outline on top
					blick_record_obb(layer, pos, rot, ext, color, true);
				} else {
					blick_record_obb(layer, pos, rot, ext, color, false);
				}
				break;
			}
		}
	}
}

int main() {
	FILE* file = fopen(PROJECT_ROOT_DIR "/collision_test_data.bin", "rb");
	if (!file) {
		printf("Failed to open " PROJECT_ROOT_DIR "/collision_test_data.bin\n");
		return 1;
	}

	blick_init();

	uint32_t frame_index = 0;
	uint32_t counts[4];

	while (fread(counts, sizeof(uint32_t), 4, file) == 4) {
		uint32_t rigid_count = counts[0];
		uint32_t static_count = counts[1];
		uint32_t shape_count = counts[2];
		uint32_t expected_col_count = counts[3];

		dx_entity* rigids = (dx_entity*)malloc(rigid_count * sizeof(dx_entity));
		dx_entity* statics = (dx_entity*)malloc(static_count * sizeof(dx_entity));
		dx_shape* shapes = (dx_shape*)malloc(shape_count * sizeof(dx_shape));
		dx_collision_full* expected_cols = (dx_collision_full*)malloc(
			expected_col_count * sizeof(dx_collision_full));

		if (rigid_count > 0) fread(rigids, sizeof(dx_entity), rigid_count, file);
		if (static_count > 0) fread(statics, sizeof(dx_entity), static_count, file);
		if (shape_count > 0) fread(shapes, sizeof(dx_shape), shape_count, file);
		if (expected_col_count > 0) {
			fread(expected_cols, sizeof(dx_collision_full), expected_col_count, file);
		}

		// Rendering
		render_entities(statics, static_count, shapes, true);
		render_entities(rigids, rigid_count, shapes, false);

		blick_refresh();
		blick_clear(0xFFFF); // Clear all 16 layers for the next frame

		// Roughly limits execution rate to 60fps
		usleep(160000);

		free(rigids);
		free(statics);
		free(shapes);
		free(expected_cols);

		frame_index++;
	}

	// blick_shutdown();
	return 0;
}
