#define CGLTF_IMPLEMENTATION
#include "cgltf.h"

#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// --- Data Structures ---

typedef struct {
	float m[16];
} mat4;
typedef struct {
	float x, y, z;
} vec3;
typedef struct {
	float x, y, z, w;
} vec4;

// Structure to store info for the final summary arrays
typedef struct {
	char safe_name[128];
	vec3 world_pos;
	vec4 world_rot;
	size_t vertex_count;
	size_t index_count;
} MeshMetadata;

// --- Math Helpers ---

mat4 mat4_mul(mat4 a, mat4 b) {
	mat4 r = {0};
	for (int col = 0; col < 4; ++col) {
		for (int row = 0; row < 4; ++row) {
			float sum = 0.0f;
			for (int k = 0; k < 4; ++k) {
				sum += a.m[k * 4 + row] * b.m[col * 4 + k];
			}
			r.m[col * 4 + row] = sum;
		}
	}
	return r;
}

mat4 get_local_matrix(cgltf_node* node) {
	float temp[16];
	cgltf_node_transform_local(node, temp);
	mat4 r;
	memcpy(r.m, temp, 16 * sizeof(float));
	return r;
}

mat4 get_world_matrix(cgltf_node* node) {
	mat4 local = get_local_matrix(node);
	if (node->parent) {
		mat4 parent_world = get_world_matrix(node->parent);
		return mat4_mul(parent_world, local);
	}
	return local;
}

void decompose_transform(mat4 mat, vec3* out_pos, vec4* out_quat, int* scale_warning) {
	// 1. Position
	out_pos->x = mat.m[12];
	out_pos->y = mat.m[13];
	out_pos->z = mat.m[14];

	// 2. Scale check
	vec3 col0 = {mat.m[0], mat.m[1], mat.m[2]};
	vec3 col1 = {mat.m[4], mat.m[5], mat.m[6]};
	vec3 col2 = {mat.m[8], mat.m[9], mat.m[10]};

	float sx = sqrtf(col0.x * col0.x + col0.y * col0.y + col0.z * col0.z);
	float sy = sqrtf(col1.x * col1.x + col1.y * col1.y + col1.z * col1.z);
	float sz = sqrtf(col2.x * col2.x + col2.y * col2.y + col2.z * col2.z);

	if (fabsf(sx - 1.0f) > 0.001f || fabsf(sy - 1.0f) > 0.001f || fabsf(sz - 1.0f) > 0.001f) {
		*scale_warning = 1;
	}

	// Normalize for rotation
	if (sx > 0) {
		col0.x /= sx;
		col0.y /= sx;
		col0.z /= sx;
	}
	if (sy > 0) {
		col1.x /= sy;
		col1.y /= sy;
		col1.z /= sy;
	}
	if (sz > 0) {
		col2.x /= sz;
		col2.y /= sz;
		col2.z /= sz;
	}

	// 3. Rotation (Matrix to Quat)
	float trace = col0.x + col1.y + col2.z;
	if (trace > 0.0f) {
		float s = 0.5f / sqrtf(trace + 1.0f);
		out_quat->w = 0.25f / s;
		out_quat->x = (col1.z - col2.y) * s;
		out_quat->y = (col2.x - col0.z) * s;
		out_quat->z = (col0.y - col1.x) * s;
	}
	else {
		if (col0.x > col1.y && col0.x > col2.z) {
			float s = 2.0f * sqrtf(1.0f + col0.x - col1.y - col2.z);
			out_quat->w = (col1.z - col2.y) / s;
			out_quat->x = 0.25f * s;
			out_quat->y = (col0.y + col1.x) / s;
			out_quat->z = (col0.z + col2.x) / s;
		}
		else if (col1.y > col2.z) {
			float s = 2.0f * sqrtf(1.0f + col1.y - col0.x - col2.z);
			out_quat->w = (col2.x - col0.z) / s;
			out_quat->x = (col0.y + col1.x) / s;
			out_quat->y = 0.25f * s;
			out_quat->z = (col1.z + col2.y) / s;
		}
		else {
			float s = 2.0f * sqrtf(1.0f + col2.z - col0.x - col1.y);
			out_quat->w = (col0.y - col1.x) / s;
			out_quat->x = (col0.z + col2.x) / s;
			out_quat->y = (col1.z + col2.y) / s;
			out_quat->z = 0.25f * s;
		}
	}
}

// --- Utils ---

void sanitize_name(const char* input, char* output, size_t size) {
	size_t i = 0;
	if (!input || strlen(input) == 0) {
		snprintf(output, size, "unnamed_node");
		return;
	}
	while (input[i] && i < size - 1) {
		char c = input[i];
		output[i] = (isalnum(c)) ? c : '_';
		i++;
	}
	output[i] = '\0';
}

void print_float(float f) {
	char buf[64];
	snprintf(buf, sizeof(buf), "%.9g", f);
	if (strchr(buf, '.') == NULL && strchr(buf, 'e') == NULL) { strcat(buf, ".0"); }
	printf("%sf", buf);
}

// --- Main ---

int main(int argc, char** argv) {
	if (argc < 2) {
		fprintf(stderr, "Usage: %s <path_to.glb>\n", argv[0]);
		return 1;
	}

	cgltf_options options = {0};
	cgltf_data* data = NULL;
	cgltf_result result = cgltf_parse_file(&options, argv[1], &data);

	if (result != cgltf_result_success) {
		fprintf(stderr, "Error parsing file: %d\n", result);
		return 1;
	}

	result = cgltf_load_buffers(&options, data, argv[1]);
	if (result != cgltf_result_success) {
		cgltf_free(data);
		return 1;
	}

	MeshMetadata* meta_list = NULL;
	size_t meta_count = 0;

	for (size_t i = 0; i < data->nodes_count; ++i) {
		cgltf_node* node = &data->nodes[i];
		if (!node->mesh) continue;

		MeshMetadata meta;
		if (node->name) sanitize_name(node->name, meta.safe_name, sizeof(meta.safe_name));
		else snprintf(meta.safe_name, sizeof(meta.safe_name), "node_%zu", i);

		mat4 world_mat = get_world_matrix(node);
		int scale_warn = 0;
		decompose_transform(world_mat, &meta.world_pos, &meta.world_rot, &scale_warn);

		if (scale_warn) {
			fprintf(stderr, "Warning: Mesh '%s' has non-uniform/non-identity scale.\n",
					meta.safe_name);
		}

		// --- Print Positions & Rotation ---
		printf("tics_vec3 %s_position = {", meta.safe_name);
		print_float(meta.world_pos.x);
		printf(", ");
		print_float(meta.world_pos.y);
		printf(", ");
		print_float(meta.world_pos.z);
		printf("};\n");

		printf("tics_quat %s_rotation = {", meta.safe_name);
		print_float(meta.world_rot.x);
		printf(", ");
		print_float(meta.world_rot.y);
		printf(", ");
		print_float(meta.world_rot.z);
		printf(", ");
		print_float(meta.world_rot.w);
		printf("};\n");

		// --- Print Vertices (One Line) ---
		printf("tics_vec3 %s_vertices[] = {", meta.safe_name);

		size_t v_count = 0;
		int first_v = 1;

		for (size_t p = 0; p < node->mesh->primitives_count; ++p) {
			cgltf_primitive* prim = &node->mesh->primitives[p];
			cgltf_accessor* acc = NULL;
			for (size_t a = 0; a < prim->attributes_count; ++a) {
				if (prim->attributes[a].type == cgltf_attribute_type_position) {
					acc = prim->attributes[a].data;
					break;
				}
			}
			if (!acc) continue;

			for (size_t v = 0; v < acc->count; ++v) {
				float temp[3];
				cgltf_accessor_read_float(acc, v, temp, 3);

				if (!first_v) printf(",");
				printf("{");
				print_float(temp[0]);
				printf(",");
				print_float(temp[1]);
				printf(",");
				print_float(temp[2]);
				printf("}");
				first_v = 0;
				v_count++;
			}
		}
		printf("};\n");
		meta.vertex_count = v_count;

		// --- Print Indices (One Line) ---
		printf("uint32_t %s_indices[] = {", meta.safe_name);

		size_t i_count = 0;
		int first_i = 1;
		size_t vertex_offset = 0;

		for (size_t p = 0; p < node->mesh->primitives_count; ++p) {
			cgltf_primitive* prim = &node->mesh->primitives[p];

			size_t prim_v_count = 0;
			for (size_t a = 0; a < prim->attributes_count; ++a) {
				if (prim->attributes[a].type == cgltf_attribute_type_position) {
					prim_v_count = prim->attributes[a].data->count;
					break;
				}
			}

			if (prim->indices) {
				for (size_t k = 0; k < prim->indices->count; ++k) {
					size_t idx = cgltf_accessor_read_index(prim->indices, k);
					if (!first_i) printf(",");
					printf("%zu", idx + vertex_offset);
					first_i = 0;
					i_count++;
				}
			}
			else {
				for (size_t k = 0; k < prim_v_count; ++k) {
					if (!first_i) printf(",");
					printf("%zu", k + vertex_offset);
					first_i = 0;
					i_count++;
				}
			}
			vertex_offset += prim_v_count;
		}
		printf("};\n\n");
		meta.index_count = i_count;

		// --- Store Metadata ---
		meta_list = realloc(meta_list, sizeof(MeshMetadata) * (meta_count + 1));
		meta_list[meta_count] = meta;
		meta_count++;
	}

	// --- Print Summary Arrays (One Line Each) ---
	if (meta_count > 0) {
		// Positions
		printf("tics_vec3 positions[] = {");
		for (size_t i = 0; i < meta_count; i++) {
			if (i > 0) printf(",");
			printf("{");
			print_float(meta_list[i].world_pos.x);
			printf(",");
			print_float(meta_list[i].world_pos.y);
			printf(",");
			print_float(meta_list[i].world_pos.z);
			printf("}");
		}
		printf("};\n");

		// Rotations
		printf("tics_quat rotations[] = {");
		for (size_t i = 0; i < meta_count; i++) {
			if (i > 0) printf(",");
			printf("{");
			print_float(meta_list[i].world_rot.x);
			printf(",");
			print_float(meta_list[i].world_rot.y);
			printf(",");
			print_float(meta_list[i].world_rot.z);
			printf(",");
			print_float(meta_list[i].world_rot.w);
			printf("}");
		}
		printf("};\n");

		// Vertex Buffers
		printf("tics_vec3* vertex_buffers[] = {");
		for (size_t i = 0; i < meta_count; i++) {
			if (i > 0) printf(",");
			printf("%s_vertices", meta_list[i].safe_name);
		}
		printf("};\n");

		// Vertex Buffer Sizes
		printf("size_t vertex_buffer_sizes[] = {");
		for (size_t i = 0; i < meta_count; i++) {
			if (i > 0) printf(",");
			printf("%zu", meta_list[i].vertex_count);
		}
		printf("};\n");

		// Index Buffers
		printf("uint32_t* index_buffers[] = {");
		for (size_t i = 0; i < meta_count; i++) {
			if (i > 0) printf(",");
			printf("%s_indices", meta_list[i].safe_name);
		}
		printf("};\n");

		// Index Buffer Sizes
		printf("size_t index_buffer_sizes[] = {");
		for (size_t i = 0; i < meta_count; i++) {
			if (i > 0) printf(",");
			printf("%zu", meta_list[i].index_count);
		}
		printf("};\n");
	}

	free(meta_list);
	cgltf_free(data);
	return 0;
}
