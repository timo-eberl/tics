#include "demo_framework.h"
#include <raymath.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// --------------------------------------------------------------------------------------
// Math & Camera
// --------------------------------------------------------------------------------------
tics_vec3 ToTicsVec(Vector3 v) {
	return (tics_vec3){v.x, v.y, v.z};
}
tics_quat ToTicsQuat(Quaternion q) {
	return (tics_quat){q.x, q.y, q.z, q.w};
}
Vector3 ToRaylibVec(tics_vec3 v) {
	return (Vector3){v.x, v.y, v.z};
}
Quaternion ToRaylibQuat(tics_quat q) {
	return (Quaternion){q.x, q.y, q.z, q.w};
}

float GetRandomFloat(float min, float max) {
	return min + (float)rand() / ((float)RAND_MAX / (max - min));
}

void UpdateFlyCamera(Camera3D* camera) {
	float dt = GetFrameTime();
	if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) DisableCursor();
	if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) EnableCursor();

	if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
		Vector2 d = GetMouseDelta();
		Vector3 fwd = Vector3Normalize(Vector3Subtract(camera->target, camera->position));
		fwd = Vector3RotateByAxisAngle(fwd, (Vector3){0, 1, 0}, -d.x * 0.003f);
		Vector3 right = Vector3CrossProduct(fwd, camera->up);
		fwd = Vector3RotateByAxisAngle(fwd, right, -d.y * 0.003f);
		camera->target = Vector3Add(camera->position, fwd);
	}

	Vector3 fwd = Vector3Normalize(Vector3Subtract(camera->target, camera->position));
	Vector3 right = Vector3Normalize(Vector3CrossProduct(fwd, camera->up));
	Vector3 dir = {0};

	if (IsKeyDown(KEY_W)) dir = Vector3Add(dir, fwd);
	if (IsKeyDown(KEY_S)) dir = Vector3Subtract(dir, fwd);
	if (IsKeyDown(KEY_D)) dir = Vector3Add(dir, right);
	if (IsKeyDown(KEY_A)) dir = Vector3Subtract(dir, right);
	if (IsKeyDown(KEY_E)) dir.y += 1.0f;
	if (IsKeyDown(KEY_Q)) dir.y -= 1.0f;

	if (Vector3Length(dir) > 0) {
		dir = Vector3Scale(Vector3Normalize(dir), 10.0f * dt);
		camera->position = Vector3Add(camera->position, dir);
		camera->target = Vector3Add(camera->target, dir);
	}
}

// --------------------------------------------------------------------------------------
// Manual Asset Loading
// --------------------------------------------------------------------------------------

static float* ReadAccessor(cgltf_accessor* acc, int comps) {
	float* out = malloc(acc->count * comps * sizeof(float));
	for (size_t i = 0; i < acc->count; i++)
		cgltf_accessor_read_float(acc, i, &out[i * comps], comps);
	return out;
}

// Helper to calculate global transform of a node by walking up parents
static Matrix GetNodeWorldMatrix(cgltf_node* node) {
	float m[16];
	cgltf_node_transform_local(node, m);

	Matrix mat = {m[0], m[4], m[8],	 m[12], m[1], m[5], m[9],  m[13],
				  m[2], m[6], m[10], m[14], m[3], m[7], m[11], m[15]};

	if (node->parent) {
		Matrix parentMat = GetNodeWorldMatrix(node->parent);
		mat = MatrixMultiply(mat, parentMat);
	}
	return mat;
}

DemoScene LoadDemoScene(const char* filename) {
	DemoScene scene = {0};
	cgltf_options opts = {0};
	cgltf_data* data = NULL;

	if (cgltf_parse_file(&opts, filename, &data) != cgltf_result_success) return scene;
	if (cgltf_load_buffers(&opts, data, filename) != cgltf_result_success) {
		cgltf_free(data);
		return scene;
	}

	// Count meshes
	int mesh_node_count = 0;
	for (size_t i = 0; i < data->nodes_count; i++) {
		if (data->nodes[i].mesh) mesh_node_count++;
	}

	scene.objects = calloc(mesh_node_count, sizeof(DemoObject));
	scene.count = 0;

	// Iterate all nodes to find meshes
	for (size_t i = 0; i < data->nodes_count; i++) {
		cgltf_node* node = &data->nodes[i];
		if (!node->mesh) continue;

		DemoObject* obj = &scene.objects[scene.count];

		// 1. Calculate World Transform
		obj->default_transform = GetNodeWorldMatrix(node);

		// 2. Extract Mesh Data
		// For simplicity, we only take the first primitive of the mesh
		if (node->mesh->primitives_count > 0) {
			cgltf_primitive* prim = &node->mesh->primitives[0];
			cgltf_accessor* pos = NULL;
			cgltf_accessor* ind = prim->indices;

			for (size_t k = 0; k < prim->attributes_count; k++) {
				if (prim->attributes[k].type == cgltf_attribute_type_position)
					pos = prim->attributes[k].data;
			}

			if (pos) {
				// Raylib Mesh
				Mesh mesh = {0};
				mesh.vertexCount = (int)pos->count;
				mesh.triangleCount = (int)((ind) ? ind->count / 3 : pos->count / 3);
				mesh.vertices = ReadAccessor(pos, 3);
				if (ind) {
					mesh.indices = malloc(ind->count * sizeof(unsigned short));
					for (size_t k = 0; k < ind->count; k++)
						mesh.indices[k] = (unsigned short)cgltf_accessor_read_index(ind, k);
				}

				UploadMesh(&mesh, false);
				obj->model = LoadModelFromMesh(mesh);

				// Physics Vertices (Keep raw copy)
				obj->vertex_count = mesh.vertexCount;
				obj->raw_vertices = malloc(obj->vertex_count * sizeof(tics_vec3));
				memcpy(obj->raw_vertices, mesh.vertices, obj->vertex_count * sizeof(tics_vec3));
			}
		}

		scene.count++;
	}

	cgltf_free(data);
	return scene;
}

void UnloadDemoScene(DemoScene scene) {
	for (int i = 0; i < scene.count; i++) {
		UnloadModel(scene.objects[i].model);
		if (scene.objects[i].raw_vertices) free(scene.objects[i].raw_vertices);
	}
	if (scene.objects) free(scene.objects);
}
