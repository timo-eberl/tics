#include <cgltf.h>

#include <raylib.h>
#include <raymath.h>
#include <tics.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// --------------------------------------------------------------------------------------
// Configuration
// --------------------------------------------------------------------------------------

#define GRAVITY_Y -9.81f
#define SPAWN_AREA_XZ 5.0f
#define SPAWN_HEIGHT_MIN 5.0f
#define SPAWN_HEIGHT_MAX 15.0f
#define DYNAMIC_OBJ_COUNT 20

typedef struct {
	const char* model_path;
	float mass;
} SpawnConfig;

const SpawnConfig ASSET_CONFIGS[] = {
	{"assets/models/icosphere.glb", 1.0f},
	{"assets/models/icosphere_lowres.glb", 1.0f},
	{"assets/models/cube.glb", 2.0f},
};

// --------------------------------------------------------------------------------------
// Abstraction Layer Types
// --------------------------------------------------------------------------------------

// Represents a single object loaded from a file
typedef struct {
	Matrix transform;	 // World Transform (Rotation + Position)
	Vector3 position;	 // Extracted Position (for convenience)
	Quaternion rotation; // Extracted Rotation (for Physics init)

	// Visuals (Raylib)
	Model model;

	// Physics Data (Raw vertices for Tics)
	tics_vec3* vertices;
	int vertex_count;
} SceneObject;

typedef struct {
	SceneObject* objects;
	int count;
} SceneData;

// --------------------------------------------------------------------------------------
// Math Converters
// --------------------------------------------------------------------------------------

tics_vec3 ToTicsVec(Vector3 v) {
	return (tics_vec3){v.x, v.y, v.z};
}
tics_quat ToTicsQuat(Quaternion q) {
	return (tics_quat){q.x, q.y, q.z, q.w};
}
Quaternion ToRaylibQuat(tics_quat q) {
	return (Quaternion){q.x, q.y, q.z, q.w};
}

float GetRandomFloat(float min, float max) {
	return min + (float)rand() / ((float)RAND_MAX / (max - min));
}

// --------------------------------------------------------------------------------------
// GLTF Loader Abstraction
// --------------------------------------------------------------------------------------

// Helper: Read raw floats from accessor
float* ReadAccessorData(cgltf_accessor* acc, int component_count) {
	size_t num_floats = acc->count * component_count;
	float* buffer = (float*)malloc(num_floats * sizeof(float));
	for (size_t i = 0; i < acc->count; i++) {
		cgltf_accessor_read_float(acc, i, &buffer[i * component_count], component_count);
	}
	return buffer;
}

// Helper: Extract Mesh Data from Primitive
void ExtractMeshData(cgltf_primitive* prim, SceneObject* out_obj) {
	cgltf_accessor* pos_acc = NULL;
	cgltf_accessor* norm_acc = NULL;
	cgltf_accessor* tex_acc = NULL;
	cgltf_accessor* ind_acc = prim->indices;

	for (size_t i = 0; i < prim->attributes_count; i++) {
		if (prim->attributes[i].type == cgltf_attribute_type_position)
			pos_acc = prim->attributes[i].data;
		if (prim->attributes[i].type == cgltf_attribute_type_normal)
			norm_acc = prim->attributes[i].data;
		if (prim->attributes[i].type == cgltf_attribute_type_texcoord)
			tex_acc = prim->attributes[i].data;
	}

	if (!pos_acc) return;

	// 1. Build Raylib Mesh
	Mesh mesh = {0};
	mesh.vertexCount = (int)pos_acc->count;
	mesh.triangleCount = (int)((ind_acc) ? ind_acc->count / 3 : pos_acc->count / 3);
	mesh.vertices = ReadAccessorData(pos_acc, 3);
	if (norm_acc) mesh.normals = ReadAccessorData(norm_acc, 3);
	if (tex_acc) mesh.texcoords = ReadAccessorData(tex_acc, 2);

	if (ind_acc) {
		mesh.indices = (unsigned short*)malloc(ind_acc->count * sizeof(unsigned short));
		for (size_t i = 0; i < ind_acc->count; i++) {
			mesh.indices[i] = (unsigned short)cgltf_accessor_read_index(ind_acc, i);
		}
	}

	UploadMesh(&mesh, false);
	out_obj->model = LoadModelFromMesh(mesh);

	// 2. Keep Raw Vertices for Physics
	// We can just point to the mesh.vertices since it's a float array, same layout as tics_vec3
	// Note: We duplicate the buffer so we can free the mesh cpu data if we wanted to (though here
	// we keep it)
	size_t size = mesh.vertexCount * sizeof(tics_vec3);
	out_obj->vertices = malloc(size);
	memcpy(out_obj->vertices, mesh.vertices, size);
	out_obj->vertex_count = mesh.vertexCount;
}

// Main Loading Function
SceneData LoadSceneObjects(const char* filename) {
	SceneData scene = {0};

	cgltf_options options = {0};
	cgltf_data* data = NULL;

	if (cgltf_parse_file(&options, filename, &data) != cgltf_result_success) return scene;
	if (cgltf_load_buffers(&options, data, filename) != cgltf_result_success) {
		cgltf_free(data);
		return scene;
	}

	// Count mesh nodes
	int count = 0;
	for (size_t i = 0; i < data->nodes_count; i++) {
		if (data->nodes[i].mesh) count++;
	}

	scene.objects = (SceneObject*)calloc(count, sizeof(SceneObject));
	scene.count = 0;

	for (size_t i = 0; i < data->nodes_count; i++) {
		cgltf_node* node = &data->nodes[i];
		if (!node->mesh || node->mesh->primitives_count == 0) continue;

		SceneObject* obj = &scene.objects[scene.count];

		// 1. Get Node World Matrix (Column-Major Array)
		float m[16];
		cgltf_node_transform_world(node, m);

		// 2. Map to Raylib Matrix (Row-Major struct)
		Matrix rm;
		rm.m0 = m[0];
		rm.m4 = m[4];
		rm.m8 = m[8];
		rm.m12 = m[12];
		rm.m1 = m[1];
		rm.m5 = m[5];
		rm.m9 = m[9];
		rm.m13 = m[13];
		rm.m2 = m[2];
		rm.m6 = m[6];
		rm.m10 = m[10];
		rm.m14 = m[14];
		rm.m3 = m[3];
		rm.m7 = m[7];
		rm.m11 = m[11];
		rm.m15 = m[15];

		obj->transform = rm;
		obj->position = (Vector3){rm.m12, rm.m13, rm.m14};

		// Normalize rotation basis to handle potential scale
		Vector3 right = (Vector3){rm.m0, rm.m1, rm.m2};
		Vector3 up = (Vector3){rm.m4, rm.m5, rm.m6};
		Vector3 fwd = (Vector3){rm.m8, rm.m9, rm.m10};

		if (Vector3Length(right) > 0.001f) {
			float sx = Vector3Length(right);
			rm.m0 /= sx;
			rm.m1 /= sx;
			rm.m2 /= sx;
		}
		if (Vector3Length(up) > 0.001f) {
			float sy = Vector3Length(up);
			rm.m4 /= sy;
			rm.m5 /= sy;
			rm.m6 /= sy;
		}
		if (Vector3Length(fwd) > 0.001f) {
			float sz = Vector3Length(fwd);
			rm.m8 /= sz;
			rm.m9 /= sz;
			rm.m10 /= sz;
		}

		obj->rotation = QuaternionFromMatrix(rm);

		// 3. Extract Mesh Data
		// Just take the first primitive for simplicity
		ExtractMeshData(&node->mesh->primitives[0], obj);

		scene.count++;
	}

	cgltf_free(data);
	return scene;
}

void UnloadSceneObjects(SceneData scene) {
	for (int i = 0; i < scene.count; i++) {
		UnloadModel(scene.objects[i].model);
		free(scene.objects[i].vertices);
	}
	free(scene.objects);
}

// --------------------------------------------------------------------------------------
// Main Application
// --------------------------------------------------------------------------------------

typedef struct {
	Model model; // Visual reference (shallow copy from SceneData or ref)
	tics_body_id body;
} DynamicObject;

typedef struct {
	SceneData scene_data;
	tics_shape_id shape;
} CachedAsset;

int main(void) {
	srand((unsigned int)time(NULL));

	InitWindow(1280, 720, "Tics Physics Demo");
	SetTargetFPS(60);

	Camera3D camera = {0};
	camera.position = (Vector3){12.0f, 10.0f, 12.0f};
	camera.target = (Vector3){0.0f, 2.0f, 0.0f};
	camera.up = (Vector3){0.0f, 1.0f, 0.0f};
	camera.fovy = 45.0f;
	camera.projection = CAMERA_PERSPECTIVE;

	// 1. Init Physics
	tics_world_desc world_desc = {.gravity = {0.0f, GRAVITY_Y, 0.0f}};
	tics_world* phys_world = tics_world_create(world_desc);

	// 2. Load Ground (Static Scene)
	SceneData groundScene = LoadSceneObjects("assets/models/ground.glb");

	// Create static bodies for every object found in the ground file
	for (int i = 0; i < groundScene.count; i++) {
		SceneObject* obj = &groundScene.objects[i];

		// Create Shape
		tics_shape_desc shape_desc = {0};
		shape_desc.type = TICS_SHAPE_CONVEX;
		shape_desc.data.convex.vertices = obj->vertices;
		shape_desc.data.convex.vertex_count = obj->vertex_count;
		tics_shape_id shape = tics_create_shape(phys_world, shape_desc);

		// Create Static Body
		tics_static_body_desc body_desc = {0};
		body_desc.transform.position = ToTicsVec(obj->position);
		body_desc.transform.rotation = ToTicsQuat(obj->rotation);
		body_desc.shape = shape;
		body_desc.elasticity = 0.5f;

		tics_world_add_static_body(phys_world, body_desc);
	}

	// 3. Pre-load Dynamic Assets
	int config_count = sizeof(ASSET_CONFIGS) / sizeof(ASSET_CONFIGS[0]);
	CachedAsset* assets = calloc(config_count, sizeof(CachedAsset));

	for (int i = 0; i < config_count; i++) {
		// Load the file
		assets[i].scene_data = LoadSceneObjects(ASSET_CONFIGS[i].model_path);

		// Assume the first object in the GLB is the one we want to spawn
		if (assets[i].scene_data.count > 0) {
			SceneObject* obj = &assets[i].scene_data.objects[0];

			tics_shape_desc shape_desc = {0};
			shape_desc.type = TICS_SHAPE_CONVEX;
			shape_desc.data.convex.vertices = obj->vertices;
			shape_desc.data.convex.vertex_count = obj->vertex_count;

			assets[i].shape = tics_create_shape(phys_world, shape_desc);
		}
	}

	// 4. Spawn Dynamic Objects
	DynamicObject* dyn_objs = malloc(DYNAMIC_OBJ_COUNT * sizeof(DynamicObject));
	for (int i = 0; i < DYNAMIC_OBJ_COUNT; i++) {
		int idx = rand() % config_count;
		if (assets[idx].scene_data.count == 0) continue;

		// Visual
		dyn_objs[i].model = assets[idx].scene_data.objects[0].model;

		// Physics
		tics_rigid_body_desc desc = {0};
		desc.shape = assets[idx].shape;
		desc.mass = ASSET_CONFIGS[idx].mass;
		desc.gravity_scale = 1.0f;
		desc.elasticity = GetRandomFloat(0.8f, 1.0f);

		desc.transform.position.x = GetRandomFloat(-SPAWN_AREA_XZ, SPAWN_AREA_XZ);
		desc.transform.position.z = GetRandomFloat(-SPAWN_AREA_XZ, SPAWN_AREA_XZ);
		desc.transform.position.y = GetRandomFloat(SPAWN_HEIGHT_MIN, SPAWN_HEIGHT_MAX);

		Quaternion rndRot = QuaternionFromEuler(GetRandomFloat(0, 360), GetRandomFloat(0, 360),
												GetRandomFloat(0, 360));
		desc.transform.rotation = ToTicsQuat(rndRot);

		dyn_objs[i].body = tics_world_add_rigid_body(phys_world, desc);
	}

	// ----------------------------------------------------------------------------------
	// Main Loop
	// ----------------------------------------------------------------------------------
	while (!WindowShouldClose()) {
		float dt = GetFrameTime();
		UpdateCamera(&camera, CAMERA_FREE);

		// Step Physics
		tics_world_step(phys_world, dt);

		BeginDrawing();
		ClearBackground(RAYWHITE);
		BeginMode3D(camera);

		// Render Ground
		for (int i = 0; i < groundScene.count; i++) {
			// Ground is static, so we can just use the transform loaded from the file
			// (or if we wanted to support moving kinematic platforms, we'd query the body here)
			SceneObject* obj = &groundScene.objects[i];
			obj->model.transform = obj->transform;
			DrawModel(obj->model, (Vector3){0, 0, 0}, 1.0f, LIGHTGRAY);
			DrawModelWires(obj->model, (Vector3){0, 0, 0}, 1.0f, GRAY);
		}

		// Render Dynamic Objects
		for (int i = 0; i < DYNAMIC_OBJ_COUNT; i++) {
			tics_transform t = tics_body_get_transform(phys_world, dyn_objs[i].body);

			Matrix matRot = QuaternionToMatrix(ToRaylibQuat(t.rotation));
			Matrix matTrans = MatrixTranslate(t.position.x, t.position.y, t.position.z);

			dyn_objs[i].model.transform = MatrixMultiply(matRot, matTrans);

			DrawModel(dyn_objs[i].model, (Vector3){0, 0, 0}, 1.0f, MAROON);
			DrawModelWires(dyn_objs[i].model, (Vector3){0, 0, 0}, 1.0f, BLACK);
		}

		EndMode3D();
		DrawFPS(10, 10);
		EndDrawing();
	}

	// ----------------------------------------------------------------------------------
	// Cleanup
	// ----------------------------------------------------------------------------------
	UnloadSceneObjects(groundScene);
	for (int i = 0; i < config_count; i++)
		UnloadSceneObjects(assets[i].scene_data);
	free(assets);
	free(dyn_objs);

	tics_world_destroy(phys_world);
	CloseWindow();

	return 0;
}
