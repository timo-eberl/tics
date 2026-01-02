#ifndef DEMO_FRAMEWORK_H
#define DEMO_FRAMEWORK_H

#include <cgltf.h>
#include <raylib.h>
#include <tics.h>

// --------------------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------------------

typedef struct {
	// Visuals
	Model model;

	// Physics Data (Raw local-space vertices)
	tics_vec3* raw_vertices;
	int vertex_count;

	// The initial World Transform from the GLTF file
	// (Used to place static objects correctly)
	Matrix default_transform;
} DemoObject;

typedef struct {
	DemoObject* objects;
	int count;
} DemoScene;

// --------------------------------------------------------------------------------------
// Helpers
// --------------------------------------------------------------------------------------

tics_vec3 ToTicsVec(Vector3 v);
tics_quat ToTicsQuat(Quaternion q);
Vector3 ToRaylibVec(tics_vec3 v);
Quaternion ToRaylibQuat(tics_quat q);
float GetRandomFloat(float min, float max);

void UpdateFlyCamera(Camera3D* camera);

// Loads all meshes found in the GLB file
DemoScene LoadDemoScene(const char* filename);
void UnloadDemoScene(DemoScene scene);

#endif
