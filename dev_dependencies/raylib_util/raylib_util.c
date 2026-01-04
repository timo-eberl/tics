#include "raylib_util.h"

#include <raymath.h>

#include <stdlib.h>
#include <string.h>

void update_fly_camera(Camera3D* camera) {
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

	Vector3 dir = {0};
	Vector3 fwd = Vector3Normalize(Vector3Subtract(camera->target, camera->position));
	Vector3 right = Vector3Normalize(Vector3CrossProduct(fwd, camera->up));

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

Model create_raylib_model(float* vertices, int vertexCount, uint32_t* indices, int indexCount) {
	Mesh mesh = {0};
	mesh.vertexCount = vertexCount;
	mesh.triangleCount = indexCount / 3;

	// Copy Vertices
	mesh.vertices = (float*)malloc(vertexCount * 3 * sizeof(float));
	memcpy(mesh.vertices, vertices, vertexCount * 3 * sizeof(float));

	// Copy Indices (Convert 32-bit to 16-bit)
	mesh.indices = (unsigned short*)malloc(indexCount * sizeof(unsigned short));
	for (int i = 0; i < indexCount; i++)
		mesh.indices[i] = (unsigned short)indices[i];

	UploadMesh(&mesh, false);
	return LoadModelFromMesh(mesh);
}
