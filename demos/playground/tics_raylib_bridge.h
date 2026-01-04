#ifndef TICS_RAYLIB_BRIDGE_H
#define TICS_RAYLIB_BRIDGE_H

#include <raylib.h>
#include <raymath.h>
#include <tics.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* -------------------------------------------------------------------------------------------------
Integration layer between Raylib (Rendering) and Tics (Physics).

- Maps fundamental types (Vectors, Quaternions) between libraries.
- Converts spatial transforms (Physics Position/Rot <-> Render Matrix).
- Provides utility functions for raylib model creation and camera controls.
------------------------------------------------------------------------------------------------- */

// clang-format off
static tics_vec3 to_tics_vec(Vector3 v) { return (tics_vec3){v.x, v.y, v.z}; }
static tics_quat to_tics_quat(Quaternion q) { return (tics_quat){q.x, q.y, q.z, q.w}; }
static Vector3 to_raylib_vec(tics_vec3 v) { return (Vector3){v.x, v.y, v.z}; }
static Quaternion to_raylib_quat(tics_quat q) { return (Quaternion){q.x, q.y, q.z, q.w}; }
// clang-format on
static Matrix to_raylib_matrix(tics_transform t) {
	Matrix mat_rotation = QuaternionToMatrix(to_raylib_quat(t.rotation));
	Matrix mat_translation = MatrixTranslate(t.position.x, t.position.y, t.position.z);
	return MatrixMultiply(mat_rotation, mat_translation);
}
static tics_transform to_tics_transform(Matrix m) {
	tics_vec3 position = {m.m12, m.m13, m.m14};
	tics_quat rotation = to_tics_quat(QuaternionFromMatrix(m));
	return (tics_transform){position, rotation};
}

// look around: hold right mouse button + move mouse
// fly around: WASD + QE
static void update_fly_camera(Camera3D* camera) {
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

static Model create_raylib_model(tics_vec3* vertices, int vertexCount, uint32_t* indices,
								 int indexCount) {
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

#endif // TICS_RAYLIB_BRIDGE_H
