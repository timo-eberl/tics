#include "raylib_util.h"

#include <raymath.h>

#include <math.h> // Required for acosf
#include <stdlib.h>
#include <string.h>

void update_fly_camera(Camera3D* camera) {
	float dt = GetFrameTime();

	if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) DisableCursor();
	if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) EnableCursor();

	// Look around (RMB + move mouse or arrow keys)
	Vector2 look_input = {0};
	// Mouse Input (Right Click Drag)
	if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
		Vector2 d = GetMouseDelta();
		look_input.x = d.x * 0.003f;
		look_input.y = d.y * 0.003f;
	}
	// Keyboard Input (Arrow Keys) - Good for touchpads
	// Multiplied by 2.0 * dt for consistent rotation speed
	float key_look_speed = 1.3f * dt;
	if (IsKeyDown(KEY_LEFT)) look_input.x -= key_look_speed;
	if (IsKeyDown(KEY_RIGHT)) look_input.x += key_look_speed;
	if (IsKeyDown(KEY_UP)) look_input.y -= key_look_speed;
	if (IsKeyDown(KEY_DOWN)) look_input.y += key_look_speed;
	// Apply Rotation
	if (look_input.x != 0 || look_input.y != 0) {
		Vector3 fwd = Vector3Normalize(Vector3Subtract(camera->target, camera->position));
		// Yaw (Rotate around global Y)
		fwd = Vector3RotateByAxisAngle(fwd, (Vector3){0, 1, 0}, -look_input.x);

		// Pitch (Rotate around local Right)
		Vector3 right = Vector3CrossProduct(fwd, camera->up);
		// Calculate current angle from Up (0 = Up, PI = Down)
		float current_angle = acosf(Clamp(Vector3DotProduct(fwd, camera->up), -1.0f, 1.0f));
		// Mouse Down (+y) -> Pitch Down -> Increase Angle
		float target_angle = current_angle + look_input.y;
		// Clamp to avoid Gimbal Lock (0.001 rad buffer)
		float clamped_angle = Clamp(target_angle, 0.001f, PI - 0.001f);
		// Calculate actual rotation needed (Negative around Right = Pitch Down)
		float apply_pitch = -(clamped_angle - current_angle);
		fwd = Vector3RotateByAxisAngle(fwd, right, apply_pitch);

		camera->target = Vector3Add(camera->position, fwd);
	}

	// Movement speed
	static float speed = 10.0f;
	// Scroll Wheel
	speed *= (1.0f + 0.1f * GetMouseWheelMove());
	// Keyboard fallback (useful for touchpads)
	if (IsKeyDown(KEY_PERIOD)) speed *= 1.02f; // Increase
	if (IsKeyDown(KEY_COMMA)) speed *= 0.98f;  // Decrease

	if (speed < 0.1f) speed = 0.1f;
	if (speed > 500.0f) speed = 500.0f;

	// Movement (WASD + QE)
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
		dir = Vector3Scale(Vector3Normalize(dir), speed * dt);
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
