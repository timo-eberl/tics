#include "models_data.h"

#include <tics_debug_view_shm.h>
#include <tics_raylib_bridge.h>

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

int main(void) {
	printf("[VIEWER] Waiting for shared memory connection...\n");
	int fd = -1;
	while (fd == -1) {
		fd = shm_open(TICS_SHM_NAME, O_RDWR, 0666);
		if (fd == -1) usleep(100000); // 100ms retry
	}
	// write mode, because we write to 'reading_idx' to tell the host which buffer we are reading
	tics_view_shm_header* shm =
		mmap(0, sizeof(tics_view_shm_header), PROT_WRITE, MAP_SHARED, fd, 0);
	if (shm == MAP_FAILED) {
		perror("mmap");
		return 1;
	}
	printf("[VIEWER] Connected.\n");

	InitWindow(1280, 720, "Tics Debug Viewer");
	// Move the debug window to the top-right corner of the current monitor.
	// This prevents it from spawning directly on top of the main simulation window.
	set_window_top_right(0);

	SetTargetFPS(60);

	Camera3D camera = {0};
	camera.position = (Vector3){15.0f, 10.0f, 15.0f};
	camera.target = (Vector3){0.0f, 2.0f, 0.0f};
	camera.up = (Vector3){0.0f, 1.0f, 0.0f};
	camera.fovy = 45.0f;
	camera.projection = CAMERA_PERSPECTIVE;

	Model static_models[static_object_count];
	for (int i = 0; i < static_object_count; i++) {
		// Create raylib model and insert mesh data
		static_models[i] =
			create_raylib_model(ground_vertex_buffers[i], (int)ground_vertex_buffer_sizes[i],
								ground_index_buffers[i], (int)ground_index_buffer_sizes[i]);
		// Convert position and rotation into matrix for rendering
		tics_transform t = {ground_positions[i], ground_rotations[i]};
		static_models[i].transform = to_raylib_matrix(t);
	}

	while (!WindowShouldClose()) {
		update_fly_camera(&camera);

		// --- DATA SYNC ---
		// Identify the latest frame
		uint32_t idx = atomic_load(&shm->latest_buffer_idx);
		// Claim it (Tell host: "Do not overwrite buffer 'idx'")
		atomic_store(&shm->reading_idx, idx);
		// Copy to local memory
		tics_view_buffer local_buf = shm->buffers[idx];
		// Release the claim
		atomic_store(&shm->reading_idx, 0xFFFFFFFF);

		BeginDrawing();
		{
			ClearBackground(RAYWHITE);
			BeginMode3D(camera);
			{
				// Draw static geometry
				for (int i = 0; i < static_object_count; i++) {
					DrawModel(static_models[i], (Vector3){0}, 1.0f, LIGHTGRAY);
					DrawModelWires(static_models[i], (Vector3){0}, 1.0f, BLACK);
				}

				// Draw Debug Data
				for (uint32_t i = 0; i < local_buf.count; i++) {
					tics_view_cmd* cmd = &local_buf.cmds[i];

					// Unpack Color: 0xAABBGGRR
					Color color;
					color.a = (cmd->color >> 24) & 0xFF;
					color.b = (cmd->color >> 16) & 0xFF;
					color.g = (cmd->color >> 8) & 0xFF;
					color.r = (cmd->color) & 0xFF;

					switch (cmd->type) {
					case TICS_VIEW_CMD_POINT: {
						Vector3 pos = {cmd->data.point.pos.x, cmd->data.point.pos.y,
									   cmd->data.point.pos.z};
						DrawSphere(pos, cmd->data.point.radius, color);
					} break;
					default:
						break;
					}
				}

				DrawGrid(20, 1.0f);
			}
			EndMode3D();
		}
		EndDrawing();
	}

	// Cleanup
	for (int i = 0; i < static_object_count; i++)
		UnloadModel(static_models[i]);

	munmap(shm, sizeof(tics_view_shm_header));
	close(fd);
	CloseWindow();
	return 0;
}
