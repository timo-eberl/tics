#include "blick_protocol.h"

#include <raylib_util.h>

#include <fcntl.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

int main(void) {
	printf("[BLICK VIEWER] Waiting for shared memory connection...\n");
	int fd = -1;
	while (fd == -1) {
		fd = shm_open(BLICK_SHM_NAME, O_RDWR, 0666);
		if (fd == -1) usleep(100000); // 100ms retry
	}

	// Mapping with PROT_WRITE because the viewer updates the 'reading_idx' atomic
	blick_shm_header* shm =
		mmap(0, sizeof(blick_shm_header), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

	if (shm == MAP_FAILED) {
		perror("mmap");
		return 1;
	}
	printf("[BLICK VIEWER] Connected.\n");

	InitWindow(1280, 720, "Blick Debug Viewer");

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

	while (!WindowShouldClose()) {
		update_fly_camera(&camera);

		// --- DATA SYNC ---
		// Identify the latest frame
		uint32_t idx = atomic_load(&shm->latest_buffer_idx);
		// Claim the buffer so the host doesn't overwrite it
		atomic_store(&shm->reading_idx, idx);
		// Copy to local memory
		blick_buffer local_buf = shm->buffers[idx];
		// Release the claim
		atomic_store(&shm->reading_idx, 0xFFFFFFFF);

		BeginDrawing();
		{
			ClearBackground(RAYWHITE);
			BeginMode3D(camera);
			{
				// Draw Debug Data
				for (uint32_t i = 0; i < local_buf.count; i++) {
					blick_cmd* cmd = &local_buf.cmds[i];

					// Unpack Color: 0xAABBGGRR
					Color color;
					color.a = (cmd->color >> 24) & 0xFF;
					color.b = (cmd->color >> 16) & 0xFF;
					color.g = (cmd->color >> 8) & 0xFF;
					color.r = (cmd->color) & 0xFF;

					switch (cmd->type) {
					case BLICK_CMD_POINT: {
						Vector3 pos = {cmd->data.point.pos.x, cmd->data.point.pos.y,
									   cmd->data.point.pos.z};
						DrawSphere(pos, cmd->data.point.radius, color);
					} break;

					case BLICK_CMD_LINE: {
						Vector3 start = {cmd->data.line.start.x, cmd->data.line.start.y,
										 cmd->data.line.start.z};
						Vector3 end = {cmd->data.line.end.x, cmd->data.line.end.y,
									   cmd->data.line.end.z};
						DrawLine3D(start, end, color);
					} break;

					// TODO Implement other cases (AABB, Triangle, etc.)

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

	munmap(shm, sizeof(blick_shm_header));
	close(fd);
	CloseWindow();

	return 0;
}
