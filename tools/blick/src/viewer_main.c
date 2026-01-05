#include "blick_protocol.h"

#include <raylib_util.h>
#include <raymath.h>
#include <rlgl.h>

#include <fcntl.h>
#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

// Calculates face normal of a triangle and applies view-dependent lighting
Color shade_triangle(Vector3 v0, Vector3 v1, Vector3 v2, Vector3 cam_pos, Color base_color) {
	Vector3 edge1 = Vector3Subtract(v1, v0);
	Vector3 edge2 = Vector3Subtract(v2, v0);
	Vector3 normal = Vector3Normalize(Vector3CrossProduct(edge1, edge2));
	Vector3 light_dir = Vector3Normalize(Vector3Subtract(cam_pos, v0)); // approximation

	float n_dot_l = Vector3DotProduct(normal, light_dir);
	n_dot_l = fabsf(n_dot_l); // use abs so backfaces are shaded too
	float intensity = fmax(pow(n_dot_l, 0.2), 0.7);

	return (Color){(unsigned char)(base_color.r * intensity),
				   (unsigned char)(base_color.g * intensity),
				   (unsigned char)(base_color.b * intensity), base_color.a};
}

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

	// Tell raylib to shut up
	SetTraceLogLevel(LOG_NONE);

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
		// --- DATA SYNC ---
		// Identify the latest frame
		uint32_t idx = atomic_load(&shm->latest_buffer_idx);
		// Claim the buffer so the host doesn't overwrite it
		atomic_store(&shm->reading_idx, idx);
		// Race condition check: Did the host update 'latest' while we were claiming it?
		if (atomic_load(&shm->latest_buffer_idx) != idx) {
			// Race condition detected! Skip this frame and try again next loop
			atomic_store(&shm->reading_idx, 0xFFFFFFFF);
			continue;
		}
		// Copy to local memory
		blick_buffer local_buf = shm->buffers[idx];
		// Release the claim
		atomic_store(&shm->reading_idx, 0xFFFFFFFF);

		update_fly_camera(&camera);

		rlDisableBackfaceCulling();
		BeginDrawing();
		{
			ClearBackground((Color){30, 30, 30, 255});
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
						Vector3 p = {cmd->data.point.pos.x, cmd->data.point.pos.y,
									 cmd->data.point.pos.z};
						float r = cmd->data.point.radius;

						// Draw a 3D Crosshair centered at the point
						DrawLine3D((Vector3){p.x - r, p.y, p.z}, (Vector3){p.x + r, p.y, p.z},
								   color);
						DrawLine3D((Vector3){p.x, p.y - r, p.z}, (Vector3){p.x, p.y + r, p.z},
								   color);
						DrawLine3D((Vector3){p.x, p.y, p.z - r}, (Vector3){p.x, p.y, p.z + r},
								   color);
					} break;

					case BLICK_CMD_LINE: {
						Vector3 start = {cmd->data.line.start.x, cmd->data.line.start.y,
										 cmd->data.line.start.z};
						Vector3 end = {cmd->data.line.end.x, cmd->data.line.end.y,
									   cmd->data.line.end.z};
						DrawLine3D(start, end, color);
					} break;

					case BLICK_CMD_TRIANGLE: {
						Vector3 a = {cmd->data.triangle.a.x, cmd->data.triangle.a.y,
									 cmd->data.triangle.a.z};
						Vector3 b = {cmd->data.triangle.b.x, cmd->data.triangle.b.y,
									 cmd->data.triangle.b.z};
						Vector3 c = {cmd->data.triangle.c.x, cmd->data.triangle.c.y,
									 cmd->data.triangle.c.z};

						Color shaded = shade_triangle(a, b, c, camera.position, color);
						DrawTriangle3D(a, b, c, shaded);
					} break;

						// TODO Add other primitives...

					case BLICK_CMD_DRAW_MESH: {
						rlPushMatrix();
						Quaternion q = {cmd->data.mesh.rot.x, cmd->data.mesh.rot.y,
										cmd->data.mesh.rot.z, cmd->data.mesh.rot.w};
						Matrix mat = QuaternionToMatrix(q);
						// Inject translation directly into the matrix (Column-Major: m12, m13, m14)
						mat.m12 = cmd->data.mesh.pos.x;
						mat.m13 = cmd->data.mesh.pos.y;
						mat.m14 = cmd->data.mesh.pos.z;
						rlMultMatrixf(MatrixToFloat(mat));

						rlColor4ub(color.r, color.g, color.b, color.a);

						uint32_t offset = cmd->data.mesh.offset;
						uint32_t v_count = cmd->data.mesh.vertex_count;
						float* pool = shm->mesh_pool;

						if (cmd->data.mesh.wireframe) {
							// Simply using RL_LINES on a triangle mesh doesn't work correctly
							// Expand Triangles (3 verts) into Lines (6 verts)
							rlBegin(RL_LINES);
							for (uint32_t v = 0; v < v_count; v += 3) {
								if (v + 2 >= v_count) break;

								float* p0 = &pool[offset + ((v + 0) * 3)];
								float* p1 = &pool[offset + ((v + 1) * 3)];
								float* p2 = &pool[offset + ((v + 2) * 3)];
								// clang-format off
								// Edge 0-1, Edge 1-2, Edge 2-0
								rlVertex3f(p0[0], p0[1], p0[2]); rlVertex3f(p1[0], p1[1], p1[2]);
								rlVertex3f(p1[0], p1[1], p1[2]); rlVertex3f(p2[0], p2[1], p2[2]);
								rlVertex3f(p2[0], p2[1], p2[2]); rlVertex3f(p0[0], p0[1], p0[2]);
								// clang-format on
							}
							rlEnd();
						}
						else {
							// Calculate local camera position for lighting
							Vector3 local_cam_pos = Vector3Transform(camera.position, MatrixInvert(mat));

							rlBegin(RL_TRIANGLES);
							for (uint32_t v = 0; v < v_count; v += 3) {
								if (v + 2 >= v_count) break;

								float* p0 = &pool[offset + ((v + 0) * 3)];
								float* p1 = &pool[offset + ((v + 1) * 3)];
								float* p2 = &pool[offset + ((v + 2) * 3)];
								Vector3 v0 = {p0[0], p0[1], p0[2]};
								Vector3 v1 = {p1[0], p1[1], p1[2]};
								Vector3 v2 = {p2[0], p2[1], p2[2]};

								// Lighting in local space
								Color shaded = shade_triangle(v0, v1, v2, local_cam_pos, color);
								rlColor4ub(shaded.r, shaded.g, shaded.b, shaded.a);

								rlVertex3f(v0.x, v0.y, v0.z);
								rlVertex3f(v1.x, v1.y, v1.z);
								rlVertex3f(v2.x, v2.y, v2.z);
							}
							rlEnd();
						}

						rlPopMatrix();
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

	munmap(shm, sizeof(blick_shm_header));
	close(fd);
	CloseWindow();

	return 0;
}
