#include <blick.h>

#include <math.h>
#include <stdio.h>
#include <unistd.h>

// Doesn't use tics. Showcases the functionality of Blick, the debug viewer.

int main() {
	// Initialize (Forks the viewer process)
	blick_init("blick_viewer");

	printf("[HOST] Simulation starting...\n");

	// Define a simple triangle
	blick_vec3 triangle_verts[] = {{-1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.5f, 0.0f}};
	int mesh_id = 5;
	blick_upload_mesh(mesh_id, triangle_verts, 3);

	blick_vec3 pos = {mesh_id, 20, 0};

	// Simulation Loop
	for (int i = 0; i < 1000; i++) {
		blick_start_frame();

		// 1. Primitive Animation
		pos.y -= 0.5f;
		blick_record_point(pos, 0.05f, 0xFF00FF00, true); // light green trail (permanent)
		blick_record_point(pos, 0.2f, 0xFF003300, false); // dark green head

		blick_vec3 vel_end = {pos.x, pos.y - 1.0f, pos.z};
		blick_record_line(pos, vel_end, 0xFFFF0000, false); // Red velocity vector

		// Mesh Animation
		// Rotate around Y axis: q = [0, sin(t/2), 0, cos(t/2)]
		float angle = i * 0.1f;
		blick_quat rot = {0.0f, sinf(angle), 0.0f, cosf(angle)};
		blick_vec3 mesh_pos = {5.0f, 5.0f, 0.0f};

		// Draw the solid mesh (Red)
		blick_record_mesh(mesh_id, mesh_pos, rot, 0xFF0000FF, false, false);
		// Draw the wireframe (Blue)
		blick_record_mesh(mesh_id, mesh_pos, rot, 0xFFFF0000, true, false);

		blick_end_frame();
		usleep(50000);
	}

	printf("[HOST] Simulation finished. Shutting down.\n");
	blick_shutdown();
	return 0;
}
