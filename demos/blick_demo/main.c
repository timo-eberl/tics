#include <blick.h>

#include <math.h>
#include <stdio.h>
#include <unistd.h>

// Showcases the functionality of Blick, the debug viewer. Does not use Tics.
// Uses all supported primitive types.

int main() {
	// Initialize (Forks the viewer process)
	blick_init("blick_viewer");

	printf("[HOST] Simulation starting...\n");

	// --- Setup Mesh (Type: BLICK_CMD_DRAW_MESH) ---
	// Winding Order: CCW (Counter-Clockwise)
	// This ensures normals point OUTWARD for correct lighting calculation.
	blick_vec3 pyramid_verts[] = {
		// Back Face
		{1.0f, -1.0f, -1.0f},
		{-1.0f, -1.0f, -1.0f},
		{0.0f, 1.0f, 0.0f},
		// Right Face
		{1.0f, -1.0f, 1.0f},
		{1.0f, -1.0f, -1.0f},
		{0.0f, 1.0f, 0.0f},
		// Front Face
		{-1.0f, -1.0f, 1.0f},
		{1.0f, -1.0f, 1.0f},
		{0.0f, 1.0f, 0.0f},
		// Left Face
		{-1.0f, -1.0f, -1.0f},
		{-1.0f, -1.0f, 1.0f},
		{0.0f, 1.0f, 0.0f},
	};
	int mesh_id = 1;
	blick_upload_mesh(mesh_id, pyramid_verts, 12);

	float time = 0.0f;

	// Static objects on layer 1 (never cleared)

	// A static blue ground line
	blick_vec3 l_start = {-20.0f, 0, 20.0f};
	blick_vec3 l_end = {20.0f, 0, -20.0f};
	blick_record_line(1, l_start, l_end, 0xFFFF0000);

	// A standalone floating triangle (Cyan)
	blick_vec3 t_a = {-5.0f, 2.0f, 5.0f};
	blick_vec3 t_b = {-3.0f, 2.0f, 5.0f};
	blick_vec3 t_c = {-4.0f, 4.0f, 5.0f};
	blick_record_triangle(1, t_a, t_b, t_c, 0xFFFFFF00);

	// A bounding box surrounding the triangle above (Yellow)
	blick_vec3 min = {-5.5f, 1.5f, 4.5f};
	blick_vec3 max = {-2.5f, 4.5f, 5.5f};
	blick_record_aabb(1, min, max, 0xFF00FFFF);

	// A static capsule (Magenta)
	blick_vec3 cap_a = {8.0f, 0.0f, 0.0f};
	blick_vec3 cap_b = {10.0f, 3.0f, 2.0f};
	blick_record_capsule(1, cap_a, cap_b, 0.75f, 0xFFFF00FF, false); // Solid
	blick_record_capsule(1, cap_a, cap_b, 0.75f, 0xFFFFFFFF, true);  // Wireframe outline

	// Simulation Loop: Dynamic objects on layer 0 (cleared every frame)
	while (true) {
		time += 0.05f;

		// Point bouncing up and down
		blick_vec3 ball_pos = {0.0f, fabs(sinf(time * 2.0f)) * 5.0f + 1.0f, 0.0f};
		blick_record_point(0, ball_pos, 0.5f, 0xFF00FF00); // Green Ball

		// Arrow visualizing velocity/direction on the ball
		blick_vec3 arrow_end = ball_pos;
		arrow_end.y += cosf(time * 2.0f) * 2.0f; // roughly the derivative
		blick_record_arrow(0, ball_pos, arrow_end, 0xFFFF00FF); // Purple Arrow

		// Text label following the ball
		blick_vec3 text_pos = ball_pos;
		blick_record_text(0, ball_pos, "Bouncing Ball", 0xFFFF99FF);

		// A spinning coordinate system gizmo (transform)
		blick_vec3 trans_pos = {5.0f, 2.0f, 5.0f};
		// Rotate around Y axis
		blick_quat trans_rot = {0.0f, sinf(time), 0.0f, cosf(time)};
		blick_record_transform(0, trans_pos, trans_rot, 1.0f);

		// Rotating Mesh
		blick_vec3 mesh_pos = {5.0f, 5.0f, -5.0f};
		blick_record_mesh(0, mesh_id, mesh_pos, trans_rot, 0xFF0000FF, false); // Solid Red
		blick_record_mesh(0, mesh_id, mesh_pos, trans_rot, 0xFFFFFFFF, true); // White Wireframe

		// An orbiting sphere (Orange)
		blick_vec3 sphere_pos = {-5.0f, 2.0f, -5.0f};
		sphere_pos.x += cosf(time * 1.5f) * 2.0f;
		sphere_pos.z += sinf(time * 1.5f) * 2.0f;
		blick_quat identity_rot = {0.0f, 0.0f, 0.0f, 1.0f};
		blick_record_sphere(0, sphere_pos, identity_rot, 1.0f, 0xFF008CFF, false);
		blick_record_sphere(0, sphere_pos, identity_rot, 1.0f, 0xFF00FFFF, true);

		// A spinning OBB (Oriented Bounding Box) (Green)
		blick_vec3 obb_pos = {0.0f, 5.0f, -5.0f};
		blick_vec3 obb_ext = {2.0f, 1.0f, 0.5f};
		blick_record_obb(0, obb_pos, trans_rot, obb_ext, 0xFF00FF00, false);
		blick_record_obb(0, obb_pos, trans_rot, obb_ext, 0xFF000000, true);

		blick_refresh();
		blick_clear(0b1); // clear layer 0
		usleep(100000);
	}

	printf("[HOST] Simulation finished.\n");
	blick_shutdown();
	return 0;
}
