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

	// Simulation Loop
	while (true) {
		blick_start_frame();
		time += 0.05f;

		// --- 2. Point (Type: BLICK_CMD_POINT) ---
		// A ball bouncing up and down
		blick_vec3 ball_pos = {0.0f, fabs(sinf(time * 2.0f)) * 5.0f + 1.0f, 0.0f};
		blick_record_point(ball_pos, 0.5f, 0xFF00FF00, false); // Green Ball

		// --- 3. Arrow (Type: BLICK_CMD_ARROW) ---
		// Visualizing velocity/direction on the ball
		blick_vec3 arrow_end = ball_pos;
		arrow_end.y += cosf(time * 2.0f) * 2.0f;					// roughly the derivative
		blick_record_arrow(ball_pos, arrow_end, 0xFFFF00FF, false); // Purple Arrow

		// --- 4. Text (Type: BLICK_CMD_TEXT) ---
		// Label following the ball
		blick_vec3 text_pos = ball_pos;
		blick_record_text(ball_pos, "Bouncing Ball", 0xFFFF99FF, false);

		// --- 5. Line (Type: BLICK_CMD_LINE) ---
		// A static red ground line
		blick_vec3 l_start = {-10.0f, 0.0f, 0.0f};
		blick_vec3 l_end = {10.0f, 0.0f, 0.0f};
		blick_record_line(l_start, l_end, 0xFFFF0000, false);

		// --- 6. Triangle (Type: BLICK_CMD_TRIANGLE) ---
		// A standalone floating triangle (Cyan)
		blick_vec3 t_a = {-5.0f, 2.0f, 5.0f};
		blick_vec3 t_b = {-3.0f, 2.0f, 5.0f};
		blick_vec3 t_c = {-4.0f, 4.0f, 5.0f};
		blick_record_triangle(t_a, t_b, t_c, 0xFFFFFF00, false);

		// --- 7. AABB (Type: BLICK_CMD_AABB) ---
		// A bounding box surrounding the triangle above (Yellow)
		blick_vec3 min = {-5.5f, 1.5f, 4.5f};
		blick_vec3 max = {-2.5f, 4.5f, 5.5f};
		blick_record_aabb(min, max, 0xFF00FFFF, false);

		// --- 8. Transform (Type: BLICK_CMD_TRANSFORM) ---
		// A spinning coordinate system gizmo
		blick_vec3 trans_pos = {5.0f, 2.0f, 5.0f};
		// Rotate around Y axis
		blick_quat trans_rot = {0.0f, sinf(time), 0.0f, cosf(time)};
		blick_record_transform(trans_pos, trans_rot, 1.0f, false);

		// --- Mesh Drawing (Type: BLICK_CMD_DRAW_MESH) ---
		// Rotating Mesh from previous demo
		blick_vec3 mesh_pos = {5.0f, 5.0f, -5.0f};
		blick_record_mesh(mesh_id, mesh_pos, trans_rot, 0xFF0000FF, false, false); // Solid Red
		blick_record_mesh(mesh_id, mesh_pos, trans_rot, 0xFFFFFFFF, true, false); // White Wireframe

		blick_end_frame();
		usleep(100000);
	}

	printf("[HOST] Simulation finished.\n");
	blick_shutdown();
	return 0;
}
