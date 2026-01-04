#include <blick.h>

#include <stdio.h>
#include <unistd.h>

// Doesn't use tics. Showcases the functionality of Blick, the debug viewer.

int main() {
	// Initialize (Forks the viewer process)
	blick_init("blick_viewer");

	printf("[HOST] Simulation starting...\n");
	blick_vec3 pos = {0, 20, 0};

	// Simulation Loop
	for (int i = 0; i < 50; i++) {
		blick_start_frame();

		pos.y -= 0.5f;

		blick_record_point(pos, 0.05f, 0xFF00FF00, true);  // light green trail (permanent)
		blick_record_point(pos, 0.2f, 0xFF003300, false); // dark green

		blick_update_frame();

		usleep(100000);

		blick_vec3 vel_end = {pos.x, pos.y - 1.0f, pos.z};
		blick_record_line(pos, vel_end, 0xFFFF0000, false); // Red

		blick_end_frame();

		usleep(100000);
	}

	printf("[HOST] Simulation finished. Shutting down.\n");
	blick_shutdown();
	return 0;
}
