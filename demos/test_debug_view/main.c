#include <tics.h>
#include <tics_internal.h>

#include <stdio.h>
#include <unistd.h>

// Doesn't use tics. Showcases the functionality of the debug viewer.

int main() {
	// 1. Initialize (Forks the viewer process)
	TICS_VIEW_INIT();

	printf("[HOST] Simulation starting...\n");
	tics_vec3 pos = {0, 20, 0};

	// 2. Simulation Loop
	for (int i = 0; i < 500; i++) {
		TICS_VIEW_FRAME_START();

		// Update fake physics
		pos.y -= 0.5f;

		// View Logic: Record a point and a velocity line
		tics_vec3 vel_end = {pos.x, pos.y - 1.0f, pos.z};

		TICS_VIEW_POINT(pos, 0.5f, 0xFF00FF00); // Green

		TICS_VIEW_FRAME_UPDATE();

		usleep(500000);

		TICS_VIEW_LINE(pos, vel_end, 0xFFFF0000); // Red

		TICS_VIEW_FRAME_END();

		usleep(500000);
	}

	printf("[HOST] Simulation finished. Shutting down.\n");
	TICS_VIEW_SHUTDOWN();
	return 0;
}
