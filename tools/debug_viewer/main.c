#include <tics_debug_view_shm.h>

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

int main() {
	printf("[VIEWER] Waiting for shared memory connection...\n");

	int fd = -1;
	while (fd == -1) {
		fd = shm_open(TICS_SHM_NAME, O_RDWR, 0666);
		if (fd == -1) usleep(100000); // 100ms retry
	}

	tics_view_shm_header* shm = mmap(0, sizeof(tics_view_shm_header), PROT_READ, MAP_SHARED, fd, 0);
	if (shm == MAP_FAILED) {
		perror("mmap");
		return 1;
	}

	printf("[VIEWER] Connected. Polling data (10Hz)...\n");

	while (1) {
		// Read the latest buffer index atomically
		uint32_t idx = atomic_load(&shm->latest_buffer_idx);
		uint32_t seq = atomic_load(&shm->buffers[idx].seq);

		tics_view_buffer* buf = &shm->buffers[idx];

		printf("[VIEWER] Frame Seq: %u | Count: %u\n", seq, buf->count);

		for (uint32_t i = 0; i < buf->count; i++) {
			tics_view_cmd* cmd = &buf->cmds[i];
			if (cmd->type == TICS_VIEW_CMD_LINE) {
				printf("  LINE [C:%X]: (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f)\n", cmd->color,
					   cmd->data.line.start.x, cmd->data.line.start.y, cmd->data.line.start.z,
					   cmd->data.line.end.x, cmd->data.line.end.y, cmd->data.line.end.z);
			}
			else if (cmd->type == TICS_VIEW_CMD_POINT) {
				printf("  PNT  [C:%X]: (%.2f, %.2f, %.2f) R:%.2f\n", cmd->color,
					   cmd->data.point.pos.x, cmd->data.point.pos.y, cmd->data.point.pos.z,
					   cmd->data.point.radius);
			}
		}

		// Simulate 10 FPS rendering loop
		usleep(100000);
	}

	return 0;
}
