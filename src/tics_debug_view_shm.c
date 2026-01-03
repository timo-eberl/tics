#ifdef TICS_ENABLE_DEBUG_VIEW

#include "tics_internal.h"

#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <unistd.h>

static tics_view_shm_header* shm = NULL;
static int current_buf_idx = 0;
static int shm_fd = -1;
static pid_t viewer_pid = -1;

void tics_view_init(void) {
	// 1. Create/Open Shared Memory
	shm_fd = shm_open(TICS_SHM_NAME, O_CREAT | O_RDWR, 0666);
	if (shm_fd == -1) {
		perror("shm_open failed");
		return;
	}

	// 2. Resize
	if (ftruncate(shm_fd, sizeof(tics_view_shm_header)) == -1) {
		perror("ftruncate failed");
		return;
	}

	// 3. Map
	shm = mmap(0, sizeof(tics_view_shm_header), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
	if (shm == MAP_FAILED) {
		perror("mmap failed");
		shm = NULL;
		return;
	}

	// 4. Init Atomic & Buffers
	atomic_init(&shm->latest_buffer_idx, 0);
	shm->buffers[0].count = 0;
	shm->buffers[1].count = 0;

	// 5. Spawn the viewer process
	pid_t pid = fork();
	if (pid == 0) {
		// If parent dies, send SIGTERM to this child immediately.
		prctl(PR_SET_PDEATHSIG, SIGTERM);
		// Safety check: If parent died BEFORE the prctl call, getppid becomes 1 (init)
		if (getppid() == 1) exit(1);

		// CHILD PROCESS: Execute the viewer
		execl("./tics_debug_viewer", "./tics_debug_viewer", NULL);
		perror("Failed to spawn tics_debug_viewer");
		exit(1);
	}
	viewer_pid = pid; // Remember the child
	printf("[TICS] View Initialized. Spawning viewer (PID: %d)\n", pid);
}

void tics_view_shutdown(void) {
	// Explicitly kill the viewer if we are shutting down gracefully
	if (viewer_pid > 0) {
		kill(viewer_pid, SIGTERM);
		viewer_pid = -1;
	}

	// Cleanup memory
	if (shm) munmap(shm, sizeof(tics_view_shm_header));
	if (shm_fd != -1) close(shm_fd);
	shm_unlink(TICS_SHM_NAME);
}

void tics_view_start_frame(void) {
	if (!shm) return;
	// Write to the buffer that ISN'T currently published
	current_buf_idx = !atomic_load(&shm->latest_buffer_idx);
	shm->buffers[current_buf_idx].count = 0;
}

void tics_view_update_frame(void) {
	if (!shm) return;

	// Publish the current state to the viewer
	shm->buffers[current_buf_idx].seq++;
	atomic_store(&shm->latest_buffer_idx, current_buf_idx);

	// We must swap buffers because we just gave ownership of the current one to the viewer
	int next_idx = !current_buf_idx;

	// Copy existing data to the new buffer to preserve what we have already drawn
	tics_view_buffer* src = &shm->buffers[current_buf_idx];
	tics_view_buffer* dst = &shm->buffers[next_idx];
	dst->count = src->count;
	if (src->count > 0) { memcpy(dst->cmds, src->cmds, src->count * sizeof(tics_view_cmd)); }

	// Switch our local write index
	current_buf_idx = next_idx;
}

void tics_view_end_frame(void) {
	if (!shm) return;
	// Increment sequence to signal "Write Complete"
	shm->buffers[current_buf_idx].seq++;
	// Publish new index
	atomic_store(&shm->latest_buffer_idx, current_buf_idx);
}

// --- Recording Implementation ---

static tics_view_cmd* push_cmd(void) {
	if (!shm) return NULL;
	tics_view_buffer* buf = &shm->buffers[current_buf_idx];
	if (buf->count >= TICS_VIEW_MAX_CMDS) return NULL;
	return &buf->cmds[buf->count++];
}

void tics_view_record_line(tics_vec3 s, tics_vec3 e, uint32_t color) {
	tics_view_cmd* cmd = push_cmd();
	if (!cmd) return;
	cmd->type = TICS_VIEW_CMD_LINE;
	cmd->color = color;
	cmd->data.line.start = (tics_view_vec3){s.x, s.y, s.z};
	cmd->data.line.end = (tics_view_vec3){e.x, e.y, e.z};
}

void tics_view_record_point(tics_vec3 p, float r, uint32_t color) {
	tics_view_cmd* cmd = push_cmd();
	if (!cmd) return;
	cmd->type = TICS_VIEW_CMD_POINT;
	cmd->color = color;
	cmd->data.point.pos = (tics_view_vec3){p.x, p.y, p.z};
	cmd->data.point.radius = r;
}

#endif // TICS_ENABLE_DEBUG_VIEW
