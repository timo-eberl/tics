#ifdef TICS_ENABLE_DEBUG_VIEW

#include "tics_debug_view_shm.h"
#include "tics_math.h"

#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
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

// Storage for Persistent Commands
static tics_view_cmd persistent_cmds[TICS_VIEW_MAX_CMDS];
static uint32_t persistent_count = 0;

void tics_view_init(void) {
	// Create/Open Shared Memory
	shm_fd = shm_open(TICS_SHM_NAME, O_CREAT | O_RDWR, 0666);
	if (shm_fd == -1) {
		perror("shm_open failed");
		return;
	}

	// Resize
	if (ftruncate(shm_fd, sizeof(tics_view_shm_header)) == -1) {
		perror("ftruncate failed");
		return;
	}

	// Map
	shm = mmap(0, sizeof(tics_view_shm_header), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
	if (shm == MAP_FAILED) {
		perror("mmap failed");
		shm = NULL;
		return;
	}

	// Zero out the memory to remove data from previous crashed runs
	memset(shm, 0, sizeof(tics_view_shm_header));
	atomic_init(&shm->latest_buffer_idx, 0);

	// Spawn the viewer process
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
	tics_view_buffer* buf = &shm->buffers[current_buf_idx];

	// Pre-fill the new frame with persistent commands
	buf->count = 0;
	if (persistent_count > 0) {
		memcpy(buf->cmds, persistent_cmds, persistent_count * sizeof(tics_view_cmd));
		buf->count = persistent_count;
	}
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

void tics_view_clear_permanent(void) {
	persistent_count = 0;
}

static inline tics_view_vec3 v3_to_shm(tics_vec3 v) {
	return (tics_view_vec3){v.x, v.y, v.z};
}
static inline tics_view_quat q_to_shm(tics_quat q) {
	return (tics_view_quat){q.x, q.y, q.z, q.w};
}

// --- Recording Implementation ---

// Helper to submit a fully constructed command
static void submit_cmd(tics_view_cmd cmd, bool permanent) {
	if (!shm) return;

	// Always write to the current frame buffer (so it appears immediately)
	tics_view_buffer* buf = &shm->buffers[current_buf_idx];
	if (buf->count < TICS_VIEW_MAX_CMDS) { buf->cmds[buf->count++] = cmd; }

	// If permanent, save to persistent storage
	if (permanent) {
		if (persistent_count < TICS_VIEW_MAX_CMDS) { persistent_cmds[persistent_count++] = cmd; }
	}
}

void tics_view_record_line(tics_vec3 s, tics_vec3 e, uint32_t color, bool permanent) {
	tics_view_cmd cmd;
	cmd.type = TICS_VIEW_CMD_LINE;
	cmd.color = color;
	cmd.data.line.start = v3_to_shm(s);
	cmd.data.line.end = v3_to_shm(e);
	submit_cmd(cmd, permanent);
}

void tics_view_record_arrow(tics_vec3 s, tics_vec3 e, uint32_t color, bool permanent) {
	tics_view_cmd cmd;
	cmd.type = TICS_VIEW_CMD_ARROW;
	cmd.color = color;
	cmd.data.arrow.start = v3_to_shm(s);
	cmd.data.arrow.end = v3_to_shm(e);
	submit_cmd(cmd, permanent);
}

void tics_view_record_point(tics_vec3 p, float r, uint32_t color, bool permanent) {
	tics_view_cmd cmd;
	cmd.type = TICS_VIEW_CMD_POINT;
	cmd.color = color;
	cmd.data.point.pos = v3_to_shm(p);
	cmd.data.point.radius = r;
	submit_cmd(cmd, permanent);
}

void tics_view_record_aabb(tics_vec3 min, tics_vec3 max, uint32_t color, bool permanent) {
	tics_view_cmd cmd;
	cmd.type = TICS_VIEW_CMD_AABB;
	cmd.color = color;
	cmd.data.aabb.min = v3_to_shm(min);
	cmd.data.aabb.max = v3_to_shm(max);
	submit_cmd(cmd, permanent);
}

void tics_view_record_triangle(tics_vec3 a, tics_vec3 b, tics_vec3 c, uint32_t color,
							   bool permanent) {
	tics_view_cmd cmd;
	cmd.type = TICS_VIEW_CMD_TRIANGLE;
	cmd.color = color;
	cmd.data.triangle.a = v3_to_shm(a);
	cmd.data.triangle.b = v3_to_shm(b);
	cmd.data.triangle.c = v3_to_shm(c);
	submit_cmd(cmd, permanent);
}

void tics_view_record_transform(tics_transform t, bool permanent) {
	tics_view_cmd cmd;
	cmd.type = TICS_VIEW_CMD_TRANSFORM;
	cmd.color = 0xFFFFFFFF; // Ignored for Transform
	cmd.data.transform.pos = v3_to_shm(t.position);
	cmd.data.transform.rot = q_to_shm(t.rotation);
	submit_cmd(cmd, permanent);
}

void tics_view_record_text(tics_vec3 pos, const char* text, uint32_t color, bool permanent) {
	tics_view_cmd cmd;
	cmd.type = TICS_VIEW_CMD_TEXT;
	cmd.color = color;
	cmd.data.text.pos = v3_to_shm(pos);
	strncpy(cmd.data.text.buffer, text, TICS_TEXT_MAX_LEN - 1);
	cmd.data.text.buffer[TICS_TEXT_MAX_LEN - 1] = '\0';
	submit_cmd(cmd, permanent);
}

#endif // TICS_ENABLE_DEBUG_VIEW
