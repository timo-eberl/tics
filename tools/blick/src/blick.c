#include "blick.h"

#include <assert.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

static blick_shm_header* shm = NULL;
static int current_buf_idx = 0;
static int shm_fd = -1;
static pid_t viewer_pid = -1;

static blick_cmd persistent_cmds[BLICK_MAX_PERS_CMDS];
static uint32_t persistent_count = 0;
static uint32_t persistent_head = 0;

void blick_init(const char* viewer_path) {
	shm_fd = shm_open(BLICK_SHM_NAME, O_CREAT | O_RDWR, 0666);
	if (shm_fd == -1) return;

	if (ftruncate(shm_fd, sizeof(blick_shm_header)) == -1) return;

	shm = mmap(0, sizeof(blick_shm_header), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
	if (shm == MAP_FAILED) {
		shm = NULL;
		return;
	}

	memset(shm, 0, sizeof(blick_shm_header));
	atomic_init(&shm->latest_buffer_idx, 0);
	atomic_init(&shm->reading_idx, 0xFFFFFFFF);

	pid_t pid = fork();
	if (pid == 0) {
		if (getppid() == 1) exit(1);
		execl(viewer_path, viewer_path, NULL);
		perror("Blick: Failed to spawn viewer");
		exit(1);
	}
	viewer_pid = pid;
}

void blick_shutdown(void) {
	if (viewer_pid > 0) {
		kill(viewer_pid, SIGTERM);
		viewer_pid = -1;
	}
	if (shm) munmap(shm, sizeof(blick_shm_header));
	if (shm_fd != -1) close(shm_fd);
	shm_unlink(BLICK_SHM_NAME);
}

static int get_next_free_buffer(void) {
	uint32_t latest = atomic_load(&shm->latest_buffer_idx);
	uint32_t reading = atomic_load(&shm->reading_idx);
	for (int i = 0; i < BLICK_SHM_BUFFER_COUNT; i++) {
		if (i != (int)latest && i != (int)reading) return i;
	}
	return (latest + 1) % BLICK_SHM_BUFFER_COUNT;
}

void blick_start_frame(void) {
	if (!shm) return;
	current_buf_idx = get_next_free_buffer();
	blick_buffer* buf = &shm->buffers[current_buf_idx];
	buf->count = 0;

	if (persistent_count > 0) {
		uint32_t copy_count =
			(persistent_count > BLICK_MAX_CMDS) ? BLICK_MAX_CMDS : persistent_count;
		uint32_t start_idx = (persistent_count < BLICK_MAX_PERS_CMDS) ? 0 : persistent_head;
		uint32_t first_chunk = BLICK_MAX_PERS_CMDS - start_idx;
		if (first_chunk > copy_count) first_chunk = copy_count;

		memcpy(buf->cmds, &persistent_cmds[start_idx], first_chunk * sizeof(blick_cmd));
		if (copy_count > first_chunk) {
			memcpy(&buf->cmds[first_chunk], &persistent_cmds[0],
				   (copy_count - first_chunk) * sizeof(blick_cmd));
		}
		buf->count = copy_count;
	}
}

void blick_update_frame(void) {
	if (!shm) return;
	atomic_store(&shm->latest_buffer_idx, current_buf_idx);
	int next_idx = get_next_free_buffer();
	blick_buffer* src = &shm->buffers[current_buf_idx];
	blick_buffer* dst = &shm->buffers[next_idx];
	dst->count = src->count;
	if (src->count > 0) memcpy(dst->cmds, src->cmds, src->count * sizeof(blick_cmd));
	current_buf_idx = next_idx;
}

void blick_end_frame(void) {
	if (!shm) return;
	atomic_store(&shm->latest_buffer_idx, current_buf_idx);
}

void blick_clear_permanent(void) {
	persistent_count = 0;
	persistent_head = 0;
}

static void submit_cmd(blick_cmd cmd, bool permanent) {
	if (!shm) return;
	blick_buffer* buf = &shm->buffers[current_buf_idx];
	if (buf->count < BLICK_MAX_CMDS) { buf->cmds[buf->count++] = cmd; }
	if (permanent) {
		persistent_cmds[persistent_head] = cmd;
		persistent_head = (persistent_head + 1) % BLICK_MAX_PERS_CMDS;
		if (persistent_count < BLICK_MAX_PERS_CMDS) persistent_count++;
	}
}

void blick_record_line(blick_vec3 start, blick_vec3 end, uint32_t color, bool permanent) {
	blick_cmd cmd = {.type = BLICK_CMD_LINE, .color = color, .data.line = {start, end}};
	submit_cmd(cmd, permanent);
}

void blick_record_arrow(blick_vec3 start, blick_vec3 end, uint32_t color, bool permanent) {
	blick_cmd cmd = {.type = BLICK_CMD_ARROW, .color = color, .data.arrow = {start, end}};
	submit_cmd(cmd, permanent);
}

void blick_record_point(blick_vec3 pos, float radius, uint32_t color, bool permanent) {
	blick_cmd cmd = {.type = BLICK_CMD_POINT, .color = color, .data.point = {pos, radius}};
	submit_cmd(cmd, permanent);
}

void blick_record_aabb(blick_vec3 min, blick_vec3 max, uint32_t color, bool permanent) {
	blick_cmd cmd = {.type = BLICK_CMD_AABB, .color = color, .data.aabb = {min, max}};
	submit_cmd(cmd, permanent);
}

void blick_record_triangle(blick_vec3 a, blick_vec3 b, blick_vec3 c, uint32_t color,
						   bool permanent) {
	blick_cmd cmd = {.type = BLICK_CMD_TRIANGLE, .color = color, .data.triangle = {a, b, c}};
	submit_cmd(cmd, permanent);
}

void blick_record_transform(blick_vec3 pos, blick_quat rot, bool permanent) {
	blick_cmd cmd = {
		.type = BLICK_CMD_TRANSFORM, .color = 0xFFFFFFFF, .data.transform = {pos, rot}};
	submit_cmd(cmd, permanent);
}

void blick_record_text(blick_vec3 pos, const char* text, uint32_t color, bool permanent) {
	blick_cmd cmd = {.type = BLICK_CMD_TEXT, .color = color, .data.text.pos = pos};
	strncpy(cmd.data.text.buffer, text, BLICK_TEXT_MAX_LEN - 1);
	cmd.data.text.buffer[BLICK_TEXT_MAX_LEN - 1] = '\0';
	submit_cmd(cmd, permanent);
}
