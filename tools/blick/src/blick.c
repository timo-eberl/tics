#include "blick.h"

#include <assert.h>
#include <fcntl.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

static blick_shm_header* shm = NULL;
static int current_buf_idx;
static int shm_fd = -1;
static pid_t viewer_pid = -1;

// --- Mesh Registry ---
typedef struct {
	bool allocated;
	uint32_t offset;
	uint32_t capacity;
	uint32_t vertex_count;
} MeshEntry;

static MeshEntry mesh_registry[BLICK_MAX_IDS];
static uint32_t pool_head = 0;

static int get_next_free_buffer(void);

void blick_init(const char* viewer_path) {
	shm_fd = shm_open(BLICK_SHM_NAME, O_CREAT | O_RDWR, 0666);
	if (shm_fd == -1) {
		perror("[BLICK] Error: shm_open failed");
		return;
	}

	if (ftruncate(shm_fd, sizeof(blick_shm_header)) == -1) {
		perror("[BLICK] Error: ftruncate failed");
		return;
	}

	shm = mmap(0, sizeof(blick_shm_header), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
	if (shm == MAP_FAILED) {
		perror("[BLICK] Error: mmap failed");
		shm = NULL;
		return;
	}

	// Initialize shared memory
	memset(shm, 0, sizeof(blick_shm_header));
	atomic_init(&shm->latest_buffer_idx, 0);
	atomic_init(&shm->reading_idx, 0xFFFFFFFF);

	// Initialize local memory
	memset(mesh_registry, 0, sizeof(mesh_registry));
	current_buf_idx = get_next_free_buffer();

	pid_t pid = fork();
	if (pid == 0) {
		if (getppid() == 1) exit(1);
		execl(viewer_path, viewer_path, NULL);
		perror("[BLICK] Error: Failed to spawn viewer");
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
	assert(false); // Unreachable if we have 3 buffers and 1 viewer
	return (latest + 1) % BLICK_SHM_BUFFER_COUNT;
}

void blick_refresh(void) {
	if (!shm) return;

	// Publish current state
	atomic_store(&shm->latest_buffer_idx, current_buf_idx);

	// Swap to next buffer
	int next_idx = get_next_free_buffer();
	// Clone state
	blick_buffer* src = &shm->buffers[current_buf_idx];
	blick_buffer* dst = &shm->buffers[next_idx];
	if (src->count > 0) { memcpy(dst->cmds, src->cmds, src->count * sizeof(blick_cmd)); }
	dst->count = src->count;

	current_buf_idx = next_idx;
}

void blick_clear(uint16_t layer_mask) {
	if (!shm) return;
	blick_buffer* buf = &shm->buffers[current_buf_idx];

	if (layer_mask == 0xFFFF) {
		buf->count = 0;
		return;
	}

	uint32_t write_idx = 0;
	for (uint32_t read_idx = 0; read_idx < buf->count; read_idx++) {
		uint8_t layer = buf->cmds[read_idx].layer;

		// Check if layer bit is set (limit to 16 layers)
		bool should_delete = (layer < 16) && ((layer_mask >> layer) & 1);

		if (!should_delete) {
			if (write_idx != read_idx) { buf->cmds[write_idx] = buf->cmds[read_idx]; }
			write_idx++;
		}
	}
	buf->count = write_idx;
}

void blick_trim_layer(uint8_t layer_id, uint32_t max_count) {
	if (!shm) return;
	blick_buffer* buf = &shm->buffers[current_buf_idx];

	if (buf->count == 0 || max_count >= BLICK_MAX_CMDS) return;

	// Count existing items in this layer
	uint32_t total = 0;
	for (uint32_t i = 0; i < buf->count; i++) {
		if (buf->cmds[i].layer == layer_id) total++;
	}

	if (total <= max_count) return;

	// Calculate how many from the START (oldest) need to be deleted
	uint32_t to_delete = total - max_count;
	uint32_t deleted_so_far = 0;

	// Compaction Pass (In-place)
	uint32_t write_idx = 0;
	for (uint32_t read_idx = 0; read_idx < buf->count; read_idx++) {
		blick_cmd* cmd = &buf->cmds[read_idx];
		bool keep = true;
		if (cmd->layer == layer_id) {
			if (deleted_so_far < to_delete) {
				keep = false;
				deleted_so_far++;
			}
		}
		if (keep) {
			if (write_idx != read_idx) { buf->cmds[write_idx] = buf->cmds[read_idx]; }
			write_idx++;
		}
	}
	buf->count = write_idx;
}

static void submit_cmd(blick_cmd cmd) {
	if (!shm) {
		static bool warned = false;
		if (!warned) {
			fprintf(stderr, "[BLICK] Error: API called before blick_init()\n");
			warned = true;
		}
		return;
	}
	blick_buffer* buf = &shm->buffers[current_buf_idx];
	if (buf->count < BLICK_MAX_CMDS) { buf->cmds[buf->count++] = cmd; }
	else {
		static bool warned = false;
		if (!warned) {
			fprintf(stderr, "[BLICK] Error: Command buffer overflow (Limit: %d)\n", BLICK_MAX_CMDS);
			warned = true;
		}
	}
}

void blick_record_line(uint8_t layer, blick_vec3 start, blick_vec3 end, uint32_t color) {
	blick_cmd cmd = {
		.type = BLICK_CMD_LINE, .layer = layer, .color = color, .data.line = {start, end}};
	submit_cmd(cmd);
}

void blick_record_arrow(uint8_t layer, blick_vec3 start, blick_vec3 end, uint32_t color) {
	blick_cmd cmd = {
		.type = BLICK_CMD_ARROW, .layer = layer, .color = color, .data.arrow = {start, end}};
	submit_cmd(cmd);
}

void blick_record_point(uint8_t layer, blick_vec3 pos, float radius, uint32_t color) {
	blick_cmd cmd = {
		.type = BLICK_CMD_POINT, .layer = layer, .color = color, .data.point = {pos, radius}};
	submit_cmd(cmd);
}

void blick_record_aabb(uint8_t layer, blick_vec3 min, blick_vec3 max, uint32_t color) {
	blick_cmd cmd = {
		.type = BLICK_CMD_AABB, .layer = layer, .color = color, .data.aabb = {min, max}};
	submit_cmd(cmd);
}

void blick_record_triangle(uint8_t layer, blick_vec3 a, blick_vec3 b, blick_vec3 c,
						   uint32_t color) {
	blick_cmd cmd = {
		.type = BLICK_CMD_TRIANGLE, .layer = layer, .color = color, .data.triangle = {a, b, c}};
	submit_cmd(cmd);
}

void blick_record_transform(uint8_t layer, blick_vec3 pos, blick_quat rot, float size) {
	blick_cmd cmd = {.type = BLICK_CMD_TRANSFORM,
					 .layer = layer,
					 .color = 0xFFFFFFFF,
					 .data.transform = {pos, rot, size}};
	submit_cmd(cmd);
}

void blick_record_sphere(uint8_t layer, blick_vec3 pos, blick_quat rot, float radius,
						 uint32_t color, bool wireframe) {
	blick_cmd cmd = {.type = BLICK_CMD_SPHERE,
					 .layer = layer,
					 .color = color,
					 .data.sphere = {pos, rot, radius, wireframe}};
	submit_cmd(cmd);
}

void blick_record_capsule(uint8_t layer, blick_vec3 p_a, blick_vec3 p_b, float radius,
						  uint32_t color, bool wireframe) {
	blick_cmd cmd = {.type = BLICK_CMD_CAPSULE,
					 .layer = layer,
					 .color = color,
					 .data.capsule = {p_a, p_b, radius, wireframe}};
	submit_cmd(cmd);
}

void blick_record_text(uint8_t layer, blick_vec3 pos, const char* text, uint32_t color) {
	blick_cmd cmd = {.type = BLICK_CMD_TEXT, .layer = layer, .color = color, .data.text.pos = pos};
	strncpy(cmd.data.text.buffer, text, BLICK_TEXT_MAX_LEN - 1);
	cmd.data.text.buffer[BLICK_TEXT_MAX_LEN - 1] = '\0';
	submit_cmd(cmd);
}

void blick_record_mesh(uint8_t layer, uint32_t id, blick_vec3 pos, blick_quat rot, uint32_t color,
					   bool wireframe) {
	if (!shm || id >= BLICK_MAX_IDS) return;
	MeshEntry* entry = &mesh_registry[id];
	if (!entry->allocated || entry->vertex_count == 0) {
		static bool warned = false;
		if (!warned) {
			fprintf(stderr, "[BLICK] Error: Attempting to draw uninitialized mesh (ID: %d)\n", id);
			warned = true;
		}
		return;
	}

	blick_cmd cmd = {.type = BLICK_CMD_DRAW_MESH,
					 .layer = layer,
					 .color = color,
					 .data.mesh = {.offset = entry->offset,
								   .vertex_count = entry->vertex_count,
								   .pos = pos,
								   .rot = rot,
								   .wireframe = wireframe}};
	submit_cmd(cmd);
}

// --- Mesh Upload ---

void blick_upload_mesh(uint32_t id, const blick_vec3* vertices, uint32_t vertex_count) {
	if (!shm) return;
	if (id >= BLICK_MAX_IDS) {
		fprintf(stderr, "[BLICK] Error: Mesh ID %d exceeds limit (%d)\n", id, BLICK_MAX_IDS);
		return;
	}

	uint32_t float_count = vertex_count * 3;
	MeshEntry* entry = &mesh_registry[id];

	// The custom allocator used here is not perfect, but works for now. It can leak memory.
	// Append-Only Leak: The allocator (pool_head) is strictly linear. When blick_upload_mesh is
	// called:
	// - If the ID is new, it allocates space.
	// - If the ID exists but the new data requires more space than previously allocated, it
	//   abandons the old space and allocates new space at the end of the pool.
	// The Problem: There is no implementation of a free list or compaction. The "abandoned" gaps
	// are never reclaimed.

	// Determine if we need to allocate new space
	bool need_alloc = !entry->allocated || (float_count > entry->capacity);

	if (need_alloc) {
		if (pool_head + float_count > BLICK_POOL_SIZE) {
			fprintf(stderr, "[BLICK] Error: Mesh pool overflow (Request: %u, Free: %u)\n",
					float_count, BLICK_POOL_SIZE - pool_head);
			return;
		}
		entry->offset = pool_head;
		entry->capacity = float_count;
		entry->allocated = true;
		pool_head += float_count;
	}
	entry->vertex_count = vertex_count;
	memcpy(&shm->mesh_pool[entry->offset], vertices, float_count * sizeof(float));
}

void blick_upload_mesh_indexed(uint32_t id, const blick_vec3* vertices, const uint32_t* indices,
							   uint32_t i_count) {
	if (i_count == 0) return;
	blick_vec3* flat = malloc(i_count * sizeof(blick_vec3));
	if (!flat) return;
	for (uint32_t i = 0; i < i_count; i++)
		flat[i] = vertices[indices[i]]; // this can segfault if indices has bad data
	blick_upload_mesh(id, flat, i_count);
	free(flat);
}
