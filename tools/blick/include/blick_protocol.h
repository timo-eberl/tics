#ifndef BLICK_PROTOCOL_H
#define BLICK_PROTOCOL_H

/**
 * @file blick_protocol.h
 * @brief Shared Memory Protocol for the Blick Visualization Library.
 */

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#define BLICK_SHM_NAME "/blick_shm"
#define BLICK_SHM_BUFFER_COUNT 3
#define BLICK_MAX_CMDS 1000000
#define BLICK_TEXT_MAX_LEN 32

// Mesh Constants
#define BLICK_POOL_SIZE (1024 * 1024) // 1 million floats (4MB)
#define BLICK_MAX_IDS 1024

// clang-format off

typedef struct { float x, y, z; } blick_vec3;
typedef struct { float x, y, z, w; } blick_quat;

typedef enum {
	BLICK_CMD_LINE,
	BLICK_CMD_ARROW,
	BLICK_CMD_POINT,
	BLICK_CMD_AABB,
	BLICK_CMD_TRIANGLE,
	BLICK_CMD_TRANSFORM,
	BLICK_CMD_SPHERE,
	BLICK_CMD_TEXT,
	BLICK_CMD_DRAW_MESH,
} blick_cmd_type;

typedef struct {
	union {
		struct { blick_vec3 start; blick_vec3 end; } line;
		struct { blick_vec3 start; blick_vec3 end; } arrow;
		struct { blick_vec3 pos; float radius; } point;
		struct { blick_vec3 min; blick_vec3 max; } aabb;
		struct { blick_vec3 a; blick_vec3 b; blick_vec3 c; } triangle;
		struct { blick_vec3 pos; blick_quat rot; float size; } transform;
		struct { blick_vec3 pos; blick_quat rot; float radius; bool wireframe; } sphere;
		struct { blick_vec3 pos; char buffer[BLICK_TEXT_MAX_LEN]; } text;
		// Mesh Draw Command (Stateless: contains offset, not ID)
		struct {
			uint32_t offset; // Offset into shm->mesh_pool
			uint32_t vertex_count;
			blick_vec3 pos;
			blick_quat rot;
			bool wireframe;
		} mesh;
	} data;

	uint32_t color; // 0xAABBGGRR
	blick_cmd_type type;
	uint8_t layer;
} blick_cmd;

typedef struct {
	uint32_t count;
	blick_cmd cmds[BLICK_MAX_CMDS];
} blick_buffer;

typedef struct {
	_Atomic uint32_t latest_buffer_idx;
	_Atomic uint32_t reading_idx;
	float mesh_pool[BLICK_POOL_SIZE];
	blick_buffer buffers[BLICK_SHM_BUFFER_COUNT];
} blick_shm_header;

// clang-format on

#endif // BLICK_PROTOCOL_H
