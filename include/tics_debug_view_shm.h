#ifndef TICS_DEBUG_VIEW_SHM_H
#define TICS_DEBUG_VIEW_SHM_H

/**
 * @file tics_debug_view_shm.h
 * @brief Inter-Process Communication (IPC) Protocol for Tics Physics Visualization.
 *
 * DESCRIPTION:
 * This header defines the shared memory layout used to communicate between the Physics Simulation
 * (Host) and the Debug Viewer (Client).
 *
 * ARCHITECTURE:
 * 1. Out-of-Process: The Debug Viewer runs as a completely separate OS process.
 * 2. Shared Memory: Communication happens via a POSIX shared memory file.
 * 3. Triple Buffering: To prevent tearing and support fast writers vs slow readers:
 *    - The Host writes to a free buffer.
 *    - The Client locks a buffer for reading via 'reading_idx'.
 *    - 'latest_buffer_idx' points to the most recently completed frame.
 *
 * USAGE:
 * This file must be included by both the library (tics) and the viewer tool.
 */

#include <stdatomic.h>
#include <stdint.h>

// ------------------------------------------------------------------------------------------------
// CONFIGURATION
// ------------------------------------------------------------------------------------------------

// The internal name used by shm_open (usually maps to /dev/shm/tics_debug_view_shm on Linux)
#define TICS_SHM_NAME "/tics_debug_view_shm"

// Increase to 3 to allow: 1 for Reader, 1 for Writer, 1 for Latest Completed
#define TICS_SHM_BUFFER_COUNT 3

// Maximum number of debug primitives per frame.
#define TICS_VIEW_MAX_CMDS 4096

// Max characters for a text label
#define TICS_TEXT_MAX_LEN 32

// ------------------------------------------------------------------------------------------------
// DATA TYPES
// ------------------------------------------------------------------------------------------------

/**
 * @brief A minimal 3D vector struct.
 */
typedef struct {
	float x, y, z;
} tics_view_vec3;

/**
 * @brief A minimal Quaternion struct.
 */
typedef struct {
	float x, y, z, w;
} tics_view_quat;

/**
 * @brief Types of debug primitives available.
 */
typedef enum {
	TICS_VIEW_CMD_LINE,
	TICS_VIEW_CMD_ARROW,
	TICS_VIEW_CMD_POINT,
	TICS_VIEW_CMD_AABB,
	TICS_VIEW_CMD_TRIANGLE,
	TICS_VIEW_CMD_TRANSFORM,
	TICS_VIEW_CMD_TEXT
} tics_view_cmd_type;

/**
 * @brief A single render command.
 */
typedef struct {
	uint32_t type;
	uint32_t color; // Hex format: 0xAABBGGRR (Alpha, Blue, Green, Red)

	union {
		struct {
			tics_view_vec3 start;
			tics_view_vec3 end;
		} line;

		struct {
			tics_view_vec3 start;
			tics_view_vec3 end;
		} arrow;

		struct {
			tics_view_vec3 pos;
			float radius;
		} point;

		struct {
			tics_view_vec3 min;
			tics_view_vec3 max;
		} aabb;

		struct {
			tics_view_vec3 a;
			tics_view_vec3 b;
			tics_view_vec3 c;
		} triangle;

		struct {
			tics_view_vec3 pos;
			tics_view_quat rot;
		} transform;

		struct {
			tics_view_vec3 pos;
			char buffer[TICS_TEXT_MAX_LEN];
		} text;
	} data;
} tics_view_cmd;

// ------------------------------------------------------------------------------------------------
// MEMORY LAYOUT
// ------------------------------------------------------------------------------------------------

/**
 * @brief Represents one frame of debug data.
 */
typedef struct {
	// Monotonically increasing sequence number.
	_Atomic uint32_t seq;

	// Number of valid commands in the 'cmds' array.
	uint32_t count;

	// The command payload.
	tics_view_cmd cmds[TICS_VIEW_MAX_CMDS];
} tics_view_buffer;

/**
 * @brief The Root Structure mapped into Shared Memory.
 */
typedef struct {
	// Indicates which buffer contains the latest complete frame.
	// The Host updates this atomically AFTER finishing a write.
	// The Client reads this to know which buffer to draw.
	_Atomic uint32_t latest_buffer_idx;

	// Indicates which buffer the Viewer is currently reading.
	// The Host will NOT write to this buffer while this value is set.
	// Set to 0xFFFFFFFF when not reading.
	_Atomic uint32_t reading_idx;

	// The buffers.
	tics_view_buffer buffers[TICS_SHM_BUFFER_COUNT];
} tics_view_shm_header;

#endif // TICS_DEBUG_VIEW_SHM_H
