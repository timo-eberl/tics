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
 * 1. Out-of-Process: The Debug Viewer runs as a completely separate OS process. This allows the
 *    simulation to be paused (e.g. via GDB breakpoint) while the Debug Viewer continues to render
 *    the last received frame, allowing inspection of the frozen state.
 * 2. Shared Memory: Communication happens via a POSIX shared memory file mapped into both
 *    processes.
 * 3. Double Buffering: To prevent read/write conflicts without expensive mutexes, we use two
 *    buffers:
 *    - The Host writes to the 'Back Buffer'.
 *    - The Client reads from the 'Front Buffer'.
 *    - An atomic index indicates which buffer is currently the Front Buffer.
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

// Maximum number of debug primitives per frame.
// If the simulation exceeds this, excess commands are simply dropped for that frame.
#define TICS_VIEW_MAX_CMDS 4096

// ------------------------------------------------------------------------------------------------
// DATA TYPES
// ------------------------------------------------------------------------------------------------

/**
 * @brief A minimal 3D vector struct.
 * We define this here to avoid the Viewer depending on the main 'tics.h' library header.
 */
typedef struct {
	float x, y, z;
} tics_view_vec3;

/**
 * @brief Types of debug primitives available.
 */
typedef enum { TICS_VIEW_CMD_LINE, TICS_VIEW_CMD_POINT } tics_view_cmd_type;

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
			tics_view_vec3 pos;
			float radius;
		} point;
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
	// The Client checks this to detect if a new frame has been published.
	// If (seq != last_seen_seq), the Client copies the new data.
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
	// Indicates which buffer (0 or 1) contains the latest complete frame.
	// The Host updates this atomically AFTER finishing a write.
	// The Client reads this to know which buffer to draw.
	_Atomic uint32_t latest_buffer_idx;

	// The double buffers.
	tics_view_buffer buffers[2];
} tics_view_shm_header;

#endif // TICS_DEBUG_VIEW_SHM_H
