#ifndef BLICK_H
#define BLICK_H

#include "blick_protocol.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- Lifecycle ---

/**
 * @brief Initialize SHM and spawn the viewer process.
 * 
 * The host automatically looks for the viewer executable named "blick_viewer" 
 * (or "blick_viewer.exe" on Windows). It is expected to be located in the same 
 * directory as the host application.
 */
void blick_init(void);

/**
 * @brief Clean up SHM and kill the viewer process.
 */
void blick_shutdown(void);

/**
 * @brief Publishes the current state to the viewer.
 *
 * IMPORTANT: The current state is CLONED to the next frame.
 * If you call blick_refresh() without clearing, the image remains static.
 *
 * Usage:
 *   // 1. Draw frame
 *   blick_record_line(0, ...); // Layer 0
 *   blick_record_line(1, ...); // Layer 1
 *
 *   // 2. Publish
 *   blick_refresh();
 *
 *   // 3. Clear specific layers for the next frame
 *   blick_clear(0b0010); // Clear Layer 1 (dynamic), keep Layer 0 (static)
 */
void blick_refresh(void);

/**
 * @brief Removes commands belonging to specific layers from the current buffer.
 * @param layer_mask Bitmask of layers to delete (1 = delete, 0 = keep).
 *                   Supports layers 0-15.
 *
 * Examples:
 *   blick_clear(0xFFFF);      // Clear All
 *   blick_clear(0b1);         // Clear Layer 0
 *   blick_clear(0b000100001); // Clear Layer 0 and 5
 */
void blick_clear(uint16_t layer_mask);

/**
 * @brief Reduces a layer to the specified number of most recent items.
 * Removes oldest items first.
 */
void blick_trim_layer(uint8_t layer_id, uint32_t max_count);

// --- Primitive Recording ---
// layer_id: 0-15 (corresponds to bits in blick_clear mask)

void blick_record_line(uint8_t layer, blick_vec3 start, blick_vec3 end, uint32_t color);
void blick_record_arrow(uint8_t layer, blick_vec3 start, blick_vec3 end, uint32_t color);
void blick_record_point(uint8_t layer, blick_vec3 pos, float radius, uint32_t color);
void blick_record_aabb(uint8_t layer, blick_vec3 min, blick_vec3 max, uint32_t color);
void blick_record_triangle(uint8_t layer, blick_vec3 a, blick_vec3 b, blick_vec3 c, uint32_t color);
void blick_record_transform(uint8_t layer, blick_vec3 pos, blick_quat rot, float size);
void blick_record_sphere(uint8_t layer, blick_vec3 pos, blick_quat rot, float radius,
						 uint32_t color, bool wireframe);
void blick_record_capsule(uint8_t layer, blick_vec3 p_a, blick_vec3 p_b, float radius,
						  uint32_t color, bool wireframe);
void blick_record_obb(uint8_t layer, blick_vec3 pos, blick_quat rot, blick_vec3 extents,
					  uint32_t color, bool wireframe);
void blick_record_text(uint8_t layer, blick_vec3 pos, const char* text, uint32_t color);

// --- Mesh API ---

/**
 * @brief Uploads or updates a mesh in the shared memory pool.
 * @param id The unique identifier for this mesh (0 to BLICK_MAX_IDS-1).
 * @param vertices Array of vertices (triangle list).
 * @param vertex_count Number of vertices.
 */
void blick_upload_mesh(uint32_t id, const blick_vec3* vertices, uint32_t vertex_count);

/**
 * @brief Uploads an indexed mesh.
 * This flattens the mesh into a triangle list on the host side,
 * ensuring it renders correctly in the viewer.
 */
void blick_upload_mesh_indexed(uint32_t id, const blick_vec3* vertices, const uint32_t* indices,
							   uint32_t i_count);

/**
 * @brief Records a command to draw a previously uploaded mesh.
 * @param id The unique identifier for the mesh to draw.
 */
void blick_record_mesh(uint8_t layer, uint32_t id, blick_vec3 pos, blick_quat rot, uint32_t color,
					   bool wireframe);

#ifdef __cplusplus
}
#endif

#endif // BLICK_H
