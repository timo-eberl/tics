#ifndef BLICK_H
#define BLICK_H

#include "blick_protocol.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize SHM and spawn the viewer process.
 * @param viewer_path Path to the blick_viewer executable.
 */
void blick_init(const char* viewer_path);

/**
 * @brief Clean up SHM and kill the viewer process.
 */
void blick_shutdown(void);

void blick_start_frame(void);
void blick_update_frame(void); // Publishes current state but keeps writing
void blick_end_frame(void);
void blick_clear_permanent(void);

// Primitive Recording
void blick_record_line(blick_vec3 start, blick_vec3 end, uint32_t color, bool permanent);
void blick_record_arrow(blick_vec3 start, blick_vec3 end, uint32_t color, bool permanent);
void blick_record_point(blick_vec3 pos, float radius, uint32_t color, bool permanent);
void blick_record_aabb(blick_vec3 min, blick_vec3 max, uint32_t color, bool permanent);
void blick_record_triangle(blick_vec3 a, blick_vec3 b, blick_vec3 c, uint32_t color,
						   bool permanent);
void blick_record_transform(blick_vec3 pos, blick_quat rot, float size, bool permanent);
void blick_record_text(blick_vec3 pos, const char* text, uint32_t color, bool permanent);

// Mesh API
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
void blick_record_mesh(uint32_t id, blick_vec3 pos, blick_quat rot, uint32_t color, bool wireframe,
					   bool permanent);

#ifdef __cplusplus
}
#endif

#endif // BLICK_H
