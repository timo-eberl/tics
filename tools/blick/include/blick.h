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
void blick_record_transform(blick_vec3 pos, blick_quat rot, bool permanent);
void blick_record_text(blick_vec3 pos, const char* text, uint32_t color, bool permanent);

#ifdef __cplusplus
}
#endif

#endif // BLICK_H
