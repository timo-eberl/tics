#ifndef TICS_DEBUG_VIEW_SHM_INTERNAL_H
#define TICS_DEBUG_VIEW_SHM_INTERNAL_H

#ifdef TICS_ENABLE_DEBUG_VIEW

#include "tics_debug_view_shm.h"
#include "tics_math.h"
#include <stdbool.h>

void tics_view_init(void);
void tics_view_shutdown(void);
void tics_view_start_frame(void);
void tics_view_update_frame(void);
void tics_view_end_frame(void);

// Clears all commands marked as permanent
void tics_view_clear_permanent(void);

// Primitives with persistence support
void tics_view_record_line(tics_vec3 start, tics_vec3 end, uint32_t color, bool permanent);
void tics_view_record_arrow(tics_vec3 start, tics_vec3 end, uint32_t color, bool permanent);
void tics_view_record_point(tics_vec3 pos, float radius, uint32_t color, bool permanent);
void tics_view_record_aabb(tics_vec3 min, tics_vec3 max, uint32_t color, bool permanent);
void tics_view_record_triangle(tics_vec3 a, tics_vec3 b, tics_vec3 c, uint32_t color,
							   bool permanent);
void tics_view_record_transform(tics_transform t, bool permanent);
void tics_view_record_text(tics_vec3 pos, const char* text, uint32_t color, bool permanent);

// --- Macros ---

#define TICS_VIEW_INIT() tics_view_init()
#define TICS_VIEW_SHUTDOWN() tics_view_shutdown()
#define TICS_VIEW_FRAME_START() tics_view_start_frame()
#define TICS_VIEW_FRAME_UPDATE() tics_view_update_frame()
#define TICS_VIEW_FRAME_END() tics_view_end_frame()
#define TICS_VIEW_CLEAR_PERM() tics_view_clear_permanent()

// Transient (Disappears next frame)
#define TICS_VIEW_LINE(s, e, c) tics_view_record_line(s, e, c, false)
#define TICS_VIEW_ARROW(s, e, c) tics_view_record_arrow(s, e, c, false)
#define TICS_VIEW_POINT(p, r, c) tics_view_record_point(p, r, c, false)
#define TICS_VIEW_AABB(min, max, c) tics_view_record_aabb(min, max, c, false)
#define TICS_VIEW_TRIANGLE(a, b, c_pos, col) tics_view_record_triangle(a, b, c_pos, col, false)
#define TICS_VIEW_TRANSFORM(t) tics_view_record_transform(t, false)
#define TICS_VIEW_TEXT(p, txt, c) tics_view_record_text(p, txt, c, false)

// Permanent (Stays until cleared)
#define TICS_VIEW_LINE_PERM(s, e, c) tics_view_record_line(s, e, c, true)
#define TICS_VIEW_ARROW_PERM(s, e, c) tics_view_record_arrow(s, e, c, true)
#define TICS_VIEW_POINT_PERM(p, r, c) tics_view_record_point(p, r, c, true)
#define TICS_VIEW_AABB_PERM(min, max, c) tics_view_record_aabb(min, max, c, true)
#define TICS_VIEW_TRIANGLE_PERM(a, b, c_pos, col) tics_view_record_triangle(a, b, c_pos, col, true)
#define TICS_VIEW_TRANSFORM_PERM(t) tics_view_record_transform(t, true)
#define TICS_VIEW_TEXT_PERM(p, txt, c) tics_view_record_text(p, txt, c, true)

#else

#define TICS_VIEW_INIT() ((void)0)
#define TICS_VIEW_SHUTDOWN() ((void)0)
#define TICS_VIEW_FRAME_START() ((void)0)
#define TICS_VIEW_FRAME_UPDATE() ((void)0)
#define TICS_VIEW_FRAME_END() ((void)0)
#define TICS_VIEW_CLEAR_PERM() ((void)0)

#define TICS_VIEW_LINE(...) ((void)0)
#define TICS_VIEW_ARROW(...) ((void)0)
#define TICS_VIEW_POINT(...) ((void)0)
#define TICS_VIEW_AABB(...) ((void)0)
#define TICS_VIEW_TRIANGLE(...) ((void)0)
#define TICS_VIEW_TRANSFORM(...) ((void)0)
#define TICS_VIEW_TEXT(...) ((void)0)

#define TICS_VIEW_LINE_PERM(...) ((void)0)
#define TICS_VIEW_ARROW_PERM(...) ((void)0)
#define TICS_VIEW_POINT_PERM(...) ((void)0)
#define TICS_VIEW_AABB_PERM(...) ((void)0)
#define TICS_VIEW_TRIANGLE_PERM(...) ((void)0)
#define TICS_VIEW_TRANSFORM_PERM(...) ((void)0)
#define TICS_VIEW_TEXT_PERM(...) ((void)0)

#endif

#endif // TICS_DEBUG_VIEW_SHM_INTERNAL_H
