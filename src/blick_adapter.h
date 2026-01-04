#ifndef BLICK_ADAPTER_H
#define BLICK_ADAPTER_H

// Internal adapter for the 'blick' visualization library.

#ifdef TICS_ENABLE_BLICK

#include "tics.h"

#include <blick.h>

// --- Type Conversion Helpers (Private) ---
#define _BLICK_V3(v) ((blick_vec3){(v).x, (v).y, (v).z})
#define _BLICK_Q(q) ((blick_quat){(q).x, (q).y, (q).z, (q).w})

// --- Lifecycle ---

#define BLICK_INIT() blick_init("blick_viewer")
#define BLICK_SHUTDOWN() blick_shutdown()
#define BLICK_FRAME_START() blick_start_frame()
#define BLICK_FRAME_UPDATE() blick_update_frame()
#define BLICK_FRAME_END() blick_end_frame()
#define BLICK_CLEAR_PERM() blick_clear_permanent()

// --- Transient Primitives (Clear next frame) ---

#define BLICK_LINE(s, e, c) blick_record_line(_BLICK_V3(s), _BLICK_V3(e), c, false)
#define BLICK_ARROW(s, e, c) blick_record_arrow(_BLICK_V3(s), _BLICK_V3(e), c, false)
#define BLICK_POINT(p, r, c) blick_record_point(_BLICK_V3(p), r, c, false)
#define BLICK_AABB(min, max, c) blick_record_aabb(_BLICK_V3(min), _BLICK_V3(max), c, false)
#define BLICK_TRIANGLE(a, b, c_pos, col)                                                           \
	blick_record_triangle(_BLICK_V3(a), _BLICK_V3(b), _BLICK_V3(c_pos), col, false)
#define BLICK_TRANSFORM(t)                                                                         \
	blick_record_transform(_BLICK_V3((t).position), _BLICK_Q((t).rotation), false)
#define BLICK_TEXT(p, txt, c) blick_record_text(_BLICK_V3(p), txt, c, false)

// --- Permanent Primitives (Persist until cleared) ---

#define BLICK_LINE_PERM(s, e, c) blick_record_line(_BLICK_V3(s), _BLICK_V3(e), c, true)
#define BLICK_ARROW_PERM(s, e, c) blick_record_arrow(_BLICK_V3(s), _BLICK_V3(e), c, true)
#define BLICK_POINT_PERM(p, r, c) blick_record_point(_BLICK_V3(p), r, c, true)
#define BLICK_AABB_PERM(min, max, c) blick_record_aabb(_BLICK_V3(min), _BLICK_V3(max), c, true)
#define BLICK_TRIANGLE_PERM(a, b, c_pos, col)                                                      \
	blick_record_triangle(_BLICK_V3(a), _BLICK_V3(b), _BLICK_V3(c_pos), col, true)
#define BLICK_TRANSFORM_PERM(t)                                                                    \
	blick_record_transform(_BLICK_V3((t).position), _BLICK_Q((t).rotation), true)
#define BLICK_TEXT_PERM(p, txt, c) blick_record_text(_BLICK_V3(p), txt, c, true)

#else

// --- No-Op Implementations ---

#define BLICK_INIT(path) ((void)0)
#define BLICK_SHUTDOWN() ((void)0)
#define BLICK_FRAME_START() ((void)0)
#define BLICK_FRAME_UPDATE() ((void)0)
#define BLICK_FRAME_END() ((void)0)
#define BLICK_CLEAR_PERM() ((void)0)

#define BLICK_LINE(...) ((void)0)
#define BLICK_ARROW(...) ((void)0)
#define BLICK_POINT(...) ((void)0)
#define BLICK_AABB(...) ((void)0)
#define BLICK_TRIANGLE(...) ((void)0)
#define BLICK_TRANSFORM(...) ((void)0)
#define BLICK_TEXT(...) ((void)0)

#define BLICK_LINE_PERM(...) ((void)0)
#define BLICK_ARROW_PERM(...) ((void)0)
#define BLICK_POINT_PERM(...) ((void)0)
#define BLICK_AABB_PERM(...) ((void)0)
#define BLICK_TRIANGLE_PERM(...) ((void)0)
#define BLICK_TRANSFORM_PERM(...) ((void)0)
#define BLICK_TEXT_PERM(...) ((void)0)

#endif // TICS_ENABLE_DEBUG_VIEW

#endif // BLICK_ADAPTER_H
