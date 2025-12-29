#ifndef TICS_MATH_H
#define TICS_MATH_H

#include <math.h>

// header-only math for best performance

// --- Data Structures ---

// clang-format off
typedef struct { float x, y, z; } tics_vec3;
typedef struct { float x, y, z, w; } tics_vec4;
typedef struct { float x, y, z, w; } tics_quat;
// clang-format on

// --- Vector Implementation ---

static inline tics_vec3 tics_vec3_add(tics_vec3 a, tics_vec3 b) {
	return (tics_vec3){a.x + b.x, a.y + b.y, a.z + b.z};
}

static inline tics_vec3 tics_vec3_sub(tics_vec3 a, tics_vec3 b) {
	return (tics_vec3){a.x - b.x, a.y - b.y, a.z - b.z};
}

static inline tics_vec3 tics_vec3_mul_f(tics_vec3 v, float s) {
	return (tics_vec3){v.x * s, v.y * s, v.z * s};
}

static inline float tics_vec3_dot(tics_vec3 a, tics_vec3 b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline tics_vec3 tics_vec3_cross(tics_vec3 a, tics_vec3 b) {
	return (tics_vec3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

static inline float tics_vec3_length_sq(tics_vec3 v) {
	return v.x * v.x + v.y * v.y + v.z * v.z;
}

static inline float tics_vec3_length(tics_vec3 v) {
	return sqrt(tics_vec3_length_sq(v));
}

static inline tics_vec3 tics_vec3_normalize(tics_vec3 v) {
	float len = tics_vec3_length(v);
	if (len > 0.00001f) {
		float inv = 1.0f / len;
		return (tics_vec3){v.x * inv, v.y * inv, v.z * inv};
	}
	return (tics_vec3){0, 0, 0};
}

static inline tics_vec3 tics_vec3_negate(tics_vec3 v) {
	return (tics_vec3){-v.x, -v.y, -v.z};
}

// --- Quaternion Implementation ---

static inline tics_quat tics_quat_identity(void) {
	return (tics_quat){0.0f, 0.0f, 0.0f, 1.0f};
}

// Grassman product (standard quaternion multiplication)
static inline tics_quat tics_quat_mul(tics_quat a, tics_quat b) {
	return (tics_quat){a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
					   a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
					   a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
					   a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

static inline tics_quat tics_quat_normalize(tics_quat q) {
	float len_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
	if (len_sq > 0.00001f) {
		float inv = 1.0f / sqrt(len_sq);
		return (tics_quat){q.x * inv, q.y * inv, q.z * inv, q.w * inv};
	}
	return (tics_quat){0, 0, 0, 1};
}

// axis must be normalized
static inline tics_quat tics_quat_from_axis_angle(tics_vec3 axis, float angle) {
	float half_angle = angle * 0.5f;
	float s = sinf(half_angle);
	return (tics_quat){axis.x * s, axis.y * s, axis.z * s, cosf(half_angle)};
}

// Rotate vector v by quaternion q: v' = q * v * q_inv
// Optimized version: v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
static inline tics_vec3 tics_quat_rotate_vec3(tics_vec3 v, tics_quat q) {
	tics_vec3 q_xyz = {q.x, q.y, q.z};
	tics_vec3 t = tics_vec3_cross(q_xyz, v);
	t = tics_vec3_add(t, t); // 2 * cross(q.xyz, v)

	// v + q.w * t + cross(q.xyz, t)
	tics_vec3 term1 = tics_vec3_mul_f(t, q.w);
	tics_vec3 term2 = tics_vec3_cross(q_xyz, t);
	return tics_vec3_add(v, tics_vec3_add(term1, term2));
}

static inline tics_quat tics_quat_lerp(tics_quat a, tics_quat b, float t) {
	// Simple linear interpolation followed by normalize
	tics_quat res;
	float one_minus_t = 1.0f - t;
	res.x = a.x * one_minus_t + b.x * t;
	res.y = a.y * one_minus_t + b.y * t;
	res.z = a.z * one_minus_t + b.z * t;
	res.w = a.w * one_minus_t + b.w * t;
	return tics_quat_normalize(res);
}

static inline tics_quat tics_quat_scale(tics_quat q, float scale) {
	// Implementation matching the specific logic previously used (NLERP towards identity)
	tics_quat res;
	res.x = q.x * scale;
	res.y = q.y * scale;
	res.z = q.z * scale;
	res.w = 1.0f + (q.w - 1.0f) * scale;
	return tics_quat_normalize(res);
}

// inverse rotation (conjugate for unit quaternions)
static inline tics_quat quat_inverse(tics_quat q) {
	return (tics_quat){-q.x, -q.y, -q.z, q.w};
}

#endif // TICS_MATH_H
