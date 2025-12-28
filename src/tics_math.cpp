#include "tics_math.h"

// Terathon Math Includes
#include <TSMatrix4D.h>
#include <TSQuaternion.h>
#include <TSVector3D.h>
#include <TSVector4D.h>

using namespace Terathon;

// Internal Helpers to convert between C structs and Terathon Classes
static inline Vector3D to_ts(tics_vec3 v) {
	return Vector3D(v.x, v.y, v.z);
}
static inline tics_vec3 from_ts(Vector3D v) {
	return {v.x, v.y, v.z};
}
static inline Quaternion to_ts(tics_quat q) {
	return Quaternion(q.x, q.y, q.z, q.w);
}
static inline tics_quat from_ts(Quaternion q) {
	return {q.x, q.y, q.z, q.w};
}

// Helper to copy a Terathon Matrix4D into our C-style array (Column-Major)
static void matrix_to_array(const Matrix4D& src, float* dst) {
	for (int j = 0; j < 4; j++) {
		const Vector4D& col = src[j]; // Returns column j
		dst[j * 4 + 0] = col.x;
		dst[j * 4 + 1] = col.y;
		dst[j * 4 + 2] = col.z;
		dst[j * 4 + 3] = col.w;
	}
}

// Helper to create a Terathon Matrix4D from our C-style array (Column-Major)
static Matrix4D array_to_matrix(const float* src) {
	return Matrix4D(Vector4D(src[0], src[1], src[2], src[3]),	 // Column 0
					Vector4D(src[4], src[5], src[6], src[7]),	 // Column 1
					Vector4D(src[8], src[9], src[10], src[11]),	 // Column 2
					Vector4D(src[12], src[13], src[14], src[15]) // Column 3
	);
}

// --- Vector Implementation ---

tics_vec3 tics_vec3_add(tics_vec3 a, tics_vec3 b) {
	return from_ts(to_ts(a) + to_ts(b));
}

tics_vec3 tics_vec3_sub(tics_vec3 a, tics_vec3 b) {
	return from_ts(to_ts(a) - to_ts(b));
}

tics_vec3 tics_vec3_mul_f(tics_vec3 v, float s) {
	return from_ts(to_ts(v) * s);
}

float tics_vec3_dot(tics_vec3 a, tics_vec3 b) {
	return Dot(to_ts(a), to_ts(b));
}

tics_vec3 tics_vec3_cross(tics_vec3 a, tics_vec3 b) {
	return from_ts(Cross(to_ts(a), to_ts(b)));
}

float tics_vec3_length_sq(tics_vec3 v) {
	return SquaredMag(to_ts(v));
}

float tics_vec3_length(tics_vec3 v) {
	return Magnitude(to_ts(v));
}

tics_vec3 tics_vec3_normalize(tics_vec3 v) {
	return from_ts(Normalize(to_ts(v)));
}

tics_vec3 tics_vec3_negate(tics_vec3 v) {
	return from_ts(-to_ts(v));
}

// --- Quaternion Implementation ---

tics_quat tics_quat_identity(void) {
	return {0.0f, 0.0f, 0.0f, 1.0f};
}

tics_quat tics_quat_mul(tics_quat q1, tics_quat q2) {
	return from_ts(to_ts(q1) * to_ts(q2));
}

tics_quat tics_quat_normalize(tics_quat q) {
	Quaternion res = to_ts(q);
	res.Normalize();
	return from_ts(res);
}

tics_quat tics_quat_from_axis_angle(tics_vec3 axis, float angle) {
	return from_ts(Quaternion::MakeRotation(angle, !to_ts(axis)));
}

tics_vec3 tics_quat_rotate_vec3(tics_vec3 v, tics_quat q) {
	return from_ts(Transform(to_ts(v), to_ts(q)));
}

tics_quat tics_quat_lerp(tics_quat a, tics_quat b, float t) {
	// Basic LERP for quaternions (Terathon style)
	Quaternion qa = to_ts(a);
	Quaternion qb = to_ts(b);
	Quaternion res = qa * (1.0f - t) + qb * t;
	res.Normalize();
	return from_ts(res);
}

tics_quat tics_quat_scale(tics_quat q, float scale) {
	Quaternion ts_q = to_ts(q);

	// Optimized NLERP: Identity is (0,0,0,1)
	// We only need to interpolate the components
	Quaternion res;
	res.x = ts_q.x * scale;
	res.y = ts_q.y * scale;
	res.z = ts_q.z * scale;
	res.w = 1.0f + (ts_q.w - 1.0f) * scale;

	res.Normalize();
	return from_ts(res);
}

// inverse rotation (conjugate for unit quaternions)
tics_quat quat_inverse(tics_quat q) {
	return {-q.x, -q.y, -q.z, q.w};
}

// --- Matrix Implementation ---

tics_mat4 tics_mat4_identity(void) {
	tics_mat4 res = {0};
	res.m[0] = 1.0f;
	res.m[5] = 1.0f;
	res.m[10] = 1.0f;
	res.m[15] = 1.0f;
	return res;
}

tics_mat4 tics_mat4_translate(tics_vec3 v) {
	Matrix4D ts_mat = Transform3D::MakeTranslation(to_ts(v));
	tics_mat4 res;
	for (int i = 0; i < 16; i++) {
		matrix_to_array(ts_mat, res.m);
	}
	return res;
}

tics_mat4 tics_mat4_from_quat(tics_quat q) {
	Matrix3D rot = to_ts(q).GetRotationMatrix();
	tics_mat4 res = tics_mat4_identity();
	// Col 0
	res.m[0] = rot(0, 0);
	res.m[1] = rot(1, 0);
	res.m[2] = rot(2, 0);
	// Col 1
	res.m[4] = rot(0, 1);
	res.m[5] = rot(1, 1);
	res.m[6] = rot(2, 1);
	// Col 2
	res.m[8] = rot(0, 2);
	res.m[9] = rot(1, 2);
	res.m[10] = rot(2, 2);
	// Col 3 and the bottom row of each column remain identity (0,0,0,1)
	return res;
}

tics_mat4 tics_mat4_mul(tics_mat4 a, tics_mat4 b) {
	Matrix4D ma = array_to_matrix(a.m);
	Matrix4D mb = array_to_matrix(b.m);

	// Use Terathon's operator*
	Matrix4D res_ts = ma * mb;

	tics_mat4 res;
	matrix_to_array(res_ts, res.m);
	return res;
}
