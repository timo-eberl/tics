#ifndef TICS_MATH_H
#define TICS_MATH_H

typedef struct tics_vec3 {
	float x, y, z;
} tics_vec3;

typedef struct tics_vec4 {
	float x, y, z, w;
} tics_vec4;

typedef struct tics_quat {
	float x, y, z, w;
} tics_quat;

typedef struct tics_mat4 {
	float m[16]; // Column-major
} tics_mat4;

// Vector Operations
tics_vec3 tics_vec3_add(tics_vec3 a, tics_vec3 b);
tics_vec3 tics_vec3_sub(tics_vec3 a, tics_vec3 b);
tics_vec3 tics_vec3_mul_f(tics_vec3 v, float s);
float tics_vec3_dot(tics_vec3 a, tics_vec3 b);
tics_vec3 tics_vec3_cross(tics_vec3 a, tics_vec3 b);
float tics_vec3_length_sq(tics_vec3 v);
float tics_vec3_length(tics_vec3 v);
tics_vec3 tics_vec3_normalize(tics_vec3 v);
tics_vec3 tics_vec3_negate(tics_vec3 v);

// Quaternion Operations
tics_quat tics_quat_identity(void);
tics_quat tics_quat_mul(tics_quat q1, tics_quat q2);
tics_quat tics_quat_normalize(tics_quat q);
tics_quat tics_quat_from_axis_angle(tics_vec3 axis, float angle);
tics_vec3 tics_quat_rotate_vec3(tics_vec3 v, tics_quat q);
tics_quat tics_quat_lerp(tics_quat a, tics_quat b, float t);
// Scales a rotation toward identity by a factor (NLERP)
// Useful for integrating angular velocity over a time step
tics_quat tics_quat_scale(tics_quat q, float scale);
tics_quat quat_inverse(tics_quat q);

// Matrix Operations
tics_mat4 tics_mat4_identity(void);
tics_mat4 tics_mat4_translate(tics_vec3 v);
tics_mat4 tics_mat4_from_quat(tics_quat q);
tics_mat4 tics_mat4_mul(tics_mat4 a, tics_mat4 b);

#endif // TICS_MATH_H
