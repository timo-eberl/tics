#ifndef TICS_RAYLIB_BRIDGE_H
#define TICS_RAYLIB_BRIDGE_H

#include <raylib.h>
#include <raymath.h>
#include <tics.h>

#include <stdint.h>

/* -------------------------------------------------------------------------------------------------
Integration layer between Raylib (Rendering) and Tics (Physics).
- Maps fundamental types (Vectors, Quaternions) between libraries.
- Converts spatial transforms (Physics Position/Rot <-> Render Matrix).
------------------------------------------------------------------------------------------------- */

// clang-format off
static tics_vec3 to_tics_vec(Vector3 v) { return (tics_vec3){v.x, v.y, v.z}; }
static tics_quat to_tics_quat(Quaternion q) { return (tics_quat){q.x, q.y, q.z, q.w}; }
static Vector3 to_raylib_vec(tics_vec3 v) { return (Vector3){v.x, v.y, v.z}; }
static Quaternion to_raylib_quat(tics_quat q) { return (Quaternion){q.x, q.y, q.z, q.w}; }
// clang-format on
static Matrix to_raylib_matrix(tics_transform t) {
	Matrix mat_rotation = QuaternionToMatrix(to_raylib_quat(t.rotation));
	Matrix mat_translation = MatrixTranslate(t.position.x, t.position.y, t.position.z);
	return MatrixMultiply(mat_rotation, mat_translation);
}
static tics_transform to_tics_transform(Matrix m) {
	tics_vec3 position = {m.m12, m.m13, m.m14};
	tics_quat rotation = to_tics_quat(QuaternionFromMatrix(m));
	return (tics_transform){position, rotation};
}

#endif // TICS_RAYLIB_BRIDGE_H
