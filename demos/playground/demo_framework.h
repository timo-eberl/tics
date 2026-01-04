#ifndef DEMO_FRAMEWORK_H
#define DEMO_FRAMEWORK_H

#include <raylib.h>
#include <tics.h>

#include <stdint.h>

// --------------------------------------------------------------------------------------
// Helpers
// --------------------------------------------------------------------------------------

tics_vec3 ToTicsVec(Vector3 v);
tics_quat ToTicsQuat(Quaternion q);
Vector3 ToRaylibVec(tics_vec3 v);
Quaternion ToRaylibQuat(tics_quat q);
float GetRandomFloat(float min, float max);

void UpdateFlyCamera(Camera3D* camera);

// Creates a Raylib model directly from raw vertex/index data.
Model LoadModelFromRaw(tics_vec3* vertices, int vertexCount, uint32_t* indices, int indexCount);

#endif
