#ifndef TICS_TEST_H
#define TICS_TEST_H

#include "tics.h"
#include "tics_math.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define DEFAULT_EPSILON 0.0001f

#define _FAIL(fmt, ...)                                                                            \
	do {                                                                                           \
		fprintf(stderr, "[TEST FAILED] %s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__);            \
		exit(EXIT_FAILURE);                                                                        \
	} while (0)

// --- Boolean Assertions ---

#define ASSERT_TRUE(cond)                                                                          \
	do {                                                                                           \
		if (!(cond)) _FAIL("Expected TRUE, got FALSE: %s", #cond);                                 \
	} while (0)

#define ASSERT_FALSE(cond)                                                                         \
	do {                                                                                           \
		if (cond) _FAIL("Expected FALSE, got TRUE: %s", #cond);                                    \
	} while (0)

// --- Integer Assertions ---

#define ASSERT_INT_EQ(actual, expected)                                                            \
	do {                                                                                           \
		int _a = (actual);                                                                         \
		int _e = (expected);                                                                       \
		if (_a != _e) _FAIL("Expected %d, got %d", _e, _a);                                        \
	} while (0)

#define ASSERT_INT_NEQ(actual, expected)                                                           \
	do {                                                                                           \
		int _a = (actual);                                                                         \
		int _e = (expected);                                                                       \
		if (_a == _e) _FAIL("Expected anything but %d, got %d", _e, _a);                           \
	} while (0)

// --- Float Assertions ---

#define ASSERT_FLOAT_WITHIN(actual, expected, tolerance)                                           \
	do {                                                                                           \
		float _a = (actual);                                                                       \
		float _e = (expected);                                                                     \
		float _t = (tolerance);                                                                    \
		if (fabsf(_a - _e) > _t)                                                                   \
			_FAIL("Expected %f (within %f), got %f (diff: %f)", _e, _t, _a, fabsf(_a - _e));       \
	} while (0)

#define ASSERT_FLOAT_NOT_WITHIN(actual, expected, tolerance)                                       \
	do {                                                                                           \
		float _a = (actual);                                                                       \
		float _e = (expected);                                                                     \
		float _t = (tolerance);                                                                    \
		if (fabsf(_a - _e) <= _t)                                                                  \
			_FAIL("Expected values to differ > %f, but %f is close to %f", _t, _a, _e);            \
	} while (0)

// Uses default epsilon
#define ASSERT_FLOAT_APPROX(actual, expected) ASSERT_FLOAT_WITHIN(actual, expected, DEFAULT_EPSILON)

// Uses default epsilon
#define ASSERT_FLOAT_NOT_APPROX(actual, expected)                                                  \
	ASSERT_FLOAT_NOT_WITHIN(actual, expected, DEFAULT_EPSILON)

// --- Vec3 Assertions ---

// Component-wise tolerance check
#define ASSERT_VEC3_WITHIN(actual, expected, tolerance)                                            \
	do {                                                                                           \
		tics_vec3 _a = (actual);                                                                   \
		tics_vec3 _e = (expected);                                                                 \
		float _t = (tolerance);                                                                    \
		if (fabsf(_a.x - _e.x) > _t || fabsf(_a.y - _e.y) > _t || fabsf(_a.z - _e.z) > _t) {       \
			_FAIL("Expected Vec3 ~{%f, %f, %f} (within %f), got {%f, %f, %f}", _e.x, _e.y, _e.z,   \
				  _t, _a.x, _a.y, _a.z);                                                           \
		}                                                                                          \
	} while (0)

// Component-wise tolerance check
#define ASSERT_VEC3_NOT_WITHIN(actual, expected, tolerance)                                        \
	do {                                                                                           \
		tics_vec3 _a = (actual);                                                                   \
		tics_vec3 _e = (expected);                                                                 \
		float _t = (tolerance);                                                                    \
		if (fabsf(_a.x - _e.x) <= _t && fabsf(_a.y - _e.y) <= _t && fabsf(_a.z - _e.z) <= _t) {    \
			_FAIL("Expected Vec3s to differ > %f, but both are within range {%f, %f, %f}", _t,     \
				  _a.x, _a.y, _a.z);                                                               \
		}                                                                                          \
	} while (0)

// Component-wise tolerance check. Uses default epsilon
#define ASSERT_VEC3_APPROX(actual, expected) ASSERT_VEC3_WITHIN(actual, expected, DEFAULT_EPSILON)

// Component-wise tolerance check. Uses default epsilon
#define ASSERT_VEC3_NOT_APPROX(actual, expected)                                                   \
	ASSERT_VEC3_NOT_WITHIN(actual, expected, DEFAULT_EPSILON)

// --- Entry Points ---

void run_api_tests(void);
void run_core_tests(void);

#endif // TICS_TEST_H
