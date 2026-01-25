#ifndef TICS_TEST_H
#define TICS_TEST_H

// Enable POSIX features for alarm() and signal()
#define _POSIX_C_SOURCE 200809L

#include "tics.h"
#include "tics_math.h"
#include <math.h>
#include <setjmp.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define DEFAULT_EPSILON 0.0001f

#define _FAIL_LOC(file, line, fmt, ...)                                                            \
	do {                                                                                           \
		fprintf(stderr, "[TEST FAILED] %s:%d: " fmt "\n", file, line, ##__VA_ARGS__);              \
		exit(EXIT_FAILURE);                                                                        \
	} while (0)

// macro using current location
#define _FAIL(fmt, ...) _FAIL_LOC(__FILE__, __LINE__, fmt, ##__VA_ARGS__)

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

// --- Quat Assertions ---

// Checks if 'actual' is close to 'expected' OR '-expected' (same rotation)
#define ASSERT_QUAT_WITHIN(actual, expected, tolerance)                                            \
	do {                                                                                           \
		tics_quat _a = (actual);                                                                   \
		tics_quat _e = (expected);                                                                 \
		float _t = (tolerance);                                                                    \
		bool _p = (fabsf(_a.x - _e.x) <= _t && fabsf(_a.y - _e.y) <= _t &&                         \
				   fabsf(_a.z - _e.z) <= _t && fabsf(_a.w - _e.w) <= _t);                          \
		bool _n = (fabsf(_a.x + _e.x) <= _t && fabsf(_a.y + _e.y) <= _t &&                         \
				   fabsf(_a.z + _e.z) <= _t && fabsf(_a.w + _e.w) <= _t);                          \
		if (!_p && !_n) {                                                                          \
			_FAIL("Expected Quat ~{%f, %f, %f, %f} (flipped allowed), got {%f, %f, %f, %f}", _e.x, \
				  _e.y, _e.z, _e.w, _a.x, _a.y, _a.z, _a.w);                                       \
		}                                                                                          \
	} while (0)

#define ASSERT_QUAT_NOT_WITHIN(actual, expected, tolerance)                                        \
	do {                                                                                           \
		tics_quat _a = (actual);                                                                   \
		tics_quat _e = (expected);                                                                 \
		float _t = (tolerance);                                                                    \
		bool _p = (fabsf(_a.x - _e.x) <= _t && fabsf(_a.y - _e.y) <= _t &&                         \
				   fabsf(_a.z - _e.z) <= _t && fabsf(_a.w - _e.w) <= _t);                          \
		bool _n = (fabsf(_a.x + _e.x) <= _t && fabsf(_a.y + _e.y) <= _t &&                         \
				   fabsf(_a.z + _e.z) <= _t && fabsf(_a.w + _e.w) <= _t);                          \
		if (_p || _n) {                                                                            \
			_FAIL("Expected Quats to differ > %f, but they are equivalent rotations", _t);         \
		}                                                                                          \
	} while (0)

#define ASSERT_QUAT_APPROX(actual, expected) ASSERT_QUAT_WITHIN(actual, expected, DEFAULT_EPSILON)

#define ASSERT_QUAT_NOT_APPROX(actual, expected)                                                   \
	ASSERT_QUAT_NOT_WITHIN(actual, expected, DEFAULT_EPSILON)

// --- Timeout / Deadlock Protection ---

// Static buffer to hold the execution state.
// We use sigjmp_buf to ensure signal masks are restored correctly.
static sigjmp_buf _test_timeout_env;

static void _test_sigalrm_handler(int sig) {
	(void)sig;
	siglongjmp(_test_timeout_env, 1);
}
// clang-format off

// Sets a timeout in seconds. If code execution takes longer, the test fails.
// Creates a local scope that ends at TEST_TIMEOUT_END.
#define TEST_TIMEOUT_BEGIN(seconds)                                                                \
	do {                                                                                           \
		const char* _timeout_file = __FILE__;                                                      \
		int _timeout_line = __LINE__;                                                              \
		signal(SIGALRM, _test_sigalrm_handler);                                                    \
		/* Save state. If returns 0, it's the initial call. */                                     \
		if (sigsetjmp(_test_timeout_env, 1) == 0) {                                                \
			alarm(seconds);                                                                        \
			{

// Cancels the timeout.
#define TEST_TIMEOUT_END()                                                                         \
			}                                                                                      \
			alarm(0);                                                                              \
		} else {                                                                                   \
			/* We returned from the signal handler via siglongjmp. */                              \
			/* It is now safe to use _FAIL_LOC, which calls exit() and triggers atexit(). */       \
			_FAIL_LOC(_timeout_file, _timeout_line,                                                \
					  "Timeout reached: Possible endless loop detected.");                         \
		}                                                                                          \
	} while (0)
// clang-format on

// --- Entry Points ---

void run_api_tests(void);
void run_core_tests(void);
void run_collision_test_tests(void);
void run_dynamics_tests(void);
void run_velocity_at_point_tests(void);

#endif // TICS_TEST_H
