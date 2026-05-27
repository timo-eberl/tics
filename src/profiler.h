#ifndef PROFILER_H
#define PROFILER_H

#include <stdint.h>

#ifdef TICS_ENABLE_PROFILER

typedef struct profile_timer {
	const char* name;
	uint64_t elapsed_ns;
	uint32_t call_count;
	int depth;
	struct profile_timer* next;
	int registered;
} profile_timer;

// Extern declarations allow all translation units to share the exact same profiling state.
extern profile_timer* prof_root;
extern profile_timer* prof_tail;
extern int prof_depth;

long long time_ns(void);
void profile_print(void);
void profile_reset(void);

#define P_CONCAT(a, b) a##b
#define P_VAR(name, line) P_CONCAT(name, line)

// This macro abuses a for-loop to inject scope-based timing. 
// It automatically records the start time, executes the user's block, and records the end time 
// during the loop's update step.
// WARNING: Using return, break, or goto inside a PROFILE block skips the update step. This will
// permanently desync prof_depth and ruin all subsequent output formatting.
#define PROFILE(NAME)                                                                              \
	static profile_timer P_VAR(_pt_, __LINE__) = {NAME, 0, 0, 0, NULL, 0};                         \
	if (!P_VAR(_pt_, __LINE__).registered) {                                                       \
		if (!prof_root) prof_root = &P_VAR(_pt_, __LINE__);                                        \
		else prof_tail->next = &P_VAR(_pt_, __LINE__);                                             \
		prof_tail = &P_VAR(_pt_, __LINE__);                                                        \
		P_VAR(_pt_, __LINE__).registered = 1;                                                      \
	}                                                                                              \
	for (uint64_t _start = (prof_depth++, time_ns()), _once = 1; _once;                            \
		 P_VAR(_pt_, __LINE__).elapsed_ns += (time_ns() - _start),                                 \
		 P_VAR(_pt_, __LINE__).call_count++,                                                       \
		 P_VAR(_pt_, __LINE__).depth = prof_depth,                                                 \
		 prof_depth--, _once = 0)

#else // TICS_ENABLE_PROFILER is not defined

// No-op definitions
#define PROFILE(NAME)
static inline long long time_ns(void) { return 0; }
static inline void profile_print(void) {}
static inline void profile_reset(void) {}

#endif // TICS_ENABLE_PROFILER

#endif // PROFILER_H
