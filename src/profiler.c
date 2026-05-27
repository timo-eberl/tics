// Feature macros MUST be defined before any standard #include
#if defined(__linux__) || defined(__APPLE__) || defined(__unix__)
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 199309L
#endif
#endif

#include "profiler.h"

#ifdef TICS_ENABLE_PROFILER

#include <stdio.h>
#include <time.h> // Required for fallback and POSIX timers

#if defined(__linux__) || defined(__APPLE__) || defined(__unix__)
#include <unistd.h>
#endif

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif

#ifdef __APPLE__
#include <mach/mach_time.h>
#endif

profile_timer* prof_root = NULL;
profile_timer* prof_tail = NULL;
int prof_depth = 0;

long long time_ns(void) {
	long long ns = 0;

#if defined(_WIN32) || defined(_WIN64)
	static LARGE_INTEGER frequency;
	static int initialized = 0;
	if (!initialized) {
		QueryPerformanceFrequency(&frequency);
		initialized = 1;
	}

	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);

	// Convert to nanoseconds:  (ticks * 1,000,000,000) / frequency
	// We assume frequency is non-zero (always true on modern Windows)
	ns = (long long)((now.QuadPart * 1000000000LL) / frequency.QuadPart);

#elif defined(__APPLE__)
	static mach_timebase_info_data_t timebase;
	static int initialized = 0;
	if (!initialized) {
		mach_timebase_info(&timebase);
		initialized = 1;
	}

	uint64_t now = mach_absolute_time();
	ns = (long long)((now * timebase.numer) / timebase.denom);

#elif defined(CLOCK_MONOTONIC)
	struct timespec ts;
	if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
		ns = (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
	}
#else
	// FALLBACK (Low precision, but standard C)
	// If we can't find anything better, rely on standard C time()
	ns = (long long)time(NULL) * 1000000000LL;
#endif

	return ns;
}

void profile_print(void) {
	profile_timer* t = prof_root;
	fprintf(stderr, "--- Profiler Stats ---\n");
	
	while (t) {
		if (t->call_count > 0) {
			double avg_ms = (double)t->elapsed_ns / (t->call_count * 1000000.0);

			// indentation for nicely formatted output
			int indent = (t->depth - 1) * 2;
			int width = 30 - indent; // Adjust '30' to be wider than your longest name
			if (width < 0) width = 0;

			// %*s prints indentation
			// %-*s prints name padded to the calculated width
			fprintf(stderr, "%*s%-*s: %8.4f ms (Avg over %d)\n", indent, "", width, t->name, avg_ms,
				   t->call_count);
		}
		t = t->next;
	}
	fprintf(stderr, "----------------------\n");
}

void profile_reset(void) {
	profile_timer* t = prof_root;
	while (t) {
		t->elapsed_ns = 0;
		t->call_count = 0;
		t = t->next;
	}
}

#endif // TICS_ENABLE_PROFILER
