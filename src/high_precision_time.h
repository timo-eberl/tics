#include <stdint.h>

// Linux / POSIX
#if defined(__linux__) || defined(__APPLE__) || defined(__unix__)
#define _POSIX_C_SOURCE 199309L
#include <time.h>
#include <unistd.h>
#endif

// Windows
#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif

// MacOS specific (for older versions fallback)
#ifdef __APPLE__
#include <mach/mach_time.h>
#endif

// Returns a timestamp in nanoseconds (platform independent)
long long time_ns() {
	long long ns = 0;

#if defined(_WIN32) || defined(_WIN64)
	// WINDOWS: Use QueryPerformanceCounter (High Resolution)
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
	// MACOS: Use mach_absolute_time for highest precision on older & newer Macs
	static mach_timebase_info_data_t timebase;
	static int initialized = 0;
	if (!initialized) {
		mach_timebase_info(&timebase);
		initialized = 1;
	}

	uint64_t now = mach_absolute_time();
	ns = (long long)((now * timebase.numer) / timebase.denom);

#elif defined(CLOCK_MONOTONIC)
	// LINUX / POSIX: Use clock_gettime
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
