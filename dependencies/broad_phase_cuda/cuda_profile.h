#ifndef CUDA_PROFILE_H
#define CUDA_PROFILE_H

#include <cuda_runtime.h>
#include <stdio.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Lightweight per-algorithm GPU profiler.
//
// Usage:
//
//   // Persistent accumulator — one per algorithm, stores running averages.
//   static cuda_profile_acc acc;
//   static bool acc_init = false;
//   if (!acc_init) { cuda_profile_acc_init(&acc); acc_init = true; }
//
//   // Per-frame profiling:
//   cuda_profile prof;
//   cuda_profile_begin(&prof);
//
//   /* ... upload work ... */
//   cuda_profile_step(&prof, "upload");
//
//   /* ... kernel work ... */
//   cuda_profile_step(&prof, "kernel");
//
//   /* ... readback work ... */
//   cuda_profile_step(&prof, "readback");
//
//   cuda_profile_end(&prof);                    // syncs, computes intervals, destroys events
//   cuda_profile_log(&prof, &acc, "grid", 10);  // accumulates & prints every 10th call
//
// ---------------------------------------------------------------------------

#define CUDA_PROFILE_MAX_STEPS 16

typedef struct {
	cudaEvent_t ev_start;
	cudaEvent_t ev_steps[CUDA_PROFILE_MAX_STEPS];
	const char* labels[CUDA_PROFILE_MAX_STEPS];
	float
		intervals[CUDA_PROFILE_MAX_STEPS]; // intervals[i] = time from step i-1 (or start) to step i
	int count;							   // number of steps recorded
} cuda_profile;

typedef struct {
	float acc[CUDA_PROFILE_MAX_STEPS];
	int count; // expected number of steps (set on first log call)
	int calls;
} cuda_profile_acc;

static inline void cuda_profile_acc_init(cuda_profile_acc* a) {
	memset(a, 0, sizeof(*a));
}

static inline void cuda_profile_begin(cuda_profile* p) {
	p->count = 0;
	cudaEventCreate(&p->ev_start);
	cudaEventRecord(p->ev_start);
}

static inline void cuda_profile_step(cuda_profile* p, const char* label) {
	if (p->count >= CUDA_PROFILE_MAX_STEPS) return;
	int i = p->count++;
	p->labels[i] = label;
	cudaEventCreate(&p->ev_steps[i]);
	cudaEventRecord(p->ev_steps[i]);
}

static inline void cuda_profile_end(cuda_profile* p) {
	if (p->count == 0) {
		cudaEventDestroy(p->ev_start);
		return;
	}

	cudaEventSynchronize(p->ev_steps[p->count - 1]);

	// Compute intervals
	cudaEvent_t prev = p->ev_start;
	for (int i = 0; i < p->count; ++i) {
		cudaEventElapsedTime(&p->intervals[i], prev, p->ev_steps[i]);
		prev = p->ev_steps[i];
	}

	// Cleanup
	cudaEventDestroy(p->ev_start);
	for (int i = 0; i < p->count; ++i)
		cudaEventDestroy(p->ev_steps[i]);
}

// Accumulate into `acc` and print a summary line every `every` calls.
// Each step is printed as "label=X.XXXms". A total is appended at the end.
static inline void cuda_profile_log(const cuda_profile* p, cuda_profile_acc* a,
									const char* algo_label, int every) {
	if (p->count == 0) return;

	// On first call, lock in the step count
	if (a->calls == 0) a->count = p->count;

	int n = p->count < a->count ? p->count : a->count;
	for (int i = 0; i < n; ++i)
		a->acc[i] += p->intervals[i];
	a->calls++;

	if (a->calls % every != 0) return;

	fprintf(stderr, "[cuda] %s (avg over %d)", algo_label, a->calls);
	float total = 0;
	for (int i = 0; i < n; ++i) {
		float avg = a->acc[i] / a->calls;
		total += avg;
		fprintf(stderr, " %s=%.3fms", p->labels[i], avg);
	}
	fprintf(stderr, " total=%.3fms\n", total);
}

#endif
