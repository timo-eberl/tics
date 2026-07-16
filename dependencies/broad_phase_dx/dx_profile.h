#ifndef DX_PROFILE_H
#define DX_PROFILE_H

#include "dx_common.h"
#include <float.h>
#include <string.h>

#define DX_PROFILE_MAX_STEPS 16

typedef struct {
	const char* labels[DX_PROFILE_MAX_STEPS];
	int start_queries[DX_PROFILE_MAX_STEPS];
	int end_queries[DX_PROFILE_MAX_STEPS];
	float intervals[DX_PROFILE_MAX_STEPS];
	int count;
	int query_count;
} dx_profile;

typedef struct {
	float acc[DX_PROFILE_MAX_STEPS];
	float min[DX_PROFILE_MAX_STEPS];
	float max[DX_PROFILE_MAX_STEPS];
	int count;
	int calls;
} dx_profile_acc;

static inline void dx_profile_acc_init(dx_profile_acc* a) {
	memset(a, 0, sizeof(*a));
	for (int i = 0; i < DX_PROFILE_MAX_STEPS; ++i) {
		a->min[i] = FLT_MAX;
		a->max[i] = -FLT_MAX;
	}
}

static inline void dx_profile_begin(dx_profile* p, dx_shared_state* sh) {
	p->count = 0;
	p->query_count = 1;
	sh->cmd_list->EndQuery(sh->query_heap, D3D12_QUERY_TYPE_TIMESTAMP, 0);
}

// Establishes a new baseline timestamp to completely skip CPU/WSL idle gaps
static inline void dx_profile_split(dx_profile* p, dx_shared_state* sh) {
	sh->cmd_list->EndQuery(sh->query_heap, D3D12_QUERY_TYPE_TIMESTAMP, p->query_count);
	p->query_count++;
}

static inline void dx_profile_step(dx_profile* p, dx_shared_state* sh, const char* label) {
	if (p->count >= DX_PROFILE_MAX_STEPS) return;
	int i = p->count;
	p->labels[i] = label;
	p->start_queries[i] = p->query_count - 1;
	p->end_queries[i] = p->query_count;
	p->count++;
	sh->cmd_list->EndQuery(sh->query_heap, D3D12_QUERY_TYPE_TIMESTAMP, p->query_count);
	p->query_count++;
}

static inline void dx_profile_resolve(dx_profile* p, dx_shared_state* sh) {
	if (p->count == 0) return;
	sh->cmd_list->ResolveQueryData(sh->query_heap, D3D12_QUERY_TYPE_TIMESTAMP, 0, 
								   p->query_count, sh->rb_query, 0);
}

static inline void dx_profile_end(dx_profile* p, dx_shared_state* sh) {
	if (p->count == 0) return;
	
	uint64_t* timestamps = nullptr;
	D3D12_RANGE read_range = {0, (size_t)(p->query_count * sizeof(uint64_t))};
	sh->rb_query->Map(0, &read_range, (void**)&timestamps);
	
	for (int i = 0; i < p->count; ++i) {
		uint64_t start = timestamps[p->start_queries[i]];
		uint64_t end = timestamps[p->end_queries[i]];
		p->intervals[i] = (float)((end - start) * 1000.0 / (double)sh->timestamp_frequency);
	}
	
	D3D12_RANGE write_range = {0, 0};
	sh->rb_query->Unmap(0, &write_range);
}

static inline void dx_profile_log_frame(const dx_profile* p, const char* algo_label) {
	if (p->count == 0) return;
	
	fprintf(stderr, "[dx12] %s (frame)", algo_label);
	float total = 0.0f;
	for (int i = 0; i < p->count; ++i) {
		total += p->intervals[i];
		fprintf(stderr, " %s=%.3fms", p->labels[i], p->intervals[i]);
	}
	fprintf(stderr, " total=%.3fms\n", total);
}

static inline void dx_profile_log(const dx_profile* p, dx_profile_acc* a, const char* algo_label, 
								  int every) {
	if (p->count == 0) return;

	if (a->calls == 0) a->count = p->count;
	if (p->count > a->count) a->count = p->count; // Dynamically expand as steps are registered

	int n = p->count < a->count ? p->count : a->count;
	for (int i = 0; i < n; ++i) {
		float val = p->intervals[i];
		a->acc[i] += val;
		if (val < a->min[i]) a->min[i] = val;
		if (val > a->max[i]) a->max[i] = val;
	}
	a->calls++;

	if (a->calls % every != 0) return;

	fprintf(stderr, "[dx12] %s (avg over %d)", algo_label, a->calls);
	float total = 0;
	for (int i = 0; i < n; ++i) {
		float avg = a->acc[i] / a->calls;
		total += avg;
		fprintf(stderr, " %s=%.3fms [%.3f-%.3f]", p->labels[i], avg, a->min[i], a->max[i]);
	}
	fprintf(stderr, " total=%.3fms\n", total);
}

#endif
