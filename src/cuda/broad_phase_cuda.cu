#include "broad_phase_cuda.h"
#include <stdlib.h>

cuda_broad_phase_pair* broad_phase_cuda(const cuda_broad_phase_proxies_soa* rigids,
										const cuda_broad_phase_proxies_soa* statics,
										size_t* out_count) {
	(void)rigids;
	(void)statics;
	*out_count = 0;
	return NULL;
}
