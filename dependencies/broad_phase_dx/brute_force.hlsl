struct dx_aabb {
	float min_x, max_x, min_y, max_y, min_z, max_z;
};

struct dx_pair {
	uint a_index;
	uint b_index;
	uint b_type; // Maps to the 1-byte uint8_t + 3 bytes struct padding in C
};

cbuffer Constants : register(b0) {
	uint rigid_count;
	uint static_count;
	uint max_pairs;
};

StructuredBuffer<dx_aabb> rigids : register(t0);
StructuredBuffer<dx_aabb> statics : register(t1);

RWStructuredBuffer<dx_pair> pairs : register(u0);
RWStructuredBuffer<uint> pair_count : register(u1);

bool aabb_overlap(dx_aabb a, dx_aabb b) {
	return a.max_x >= b.min_x && a.min_x <= b.max_x &&
	       a.max_y >= b.min_y && a.min_y <= b.max_y &&
	       a.max_z >= b.min_z && a.min_z <= b.max_z;
}

[numthreads(256, 1, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {
	uint i = DTid.x;
	if (i >= rigid_count) return;

	dx_aabb ri = rigids[i];

	for (uint j = i + 1; j < rigid_count; ++j) {
		if (aabb_overlap(ri, rigids[j])) {
			uint idx;
			InterlockedAdd(pair_count[0], 1, idx);
			if (idx < max_pairs) {
				dx_pair p = { i, j, 1 };
				pairs[idx] = p;
			}
		}
	}

	for (uint k = 0; k < static_count; ++k) {
		if (aabb_overlap(ri, statics[k])) {
			uint idx;
			InterlockedAdd(pair_count[0], 1, idx);
			if (idx < max_pairs) {
				dx_pair p = { i, k, 0 };
				pairs[idx] = p;
			}
		}
	}
}
