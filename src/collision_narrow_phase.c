#include "blick_adapter.h"
#include "tics_internal.h"
#include "tics_math.h"

#include <stb_ds.h>

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

// Ensures the pair is ordered deterministically before collision detection.
// Order:
// 1. By Type (RIGID_BODY, STATIC_BODY)
// 2. By Index (Low < High)
static inline void canonicalize_pair(broad_phase_pair* p) {
	int swap = 0;

	// Rule 1: Sort by Type (Rigid=1 Static=0)
	if (p->a.type < p->b.type) { swap = 1; }
	// Rule 2: If types are identical, Sort by Index
	else if (p->a.type == p->b.type && p->a.index > p->b.index) { swap = 1; }

	if (swap) {
		body_ref temp = p->a;
		p->a = p->b;
		p->b = temp;
	}
}

static int compare_collisions(const void* lhs, const void* rhs);

collision* narrow_phase(const broad_phase_pair* pairs, size_t pair_count,
						const rigid_body_data* r_bodies, const static_body_data* s_bodies) {

	// Early exit if broadphase found nothing
	if (pair_count == 0) return NULL;

	// Setup per-thread storage
	int max_threads = omp_get_max_threads();
	collision** thread_buffers = calloc(max_threads, sizeof(collision*));

	// Uncomment this to disable multi-threading
	// omp_set_num_threads(1);

	// Parallel loop over potential collision pairs
	// We use 'static' because chunks have roughly the same workload
#pragma omp parallel for schedule(static)
	for (size_t i = 0; i < pair_count; ++i) {
		int tid = omp_get_thread_num();

		broad_phase_pair p = pairs[i];
		canonicalize_pair(&p);

		// A
		const shape_data* shape_a;
		tics_transform trans_a;
		if (p.a.type == RIGID_BODY) {
			shape_a = &r_bodies[p.a.index].shape;
			trans_a = r_bodies[p.a.index].transform;
		}
		else {
			shape_a = &s_bodies[p.a.index].shape;
			trans_a = s_bodies[p.a.index].transform;
		}
		// B
		const shape_data* shape_b;
		tics_transform trans_b;
		if (p.b.type == RIGID_BODY) {
			shape_b = &r_bodies[p.b.index].shape;
			trans_b = r_bodies[p.b.index].transform;
		}
		else {
			shape_b = &s_bodies[p.b.index].shape;
			trans_b = s_bodies[p.b.index].transform;
		}

		// --- Actual Geometric Test ---
		collision_result res = collision_test(shape_a, trans_a, shape_b, trans_b);

		if (res.has_collision) {
			collision col;
			col.body_a_ref = p.a;
			col.body_b_ref = p.b;
			col.result = res;

			// Write to thread-local buffer
			arrput(thread_buffers[tid], col);
		}
	} // implicit barrier

	// Merge Phase
	// Calculate total collisions to allocate exact memory once
	size_t total_count = 0;
	for (int i = 0; i < max_threads; ++i) {
		total_count += arrlen(thread_buffers[i]);
	}

	collision* collisions = NULL;
	arrsetlen(collisions, total_count);

	size_t offset = 0;
	for (int i = 0; i < max_threads; ++i) {
		size_t count = arrlen(thread_buffers[i]);
		if (count > 0) {
			memcpy(collisions + offset, thread_buffers[i], count * sizeof(collision));
			offset += count;
		}
		arrfree(thread_buffers[i]);
	}
	free(thread_buffers);

	// Sort the collisions so the order is exactly the same -> Determinism with Multi-Threading
	size_t col_size = arrlen(collisions);
	if (col_size > 0) { qsort(collisions, col_size, sizeof(collision), compare_collisions); }

	return collisions;
}

static int compare_collisions(const void* lhs, const void* rhs) {
	const collision* a = (const collision*)lhs;
	const collision* b = (const collision*)rhs;

	// 1. Compare Body A Type
	if (a->body_a_ref.type != b->body_a_ref.type)
		return (int)a->body_a_ref.type - (int)b->body_a_ref.type;

	// 2. Compare Body A Index
	if (a->body_a_ref.index != b->body_a_ref.index)
		return (a->body_a_ref.index < b->body_a_ref.index) ? -1 : 1;

	// 3. Compare Body B Type
	if (a->body_b_ref.type != b->body_b_ref.type)
		return (int)a->body_b_ref.type - (int)b->body_b_ref.type;

	// 4. Compare Body B Index
	if (a->body_b_ref.index != b->body_b_ref.index)
		return (a->body_b_ref.index < b->body_b_ref.index) ? -1 : 1;

	return 0;
}

// support point on minkowski difference
// point on shape b is calculated as a - m
typedef struct {
	tics_vec3 m; // minkowski difference
	tics_vec3 a; // corresponding point on shape a
} mink_support;

typedef struct {
	tics_vec3 normal;
	float distance;
} face_plane;

typedef struct {
	uint32_t a;
	uint32_t b;
} edge;

// A support function takes a direction d and returns a point on the boundary of a shape "furthest"
// in direction d
static tics_vec3 support_point_mesh(const shape_data* c, tics_transform t, tics_vec3 d) {
	assert(c->type == SHAPE_CONVEX);

	tics_vec3 local_d = quat_rotate_vec3(d, quat_inverse(t.rotation));

	const tics_vec3* vertices = c->data.convex.vertices;
	size_t count = c->data.convex.count;

	// find the support point in local space
	float support_point_dot = -FLT_MAX;
	tics_vec3 support = {0, 0, 0};

	for (size_t i = 0; i < count; ++i) {
		float p_dot_d = vec3_dot(vertices[i], local_d);
		if (p_dot_d > support_point_dot) {
			support_point_dot = p_dot_d;
			support = vertices[i];
		}
	}

	// this fails if the center position of a mesh is not inside the mesh
	assert(support_point_dot >= 0.0);

	support = quat_rotate_vec3(support, t.rotation);
	support = vec3_add(support, t.position);

	return support;
}

static mink_support support_point_on_minkowski_diff_mesh_mesh(const shape_data* ca,
															  tics_transform ta,
															  const shape_data* cb,
															  tics_transform tb, tics_vec3 d) {
	assert(ca->type == SHAPE_CONVEX);
	assert(cb->type == SHAPE_CONVEX);

	mink_support point;
	point.a = support_point_mesh(ca, ta, d);
	// point.b = support_point_mesh(cb, tb, - d);
	// point.m = point.a - point.b;
	tics_vec3 b_supp = support_point_mesh(cb, tb, vec3_negate(d));
	point.m = vec3_sub(point.a, b_supp);

	return point;
}

static void add_if_unique_edge(edge** edges, uint32_t edge_a, uint32_t edge_b) {
	size_t count = arrlen(*edges);
	ptrdiff_t found_idx = -1;

	for (size_t i = 0; i < count; ++i) {
		if ((*edges)[i].a == edge_b && (*edges)[i].b == edge_a) {
			found_idx = i;
			break;
		}
	}

	// edge was already present -> remove it
	if (found_idx != -1) { arrdel(*edges, found_idx); }
	else {
		edge new_edge = {edge_a, edge_b};
		arrput(*edges, new_edge);
	}
}

// When the origin lies exactly on a simplex feature (point, line, or triangle),  the search
// direction becomes a zero-vector. Since EPA requires a non-degenerate tetrahedron, we expand into
// a tetrahedron.
static void pad_simplex_to_tetrahedron(const shape_data* as, tics_transform ta,
									   const shape_data* bs, tics_transform tb,
									   mink_support simplex[4], int* count_ptr) {
	int count = *count_ptr;

	// Point to Line Expansion
	if (count == 1) {
		tics_vec3 A = simplex[0].m;
		tics_vec3 axes[6] = {
			{1, 0, 0}, {-1, 0, 0},
			{0, 1, 0}, {0, -1, 0},
			{0, 0, 1}, {0, 0, -1}
		};

		float max_dist_sq = -1.0f;
		mink_support best_p = simplex[0];

		for (int i = 0; i < 6; ++i) {
			mink_support p = support_point_on_minkowski_diff_mesh_mesh(as, ta, bs, tb, axes[i]);
			float dist_sq = vec3_length_sq(vec3_sub(p.m, A));
			if (dist_sq > max_dist_sq) {
				max_dist_sq = dist_sq;
				best_p = p;
			}
		}

		assert(max_dist_sq > 0.0f &&
			   "Minkowski difference is a single point (input geometry has 0 volume).");
		simplex[1] = best_p;
		count = 2;
	}

	// Line to Triangle Expansion
	if (count == 2) {
		tics_vec3 A = simplex[1].m;
		tics_vec3 B = simplex[0].m;
		tics_vec3 AB = vec3_sub(B, A);

		// Build an orthogonal basis around the line segment AB.
		// We use the cross product against a global axis to find a perpendicular vector 'u'.
		// If AB is strongly aligned with the X-axis, we use Y to avoid collinearity.
		tics_vec3 axis =
			(fabsf(AB.x) > 0.9f * vec3_length(AB)) ? (tics_vec3){0, 1, 0} : (tics_vec3){1, 0, 0};
		tics_vec3 u = vec3_normalize(vec3_cross(AB, axis));
		tics_vec3 w = vec3_cross(vec3_normalize(AB), u);

		tics_vec3 dirs[4] = {u, vec3_negate(u), w, vec3_negate(w)};

		float max_area_sq = -1.0f;
		mink_support best_p = simplex[1];

		for (int i = 0; i < 4; ++i) {
			mink_support p = support_point_on_minkowski_diff_mesh_mesh(as, ta, bs, tb, dirs[i]);
			// The area of the triangle is proportional to the length of the cross product of its
			// two edge vectors. We want to maximize this to find the biggest triangle.
			float area_sq = vec3_length_sq(vec3_cross(AB, vec3_sub(p.m, A)));
			if (area_sq > max_area_sq) {
				max_area_sq = area_sq;
				best_p = p;
			}
		}

		assert(max_area_sq > 0.0f &&
			   "Minkowski difference is completely 1-dimensional (flat input geometry).");
		simplex[2] = best_p;
		count = 3;
	}

	// Triangle to Tetrahedron Expansion
	if (count == 3) {
		tics_vec3 A = simplex[2].m;
		tics_vec3 B = simplex[1].m;
		tics_vec3 C = simplex[0].m;

		tics_vec3 n = vec3_normalize(vec3_cross(vec3_sub(B, A), vec3_sub(C, A)));
		tics_vec3 dirs[2] = { n, vec3_negate(n) };

		float max_dist = -1.0f;
		mink_support best_p = simplex[2];
		bool needs_swap = false;

		for (int i = 0; i < 2; ++i) {
			mink_support p = support_point_on_minkowski_diff_mesh_mesh(as, ta, bs, tb, dirs[i]);
			float dist = fabsf(vec3_dot(n, vec3_sub(p.m, A)));
			if (dist > max_dist) {
				max_dist = dist;
				best_p = p;
				// If we picked the point from the -n direction (i == 1), it lies below the
				// triangle and we swap B and C to fix the winding
				needs_swap = (i == 1);
			}
		}

		assert(max_dist > 0.0f &&
			   "Minkowski difference is completely flat (2D planar input geometry).");
		simplex[3] = best_p;

		// We must guarantee that the base triangle ABC is wound counter-clockwise relative to the
		// new vertex D, ensuring the normal points outwards.
		if (needs_swap) {
			mink_support temp = simplex[1];
			simplex[1] = simplex[0];
			simplex[0] = temp;
		}
		count = 4;
	}

	assert(count == 4 && "Simplex padding failed to construct a valid 4-vertex tetrahedron.");
	*count_ptr = count;
}

// Collision detection is based on the GJK Algorithm.
//   First implementation is based on https://youtu.be/ajv46BSqcK4
//   However that implementation didn't cover all cases for 3D that caused cycling.
//   Added improvements based on https://gist.github.com/vurtun/29727217c269a2fbf4c0ed9a1d11cb40
static bool run_gjk(const shape_data* as, tics_transform ta,
					const shape_data* bs, tics_transform tb, mink_support simplex[4]) {

	// first direction is arbitrary - we use the direction from the origin of one shape to the other
	tics_vec3 d = vec3_sub(tb.position, ta.position);
	// if this is zero, we use a fallback
	if (vec3_length_sq(d) == 0) d = (tics_vec3){1, 0, 0};

	// Simplex size. Can be a point (1), line segment (2), triangle (3) or polyhedron (4)
	int count = 0;
	bool boundary_contact = false;

	int gjk_iter = 0;
	while (true) {
		gjk_iter++;
		if (gjk_iter > 100) {
			assert(false && "GJK is cycling.");
			return false;
		}

		// find the next support point
		simplex[count] = support_point_on_minkowski_diff_mesh_mesh(as, ta, bs, tb, d);
		// if the new support point does not "pass" the origin, the shapes do not intersect
		if (vec3_dot(simplex[count].m, d) < 0.001f) { return false; }
		count++;

		// For case 1,2,3 the simplex stays the same size or is expanded. For case 4, the simplex
		// may be reduced (special case for GJK in 3D).
		switch (count) {
		// point
		case 1: {
			// trigger special handling if origin lies on the first support point
			if (vec3_length_sq(simplex[0].m) < 0.000001f) {
				boundary_contact = true;
				break;
			}
			// the second direction is towards the origin
			d = vec3_negate(simplex[0].m);
		} break;
		// line segment
		case 2: {
			// A = most recently added vertex, O = Origin
			tics_vec3 AB = vec3_sub(simplex[0].m, simplex[1].m);
			tics_vec3 AO = vec3_negate(simplex[1].m);
			// triple product: vector perpendicular to AB pointing toward the origin
			d = vec3_cross(vec3_cross(AB, AO), AB);

			// trigger special handling if origin lies on the line segment
			if (vec3_length_sq(d) < 0.000001f) {
				boundary_contact = true;
				break;
			}
		} break;
		// triangle
		case 3: {
			// We could check if the origin lies on the line AB or AC, but we forward that problem
			// to case 4.

			// A = most recently added vertex, O = Origin
			tics_vec3 AB = vec3_sub(simplex[1].m, simplex[2].m);
			tics_vec3 AC = vec3_sub(simplex[0].m, simplex[2].m);
			tics_vec3 AO = vec3_negate(simplex[2].m);

			// triple products to define regions R_AB and R_AC
			tics_vec3 ABC_normal = vec3_cross(AB, AC);
			tics_vec3 AB_normal = vec3_cross(vec3_negate(ABC_normal), AB); // (AC x AB) x AB
			tics_vec3 AC_normal = vec3_cross(ABC_normal, AC);			   // (AB x AC) x AC

			if (vec3_dot(AB_normal, AO) > 0) {
				// We are in region AB
				// Remove current C, shift the array so newest vertex is at simplex[2]
				simplex[0] = simplex[1];
				simplex[1] = simplex[2];
				d = AB_normal;
				count = 2;
			}
			else if (vec3_dot(AC_normal, AO) > 0) {
				// We are in region AC
				// Remove current B, shift the array so newest vertex is at simplex[2]
				simplex[1] = simplex[2];
				d = AC_normal;
				count = 2;
			}
			else {
				// We are in region ABC. Check if the origin is above or below ABC and move on.

				float abc_dot_ao = vec3_dot(ABC_normal, AO);
				// trigger special handling if the origin lies on the plane of the triangle
				if (abc_dot_ao == 0.0f) {
					boundary_contact = true;
					break;
				}

				if (abc_dot_ao > 0) {
					// above ABC
					d = ABC_normal;
				}
				else {
					// below ABC
					// swap current C and B (change winding order), so we are above ABC again
					mink_support B = simplex[1];
					simplex[1] = simplex[0];
					simplex[0] = B;
					d = vec3_negate(ABC_normal);
				}
			}
		} break;
		// tetrahedron
		case 4: {
			// We could check if the origin lies exactly on any relevant line/plane.
			// We "forward" that problem

			mink_support A = simplex[3];
			mink_support B = simplex[2];
			mink_support C = simplex[1];
			mink_support D = simplex[0];

			tics_vec3 AB = vec3_sub(B.m, A.m);
			tics_vec3 AC = vec3_sub(C.m, A.m);
			tics_vec3 AD = vec3_sub(D.m, A.m);
			tics_vec3 AO = vec3_negate(A.m);

			tics_vec3 ABC_normal = vec3_cross(AB, AC);
			tics_vec3 ACD_normal = vec3_cross(AC, AD);
			tics_vec3 ADB_normal = vec3_cross(AD, AB);

			// Barycentric Coordinate Determinants
			// We calculate cross products of the vertices (relative to origin).
			// These represent the normals of the sub-triangles formed by the Origin and the Edge.
			// By dotting these with the Face Normal, we find the signed volume/area contribution.
			// if (Normal . (A x B)) < 0, the origin is on the 'outside' of edge AB relative to the
			// face.

			tics_vec3 OAB_normal = vec3_cross(A.m, B.m);
			tics_vec3 OAC_normal = vec3_cross(A.m, C.m);
			tics_vec3 OAD_normal = vec3_cross(A.m, D.m);
			tics_vec3 OBC_normal = vec3_cross(B.m, C.m);
			tics_vec3 OCD_normal = vec3_cross(C.m, D.m);
			tics_vec3 ODB_normal = vec3_cross(D.m, B.m);

			// Barycentrics for Face ABC
			// w_abc: vertex C's contribution. Negative -> outside ABC across edge AB.
			// v_abc: vertex B's contribution. Negative -> outside ABC across edge AC.
			// u_abc: vertex A's contribution. Negative -> outside ABC across edge BC.
			float w_abc = vec3_dot(ABC_normal, OAB_normal);
			// Note: OAC_normal is A->C, winding flip for AC edge check inside ABC
			float v_abc = vec3_dot(ABC_normal, vec3_negate(OAC_normal));
			float u_abc = vec3_dot(ABC_normal, OBC_normal);

			// Barycentrics for Face ACD
			// v_acd: D's contribution
			// u_acd: C's contribution
			// w_acd: A's contribution
			float v_acd = vec3_dot(ACD_normal, OAC_normal);
			float u_acd = vec3_dot(ACD_normal, vec3_negate(OAD_normal));
			float w_acd = vec3_dot(ACD_normal, OCD_normal);

			// Barycentrics for Face ADB
			// u_adb: B's contribution
			// w_adb: D's contribution
			// v_adb: A's contribution
			float u_adb = vec3_dot(ADB_normal, OAD_normal);
			float w_adb = vec3_dot(ADB_normal, vec3_negate(OAB_normal));
			float v_adb = vec3_dot(ADB_normal, ODB_normal);

			// Note: We only check faces connected to A (ABC, ACD, ADB) because the origin can not
			// be in the region of BDC. If it were, the "pass origin" check would have failed
			// earlier. We need to check all edge regions.

			// --- CHECK EDGES CONNECTED TO A ---
			// An edge is the closest feature if the origin is outside of the edges two triangles
			// across said edge.

			// Check Edge AB
			// w_abc <= 0: outside ABC across edge AB
			// w_adb <= 0: outside ADB across edge AB
			if (w_abc <= 0 && w_adb <= 0) {
				// Reduce to Line AB
				simplex[0] = B;
				simplex[1] = A;
				count = 2;
				d = vec3_cross(vec3_cross(AB, AO), AB);
				break;
			}
			// Check Edge AC
			if (v_abc <= 0 && v_acd <= 0) {
				// Reduce to Line AC
				simplex[0] = C;
				simplex[1] = A;
				count = 2;
				d = vec3_cross(vec3_cross(AC, AO), AC);
				break;
			}
			// Check Edge AD
			if (u_acd <= 0 && u_adb <= 0) {
				// Reduce to Line AD
				simplex[0] = D;
				simplex[1] = A;
				count = 2;
				d = vec3_cross(vec3_cross(AD, AO), AD);
				break;
			}

			// --- CHECK FACES ---
			// To be in a Face Voronoi region, the origin must be in front of the face plane (dot >
			// 0) AND inside the triangular prism defined by the edges (all barycentrics > 0).
			// Checking only the dot product does not suffice, because multiple dot products can be
			// positive.

			if (vec3_dot(ABC_normal, AO) > 0 && w_abc > 0 && v_abc > 0 && u_abc > 0) {
				// We are strictly in region ABC
				simplex[0] = C;
				simplex[1] = B;
				simplex[2] = A;
				count = 3;
				d = ABC_normal;
				break;
			}
			if (vec3_dot(ACD_normal, AO) > 0 && v_acd > 0 && u_acd > 0 && w_acd > 0) {
				// We are strictly in region ACD
				simplex[0] = D;
				simplex[1] = C;
				simplex[2] = A;
				count = 3;
				d = ACD_normal;
				break;
			}
			if (vec3_dot(ADB_normal, AO) > 0 && u_adb > 0 && w_adb > 0 && v_adb > 0) {
				// We are strictly in region ADB
				simplex[0] = B;
				simplex[1] = D;
				simplex[2] = A;
				count = 3;
				d = ADB_normal;
				break;
			}

			// --- CHECK BASE EDGES ---
			// For these edges, we must verify the origin is outside BOTH adjacent faces (The upper
			// face and BDC). Those edges are not connected to A, so we theoretically only need to
			// check that condition for the triangle that is connected to A, because the other
			// triangle is BDC and we can not be in its voronoi region. However we need to keep in
			// mind that it's possible that the origin lies exactly on BDC. In that case we will not
			// reduce, but instead continue to EPA. We could have handled the "exaclty on" cases
			// beforehand separately, but since we didn't we need to deal with it here.

			// Calculate Base Face Normal (BDC): BC x BD
			tics_vec3 BCD_normal = vec3_cross(vec3_sub(C.m, B.m), vec3_sub(D.m, B.m));
			// Barycentrics for Face BDC (Base)
			// d_bcd: vertex D's contribution. Negative -> outside BDC across edge BC.
			// b_bcd: vertex B's contribution. Negative -> outside BDC across edge CD.
			// c_bcd: vertex C's contribution. Negative -> outside BDC across edge DB.
			float d_bcd = vec3_dot(BCD_normal, OBC_normal);
			float b_bcd = vec3_dot(BCD_normal, OCD_normal);
			float c_bcd = vec3_dot(BCD_normal, ODB_normal);

			// Check Edge BC
			// u_abc <= 0: outside ABC across edge BC
			// d_bcd <= 0: outside BDC across edge BC
			if (u_abc <= 0 && d_bcd <= 0) {
				// Reduce to Line BC
				simplex[0] = C;
				simplex[1] = B;
				count = 2;
				tics_vec3 BC = vec3_sub(C.m, B.m);
				d = vec3_cross(vec3_cross(BC, vec3_negate(B.m)), BC);
				break;
			}
			// Check Edge CD
			// w_acd <= 0: outside ACD across edge CD
			// b_bcd <= 0: outside BDC across edge CD
			if (w_acd <= 0 && b_bcd <= 0) {
				// Reduce to Line CD
				simplex[0] = D;
				simplex[1] = C;
				count = 2;
				tics_vec3 CD = vec3_sub(D.m, C.m);
				d = vec3_cross(vec3_cross(CD, vec3_negate(C.m)), CD);
				break;
			}
			// Check Edge DB
			// v_adb <= 0: outside ADB across edge DB
			// c_bcd <= 0: outside BDC across edge DB
			if (v_adb <= 0 && c_bcd <= 0) {
				// Reduce to Line DB
				simplex[0] = B;
				simplex[1] = D;
				count = 2;
				tics_vec3 DB = vec3_sub(B.m, D.m);
				d = vec3_cross(vec3_cross(DB, vec3_negate(D.m)), DB);
				break;
			}

			// Collision detected!
			return true;
		} break;
		} // switch

		if (boundary_contact) { break; }

		assert((d.x != 0.0f || d.y != 0.0f || d.z != 0.0f) &&
			   "Search direction became zero but boundary contact was not detected.");
	}

	pad_simplex_to_tetrahedron(as, ta, bs, tb, simplex, &count);
	return true;
}

static collision_result run_epa(const shape_data* as, tics_transform ta, const shape_data* bs,
								tics_transform tb, const mink_support simplex[4]) {
	collision_result result = {0};
	result.has_collision = true;

	// Ensure the input simplex is a valid, non-degenerate tetrahedron with correct winding.
	// This means vertex 3 must strictly lie in the negative half-space of face 0,1,2.
	assert(vec3_dot(vec3_cross(vec3_sub(simplex[1].m, simplex[0].m),
							   vec3_sub(simplex[2].m, simplex[0].m)),
					vec3_sub(simplex[3].m, simplex[0].m)) < 0.0f &&
		   "EPA input simplex is degenerate or has incorrect winding order.");

	// EPA (Expanding Polytope Algorithm): GJK Extension for collision information
	// We want to find the normal of the collision.
	//
	// normal of collision = b - a
	// if a and b are each the furthest points of the one shape into the other. This normal is the
	// normal of the face of the minkowski difference that is closest to the origin.
	//
	// Problem: The simplex we found in which the origin lies is a subspace of the minkowski
	// difference. It does not necessarily contain the required face.
	//
	// Solution: We are adding vertices to the simplex (making it a polytope) until we find the
	// shortest normal from a face that is on the original mesh

	// we find the face that is closest
	// then we try to expand the polytope in the direction of the faces normal
	// if we were able to expand - repeat
	// if not, we found the closest face

	// initialize the polytope with the data from the simplex
	mink_support* polytope_positions = NULL;
	arrput(polytope_positions, simplex[0]);
	arrput(polytope_positions, simplex[1]);
	arrput(polytope_positions, simplex[2]);
	arrput(polytope_positions, simplex[3]);

	// order the vertices of the triangles so that the normals are always pointing outwards
	uint32_t* polytope_indices = NULL;
	// 0,1,2 ; 0,3,1 ; 0,2,3 ; 1,3,2
	arrput(polytope_indices,0); arrput(polytope_indices,1); arrput(polytope_indices,2);
	arrput(polytope_indices,0); arrput(polytope_indices,3); arrput(polytope_indices,1);
	arrput(polytope_indices,0); arrput(polytope_indices,2); arrput(polytope_indices,3);
	arrput(polytope_indices,1); arrput(polytope_indices,3); arrput(polytope_indices,2);

	// calculate face normals (normal, distance) and find the face closest to the origin
	face_plane* polytope_normals = NULL;
	float closest_distance = FLT_MAX;
	size_t closest_index = 0;

	size_t num_faces = arrlen(polytope_indices) / 3;
	for (size_t k = 0; k < num_faces; k++) {
		tics_vec3 a = polytope_positions[polytope_indices[k * 3 + 0]].m;
		tics_vec3 b = polytope_positions[polytope_indices[k * 3 + 1]].m;
		tics_vec3 c = polytope_positions[polytope_indices[k * 3 + 2]].m;

		tics_vec3 normal = vec3_normalize(vec3_cross(vec3_sub(b, a), vec3_sub(c, a)));
		float distance = vec3_dot(normal, a); // works with any vertex of the plane

		face_plane plane = {normal, distance};
		arrput(polytope_normals, plane);

		if (distance < closest_distance) {
			closest_distance = distance;
			closest_index = k;
		}
	}

	while (true) {
		// search for a new support point in the direction of the normal of the closest face
		tics_vec3 d = polytope_normals[closest_index].normal;
		mink_support new_supp_p = support_point_on_minkowski_diff_mesh_mesh(as, ta, bs, tb, d);
		float support_distance = vec3_dot(d, new_supp_p.m);

		// check if the support point lies on the same plane as the closest face
		// if it does, the polytype cannot be further expanded
		if (fabsf(support_distance - closest_distance) <= 0.001f) {
			break; // cannot be expanded - found the closest face!
		}

		// expand the polytope by adding the support point
		// to make sure the polytope stays convex, we remove all faces that point
		// towards the support point and create new faces afterwards

		edge* unique_edges = NULL;

		size_t k = 0;
		while (k < arrlen(polytope_normals)) {
			// check if the support point is in front of the triangle
			tics_vec3 face_normal = polytope_normals[k].normal;
			tics_vec3 p_on_face = polytope_positions[polytope_indices[k * 3]].m;
			float dotp = vec3_dot(face_normal, vec3_sub(new_supp_p.m, p_on_face));

			if (dotp > 0) {
				// if it is, collect all unique edges
				add_if_unique_edge(&unique_edges, polytope_indices[k * 3 + 0],
								   polytope_indices[k * 3 + 1]);
				add_if_unique_edge(&unique_edges, polytope_indices[k * 3 + 1],
								   polytope_indices[k * 3 + 2]);
				add_if_unique_edge(&unique_edges, polytope_indices[k * 3 + 2],
								   polytope_indices[k * 3 + 0]);

				// Remove this face (indices and normal)
				arrdel(polytope_indices, k * 3); // arrdel removes 1 item
				arrdel(polytope_indices, k * 3);
				arrdel(polytope_indices, k * 3);
				arrdel(polytope_normals, k);
			}
			else {
				// Only move to the next index if we didn't remove the current one
				k++;
			}
		}

		// create new vertex and faces
		uint32_t new_vertex_index = (uint32_t)arrlen(polytope_positions);
		arrput(polytope_positions, new_supp_p);

		for (size_t k = 0; k < arrlen(unique_edges); k++) {
			uint32_t edge_index_a = unique_edges[k].a;
			uint32_t edge_index_b = unique_edges[k].b;

			arrput(polytope_indices, edge_index_a);
			arrput(polytope_indices, edge_index_b);
			arrput(polytope_indices, new_vertex_index);

			tics_vec3 a = polytope_positions[edge_index_a].m;
			tics_vec3 b = polytope_positions[edge_index_b].m;
			tics_vec3 c = polytope_positions[new_vertex_index].m;

			tics_vec3 normal = vec3_normalize(vec3_cross(vec3_sub(b, a), vec3_sub(c, a)));
			float distance = vec3_dot(normal, a);

			assert(distance > -0.001 && "Triangles have incorrect winding order");

			face_plane plane = {normal, distance};
			arrput(polytope_normals, plane);
		}

		arrfree(unique_edges);

		// (re)iterate over all faces and find the closest
		closest_distance = FLT_MAX;
		closest_index = 0;
		for (size_t k = 0; k < arrlen(polytope_normals); k++) {
			float distance = polytope_normals[k].distance;
			if (distance < closest_distance) {
				closest_distance = distance;
				closest_index = k;
			}
		}
	}

	tics_vec3 result_normal = polytope_normals[closest_index].normal;
	result.normal = vec3_negate(result_normal);
	result.depth = closest_distance;

	// Algorithm that finds the collision points on the original shapes a and b

	// get vertices of face the farthest from the origin in minkowski space
	mink_support a = polytope_positions[polytope_indices[closest_index * 3 + 0]];
	mink_support b = polytope_positions[polytope_indices[closest_index * 3 + 1]];
	mink_support c = polytope_positions[polytope_indices[closest_index * 3 + 2]];

	// first, we find the closest point to the origin of the face in minkowski space
	tics_vec3 p = vec3_mul_f(result_normal, polytope_normals[closest_index].distance);

	// now, we calculate the barycentric coordinates of this point on the minkowski space face the
	// areas of the triangles BCP,CAP,ABP are proportional to the barycentric coordinates u,v,w

	float bcp_area = vec3_length(vec3_cross(vec3_sub(p, b.m), vec3_sub(p, c.m)));
	float cap_area = vec3_length(vec3_cross(vec3_sub(p, c.m), vec3_sub(p, a.m)));
	float abp_area = vec3_length(vec3_cross(vec3_sub(p, a.m), vec3_sub(p, b.m)));

	float face_area = cap_area + abp_area + bcp_area;
	// barycentric coordinates
	float u = bcp_area / face_area; // a
	float v = cap_area / face_area; // b
	float w = abp_area / face_area; // c

	// reconstruct p to see if the barycentric coordinates are correct:
	// p_reconstructed = ( a.m * u + b.m * v + c.m * w );
	// reconstructed_distance = length(p_reconstructed - p);
	// TODO Fix: sometimes the values are off, because p does not lie on the plane abc
	// TODO Check if this is still the case

	// now, we reconstruct the collision points of the original shapes a and b
	tics_vec3 term_a = vec3_mul_f(a.a, u);
	tics_vec3 term_b = vec3_mul_f(b.a, v);
	tics_vec3 term_c = vec3_mul_f(c.a, w);

	result.point_a = vec3_add(vec3_add(term_a, term_b), term_c);
	result.point_b = vec3_add(result.point_a, vec3_mul_f(result.normal, result.depth));

	// cleanup
	arrfree(polytope_positions);
	arrfree(polytope_indices);
	arrfree(polytope_normals);

	return result;
}

static collision_result collision_test_convex_convex(const shape_data* as, tics_transform ta,
													 const shape_data* bs, tics_transform tb) {
	assert(as->type == SHAPE_CONVEX);
	assert(bs->type == SHAPE_CONVEX);

	collision_result result = {0};
	mink_support simplex[4] = {0};

	if (run_gjk(as, ta, bs, tb, simplex)) {
		// If a collision is found the EPA algorithm is used to get detailed collision information.
		result = run_epa(as, ta, bs, tb, simplex);
	}

	return result;
}

static collision_result collision_test_sphere_sphere(const shape_data* as, tics_transform ta,
													 const shape_data* bs, tics_transform tb) {
	assert(as->type == SHAPE_SPHERE);
	assert(bs->type == SHAPE_SPHERE);

	collision_result result = {0};

	// Calculate global center positions
	// Apply rotation to the local center offset, then add to body position
	tics_vec3 center_a =
		vec3_add(ta.position, quat_rotate_vec3(as->data.sphere.center, ta.rotation));
	tics_vec3 center_b =
		vec3_add(tb.position, quat_rotate_vec3(bs->data.sphere.center, tb.rotation));

	float radius_a = as->data.sphere.radius;
	float radius_b = bs->data.sphere.radius;
	float radius_sum = radius_a + radius_b;

	// Vector from A to B
	tics_vec3 delta = vec3_sub(center_b, center_a);
	float dist_sq = vec3_length_sq(delta);

	// Early exit: no collision if distance squared > radius sum squared
	if (dist_sq > radius_sum * radius_sum) { return result; }

	result.has_collision = true;
	float distance = sqrtf(dist_sq);

	// Handle degenerate case: spheres are at the exact same position
	if (distance < 0.0001f) {
		result.depth = radius_sum;
		result.normal = (tics_vec3){0.0f, 1.0f, 0.0f}; // Arbitrary normal (Up)
	}
	else {
		result.depth = radius_sum - distance;
		result.normal = vec3_mul_f(delta, -1.0f / distance); // Normalized center_b -> center_a
	}

	// Point on Surface A closest to B
	result.point_a = vec3_add(center_a, vec3_mul_f(result.normal, -radius_a));
	// Point on Surface B closest to A (center_b - normal * radius_b)
	result.point_b = vec3_add(center_b, vec3_mul_f(result.normal, radius_b));

	return result;
}

// Projects a point onto a line segment defined by endpoints a and b. Clamps the projection
// parameter t to [0.0, 1.0] to find the closest point strictly on the segment.
static inline tics_vec3 closest_point_on_segment_to_point(tics_vec3 p, tics_vec3 a, tics_vec3 b) {
	tics_vec3 ab = vec3_sub(b, a);
	float len_sq = vec3_length_sq(ab);

	// Handle degenerate segment where start and end points are identical
	if (len_sq <= 0.00001f) {
		return a;
	}

	float t = vec3_dot(vec3_sub(p, a), ab) / len_sq;
	t = fmaxf(0.0f, fminf(1.0f, t));

	return vec3_add(a, vec3_mul_f(ab, t));
}

// Calculates the closest points between two line segments (p1 to q1) and (p2 to q2).
// The closest point on the first segment is stored in c1, and the closest on the second in c2.
static inline void closest_points_between_segments(tics_vec3 p1, tics_vec3 q1, tics_vec3 p2,
												   tics_vec3 q2, tics_vec3* c1, tics_vec3* c2) {
	tics_vec3 d1 = vec3_sub(q1, p1);
	tics_vec3 d2 = vec3_sub(q2, p2);
	tics_vec3 r = vec3_sub(p1, p2);

	float a = vec3_length_sq(d1);
	float e = vec3_length_sq(d2);
	float f = vec3_dot(d2, r);

	// Check if both segments degenerate to single points
	if (a <= 0.00001f && e <= 0.00001f) {
		*c1 = p1;
		*c2 = p2;
		return;
	}

	float s = 0.0f;
	float t = 0.0f;

	if (a <= 0.00001f) {
		// First segment is a point, solve directly for t
		t = fmaxf(0.0f, fminf(1.0f, f / e));
	}
	else {
		float c = vec3_dot(d1, r);
		if (e <= 0.00001f) {
			// Second segment is a point, solve directly for s
			s = fmaxf(0.0f, fminf(1.0f, -c / a));
		}
		else {
			// Solve the general system for non-degenerate segments
			float b = vec3_dot(d1, d2);
			float denom = a * e - b * b;

			// Solve for unconstrained s if segments are not parallel
			if (denom != 0.0f) {
				s = fmaxf(0.0f, fminf(1.0f, (b * f - c * e) / denom));
			}

			// Solve for t based on chosen s
			t = (b * s + f) / e;

			// Handle boundary cases where t falls outside [0, 1] by clamping and re-solving s
			if (t < 0.0f) {
				t = 0.0f;
				s = fmaxf(0.0f, fminf(1.0f, -c / a));
			}
			else if (t > 1.0f) {
				t = 1.0f;
				s = fmaxf(0.0f, fminf(1.0f, (b - c) / a));
			}
		}
	}

	*c1 = vec3_add(p1, vec3_mul_f(d1, s));
	*c2 = vec3_add(p2, vec3_mul_f(d2, t));
}

static collision_result collision_test_sphere_capsule(const shape_data* as, tics_transform ta,
													  const shape_data* bs, tics_transform tb) {
	assert(as->type == SHAPE_SPHERE);
	assert(bs->type == SHAPE_CAPSULE);

	collision_result result = {0};

	// Calculate global sphere center
	tics_vec3 center_a =
		vec3_add(ta.position, quat_rotate_vec3(as->data.sphere.center, ta.rotation));

	// Calculate global capsule segment points
	tics_vec3 cap_p_a = vec3_add(tb.position, quat_rotate_vec3(bs->data.capsule.p_a, tb.rotation));
	tics_vec3 cap_p_b = vec3_add(tb.position, quat_rotate_vec3(bs->data.capsule.p_b, tb.rotation));

	// Find the point on the capsule's inner segment closest to the sphere's center
	tics_vec3 closest_on_cap = closest_point_on_segment_to_point(center_a, cap_p_a, cap_p_b);

	float radius_a = as->data.sphere.radius;
	float radius_b = bs->data.capsule.radius;
	float radius_sum = radius_a + radius_b;

	// By substituting the closest inner point for a second center, we essentially perform a
	// sphere-to-sphere collision test from here onwards
	tics_vec3 delta = vec3_sub(closest_on_cap, center_a);
	float dist_sq = vec3_length_sq(delta);

	if (dist_sq > radius_sum * radius_sum) { return result; }

	result.has_collision = true;
	float distance = sqrtf(dist_sq);

	if (distance < 0.0001f) {
		result.depth = radius_sum;
		result.normal = (tics_vec3){0.0f, 1.0f, 0.0f};
	}
	else {
		result.depth = radius_sum - distance;
		result.normal = vec3_mul_f(delta, -1.0f / distance);
	}

	result.point_a = vec3_add(center_a, vec3_mul_f(result.normal, -radius_a));
	result.point_b = vec3_add(closest_on_cap, vec3_mul_f(result.normal, radius_b));

	return result;
}

static collision_result collision_test_capsule_capsule(const shape_data* as, tics_transform ta,
													   const shape_data* bs, tics_transform tb) {
	assert(as->type == SHAPE_CAPSULE);
	assert(bs->type == SHAPE_CAPSULE);

	collision_result result = {0};

	// Calculate global segment points for both capsules
	tics_vec3 a_p_a = vec3_add(ta.position, quat_rotate_vec3(as->data.capsule.p_a, ta.rotation));
	tics_vec3 a_p_b = vec3_add(ta.position, quat_rotate_vec3(as->data.capsule.p_b, ta.rotation));

	tics_vec3 b_p_a = vec3_add(tb.position, quat_rotate_vec3(bs->data.capsule.p_a, tb.rotation));
	tics_vec3 b_p_b = vec3_add(tb.position, quat_rotate_vec3(bs->data.capsule.p_b, tb.rotation));

	tics_vec3 closest_a, closest_b;
	closest_points_between_segments(a_p_a, a_p_b, b_p_a, b_p_b, &closest_a, &closest_b);

	float radius_a = as->data.capsule.radius;
	float radius_b = bs->data.capsule.radius;
	float radius_sum = radius_a + radius_b;

	// Vector from the closest point on A to the closest point on B
	tics_vec3 delta = vec3_sub(closest_b, closest_a);
	float dist_sq = vec3_length_sq(delta);

	if (dist_sq > radius_sum * radius_sum) { return result; }

	result.has_collision = true;
	float distance = sqrtf(dist_sq);

	if (distance < 0.0001f) {
		result.depth = radius_sum;
		result.normal = (tics_vec3){0.0f, 1.0f, 0.0f};
	}
	else {
		result.depth = radius_sum - distance;
		result.normal = vec3_mul_f(delta, -1.0f / distance);
	}

	result.point_a = vec3_add(closest_a, vec3_mul_f(result.normal, -radius_a));
	result.point_b = vec3_add(closest_b, vec3_mul_f(result.normal, radius_b));

	return result;
}

static collision_result collision_test_sphere_box(const shape_data* as, tics_transform ta,
												  const shape_data* bs, tics_transform tb) {
	assert(as->type == SHAPE_SPHERE);
	assert(bs->type == SHAPE_BOX);
	collision_result result = {0};

	// global center of the sphere
	tics_vec3 center_a = vec3_add(ta.position, quat_rotate_vec3(as->data.sphere.center, ta.rotation));
	// Transform the sphere's center into the Box's local coordinate space.
	tics_vec3 local_center = world_to_local(tb, center_a);
	tics_vec3 ext = bs->data.box.half_extents;

	// Find the closest point on the AABB to the sphere center by clamping the coordinates.
	tics_vec3 clamped = {
		fmaxf(-ext.x, fminf(ext.x, local_center.x)),
		fmaxf(-ext.y, fminf(ext.y, local_center.y)),
		fmaxf(-ext.z, fminf(ext.z, local_center.z))
	};

	tics_vec3 delta = vec3_sub(local_center, clamped);
	float dist_sq = vec3_length_sq(delta);
	float radius = as->data.sphere.radius;

	tics_vec3 local_normal;
	float depth;

	// Deep Penetration Handling
	// If the sphere's center is inside the box (or exactly on its surface), the clamped point will
	// equal the center point. This results in a zero distance vector, which cannot be normalized.
	// To resolve this, we find the closest geometric face of the AABB and project the center point
	// on that face.
	if (dist_sq < 0.00001f) {
		float dist_x = ext.x - fabsf(local_center.x);
		float dist_y = ext.y - fabsf(local_center.y);
		float dist_z = ext.z - fabsf(local_center.z);

		if (dist_x <= dist_y && dist_x <= dist_z) {
			clamped.x = local_center.x > 0.0f ? ext.x : -ext.x;
			local_normal = (tics_vec3){local_center.x > 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f};
			depth = radius + dist_x;
		}
		else if (dist_y <= dist_x && dist_y <= dist_z) {
			clamped.y = local_center.y > 0.0f ? ext.y : -ext.y;
			local_normal = (tics_vec3){0.0f, local_center.y > 0.0f ? 1.0f : -1.0f, 0.0f};
			depth = radius + dist_y;
		}
		else {
			clamped.z = local_center.z > 0.0f ? ext.z : -ext.z;
			local_normal = (tics_vec3){0.0f, 0.0f, local_center.z > 0.0f ? 1.0f : -1.0f};
			depth = radius + dist_z;
		}
	}
	else {
		// Normal Case (spheres center is outside the box)
		// No collision if the closest point on the box is further away than the sphere's radius.
		if (dist_sq > radius * radius) { return result; }

		float dist = sqrtf(dist_sq);
		depth = radius - dist;
		// The normal points from the clamped point (Box) to the local center (Sphere).
		local_normal = vec3_mul_f(delta, 1.0f / dist);
	}

	result.has_collision = true;
	result.depth = depth;

	// Convert local calculations back into world space
	result.normal = quat_rotate_vec3(local_normal, tb.rotation);
	result.point_b = local_to_world(tb, clamped);

	// The deepest penetrating point on the sphere lies opposite to the collision normal
	result.point_a = vec3_add(center_a, vec3_mul_f(result.normal, -radius));

	return result;
}

// Extracts the specific line segment that forms the deepest edge of the box in a given direction.
static inline void get_box_support_edge(const shape_data* box, tics_transform t,
										tics_vec3 dir, tics_vec3* p1, tics_vec3* p2) {
	tics_vec3 local_dir = quat_rotate_vec3(dir, quat_inverse(t.rotation));
	tics_vec3 ext = box->data.box.half_extents;

	// Find the deepest vertex
	tics_vec3 local_support = {
		local_dir.x > 0.0f ? ext.x : -ext.x,
		local_dir.y > 0.0f ? ext.y : -ext.y,
		local_dir.z > 0.0f ? ext.z : -ext.z
	};

	// The edge is formed along the local axis that is most perpendicular to the search direction
	float abs_x = fabsf(local_dir.x);
	float abs_y = fabsf(local_dir.y);
	float abs_z = fabsf(local_dir.z);

	tics_vec3 edge_start = local_support;
	tics_vec3 edge_end = local_support;

	if (abs_x <= abs_y && abs_x <= abs_z) {
		edge_start.x = -ext.x;
		edge_end.x = ext.x;
	}
	else if (abs_y <= abs_x && abs_y <= abs_z) {
		edge_start.y = -ext.y;
		edge_end.y = ext.y;
	}
	else {
		edge_start.z = -ext.z;
		edge_end.z = ext.z;
	}

	*p1 = local_to_world(t, edge_start);
	*p2 = local_to_world(t, edge_end);
}

// Analytic Capsule-Box Collision Detection.
// Algorithm reference: Christer Ericson, "Real-Time Collision Detection", Section 5.5.7
//
// The test determines intersection, contact normal, penetration depth, and contact points
// between a Capsule (Shape A) and an OBB (Shape B).
//
// Strategy Overview:
//
// - Space Transformation:
//   Capsule segment endpoints are transformed into Box local space. In this space, the OBB
//   becomes an Axis-Aligned Bounding Box (AABB) centered at the origin.
//
// - Shallow Contact Query (Ericson 5.5.7):
//   The segment is intersected against the AABB expanded by the capsule radius. If the ray
//   hits the expanded box, the entry point classifies which Voronoi region of the box the
//   contact lies in (Face, Edge, or Vertex region).
//   For Face regions, endpoints are compared using signed axis coordinates to ensure the
//   deepest penetrating endpoint is selected when the capsule tilts relative to the face plane.
//   If the closest-point distance between the capsule segment and box surface is within the
//   capsule radius (and greater than zero), shallow contact data is returned directly.
//
// - Deep Penetration Fallback (SAT Query):
//   If the core segment penetrates inside the box volume (distance near zero), distance
//   queries cannot uniquely establish a contact normal. The solver falls back to the
//   Separating Axis Theorem (SAT) across 6 candidate axes (3 Box face normals and 3
//   Edge-Segment cross products).
//   The axis yielding minimum positive overlap defines the collision normal. A slight bias
//   favors Box face axes over Edge axes to prevent jitter during flat resting contact.
//   Feature-based contact points are extracted by projecting the deepest segment endpoint
//   (for Face contacts) or calculating edge segment crossings (for Edge contacts).
static inline bool test_capsule_box_axis(tics_vec3 axis, tics_vec3 ext, tics_vec3 A, tics_vec3 B,
										 float r, float* min_overlap, tics_vec3* best_axis,
										 int type, int* best_type) {
	float len_sq = vec3_length_sq(axis);
	if (len_sq < 0.00001f) return true; // Safely skip degenerate cross products

	float len = sqrtf(len_sq);
	tics_vec3 n = vec3_mul_f(axis, 1.0f / len);

	// Project Box extents
	float r_box = ext.x * fabsf(n.x) + ext.y * fabsf(n.y) + ext.z * fabsf(n.z);

	// Project Capsule segment
	float p_a = vec3_dot(A, n);
	float p_b = vec3_dot(B, n);
	float min_cap = fminf(p_a, p_b) - r;
	float max_cap = fmaxf(p_a, p_b) + r;

	// Early exit if separated on this axis
	if (min_cap > r_box || max_cap < -r_box) return false;

	// Calculate depths to push the capsule out of the box in either direction
	float d1 = r_box - min_cap;
	float d2 = max_cap + r_box;

	float overlap;
	tics_vec3 push_n;
	if (d1 < d2) {
		overlap = d1;
		push_n = n; // Normal inherently points from Box to Capsule
	} else {
		overlap = d2;
		push_n = vec3_negate(n); // Flipped, but still points from Box to Capsule
	}

	if (overlap < *min_overlap) {
		// Face bias to prevent jittering when resting flat
		if (*best_type == 0 && type == 1 && overlap > *min_overlap * 0.999f) return true;

		*min_overlap = overlap;
		*best_axis = push_n;
		*best_type = type;
	}
	return true;
}

static collision_result collision_test_capsule_box(const shape_data* as, tics_transform ta,
												   const shape_data* bs, tics_transform tb) {
	assert(as->type == SHAPE_CAPSULE);
	assert(bs->type == SHAPE_BOX);

	collision_result result = {0};

	// Transform Capsule segment into Box local space
	tics_vec3 world_a = local_to_world(ta, as->data.capsule.p_a);
	tics_vec3 world_b = local_to_world(ta, as->data.capsule.p_b);

	tics_vec3 A = world_to_local(tb, world_a);
	tics_vec3 B = world_to_local(tb, world_b);
	tics_vec3 D = vec3_sub(B, A);

	float r = as->data.capsule.radius;
	tics_vec3 ext = bs->data.box.half_extents;
	tics_vec3 exp_ext = {ext.x + r, ext.y + r, ext.z + r};

	// Test the core segment against the Box expanded by the capsule radius.
	float t_min = 0.0f;
	float t_max = 1.0f;
	float E_arr[3] = {exp_ext.x, exp_ext.y, exp_ext.z};
	float A_arr[3] = {A.x, A.y, A.z};
	float D_arr[3] = {D.x, D.y, D.z};

	for (int i = 0; i < 3; i++) {
		if (fabsf(D_arr[i]) < 0.00001f) {
			if (A_arr[i] < -E_arr[i] || A_arr[i] > E_arr[i]) return result;
		} else {
			float invD = 1.0f / D_arr[i];
			float t0 = (-E_arr[i] - A_arr[i]) * invD;
			float t1 = ( E_arr[i] - A_arr[i]) * invD;
			if (invD < 0.0f) { float tmp = t0; t0 = t1; t1 = tmp; }
			if (t0 > t_min) t_min = t0;
			if (t1 < t_max) t_max = t1;
			if (t_max < t_min) return result; // Separated
		}
	}

	// Shallow Test (Golden Section Search)
	// We must find the exact closest point between the capsule core segment and the unexpanded Box.
	// Since the distance between a line segment and an AABB is a strictly convex function, we can
	// use a Golden Section Search to robustly find the global minimum distance in exactly 32
	// iterations, without any branching on complex Voronoi regions.
	float t0 = fmaxf(0.0f, t_min);
	float t1 = fminf(1.0f, t_max);

	const float inv_phi = 0.6180339887f;
	const float inv_phi_2 = 0.3819660113f;

	float t_a = t0 + inv_phi_2 * (t1 - t0);
	float t_b = t0 + inv_phi * (t1 - t0);

	tics_vec3 pos_a = vec3_add(A, vec3_mul_f(D, t_a));
	tics_vec3 proj_a = {
		fmaxf(-ext.x, fminf(ext.x, pos_a.x)),
		fmaxf(-ext.y, fminf(ext.y, pos_a.y)),
		fmaxf(-ext.z, fminf(ext.z, pos_a.z))
	};
	float dist_sq_a = vec3_length_sq(vec3_sub(pos_a, proj_a));

	tics_vec3 pos_b = vec3_add(A, vec3_mul_f(D, t_b));
	tics_vec3 proj_b = {
		fmaxf(-ext.x, fminf(ext.x, pos_b.x)),
		fmaxf(-ext.y, fminf(ext.y, pos_b.y)),
		fmaxf(-ext.z, fminf(ext.z, pos_b.z))
	};
	float dist_sq_b = vec3_length_sq(vec3_sub(pos_b, proj_b));

	for (int i = 0; i < 32; i++) {
		if (dist_sq_a < dist_sq_b) {
			t1 = t_b;
			t_b = t_a;
			dist_sq_b = dist_sq_a;
			t_a = t0 + inv_phi_2 * (t1 - t0);
			pos_a = vec3_add(A, vec3_mul_f(D, t_a));
			proj_a = (tics_vec3){
				fmaxf(-ext.x, fminf(ext.x, pos_a.x)),
				fmaxf(-ext.y, fminf(ext.y, pos_a.y)),
				fmaxf(-ext.z, fminf(ext.z, pos_a.z))
			};
			dist_sq_a = vec3_length_sq(vec3_sub(pos_a, proj_a));
		} else {
			t0 = t_a;
			t_a = t_b;
			dist_sq_a = dist_sq_b;
			t_b = t0 + inv_phi * (t1 - t0);
			pos_b = vec3_add(A, vec3_mul_f(D, t_b));
			proj_b = (tics_vec3){
				fmaxf(-ext.x, fminf(ext.x, pos_b.x)),
				fmaxf(-ext.y, fminf(ext.y, pos_b.y)),
				fmaxf(-ext.z, fminf(ext.z, pos_b.z))
			};
			dist_sq_b = vec3_length_sq(vec3_sub(pos_b, proj_b));
		}
	}

	float best_t = (t0 + t1) * 0.5f;
	tics_vec3 p_seg = vec3_add(A, vec3_mul_f(D, best_t));
	tics_vec3 q_box = {
		fmaxf(-ext.x, fminf(ext.x, p_seg.x)),
		fmaxf(-ext.y, fminf(ext.y, p_seg.y)),
		fmaxf(-ext.z, fminf(ext.z, p_seg.z))
	};

	tics_vec3 delta = vec3_sub(p_seg, q_box);
	float dist_sq = vec3_length_sq(delta);

	// Reject false positives caused by the sharp corners of the expanded AABB
	if (dist_sq > r * r) return result;

	// If distance is near zero, it means the core segment has breached the interior of the Box.
	// We skip shallow resolution and fall back to SAT below.
	if (dist_sq > 0.00001f) {
		float dist = sqrtf(dist_sq);
		result.has_collision = true;
		result.depth = r - dist;

		tics_vec3 local_normal = vec3_mul_f(delta, 1.0f / dist);
		result.normal = quat_rotate_vec3(local_normal, tb.rotation);

		result.point_b = local_to_world(tb, q_box);
		result.point_a = vec3_sub(result.point_b, vec3_mul_f(result.normal, result.depth));
		return result;
	}

	// Deep Penetration Test (SAT Fallback)
	// We must use SAT across the 6 major axes to find the exact push-out vector and depth.
	float min_overlap = FLT_MAX;
	tics_vec3 best_axis = {0,0,0};
	int best_type = -1; // 0 for Face, 1 for Edge

	if (!test_capsule_box_axis((tics_vec3){1,0,0}, ext, A, B, r, &min_overlap, &best_axis, 0, &best_type)) return result;
	if (!test_capsule_box_axis((tics_vec3){0,1,0}, ext, A, B, r, &min_overlap, &best_axis, 0, &best_type)) return result;
	if (!test_capsule_box_axis((tics_vec3){0,0,1}, ext, A, B, r, &min_overlap, &best_axis, 0, &best_type)) return result;

	if (!test_capsule_box_axis((tics_vec3){0, -D.z, D.y}, ext, A, B, r, &min_overlap, &best_axis, 1, &best_type)) return result;
	if (!test_capsule_box_axis((tics_vec3){D.z, 0, -D.x}, ext, A, B, r, &min_overlap, &best_axis, 1, &best_type)) return result;
	if (!test_capsule_box_axis((tics_vec3){-D.y, D.x, 0}, ext, A, B, r, &min_overlap, &best_axis, 1, &best_type)) return result;

	result.has_collision = true;
	result.depth = min_overlap;
	result.normal = quat_rotate_vec3(best_axis, tb.rotation); // Already points from Box to Capsule

	// Deep Feature-Based Contact Generation
	// Since result.normal points outward from the box, the endpoint deeper inside the box
	// has the smaller dot product with result.normal (furthest in direction -normal).
	float dot_a = vec3_dot(world_a, result.normal);
	float dot_b = vec3_dot(world_b, result.normal);
	tics_vec3 deepest_core = (dot_a < dot_b) ? world_a : world_b;

	if (best_type == 0) {
		// Box Face axis won: offset deepest endpoint by radius along negative normal
		result.point_a = vec3_add(deepest_core, vec3_mul_f(result.normal, -r));
	}
	else {
		// Edge axis won: calculate exact crossing between colliding edges
		tics_vec3 box_p1, box_p2;
		get_box_support_edge(bs, tb, result.normal, &box_p1, &box_p2);

		tics_vec3 core_a;
		closest_points_between_segments(world_a, world_b, box_p1, box_p2,
						&core_a, &result.point_b);
		result.point_a = vec3_add(core_a, vec3_mul_f(result.normal, -r));
	}

	// Enforce the strict engine invariant: p_b = p_a + n * d
	result.point_b = vec3_add(result.point_a, vec3_mul_f(result.normal, result.depth));

	return result;
}

// --- Box-Box SAT Utilities ---

// A 3x3 matrix defined by its column vectors, which directly represent the local X, Y, and Z axes
// of a transformed object -> convenient for SAP
typedef struct { tics_vec3 x_axis; tics_vec3 y_axis; tics_vec3 z_axis; } sat_mat3;

static inline sat_mat3 quat_to_sat_mat3(tics_quat q) {
	float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
	float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
	float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;

	sat_mat3 res;
	res.x_axis.x = 1.0f - 2.0f * (yy + zz);
	res.x_axis.y = 2.0f * (xy + wz);
	res.x_axis.z = 2.0f * (xz - wy);

	res.y_axis.x = 2.0f * (xy - wz);
	res.y_axis.y = 1.0f - 2.0f * (xx + zz);
	res.y_axis.z = 2.0f * (yz + wx);

	res.z_axis.x = 2.0f * (xz + wy);
	res.z_axis.y = 2.0f * (yz - wx);
	res.z_axis.z = 1.0f - 2.0f * (xx + yy);

	return res;
}

// Returns a matrix where every element is the absolute value of the input matrix.
// Adds a tiny epsilon to prevent division-by-zero or zero-length vectors during edge-edge cross
// products when two boxes are perfectly axis-aligned.
static inline sat_mat3 sat_mat3_abs_eps(sat_mat3 m) {
	const float eps = 0.000001f;
	sat_mat3 res;
	res.x_axis = (tics_vec3){fabsf(m.x_axis.x)+eps, fabsf(m.x_axis.y)+eps, fabsf(m.x_axis.z)+eps};
	res.y_axis = (tics_vec3){fabsf(m.y_axis.x)+eps, fabsf(m.y_axis.y)+eps, fabsf(m.y_axis.z)+eps};
	res.z_axis = (tics_vec3){fabsf(m.z_axis.x)+eps, fabsf(m.z_axis.y)+eps, fabsf(m.z_axis.z)+eps};
	return res;
}

static inline tics_vec3 get_box_support_point(const shape_data* box, tics_transform t,
											  tics_vec3 dir) {
	tics_vec3 local_dir = quat_rotate_vec3(dir, quat_inverse(t.rotation));
	tics_vec3 ext = box->data.box.half_extents;
	// The deepest vertex is determined by the signs of the search direction
	tics_vec3 local_support = {
		local_dir.x > 0.0f ? ext.x : -ext.x,
		local_dir.y > 0.0f ? ext.y : -ext.y,
		local_dir.z > 0.0f ? ext.z : -ext.z
	};
	return vec3_add(t.position, quat_rotate_vec3(local_support, t.rotation));
}

static inline bool test_edge_axis(tics_vec3 axis, float r_a, float r_b, float abs_t,
								  float* min_overlap, tics_vec3* best_axis, int* best_type) {
	float len_sq = vec3_length_sq(axis);
	// If the squared length of the cross-product axis is small, it means the two edges are
	// parallel. Parallel edges generate a separating axis identical to one of the 6 face normals we
	// already checked, so we skip it.
	if (len_sq > 0.00001f) {
		float len = sqrtf(len_sq);
		float overlap = (r_a + r_b - abs_t) / len;

		if (overlap < 0.0f) return false;

		if (overlap < *min_overlap) {
			// We favor Face-axes (type 0 or 1) over Edge-Edge axes (type 2) to prevent
			// floating-point inaccuracies from choosing a edge normal during a perfectly flat
			// face-to-face collision. If an edge overlap is only marginally smaller than an
			// already found face overlap, we discard it and keep the stable face normal.
			if (*best_type < 2 && overlap > *min_overlap * 0.999f) {
				return true;
			}

			*min_overlap = overlap;
			*best_axis = vec3_mul_f(axis, 1.0f / len);
			*best_type = 2;
		}
	}
	return true;
}

// Implements the Separating Axis Theorem (SAT) for Oriented Bounding Boxes (OBB).
// To optimize the 15 required axis tests, the calculations are performed in the local coordinate
// space of Box A. This simplifies Box A into an AABB centered at the origin, aligning its face
// normals with the X, Y, and Z axes. Box B's relative orientation is extracted as a 3x3 matrix
// where the columns directly represent its local axes. We test 3 face axes from A, 3 from B, and 9
// edge-edge cross products, keeping the axis with the minimum penetration overlap. If all 15 axes
// overlap, a collision has occurred.
static collision_result collision_test_box_box(const shape_data* as, tics_transform ta,
											   const shape_data* bs, tics_transform tb) {
	assert(as->type == SHAPE_BOX);
	assert(bs->type == SHAPE_BOX);
	collision_result result = {0};

	tics_vec3 T = world_to_local(ta, tb.position); // B's position in A's local space
	tics_quat rel_q = quat_mul(quat_inverse(ta.rotation), tb.rotation);
	sat_mat3 R = quat_to_sat_mat3(rel_q);
	sat_mat3 AbsR = sat_mat3_abs_eps(R);

	tics_vec3 eA = as->data.box.half_extents;
	tics_vec3 eB = bs->data.box.half_extents;

	float min_overlap = FLT_MAX;
	tics_vec3 best_axis = {0, 0, 0};
	int best_type = -1; // 0 for Face A, 1 for Face B, 2 for Edge
	float ra, rb, t, overlap;

	// --- 3 Face Axes of Box A ---
	// Axis X
	ra = eA.x;
	rb = eB.x * AbsR.x_axis.x + eB.y * AbsR.y_axis.x + eB.z * AbsR.z_axis.x;
	overlap = ra + rb - fabsf(T.x);
	if (overlap < 0.0f) return result;
	min_overlap = overlap; best_axis = (tics_vec3){1, 0, 0}; best_type = 0;
	// Axis Y
	ra = eA.y;
	rb = eB.x * AbsR.x_axis.y + eB.y * AbsR.y_axis.y + eB.z * AbsR.z_axis.y;
	overlap = ra + rb - fabsf(T.y);
	if (overlap < 0.0f) return result;
	if (overlap < min_overlap) { min_overlap=overlap; best_axis=(tics_vec3){0,1,0}; best_type=0; }
	// Axis Z
	ra = eA.z;
	rb = eB.x * AbsR.x_axis.z + eB.y * AbsR.y_axis.z + eB.z * AbsR.z_axis.z;
	overlap = ra + rb - fabsf(T.z);
	if (overlap < 0.0f) return result;
	if (overlap < min_overlap) { min_overlap=overlap; best_axis=(tics_vec3){0,0,1}; best_type=0; }

	// --- 3 Face Axes of Box B ---
	// Box B's axes in A's space are simply the columns of the rotation matrix R
	// Axis X
	ra = eA.x * AbsR.x_axis.x + eA.y * AbsR.x_axis.y + eA.z * AbsR.x_axis.z;
	rb = eB.x;
	overlap = ra + rb - fabsf(vec3_dot(T, R.x_axis));
	if (overlap < 0.0f) return result;
	if (overlap < min_overlap) { min_overlap=overlap; best_axis=R.x_axis; best_type=1; }
	// Axis Y
	ra = eA.x * AbsR.y_axis.x + eA.y * AbsR.y_axis.y + eA.z * AbsR.y_axis.z;
	rb = eB.y;
	overlap = ra + rb - fabsf(vec3_dot(T, R.y_axis));
	if (overlap < 0.0f) return result;
	if (overlap < min_overlap) { min_overlap=overlap; best_axis=R.y_axis; best_type=1; }
	// Axis Z
	ra = eA.x * AbsR.z_axis.x + eA.y * AbsR.z_axis.y + eA.z * AbsR.z_axis.z;
	rb = eB.z;
	overlap = ra + rb - fabsf(vec3_dot(T, R.z_axis));
	if (overlap < 0.0f) return result;
	if (overlap < min_overlap) { min_overlap=overlap; best_axis=R.z_axis; best_type=1; }

	// --- 9 Edge-Edge Axes ---
	// If the axis length is extremely small, the edges are parallel. Parallel edges generate a
	// separating axis identical to one of the 6 face normals we already checked, so we skip it.

	// Ax x Bx, By, Bz
	if (!test_edge_axis((tics_vec3){0.0f, -R.x_axis.z, R.x_axis.y},
		eA.y * AbsR.x_axis.z + eA.z * AbsR.x_axis.y,
		eB.y * AbsR.z_axis.x + eB.z * AbsR.y_axis.x, fabsf(T.y * R.x_axis.z - T.z * R.x_axis.y),
		&min_overlap, &best_axis, &best_type)) return result;
	if (!test_edge_axis((tics_vec3){0.0f, -R.y_axis.z, R.y_axis.y},
		eA.y * AbsR.y_axis.z + eA.z * AbsR.y_axis.y,
		eB.x * AbsR.z_axis.x + eB.z * AbsR.x_axis.x, fabsf(T.y * R.y_axis.z - T.z * R.y_axis.y),
		&min_overlap, &best_axis, &best_type)) return result;
	if (!test_edge_axis((tics_vec3){0.0f, -R.z_axis.z, R.z_axis.y},
		eA.y * AbsR.z_axis.z + eA.z * AbsR.z_axis.y,
		eB.x * AbsR.y_axis.x + eB.y * AbsR.x_axis.x, fabsf(T.y * R.z_axis.z - T.z * R.z_axis.y),
		&min_overlap, &best_axis, &best_type)) return result;

	// Ay x Bx, By, Bz
	if (!test_edge_axis((tics_vec3){R.x_axis.z, 0.0f, -R.x_axis.x},
		eA.x * AbsR.x_axis.z + eA.z * AbsR.x_axis.x,
		eB.y * AbsR.z_axis.y + eB.z * AbsR.y_axis.y, fabsf(T.z * R.x_axis.x - T.x * R.x_axis.z),
		&min_overlap, &best_axis, &best_type)) return result;
	if (!test_edge_axis((tics_vec3){R.y_axis.z, 0.0f, -R.y_axis.x},
		eA.x * AbsR.y_axis.z + eA.z * AbsR.y_axis.x,
		eB.x * AbsR.z_axis.y + eB.z * AbsR.x_axis.y, fabsf(T.z * R.y_axis.x - T.x * R.y_axis.z),
		&min_overlap, &best_axis, &best_type)) return result;
	if (!test_edge_axis((tics_vec3){R.z_axis.z, 0.0f, -R.z_axis.x},
		eA.x * AbsR.z_axis.z + eA.z * AbsR.z_axis.x,
		eB.x * AbsR.y_axis.y + eB.y * AbsR.x_axis.y, fabsf(T.z * R.z_axis.x - T.x * R.z_axis.z),
		&min_overlap, &best_axis, &best_type)) return result;

	// Az x Bx, By, Bz
	if (!test_edge_axis((tics_vec3){-R.x_axis.y, R.x_axis.x, 0.0f},
		eA.x * AbsR.x_axis.y + eA.y * AbsR.x_axis.x,
		eB.y * AbsR.z_axis.z + eB.z * AbsR.y_axis.z, fabsf(T.x * R.x_axis.y - T.y * R.x_axis.x),
		&min_overlap, &best_axis, &best_type)) return result;
	if (!test_edge_axis((tics_vec3){-R.y_axis.y, R.y_axis.x, 0.0f},
		eA.x * AbsR.y_axis.y + eA.y * AbsR.y_axis.x,
		eB.x * AbsR.z_axis.z + eB.z * AbsR.x_axis.z, fabsf(T.x * R.y_axis.y - T.y * R.y_axis.x),
		&min_overlap, &best_axis, &best_type)) return result;
	if (!test_edge_axis((tics_vec3){-R.z_axis.y, R.z_axis.x, 0.0f},
		eA.x * AbsR.z_axis.y + eA.y * AbsR.z_axis.x,
		eB.x * AbsR.y_axis.z + eB.y * AbsR.x_axis.z, fabsf(T.x * R.z_axis.y - T.y * R.z_axis.x),
		&min_overlap, &best_axis, &best_type)) return result;

	if (min_overlap <= 0.0f || min_overlap == FLT_MAX) { return result; }

	result.has_collision = true;
	result.depth = min_overlap;

	// Enforce convention: Normal must strictly point from Shape B to Shape A.
	// Since T points from A to B in local space, we want the normal to point opposite to T.
	if (vec3_dot(best_axis, T) > 0.0f) {
		best_axis = vec3_negate(best_axis);
	}

	result.normal = quat_rotate_vec3(best_axis, ta.rotation);

	// --- Feature-Based Contact Generation ---
	// Instead of blindly taking the deepest vertex of both boxes (which causes a tiny box hitting a
	// big box/wall to return a point at the far edge of the wall), we determine which features are
	// touching based on the winning SAT axis (best_type).

	if (best_type == 0) {
		// Face A (Box A is the Reference, Box B is the Incident)
		// We only extract the point from the Incident shape (the shape crashing into the wall).
		result.point_b = get_box_support_point(bs, tb, result.normal);
		result.point_a = vec3_sub(result.point_b, vec3_mul_f(result.normal, result.depth));
	}
	else if (best_type == 1) {
		// Face B (Box B is the Reference, Box A is the Incident)
		result.point_a = get_box_support_point(as, ta, vec3_negate(result.normal));
		result.point_b = vec3_add(result.point_a, vec3_mul_f(result.normal, result.depth));
	}
	else {
		// Edge-Edge (best_type == 2)
		// The collision point is somewhere in the middle of two edges. We extract the 3D line
		// segments forming the colliding edge on both boxes, then use our existing segment
		// intersection math to find the exact closest points on those lines.
		tics_vec3 a_p1, a_p2, b_p1, b_p2;

		get_box_support_edge(as, ta, vec3_negate(result.normal), &a_p1, &a_p2);
		get_box_support_edge(bs, tb, result.normal, &b_p1, &b_p2);

		closest_points_between_segments(a_p1, a_p2, b_p1, b_p2, &result.point_a, &result.point_b);
	}

	// Limitation Note: Because our pipeline only returns a single contact point, flat Face-Face
	// stacking might balance on a single incident vertex. Real resting stability for boxes requires
	// gathering a full manifold by clipping the incident face against the reference face.

	return result;
}

static collision_result collision_test_sphere_convex(const shape_data* as, tics_transform ta,
													 const shape_data* bs, tics_transform tb) {
	assert(as->type == SHAPE_SPHERE);
	assert(bs->type == SHAPE_CONVEX);
	collision_result result = {0};
	assert(false && "Not implemented");
	return result;
}

static collision_result collision_test_capsule_convex(const shape_data* as, tics_transform ta,
													  const shape_data* bs, tics_transform tb) {
	assert(as->type == SHAPE_CAPSULE);
	assert(bs->type == SHAPE_CONVEX);
	collision_result result = {0};
	assert(false && "Not implemented");
	return result;
}

static collision_result collision_test_box_convex(const shape_data* as, tics_transform ta,
												  const shape_data* bs, tics_transform tb) {
	assert(as->type == SHAPE_BOX);
	assert(bs->type == SHAPE_CONVEX);
	collision_result result = {0};
	assert(false && "Not implemented");
	return result;
}

// function type for a collision test function
typedef collision_result (*collision_test_func)(const shape_data*, tics_transform,
												const shape_data*, tics_transform);

collision_result collision_test(const shape_data* as, tics_transform at, const shape_data* bs,
								tics_transform bt) {
	// a collision table as described by valve in this pdf on page 33
	// https://media.steampowered.com/apps/valve/2015/DirkGregorius_Contacts.pdf
	// NULL means unreachable / invalid
	static const collision_test_func collision_table[4][4] = {
		//           Sphere                       Capsule                        Box                        Convex
		/*Sphere */ {collision_test_sphere_sphere,collision_test_sphere_capsule, collision_test_sphere_box, collision_test_sphere_convex },
		/*Capsule*/ {NULL,                        collision_test_capsule_capsule,collision_test_capsule_box,collision_test_capsule_convex},
		/*Box    */ {NULL,                        NULL,                          collision_test_box_box,    collision_test_box_convex    },
		/*Convex */ {NULL,                        NULL,                          NULL,                      collision_test_convex_convex },
	};

	// make sure the colliders are in the correct order
	// example: (convex, sphere) gets swapped to (sphere, convex)
	bool swap = as->type > bs->type;

	const shape_data* sorted_a = swap ? bs : as;
	const shape_data* sorted_b = swap ? as : bs;
	tics_transform sorted_at = swap ? bt : at;
	tics_transform sorted_bt = swap ? at : bt;

	// pick the function that matches the collider types from the table
	collision_test_func func = collision_table[sorted_a->type][sorted_b->type];
	assert(func != NULL && "Collider type combination not available");

	collision_result result = func(sorted_a, sorted_at, sorted_b, sorted_bt);

	// BLICK_CLEAR(0b1000000);
	// // draw shapes in different colors
	// BLICK_DRAW_SHAPE(6, *sorted_a, sorted_at, 0xFFFF5555, true);
	// BLICK_DRAW_SHAPE(6, *sorted_a, sorted_at, 0x22FF5555, false);
	// BLICK_DRAW_SHAPE(6, *sorted_b, sorted_bt, 0xFF55FF55, true);
	// BLICK_DRAW_SHAPE(6, *sorted_b, sorted_bt, 0x2255FF55, false);
	// // draw a red arrow between collision points (might be very small)
	// BLICK_ARROW(6, result.point_a, result.point_b, 0xFF0000FF);
	// // draw two yellow lines with a fixed length extending in both directions of the arrow
	// tics_vec3 target_a = vec3_add(result.point_a, vec3_mul_f(result.normal, -0.1f));
	// tics_vec3 target_b = vec3_add(result.point_b, vec3_mul_f(result.normal, 0.1f));
	// BLICK_LINE(6, result.point_a, target_a, 0xFF00FFFF);
	// BLICK_LINE(6, result.point_b, target_b, 0xFF00FFFF);
	// BLICK_REFRESH();

	// if we swapped the input colliders, we need to invert the collision data
	if (swap) {
		result.normal = vec3_negate(result.normal);
		tics_vec3 temp = result.point_a;
		result.point_a = result.point_b;
		result.point_b = temp;
	}

	if (result.has_collision) {
		// point_b must equal point_a + (normal * depth)
		tics_vec3 expected_pb = vec3_add(result.point_a, vec3_mul_f(result.normal, result.depth));
		tics_vec3 error_delta = vec3_sub(result.point_b, expected_pb);
		assert(vec3_length_sq(error_delta) < 0.001f &&
			   "Collision invariant failed: point_b != point_a + normal * depth");
	}
	return result;
};
