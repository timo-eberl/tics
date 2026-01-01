#include "tics_internal.h"

#include <stb_ds.h>

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

typedef struct {
	tics_vec3 m; // minkowski difference
	tics_vec3 a; // point on shape a
				 // point on shape b: calculated as a - m
} support_point;

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
	assert(c->type == TICS_SHAPE_CONVEX);

	tics_vec3 local_d = tics_quat_rotate_vec3(d, quat_inverse(t.rotation));

	const tics_vec3* vertices = c->data.convex.vertices;
	size_t count = c->data.convex.count;

	// find the support point in local space
	float support_point_dot = -FLT_MAX;
	tics_vec3 support_point = {0, 0, 0};

	for (size_t i = 0; i < count; ++i) {
		float p_dot_d = tics_vec3_dot(vertices[i], local_d);
		if (p_dot_d > support_point_dot) {
			support_point_dot = p_dot_d;
			support_point = vertices[i];
		}
	}

	// this fails if the center position of a mesh is not inside the mesh
	assert(support_point_dot >= 0.0);

	support_point = tics_quat_rotate_vec3(support_point, t.rotation);
	support_point = tics_vec3_add(support_point, t.position);

	return support_point;
}

static support_point support_point_on_minkowski_diff_mesh_mesh(const shape_data* ca,
															   tics_transform ta,
															   const shape_data* cb,
															   tics_transform tb, tics_vec3 d) {
	assert(ca->type == TICS_SHAPE_CONVEX);
	assert(cb->type == TICS_SHAPE_CONVEX);

	support_point point;
	point.a = support_point_mesh(ca, ta, d);
	// point.b = support_point_mesh(cb, tb, - d);
	// point.m = point.a - point.b;
	tics_vec3 b_supp = support_point_mesh(cb, tb, tics_vec3_negate(d));
	point.m = tics_vec3_sub(point.a, b_supp);

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

// Mesh vs Mesh collisions use the GJK and EPA Algorithm
static collision_result collision_test_convex_convex(const shape_data* as, tics_transform ta,
													 const shape_data* bs, tics_transform tb) {
	assert(as->type == TICS_SHAPE_CONVEX);
	assert(bs->type == TICS_SHAPE_CONVEX);

	collision_result result = {{0}, {0}, {0}, 0, false};

	// GJK Algorithm https://youtu.be/ajv46BSqcK4

	// the first direction is arbitrary. we choose the direction from the origin of one shape to the
	// other
	tics_vec3 d = tics_vec3_normalize(tics_vec3_sub(tb.position, ta.position));
	if (tics_vec3_length_sq(d) < 0.00001f) d = (tics_vec3){1, 0, 0};

	support_point simplex[4] = {0};
	// find the first support point on the minkowski difference in direction d
	simplex[0] = support_point_on_minkowski_diff_mesh_mesh(as, ta, bs, tb, d);

	// the next direction is towards the origin
	d = tics_vec3_negate(simplex[0].m);

	// find the second support point
	simplex[1] = support_point_on_minkowski_diff_mesh_mesh(as, ta, bs, tb, d);
	// if the next support point did not "pass" the origin, the shapes do not intersect
	if (tics_vec3_dot(simplex[1].m, d) < 0.001f) { return result; }

	// A = most recently added vertex, O = Origin
	tics_vec3 AB = tics_vec3_sub(simplex[0].m, simplex[1].m);
	tics_vec3 AO = tics_vec3_negate(simplex[1].m);

	// TODO Fix: AB and AO get reused later in a different scope, is a bit confusing

	// triple product: vector perpendicular to AB pointing toward the origin
	d = tics_vec3_cross(tics_vec3_cross(AB, AO), AB);

	// find the third support point
	while (true) {
		simplex[2] = support_point_on_minkowski_diff_mesh_mesh(as, ta, bs, tb, d);

		// if the new support point did not "pass" the origin, the shapes do not intersect
		if (tics_vec3_dot(simplex[2].m, d) < 0.001f) { return result; }

		// A = most recently added vertex, O = Origin
		AB = tics_vec3_sub(simplex[1].m, simplex[2].m);
		tics_vec3 AC = tics_vec3_sub(simplex[0].m, simplex[2].m);
		AO = tics_vec3_negate(simplex[2].m);

		// triple products to define regions R_AB and R_AC
		tics_vec3 ABC_normal = tics_vec3_cross(AB, AC);
		tics_vec3 AB_normal = tics_vec3_cross(tics_vec3_cross(AC, AB), AB);
		tics_vec3 AC_normal = tics_vec3_cross(ABC_normal, AC);

		// TODO: Add check if the origin lies on the line AB or AC

		if (tics_vec3_dot(AB_normal, AO) > 0) {
			// We are in region AB
			// Remove current C, move the array so that the most recently added vertex is always at
			// simplex[2]
			simplex[0] = simplex[1];
			simplex[1] = simplex[2];
			d = AB_normal;
		}
		else if (tics_vec3_dot(AC_normal, AO) > 0) {
			// We are in region AC
			// Remove current B, move the array so that the most recently added vertex is always at
			// simplex[2]
			simplex[1] = simplex[2];
			d = AC_normal;
		}
		else {
			// We are in region ABC. Check if the origin is above or below ABC and move on.

			if (tics_vec3_dot(ABC_normal, AO) > 0) {
				// above ABC
				d = ABC_normal;
			}
			else {
				// below ABC
				// swap current C and B (change winding order), so we are above ABC again
				support_point B = simplex[1];
				simplex[1] = simplex[0];
				simplex[0] = B;
				d = tics_vec3_negate(ABC_normal);
			}

			break;
		}
	}

	// find the fourth (last) support point
	// only iterate a limited number of times to work around being stuck in a loop
	// This is a workaround, because that shouldn't happen
	// TODO make it not happen (lol) and switch to while(true)
	for (size_t i = 0; i < 100; i++) {
		simplex[3] = support_point_on_minkowski_diff_mesh_mesh(as, ta, bs, tb, d);

		// if the new support point did not "pass" the origin, the shapes do not intersect
		if (tics_vec3_dot(simplex[3].m, d) < 0.001f) { return result; }

		support_point A = simplex[3];
		support_point B = simplex[2];
		support_point C = simplex[1];
		support_point D = simplex[0];

		AB = tics_vec3_sub(B.m, A.m);
		tics_vec3 AC = tics_vec3_sub(C.m, A.m);
		tics_vec3 AD = tics_vec3_sub(D.m, A.m);
		AO = tics_vec3_negate(A.m);

		tics_vec3 ABC_normal = tics_vec3_normalize(tics_vec3_cross(AB, AC));
		tics_vec3 ACD_normal = tics_vec3_normalize(tics_vec3_cross(AC, AD));
		tics_vec3 ADB_normal = tics_vec3_normalize(tics_vec3_cross(AD, AB));

		// Check in which region we are. Remove the vertex that is not part of that region
		if (tics_vec3_dot(ABC_normal, AO) > 0.001f) {
			simplex[2] = A;
			simplex[1] = B;
			simplex[0] = C;
			d = ABC_normal;
		}
		else if (tics_vec3_dot(ACD_normal, AO) > 0.001f) {
			simplex[2] = A;
			simplex[1] = C;
			simplex[0] = D;
			d = ACD_normal;
		}
		else if (tics_vec3_dot(ADB_normal, AO) > 0.001f) {
			simplex[2] = A;
			simplex[1] = D;
			simplex[0] = B;
			d = ADB_normal;
		}
		else {
			// Collision detected!
			result.has_collision = true;

			// EPA (Expanding Polytope Algorithm): GJK Extension for collision information
			// We want to find the normal of the collision.
			//
			// normal of collision = b - a
			// if a and b are each the furthest points of the one shape into the other. This normal
			// is the normal of the face of the minkowski difference that is closest to the origin.
			//
			// Problem: The simplex we found in which the origin lies is a subspace of the minkowski
			// difference. It does not necessarily contain the required face.
			//
			// Solution: We are adding vertices to the simplex (making it a polytope) until we find
			// the shortest normal from a face that is on the original mesh

			// we find the face that is closest
			// then we try to expand the polytope in the direction of the faces normal
			// if we were able to expand - repeat
			// if not, we found the closest face

			// initialize the polytope with the data from the simplex
			support_point* polytope_positions = NULL;
			arrput(polytope_positions, simplex[0]);
			arrput(polytope_positions, simplex[1]);
			arrput(polytope_positions, simplex[2]);
			arrput(polytope_positions, simplex[3]);

			// order the vertices of the triangles so that the normals are always pointing outwards
			uint32_t* polytope_indices = NULL;
			// clang-format off
			// 0, 1, 2
			arrput(polytope_indices, 0); arrput(polytope_indices, 1); arrput(polytope_indices, 2);
			// 0, 3, 1
			arrput(polytope_indices, 0); arrput(polytope_indices, 3); arrput(polytope_indices, 1);
			// 0, 2, 3
			arrput(polytope_indices, 0); arrput(polytope_indices, 2); arrput(polytope_indices, 3);
			// 1, 3, 2
			arrput(polytope_indices, 1); arrput(polytope_indices, 3); arrput(polytope_indices, 2);
			// clang-format on

			// calculate face normals (normal, distance)
			// and find the face closest to the origin
			face_plane* polytope_normals = NULL;
			float closest_distance = FLT_MAX;
			size_t closest_index = 0;

			size_t num_faces = arrlen(polytope_indices) / 3;
			for (size_t k = 0; k < num_faces; k++) {
				tics_vec3 a = polytope_positions[polytope_indices[k * 3 + 0]].m;
				tics_vec3 b = polytope_positions[polytope_indices[k * 3 + 1]].m;
				tics_vec3 c = polytope_positions[polytope_indices[k * 3 + 2]].m;

				tics_vec3 normal =
					tics_vec3_normalize(tics_vec3_cross(tics_vec3_sub(b, a), tics_vec3_sub(c, a)));
				float distance = tics_vec3_dot(normal, a); // works with any vertex of the plane

				face_plane plane = {normal, distance};
				arrput(polytope_normals, plane);

				if (distance < closest_distance) {
					closest_distance = distance;
					closest_index = k;
				}
			}

			while (true) {
				// search for a new support point in the direction of the normal of the closest face
				d = polytope_normals[closest_index].normal;
				support_point new_supp_p =
					support_point_on_minkowski_diff_mesh_mesh(as, ta, bs, tb, d);
				float support_distance = tics_vec3_dot(d, new_supp_p.m);

				// check if the support point lies on the same plane as the closest face
				// if it does, the polytype cannot be further expanded
				if (fabsf(support_distance - closest_distance) <= 0.001f) {
					break; // cannot be expanded - found the closest face!
				}

				// expand the polytope by adding the support point
				// to make sure the polytope stays convex, we remove all faces that point towards
				// the support point and create new faces afterwards

				edge* unique_edges = NULL;

				size_t k = 0;
				while (k < arrlen(polytope_normals)) {
					// check if the support point is in front of the triangle
					tics_vec3 face_normal = polytope_normals[k].normal;
					tics_vec3 p_on_face = polytope_positions[polytope_indices[k * 3]].m;
					float dotp = tics_vec3_dot(face_normal, tics_vec3_sub(new_supp_p.m, p_on_face));

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

					tics_vec3 normal = tics_vec3_normalize(
						tics_vec3_cross(tics_vec3_sub(b, a), tics_vec3_sub(c, a)));
					float distance = tics_vec3_dot(normal, a);

					if (distance < 0) {
						normal = tics_vec3_negate(normal);
						distance = -distance;
					}

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
			result.normal = tics_vec3_negate(result_normal);
			result.depth = closest_distance;

			// Algorithm that finds the collision points on the original shapes a and b

			// get vertices of face the farthest from the origin in minkowski space
			support_point a = polytope_positions[polytope_indices[closest_index * 3 + 0]];
			support_point b = polytope_positions[polytope_indices[closest_index * 3 + 1]];
			support_point c = polytope_positions[polytope_indices[closest_index * 3 + 2]];

			// first, we find the closest point to the origin of the face in minkowski space
			tics_vec3 p = tics_vec3_mul_f(result_normal, polytope_normals[closest_index].distance);

			// now, we calculate the barycentric coordinates of this point on the minkowski space
			// face
			// the areas of the triangles BCP,CAP,ABP are proportional to the barycentric
			// coordinates u,v,w

			float bcp_area =
				tics_vec3_length(tics_vec3_cross(tics_vec3_sub(p, b.m), tics_vec3_sub(p, c.m)));
			float cap_area =
				tics_vec3_length(tics_vec3_cross(tics_vec3_sub(p, c.m), tics_vec3_sub(p, a.m)));
			float abp_area =
				tics_vec3_length(tics_vec3_cross(tics_vec3_sub(p, a.m), tics_vec3_sub(p, b.m)));

			float face_area = cap_area + abp_area + bcp_area;
			// barycentric coordinates
			float u = bcp_area / face_area; // a
			float v = cap_area / face_area; // b
			float w = abp_area / face_area; // c

			// reconstruct p to see if the barycentric coordinates are correct:
			// p_reconstructed = ( a.m * u + b.m * v + c.m * w );
			// reconstructed_distance = length(p_reconstructed - p);
			// TODO Fix: sometimes the values are off, because p does not lie on the plane abc which
			// is the fault of EPA

			// now, we reconstruct the collision points of the original shapes a and b
			tics_vec3 term_a = tics_vec3_mul_f(a.a, u);
			tics_vec3 term_b = tics_vec3_mul_f(b.a, v);
			tics_vec3 term_c = tics_vec3_mul_f(c.a, w);

			result.point_a = tics_vec3_add(tics_vec3_add(term_a, term_b), term_c);
			result.point_b =
				tics_vec3_add(result.point_a, tics_vec3_mul_f(result.normal, result.depth));

			// cleanup
			arrfree(polytope_positions);
			arrfree(polytope_indices);
			arrfree(polytope_normals);

			return result;
		}
	}
	// We end up here if we were stuck in a loop, which shouldn't happen
	return result;
}

// function type for a collision test function
typedef collision_result (*collision_test_func)(const shape_data*, tics_transform,
												const shape_data*, tics_transform);

collision_result collision_test(const shape_data* as, tics_transform at, const shape_data* bs,
								tics_transform bt) {
	// a collision table as described by valve in this pdf on page 33
	// https://media.steampowered.com/apps/valve/2015/DirkGregorius_Contacts.pdf

#define XXX NULL // Unreachable/Invalid

	static const collision_test_func function_table[3][3] = {
		// clang-format off
		// Sphere         Plane             Convex
		{  NULL /*TODO*/, NULL /*TODO*/,    NULL /*TODO*/                },  // Sphere
		{  XXX,           NULL /*invalid*/, NULL /*TODO*/                },  // Plane
		{  XXX,           XXX,              collision_test_convex_convex },  // Convex
		// clang-format on
	};

	// make sure the colliders are in the correct order
	// example: (convex, sphere) gets swapped to (sphere, convex)
	bool swap = as->type > bs->type;

	const shape_data* sorted_a = swap ? bs : as;
	const shape_data* sorted_b = swap ? as : bs;
	tics_transform sorted_at = swap ? bt : at;
	tics_transform sorted_bt = swap ? at : bt;

	// pick the function that matches the collider types from the table
	collision_test_func func = function_table[sorted_a->type][sorted_b->type];
	// check if collision test function is defined for the given colliders
	assert(func != NULL);

	collision_result result = func(sorted_a, sorted_at, sorted_b, sorted_bt);

	// if we swapped the input colliders, we need to invert the collision data
	if (swap) {
		result.normal = tics_vec3_negate(result.normal);
		tics_vec3 temp = result.point_a;
		result.point_a = result.point_b;
		result.point_b = temp;
	}

	return result;
};
