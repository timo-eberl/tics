#include "tics.h"

#include <cassert>
#include <iostream>
#include <limits>
#include <algorithm>

using tics::Transform;
using tics::CollisionPoints;
using tics::ColliderType;
using tics::Collider;
using tics::SphereCollider;
using tics::PlaneCollider;
using tics::MeshCollider;

struct SupportPoint {
	tics_vec3 m = {0,0,0}; // minkowski difference
	tics_vec3 a = {0,0,0}; // on shape a
	// tics_vec3 b = {0,0,0}; // on shape b
};

// A support function takes a direction d and returns a point on the boundary of a shape "furthest" in direction d
tics_vec3 support_point_mesh(
	const Collider &c, const Transform &t, const tics_vec3 &d
) {
	assert(c.type == ColliderType::MESH);

	const auto &collider = static_cast<const MeshCollider&>(c);

	const auto local_d = tics_quat_rotate_vec3(d, quat_inverse(t.get_rotation()));

	// find the support point in local space
	auto support_point_dot = -1.0f;
	auto support_point = tics_vec3{0,0,0};
	for (const auto &p : collider.positions) {
		const auto p_dot_d = tics_vec3_dot(p, local_d);
		if (p_dot_d > support_point_dot) {
			support_point_dot = p_dot_d;
			support_point = p;
		}
	}

	// this fails if the center position of a mesh is not inside the mesh
	assert(support_point_dot >= 0.0);

	support_point = tics_quat_rotate_vec3(support_point, t.get_rotation());
	support_point = tics_vec3_add(support_point, t.get_position());

	return support_point;
}

SupportPoint support_point_on_minkowski_diff_mesh_mesh(
	const Collider &ca, const Transform &ta,
	const Collider &cb, const Transform &tb,
	const tics_vec3 &d
) {
	assert(ca.type == ColliderType::MESH);
	assert(cb.type == ColliderType::MESH);

	auto point = SupportPoint();
	point.a = support_point_mesh(ca, ta, d);
	// point.b = support_point_mesh(cb, tb, - d);
	// point.m = point.a - point.b;
	tics_vec3 b_supp = support_point_mesh(cb, tb, tics_vec3_negate(d));
	point.m = tics_vec3_sub(point.a, b_supp);

	return point;
}

void add_if_unique_edge(
	std::vector<std::pair<uint32_t, uint32_t>>& edges,
	const std::vector<uint32_t>& faces, uint32_t edge_a, uint32_t edge_b
) {
	const auto reverse = std::find(
		edges.begin(), edges.end(),
		std::make_pair(edge_b, edge_a)
	);

	// edge was already present -> remove it
	if (reverse != edges.end()) {
		edges.erase(reverse);
	}
	else {
		edges.emplace_back(edge_a, edge_b);
	}
}

// Mesh vs Mesh collisions use the GJK and EPA Algorithm
CollisionPoints collision_test_mesh_mesh(
	const Collider& a, const Transform& ta,
	const Collider& b, const Transform& tb
) {
	assert(a.type == ColliderType::MESH);
	assert(b.type == ColliderType::MESH);

	auto& a_collider = static_cast<const MeshCollider&>(a);
	auto& b_collider = static_cast<const MeshCollider&>(b);

	// GJK Algorithm https://youtu.be/ajv46BSqcK4

	// the first direction is arbitrary. we choose the direction from the origin of one shape to the other
	auto d = tics_vec3_normalize( tics_vec3_sub(tb.get_position(), ta.get_position()) );

	SupportPoint simplex [4] = { SupportPoint(), SupportPoint(), SupportPoint(), SupportPoint() };
	// find the first support point on the minkowski difference in direction d
	simplex[0] = support_point_on_minkowski_diff_mesh_mesh(a_collider, ta, b_collider, tb, d);

	// the next direction is towards the origin
	d = tics_vec3_negate(simplex[0].m);

	// find the second support point
	simplex[1] = support_point_on_minkowski_diff_mesh_mesh(a_collider, ta, b_collider, tb, d);
	// if the next support point did not "pass" the origin, the shapes do not intersect
	if (tics_vec3_dot(simplex[1].m, d) < 0.001f) {
		return CollisionPoints();
	}

	// A = most recently added vertex, O = Origin
	const auto AB = tics_vec3_sub(simplex[0].m, simplex[1].m);
	const auto AO = tics_vec3_negate(simplex[1].m);

	// triple product: vector perpendicular to AB pointing toward the origin
	d = tics_vec3_cross ( tics_vec3_cross (AB, AO), AB );

	// find the third support point
	while (true) {
		simplex[2] = support_point_on_minkowski_diff_mesh_mesh(a_collider, ta, b_collider, tb, d);

		// if the new support point did not "pass" the origin, the shapes do not intersect
		if (tics_vec3_dot(simplex[2].m, d) < 0.001f) {
			return CollisionPoints();
		}

		// A = most recently added vertex, O = Origin
		const auto AB = tics_vec3_sub(simplex[1].m, simplex[2].m);
		const auto AC = tics_vec3_sub(simplex[0].m, simplex[2].m);
		const auto AO = tics_vec3_negate(simplex[2].m);

		// triple products to define regions R_AB and R_AC
		const auto ABC_normal = tics_vec3_cross(AB, AC);
		const auto AB_normal = tics_vec3_cross( tics_vec3_cross(AC, AB), AB );
		const auto AC_normal = tics_vec3_cross( ABC_normal, AC );

		// TODO: Add check if the origin lies on the line AB or AC

		if (tics_vec3_dot( AB_normal, AO ) > 0) {
			// We are in region AB
			// Remove current C, move the array so that the most recently added vertex is always at simplex[2]
			simplex[0] = simplex[1]; simplex[1] = simplex[2];
			d = AB_normal;
		}
		else if (tics_vec3_dot( AC_normal, AO ) > 0) {
			// We are in region AC
			// Remove current B, move the array so that the most recently added vertex is always at simplex[2]
			simplex[1] = simplex[2];
			d = AC_normal;
		}
		else {
			// We are in region ABC. Check if the origin is above or below ABC and move on.

			if (tics_vec3_dot( ABC_normal, AO ) > 0) {
				// above ABC
				d = ABC_normal;
			}
			else {
				// below ABC
				// swap current C and B (change winding order), so we are above ABC again
				const auto B = simplex[1]; simplex[1] = simplex[0]; simplex[0] = B;
				d = tics_vec3_negate(ABC_normal);
			}

			break;
		}
	}

	// find the fourth (last) support point
	// only iterate a limited number of times to work around being stuck in a loop
	for (size_t i = 0; i < 100; i++) {
	// while (true) {
		simplex[3] = support_point_on_minkowski_diff_mesh_mesh(a_collider, ta, b_collider, tb, d);

		// if the new support point did not "pass" the origin, the shapes do not intersect
		if (tics_vec3_dot(simplex[3].m, d) < 0.001f) {
			return CollisionPoints();
		}

		const auto A = simplex[3];
		const auto B = simplex[2];
		const auto C = simplex[1];
		const auto D = simplex[0];

		const auto AB = tics_vec3_sub(B.m, A.m);
		const auto AC = tics_vec3_sub(C.m, A.m);
		const auto AD = tics_vec3_sub(D.m, A.m);
		const auto AO = tics_vec3_negate(A.m);

		const auto ABC_normal = tics_vec3_normalize( tics_vec3_cross(AB, AC) );
		const auto ACD_normal = tics_vec3_normalize( tics_vec3_cross(AC, AD) );
		const auto ADB_normal = tics_vec3_normalize( tics_vec3_cross(AD, AB) );

		// Check in which region we are. Remove the vertex that is not part of that region
		if (tics_vec3_dot( ABC_normal, AO ) > 0.001f) {
			simplex[2] = A; simplex[1] = B; simplex[0] = C;
			d = ABC_normal;
		}
		else if (tics_vec3_dot( ACD_normal, AO ) > 0.001f) {
			simplex[2] = A; simplex[1] = C; simplex[0] = D;
			d = ACD_normal;
		}
		else if (tics_vec3_dot( ADB_normal, AO ) > 0.001f) {
			simplex[2] = A; simplex[1] = D; simplex[0] = B;
			d = ADB_normal;
		}
		else {
			// Collision detected!
			auto collision_points = CollisionPoints();
			collision_points.has_collision = true;

			// EPA (Expanding Polytope Algorithm): GJK Extension for collision information
			// We want to find the normal of the collision
			// normal of collision = (b - a if a a nd b are each the furthest points of the one shape into the other)
			// This normal is the normal of the face of the minkowski difference that is closest to the origin
			// Problem: The simplex we found in which the origin lies is a subspace of the minkowski difference.
			//          It does not necessarily contain the required face.
			// Solution: We are adding vertices to the simplex (making it a polytope) until we find the shortest normal
			//           from a face that is on the original mesh

			// we find the face that is closest
			// then we try to expand the polytope in the direction of the faces normal
			// if we were able to expand - repeat
			// if not, we found the closest face

			// initialize the polytope with the data from the simplex
			std::vector<SupportPoint> polytope_positions = {};
			polytope_positions.insert(polytope_positions.end(), { simplex[0], simplex[1], simplex[2], simplex[3] });
			// order the vertices of the triangles so that the normals are always pointing outwards
			std::vector<uint32_t> polytope_indices = {};
			polytope_indices.insert(polytope_indices.end(), {
				0, 1, 2,
				0, 3, 1,
				0, 2, 3,
				1, 3, 2
			});

			// calculate face normals vec4(vec3(normal), distance)
			// and find the face closest to the origin
			std::vector<tics_vec4> polytope_normals = {};
			auto closest_distance = std::numeric_limits<float>::max();
			size_t closest_index = 0;
			for (size_t i = 0; i < polytope_indices.size() / 3; i++) {
				const auto a = polytope_positions[polytope_indices[i * 3    ]].m;
				const auto b = polytope_positions[polytope_indices[i * 3 + 1]].m;
				const auto c = polytope_positions[polytope_indices[i * 3 + 2]].m;

				const auto normal = tics_vec3_normalize( tics_vec3_cross(tics_vec3_sub(b, a), tics_vec3_sub(c, a)) );
				const float distance = tics_vec3_dot(normal, a); // works with any vertex of the plane

				polytope_normals.push_back({normal.x, normal.y, normal.z, distance});

				if (distance < closest_distance) {
					closest_distance = distance;
					closest_index = i;
				}
			}

			while (true) {
				// search for a new support point in the direction of the normal of the closest face
				const auto tmp_d = polytope_normals[closest_index];
				d = {tmp_d.x, tmp_d.y, tmp_d.z};
				const auto new_supp_p = support_point_on_minkowski_diff_mesh_mesh(a_collider, ta, b_collider, tb, d);
				const auto support_distance = tics_vec3_dot(d, new_supp_p.m);

				// check if the support point lies on the same plane as the closest face
				// if it does, the polytype cannot be further expanded
				if (std::abs(support_distance - closest_distance) <= 0.001f) {
					break; // cannot be expanded - found the closest face!
				}

				// expand the polytope by adding the support point
				// to make sure the polytope stays convex, we remove all faces that point towards the support point
				// and create new faces afterwards

				std::vector<std::pair<uint32_t, uint32_t>> unique_edges;

				for (size_t i = 0; i < polytope_indices.size() / 3; i++) {
					// check if the support point is in front of the triangle
					tics_vec3 face_normal = {polytope_normals[i].x, polytope_normals[i].y, polytope_normals[i].z};
					const auto dotp = tics_vec3_dot(
						face_normal,
						tics_vec3_sub(new_supp_p.m, polytope_positions[polytope_indices[i * 3]].m)
					);
					if (dotp > 0) {
						// if it is, collect all unique edges
						add_if_unique_edge(
							unique_edges, polytope_indices, polytope_indices[i*3    ], polytope_indices[i*3 + 1]);
						add_if_unique_edge(
							unique_edges, polytope_indices, polytope_indices[i*3 + 1], polytope_indices[i*3 + 2]);
						add_if_unique_edge(
							unique_edges, polytope_indices, polytope_indices[i*3 + 2], polytope_indices[i*3    ]);

						polytope_indices.erase(polytope_indices.begin() + i*3, polytope_indices.begin() + i*3 + 3);
						polytope_normals.erase(polytope_normals.begin() + i);

						i--;
					}
				}

				// create new vertex and faces
				const auto new_vertex_index = polytope_positions.size();

				polytope_positions.push_back(new_supp_p);

				for (auto [edge_index_a, edge_index_b] : unique_edges) {
					polytope_indices.push_back(edge_index_a);
					polytope_indices.push_back(edge_index_b);
					polytope_indices.push_back(new_vertex_index);

					const auto a = polytope_positions[edge_index_a].m;
					const auto b = polytope_positions[edge_index_b].m;
					const auto c = polytope_positions[new_vertex_index].m;

					auto normal = tics_vec3_normalize( tics_vec3_cross(tics_vec3_sub(b, a), tics_vec3_sub(c, a)) );
					float distance = tics_vec3_dot(normal, a);

					if (distance < 0) {
						normal = tics_vec3_negate(normal);
						distance = -distance;
					}

					polytope_normals.push_back({normal.x, normal.y, normal.z, distance});
				}

				// (re)iterate over all faces and find the closest
				closest_distance = std::numeric_limits<float>::max();
				closest_index = 0;
				for (size_t i = 0; i < polytope_indices.size() / 3; i++) {
					const float distance = polytope_normals[i].w;
					if (distance < closest_distance) {
						closest_distance = distance;
						closest_index = i;
					}
				}
			}

			tics_vec3 result_normal = {polytope_normals[closest_index].x, polytope_normals[closest_index].y, polytope_normals[closest_index].z};
			collision_points.normal = tics_vec3_negate(result_normal);
			collision_points.depth = closest_distance;

			// Algorithm that finds the collision points on the original shapes a and b

			// get vertices of face the farthest from the origin in minkowski space
			const auto a = polytope_positions[polytope_indices[closest_index*3    ]];
			const auto b = polytope_positions[polytope_indices[closest_index*3 + 1]];
			const auto c = polytope_positions[polytope_indices[closest_index*3 + 2]];
			// first, we find the closest point to the origin of the face in minkowski space
			const auto p = tics_vec3_mul_f(result_normal, polytope_normals[closest_index].w);
			// now, we calculate the barycentric coordinates of this point on the minkowski space face
			// the areas of the triangles BCP,CAP,ABP are proportional to the barycentric coordinates u,v,w

			const auto bcp_area = tics_vec3_length( tics_vec3_cross(tics_vec3_sub(p, b.m), tics_vec3_sub(p, c.m)) );
			const auto cap_area = tics_vec3_length( tics_vec3_cross(tics_vec3_sub(p, c.m), tics_vec3_sub(p, a.m)) );
			const auto abp_area = tics_vec3_length( tics_vec3_cross(tics_vec3_sub(p, a.m), tics_vec3_sub(p, b.m)) );

			const auto face_area = cap_area + abp_area + bcp_area;
			// barycentric coordinates
			const auto u = bcp_area / face_area; // a
			const auto v = cap_area / face_area; // b
			const auto w = abp_area / face_area; // c
			// reconstruct p to see if the barycentric coordinates are correct
			// const auto p_reconstructed = ( a.m * u + b.m * v + c.m * w );
			// sometimes the values are off, because p does not lie on the plane abc which is the fault of EPA
			// const auto reconstructed_distance = Terathon::Magnitude(p_reconstructed - p);
			// std::cout << "reconstructed_distance: " << reconstructed_distance << "\n";
			// if (reconstructed_distance > 2.00) {
			// 	std::cout << "alarm\n";
			// }
			// now, we reconstruct the collision points of the original shapes a and b
			tics_vec3 term_a = tics_vec3_mul_f(a.a, u);
			tics_vec3 term_b = tics_vec3_mul_f(b.a, v);
			tics_vec3 term_c = tics_vec3_mul_f(c.a, w);

			collision_points.a = tics_vec3_add(tics_vec3_add(term_a, term_b), term_c);
			collision_points.b = tics_vec3_add(collision_points.a, tics_vec3_mul_f(collision_points.normal, collision_points.depth));

			return collision_points;
		}
	}
	// std::cout << "Terminating GJK (stuck in a loop)\n";
	return CollisionPoints(); // workaround
}

// define the function type for a collision test function
using CollisionTestFunc = CollisionPoints(*)(
	const Collider&, const Transform&,
	const Collider&, const Transform&
);

CollisionPoints tics::collision_test(
	const Collider& a, const Transform& at,
	const Collider& b, const Transform& bt
) {
	// a collision table as described by valve in this pdf on page 33
	// https://media.steampowered.com/apps/valve/2015/DirkGregorius_Contacts.pdf
	static const CollisionTestFunc function_table[3][3] = {
		  // Sphere        Plane           // Mesh
		{ nullptr,         nullptr,        nullptr                    },  // Sphere
		{ nullptr,         nullptr       , nullptr                    },  // Plane
		{ nullptr,         nullptr       , collision_test_mesh_mesh   },  // Mesh
	};

	// make sure the colliders are in the correct order
	// example: (mesh, sphere) gets swapped to (sphere, mesh)
	bool swap = a.type > b.type;

	auto& sorted_a = swap ? b : a;
	auto& sorted_b = swap ? a : b;
	auto& sorted_at = swap ? bt : at;
	auto& sorted_bt = swap ? at : bt;

	// pick the function that matches the collider types from the table
	auto collision_test_function = function_table[sorted_a.type][sorted_b.type];
	// check if collision test function is defined for the given colliders
	assert(collision_test_function != nullptr);

	CollisionPoints points = collision_test_function(sorted_a, sorted_at, sorted_b, sorted_bt);
	// if we swapped the input colliders, we need to invert the collision data
	if (swap) {
		points.normal = tics_vec3_negate(points.normal);
	}

	return points;
};
