#include "tics.h"

#include <iostream>
#include <sstream>
#include <cassert>

static std::string vec_to_str(tics_vec3 v) {
	std::stringstream stream;
	stream << "{ " << v.x << ", " << v.y << ", " << v.z << " }";
	return stream.str();
}

bool tics::raycast(const MeshCollider &mesh_collider, const tics_vec3 p, const tics_vec3 v) {
	for (size_t triangle_index = 0; triangle_index < mesh_collider.indices.size() / 3; triangle_index++) {
		auto a = mesh_collider.positions.at(mesh_collider.indices.at(triangle_index * 3 + 0));
		auto b = mesh_collider.positions.at(mesh_collider.indices.at(triangle_index * 3 + 1));
		auto c = mesh_collider.positions.at(mesh_collider.indices.at(triangle_index * 3 + 2));

		// Let l be a line containing the point p and running parallel to the unit vector v

		// Translate the vertices so that p coincides with the origin
		a = tics_vec3_sub(a, p);
		b = tics_vec3_sub(b, p);
		c = tics_vec3_sub(c, p);

		// Calculate the scalar triple products (a x b) dot v, (b x c) dot v, (c x a) dot v
		const auto scalar_triple_product_ab = tics_vec3_dot( tics_vec3_cross(a, b), v );
		const auto scalar_triple_product_bc = tics_vec3_dot( tics_vec3_cross(b, c), v );
		const auto scalar_triple_product_ca = tics_vec3_dot( tics_vec3_cross(c, a), v );
		// If any of these products is positive, then the line does not intersect the triangle.
		const auto any_positive = (
			scalar_triple_product_ab > 0 ||
			scalar_triple_product_bc > 0 ||
			scalar_triple_product_ca > 0
		);
		if (!any_positive) {
			return true;
		}
	}

	return false;
}
