#include "tics.h"

#include <iostream>
#include <sstream>
#include <cassert>

#include <TSRigid3D.h>
#include <TSVector4D.h>
#include <TSVector3D.h>
#include <TSBivector3D.h>
#include <TSMatrix3D.h>
#include <TSMatrix4D.h>
#include <TSMotor3D.h>

static std::string vec_to_str(Terathon::Vector3D v) {
	std::stringstream stream;
	stream << "{ " << v.x << ", " << v.y << ", " << v.z << " }";
	return stream.str();
}

bool tics::pga_raycast(const MeshCollider &mesh_collider, const Terathon::Point3D p, const Terathon::Vector3D v) {
	// the line l can be calculated once and used for all triangles
	const auto pre_l = Terathon::Wedge(p, p + v);

	for (size_t triangle_index = 0; triangle_index < mesh_collider.indices.size() / 3; triangle_index++) {
		// vertex positions of current triangle
		auto a = Terathon::Point3D(mesh_collider.positions.at(mesh_collider.indices.at(triangle_index * 3 + 0)));
		auto b = Terathon::Point3D(mesh_collider.positions.at(mesh_collider.indices.at(triangle_index * 3 + 1)));
		auto c = Terathon::Point3D(mesh_collider.positions.at(mesh_collider.indices.at(triangle_index * 3 + 2)));
		// precalculated edge
		const auto e_bc = mesh_collider.edges.at(triangle_index);

		auto l = pre_l;
		// Translate the line by subtracting (l.v cross a) from its moment l.m
		// NOTE: The PGA Illuminated book uses a cross l.v but that gives incorrect results
		l.m -= Terathon::Wedge(l.v, a);

		// Translate vertices so a is the origin
		b -= a;
		c -= a;
		// a = Terathon::Point3D::zero;

		// Lines representing the edges of the triangle
		const auto e_ab = Terathon::Line3D(b, Terathon::Bivector3D::zero);
		// const auto e_bc = Terathon::Wedge(b, c);
		const auto e_ca = Terathon::Line3D(-c, Terathon::Bivector3D::zero);

		// If any of the Antiwedge products is negative, then the line does not intersect the triangle.
		const auto any_negative = (
			Terathon::Antiwedge(l, e_ab) < 0 ||
			Terathon::Antiwedge(l, e_bc) < 0 ||
			Terathon::Antiwedge(l, e_ca) < 0
		);
		if (!any_negative) {
			return true;
		}
	}

	return false;
}

bool tics::raycast(const MeshCollider &mesh_collider, const Terathon::Vector3D p, const Terathon::Vector3D v) {
	for (size_t triangle_index = 0; triangle_index < mesh_collider.indices.size() / 3; triangle_index++) {
		auto a = Terathon::Point3D(mesh_collider.positions.at(mesh_collider.indices.at(triangle_index * 3 + 0)));
		auto b = Terathon::Point3D(mesh_collider.positions.at(mesh_collider.indices.at(triangle_index * 3 + 1)));
		auto c = Terathon::Point3D(mesh_collider.positions.at(mesh_collider.indices.at(triangle_index * 3 + 2)));

		// Let l be a line containing the point p and running parallel to the unit vector v

		// Translate the vertices so that p coincides with the origin
		a -= p;
		b -= p;
		c -= p;

		// Calculate the scalar triple products (a x b) dot v, (b x c) dot v, (c x a) dot v
		const auto scalar_triple_product_ab = Terathon::Dot( Terathon::Cross(a, b), v );
		const auto scalar_triple_product_bc = Terathon::Dot( Terathon::Cross(b, c), v );
		const auto scalar_triple_product_ca = Terathon::Dot( Terathon::Cross(c, a), v );
		// If any of these products is positive, then the line does not intersect the triangle.
		const auto any_positive = (
			scalar_triple_product_ab > 0 ||
			scalar_triple_product_bc > 0 ||
			scalar_triple_product_ca > 0
		);
		if (!any_positive) {
			return true;
		}

		// // implicit plane of the current triangle
		// Terathon::Vector4D plane;
		// plane.xyz = Terathon::Cross( (b-a), (c-a) );
		// plane.w = Terathon::Dot( -plane.xyz, a - p );

		// // find the point of intersection q
		// Terathon::Vector3D q =
		// 	p +
		// 	( -plane.w / Terathon::Dot(plane.xyz, v) ) * v;

		// a -= q;
		// b -= q;
		// c -= q;
		// q = Terathon::Vector3D(0.0f, 0.0f, 0.0f);

		// // edge 0
		// const Terathon::Vector3D edge0 = b - a;
		// const Terathon::Vector3D aq = q - a;
		// const auto scalar_triple_product_edge0_aq = Terathon::Dot( plane.xyz * plane.w, Terathon::Cross(edge0, aq) );

		// // edge 1
		// const Terathon::Vector3D edge1 = c - b;
		// const Terathon::Vector3D bq = q - b;
		// const auto scalar_triple_product_edge1_bq = Terathon::Dot( plane.xyz * plane.w, Terathon::Cross(edge1, bq) );

		// // edge 2
		// const Terathon::Vector3D edge2 = a - c;
		// const Terathon::Vector3D cq = q - c;
		// const auto scalar_triple_product_edge2_cq = Terathon::Dot( plane.xyz * plane.w, Terathon::Cross(edge2, cq) );

		// std::cout << "scalar_triple_product_edge0_aq: " << scalar_triple_product_edge0_aq << std::endl;
		// std::cout << "scalar_triple_product_edge1_bq: " << scalar_triple_product_edge1_bq << std::endl;
		// std::cout << "scalar_triple_product_edge2_cq: " << scalar_triple_product_edge2_cq << std::endl;

		// return !(
		// 	scalar_triple_product_edge0_aq < 0 ||
		// 	scalar_triple_product_edge1_bq < 0 ||
		// 	scalar_triple_product_edge2_cq < 0
		// );
	}

	return false;
}
