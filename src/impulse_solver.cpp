#include "tics.h"

#include <cassert>
#include <math.h>

using tics::ImpulseSolver;

static Terathon::Vector3D get_velocity(tics::RigidBody *rb, const Terathon::Vector3D &point) {
	const auto no_rotation = Terathon::Magnitude(rb->angular_velocity.xyz) < 0.01;
	auto axis = no_rotation ? Terathon::Vector3D(1,0,0) : !Terathon::Normalize(rb->angular_velocity.xyz);
	auto half_angle = no_rotation ? 0.0f : acos(rb->angular_velocity.w);
	if (half_angle != half_angle) { // NaN, because Rotor represents neutral rotation
		axis = Terathon::Vector3D(1,0,0);
		half_angle = 0.0;
	}
	const auto lin_vel = rb->velocity;
	const auto rotation_center = rb->get_transform().lock()->get_position();

#ifdef TICS_GA
	// the (world sapce) line about which the rotation occurs
	const auto l = Terathon::Wedge(Terathon::Point3D(rotation_center), axis) * sin(half_angle);
	// construct motor from l
	const auto R = Terathon::Motor3D(l.v.x, l.v.y, l.v.z, cos(half_angle), l.m.x, l.m.y, l.m.z, 0.0);
	const auto T = Terathon::Motor3D::MakeTranslation(lin_vel * 0.1);
	const auto Q = T * R; // motor that represents the current angular + linear velocity of the body

	const auto premoved_point = Terathon::Transform(Terathon::Point3D(point), Q);
	const auto total_vel = (premoved_point - Terathon::Point3D(point)) * 10.0;
	return total_vel;
#else
	// move to local space of rigid body
	const auto local = point - rotation_center;
	// "move" the point according to the angular velocity
	const auto rotated = Terathon::Transform(
		local, Terathon::Quaternion::MakeRotation(half_angle * 2.0, !axis)
	);
	const auto rotated_world_space = rotated + rotation_center;
	const auto ws_premoved_point = rotated_world_space + lin_vel * 0.1;
	const auto total_v = (ws_premoved_point - point) * 10.0;
	return total_v;
#endif
}

void ImpulseSolver::solve(const std::vector<Collision>& collisions, float delta) {
	for (auto collision : collisions) {
		auto sp_a = collision.a.lock();
		auto sp_b = collision.b.lock();
		if (!sp_a || !sp_b) { continue; }

		const auto rb_a = dynamic_cast<RigidBody *>(sp_a.get());
		const auto rb_b = dynamic_cast<RigidBody *>(sp_b.get());
		const auto sb_a = dynamic_cast<StaticBody *>(sp_a.get());
		const auto sb_b = dynamic_cast<StaticBody *>(sp_b.get());

		// continue if the objects are no valid object combination
		if (!( (rb_a && rb_b) || (rb_a && sb_b) || (sb_a && rb_b) )) { continue; }

		// hacky fix for the case when a rigid body collides with 2 other bodies: limit of 1 collision response/body
		// very problematic, when a rigid body collides with two static bodies (e.g. intersecting static bodies)
		// the best solution: change order to solver_1 -> update velocity -> solver_2 -> update velocity
		// if (
		// 	   (rb_a && rb_a->impulse != Terathon::Vector3D(0,0,0))
		// 	|| (rb_b && rb_b->impulse != Terathon::Vector3D(0,0,0))
		// ) { continue; }

		const auto velocity_a = rb_a ? get_velocity(rb_a, collision.points.a) : Terathon::Vector3D(0.0, 0.0, 0.0);
		const auto velocity_b = rb_b ? get_velocity(rb_b, collision.points.b) : Terathon::Vector3D(0.0, 0.0, 0.0);

		const auto r_a = collision.points.a - sp_a->get_transform().lock()->get_position();
		const auto r_b = collision.points.b - sp_b->get_transform().lock()->get_position();

		auto r_a_dist_squared = Terathon::Magnitude(r_a);
		r_a_dist_squared *= r_a_dist_squared;
		auto r_b_dist_squared = Terathon::Magnitude(r_b);
		r_b_dist_squared *= r_b_dist_squared;

		const auto n = collision.points.normal;

		const auto v_r = velocity_a - velocity_b;
		// relative velocity in the collision normal direction
		const auto n_dot_vr = Terathon::Dot(v_r, n);
		// n_dot_v is > 0 if the bodies are moving away from each other
		if (n_dot_vr >= 0) {
			continue;
		}

		// coefficient of restitution is the ratio of the relative velocity of
		// separation after collision to the relative velocity of approach before collision.
		// it is a property of BOTH collision objects (their "bounciness").
		const auto cor = (rb_a ? rb_a->elasticity : sb_a->elasticity) * (rb_b ? rb_b->elasticity : sb_b->elasticity);

		const auto inv_mass_a = rb_a ? 1.0f/rb_a->mass : 0.0f;
		const auto inv_mass_b = rb_b ? 1.0f/rb_b->mass : 0.0f;

		const auto inv_moment_of_inertia_a = rb_a
			? 1.0f/(rb_a->mass * r_a_dist_squared)
			: 0.0f;
		const auto inv_moment_of_inertia_b = rb_b
			? 1.0f/(rb_b->mass * r_b_dist_squared)
			: 0.0f;

		// https://en.wikipedia.org/wiki/Collision_response
		const auto impulse_magnitude = (
			(-(1.0f + cor) * n_dot_vr)
			/ (
				inv_mass_a + inv_mass_b + Terathon::Dot(n,
					  inv_moment_of_inertia_a * (Terathon::Cross(Terathon::Cross(r_a, n), r_a))
					+ inv_moment_of_inertia_b * (Terathon::Cross(Terathon::Cross(r_b, n), r_b))
				)
			)
		);

		// add impulse-based friction
		const auto dynamic_friction_coefficient = 0.07;
		const auto collision_tangent = Terathon::Normalize( v_r - (Terathon::Dot(v_r, n) * n) );
		const auto friction_impulse = (
			(impulse_magnitude * dynamic_friction_coefficient) * collision_tangent
		);

		const auto impulse = (impulse_magnitude * n) - friction_impulse;

		// apply impulses only to rigid bodies
		if (rb_a) {
			rb_a->impulse += impulse;

			const auto angular_impulse = Terathon::Cross(r_a, impulse);
			if (angular_impulse != Terathon::Vector3D(0,0,0)) {
				auto str = Terathon::Magnitude(angular_impulse) * 0.1f / r_a_dist_squared;
				const auto axis = Terathon::Normalize(angular_impulse);
				rb_a->an_imp_div_sq_dst = Terathon::Quaternion::MakeRotation(str, !axis);
			}
		}
		if (rb_b) {
			rb_b->impulse -= impulse;

			const auto angular_impulse = Terathon::Cross(r_b, -impulse);
			if (angular_impulse != Terathon::Vector3D(0,0,0)) {
				auto str = Terathon::Magnitude(angular_impulse) * 0.1f / r_b_dist_squared;
				const auto axis = Terathon::Normalize(angular_impulse);
				rb_b->an_imp_div_sq_dst = Terathon::Quaternion::MakeRotation(str, !axis);
			}
		}
	}
}
