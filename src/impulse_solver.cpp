#include "tics_old.h"

#include <cassert>
#include <math.h>

using tics::ImpulseSolver;

static tics_vec3 get_velocity(tics::RigidBody *rb, const tics_vec3 &point) {
	const auto no_rotation = tics_vec3_length({rb->angular_velocity.x, rb->angular_velocity.y, rb->angular_velocity.z}) < 0.01f;
	auto axis = no_rotation ? tics_vec3{1,0,0} : tics_vec3_negate(tics_vec3_normalize({rb->angular_velocity.x, rb->angular_velocity.y, rb->angular_velocity.z}));
	auto half_angle = no_rotation ? 0.0f : acos(rb->angular_velocity.w);
	if (half_angle != half_angle) { // NaN check
		axis = {1,0,0};
		half_angle = 0.0;
	}
	const auto lin_vel = rb->velocity;
	const auto rotation_center = rb->get_transform().lock()->get_position();

	// move to local space of rigid body
	const auto local = tics_vec3_sub(point, rotation_center);
	// "move" the point according to the angular velocity
	const auto rotated = tics_quat_rotate_vec3(
		local, tics_quat_from_axis_angle(axis, half_angle * 2.0f)
	);
	const auto rotated_world_space = tics_vec3_add(rotated, rotation_center);
	const auto ws_premoved_point = tics_vec3_add(rotated_world_space, tics_vec3_mul_f(lin_vel, 0.1f));
	const auto total_v = tics_vec3_mul_f(tics_vec3_sub(ws_premoved_point, point), 10.0f);
	return total_v;
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

		const auto velocity_a = rb_a ? get_velocity(rb_a, collision.points.a) : tics_vec3{0.0f, 0.0f, 0.0f};
		const auto velocity_b = rb_b ? get_velocity(rb_b, collision.points.b) : tics_vec3{0.0f, 0.0f, 0.0f};

		const auto r_a = tics_vec3_sub(collision.points.a, sp_a->get_transform().lock()->get_position());
		const auto r_b = tics_vec3_sub(collision.points.b, sp_b->get_transform().lock()->get_position());

		auto r_a_dist_squared = tics_vec3_length(r_a);
		r_a_dist_squared *= r_a_dist_squared;
		auto r_b_dist_squared = tics_vec3_length(r_b);
		r_b_dist_squared *= r_b_dist_squared;

		const auto n = collision.points.normal;

		const auto v_r = tics_vec3_sub(velocity_a, velocity_b);
		// relative velocity in the collision normal direction
		const auto n_dot_vr = tics_vec3_dot(v_r, n);
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
		// denom calculation: inv_mass_a + inv_mass_b + dot(n, ...)
		tics_vec3 term1 = tics_vec3_cross(tics_vec3_cross(r_a, n), r_a);
		term1 = tics_vec3_mul_f(term1, inv_moment_of_inertia_a);
		tics_vec3 term2 = tics_vec3_cross(tics_vec3_cross(r_b, n), r_b);
		term2 = tics_vec3_mul_f(term2, inv_moment_of_inertia_b);
		const auto denom = inv_mass_a + inv_mass_b + tics_vec3_dot(n, tics_vec3_add(term1, term2));

		const auto impulse_magnitude = (-(1.0f + cor) * n_dot_vr) / denom;

		// add impulse-based friction
		const auto dynamic_friction_coefficient = 0.07f;
		// collision_tangent = Normalize( v_r - (Dot(v_r, n) * n) )
		const auto normal_comp = tics_vec3_mul_f(n, tics_vec3_dot(v_r, n));
		const auto collision_tangent = tics_vec3_normalize( tics_vec3_sub(v_r, normal_comp) );
		
		const auto friction_impulse = tics_vec3_mul_f(collision_tangent, impulse_magnitude * dynamic_friction_coefficient);

		// impulse = (magnitude * n) - friction
		const auto impulse = tics_vec3_sub( tics_vec3_mul_f(n, impulse_magnitude), friction_impulse );

		// apply impulses only to rigid bodies
		if (rb_a) {
			rb_a->impulse = tics_vec3_add(rb_a->impulse, impulse);

			const auto angular_impulse = tics_vec3_cross(r_a, impulse);
			if (angular_impulse.x != 0 || angular_impulse.y != 0 || angular_impulse.z != 0) {
				auto str = tics_vec3_length(angular_impulse) * 0.1f / r_a_dist_squared;
				const auto axis = tics_vec3_normalize(angular_impulse);
				rb_a->an_imp_div_sq_dst = tics_quat_from_axis_angle(tics_vec3_negate(axis), str);
			}
		}
		if (rb_b) {
			rb_b->impulse = tics_vec3_sub(rb_b->impulse, impulse);

			const auto angular_impulse = tics_vec3_cross(r_b, tics_vec3_negate(impulse));
			if (angular_impulse.x != 0 || angular_impulse.y != 0 || angular_impulse.z != 0) {
				auto str = tics_vec3_length(angular_impulse) * 0.1f / r_b_dist_squared;
				const auto axis = tics_vec3_normalize(angular_impulse);
				rb_b->an_imp_div_sq_dst = tics_quat_from_axis_angle(tics_vec3_negate(axis), str);
			}
		}
	}
}
