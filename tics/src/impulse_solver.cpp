#include "tics.h"

#include <cassert>

using tics::ImpulseSolver;

void ImpulseSolver::solve(std::vector<Collision>& collisions, float delta) {
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
		// continue if a rigid body has an impulse that was not yet added to its velocity
		if (
			   (rb_a && rb_a->impulse != Terathon::Vector3D(0,0,0))
			|| (rb_b && rb_b->impulse != Terathon::Vector3D(0,0,0))
		) { continue; }

		const auto velocity_a = rb_a ? rb_a->velocity : Terathon::Vector3D(0.0, 0.0, 0.0);
		const auto velocity_b = rb_b ? rb_b->velocity : Terathon::Vector3D(0.0, 0.0, 0.0);
		const auto relative_velocity = velocity_a - velocity_b;
		// relative velocity in the collision normal direction
		const auto n_dot_v = Terathon::Dot(relative_velocity, collision.points.normal);

		// coefficient of restitution is the ratio of the relative velocity of
		// separation after collision to the relative velocity of approach before collision.
		// it is a property of BOTH collision objects (their "bounciness").
		const auto cor = 0.75f;

		const auto inv_mass_a = rb_a ? 1.0f/rb_a->mass : 0.0f;
		const auto inv_mass_b = rb_b ? 1.0f/rb_b->mass : 0.0f;

		// from my "game physics" lecture
		// an impulse models an instantaneous change in momentum, unit: m/s * kg
		const auto impulse_magnitude = (-(1.0f + cor) * n_dot_v) / ( inv_mass_a + inv_mass_b );
		const auto impulse = impulse_magnitude * collision.points.normal;

		// apply impulses only to rigid bodies
		if (rb_a) { rb_a->impulse += impulse; };
		if (rb_b) { rb_b->impulse -= impulse; }
	}
}
