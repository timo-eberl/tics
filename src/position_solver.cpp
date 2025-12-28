#include "tics.h"

#include <cmath>

using tics::NonIntersectionConstraintSolver;

enum ObjectCombination { Invalid, RigidBodyRigidBody, RigidBodyStaticBody, StaticBodyRigidBody };

static void add_pos_offset(tics::ICollisionObject &object, Terathon::Vector3D offset) {
	const auto &transform = object.get_transform().lock();

	#ifdef TICS_GA
		const auto offset_motor = Terathon::Motor3D::MakeTranslation(offset);
		transform->motor = offset_motor * transform->motor;
	#else
		transform->position += offset;
	#endif
}

void NonIntersectionConstraintSolver::solve(const std::vector<Collision>& collisions, float delta) {
	for (auto collision : collisions) {
		auto sp_a = collision.a.lock();
		auto sp_b = collision.b.lock();
		if (!sp_a || !sp_b) { continue; } // are pointers valid?

		// check the objects are a valid combination
		const auto rb_a = dynamic_cast<RigidBody *>(sp_a.get());
		const auto rb_b = dynamic_cast<RigidBody *>(sp_b.get());
		const auto sb_a = dynamic_cast<StaticBody *>(sp_a.get());
		const auto sb_b = dynamic_cast<StaticBody *>(sp_b.get());
		auto object_combination = ObjectCombination();
		if (rb_a && rb_b) { object_combination = RigidBodyRigidBody; }
		else if (rb_a && sb_b) { object_combination = RigidBodyStaticBody; }
		else if (sb_a && rb_b) { object_combination = StaticBodyRigidBody; }
		else { continue; } // no valid object combination combination

		const auto percent = 0.8f;
		const auto depth_tolerance = 0.01f; // how much they are allowed to glitch into another

		const float depth_with_tolerance = fmax(collision.points.depth - depth_tolerance, 0.0f);
		// distance that the objects are moved away from each other
		const auto correction = collision.points.normal * ( percent *  depth_with_tolerance);

		switch (object_combination) {
			case RigidBodyRigidBody: {
				const auto b_percentage_of_total_mass = (rb_b->mass) / (rb_a->mass + rb_b->mass);
				// if b is heavier, move a more
				add_pos_offset(*sp_a,  correction * b_percentage_of_total_mass);
				add_pos_offset(*sp_b, -correction * (1.0f - b_percentage_of_total_mass));
			} break;
			case RigidBodyStaticBody: // b is static -> move only a
				add_pos_offset(*sp_a,  correction);
				break;
			case StaticBodyRigidBody: // a is static -> move only b
				add_pos_offset(*sp_b, -correction);
			default: break;
		}
	}
}
