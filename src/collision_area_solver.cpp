#include "tics.h"

#include <algorithm>

using tics::CollisionAreaSolver;

void CollisionAreaSolver::solve(const std::vector<Collision>& collisions, float delta) {
	AreasCollisionRecord current_collisions = {};

	for (const auto &collision : collisions) {
		auto sp_a = collision.a.lock();
		auto sp_b = collision.b.lock();
		if (!sp_a || !sp_b) { continue; } // are pointers valid?

		const auto area_a = dynamic_cast<CollisionArea *>(sp_a.get());
		const auto area_b = dynamic_cast<CollisionArea *>(sp_b.get());

		if (area_a) {
			if (!current_collisions.contains(area_a)) {
				current_collisions[area_a] = {};
			}
			current_collisions[area_a].push_back( ObjectAndCollisionData(sp_b, collision.points, true) );
		}

		if (area_b) {
			if (!current_collisions.contains(area_b)) {
				current_collisions[area_b] = {};
			}
			current_collisions[area_b].push_back( ObjectAndCollisionData(sp_a, collision.points, false) );
		}
	}

	for (const auto &[previous_area, previous_objects] : m_areas_collision_record) {
		if (!previous_area->on_collision_exit) { continue; } // there is no listener

		for (const auto &prev : previous_objects) {
			const auto sp_prev = prev.object.lock();
			if (current_collisions.contains(previous_area) && sp_prev) {
				const auto &curr_objects = current_collisions[previous_area];
				auto previous_is_still_colliding = false;
				for (const auto &curr : curr_objects) {
					if (!curr.object.expired() && curr.object.lock() == sp_prev) {
						previous_is_still_colliding = true;
						continue;
					}
				}
				if (previous_is_still_colliding) { continue; }
			}
			// an element that collided previously is not colliding anymore
			previous_area->on_collision_exit(prev.object);
		}
	}

	for (const auto &[current_area, curr_objects] : current_collisions) {
		if (!current_area->on_collision_enter) { continue; } // there is no listener

		for (const auto &curr : curr_objects) {
			const auto sp_curr = curr.object.lock();
			if (m_areas_collision_record.contains(current_area) && sp_curr) {
				const auto &prev_objects = m_areas_collision_record[current_area];
				auto current_was_already_colliding = false;
				for (const auto &prev : prev_objects) {
					if (!prev.object.expired() && prev.object.lock() == sp_curr) {
						current_was_already_colliding = true;
						continue;
					}
				}
				if (current_was_already_colliding) { continue; }
			}
			// an element that was not colliding previously is now colliding
			current_area->on_collision_enter(
				curr.object,
				curr.collision_points_swapped
					? CollisionPoints(
						curr.collision_points.b, curr.collision_points.a,
						-curr.collision_points.normal, curr.collision_points.depth,
						curr.collision_points.has_collision
					)
					: curr.collision_points
			);
		}
	}

	m_areas_collision_record = current_collisions;
}
