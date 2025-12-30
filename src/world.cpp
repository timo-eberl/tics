#include "tics_old.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <chrono>

using namespace std::chrono_literals;

using tics::World;

void World::add_object(const std::weak_ptr<tics::ICollisionObject> object) {
	m_objects.emplace_back(object);
}

void World::remove_object(const std::weak_ptr<tics::ICollisionObject> object) {
	auto is_equals = [object](std::weak_ptr<tics::ICollisionObject> obj) {
		return !obj.expired() && !object.expired() && object.lock() == obj.lock();
	};
	// find the object, move it to the end of the list and erase it
	m_objects.erase(std::remove_if(m_objects.begin(), m_objects.end(), is_equals), m_objects.end());
}

void World::add_solver(const std::weak_ptr<ISolver> solver) {
	m_solvers.emplace_back(solver);
}

void World::remove_solver(const std::weak_ptr<ISolver> solver) {
	auto is_equals = [solver](std::weak_ptr<tics::ISolver> s) {
		return !s.expired() && !solver.expired() && solver.lock() == s.lock();
	};
	// find the solver, move it to the end of the list and erase it
	m_solvers.erase(std::remove_if(m_solvers.begin(), m_solvers.end(), is_equals), m_solvers.end());
}

static void apply_dynamics(tics::RigidBody &rigid_body, const float delta, const tics_vec3 &gravity) {
	const auto &transform = rigid_body.get_transform().lock();

	// add gravity
	rigid_body.impulse = tics_vec3_add(
		rigid_body.impulse,
		tics_vec3_mul_f(gravity, rigid_body.mass * delta * rigid_body.gravity_scale)
	);

	// apply impulses to velocities
	assert(rigid_body.mass != 0.0f);
	// linear
	rigid_body.velocity = tics_vec3_add(
		rigid_body.velocity,
		tics_vec3_mul_f(rigid_body.impulse, 1.0f / rigid_body.mass)
	);
	
	// angular
	// NOTE: angular velocity is stored in rad / 0.1s, because a quaternion/rotor
	//       using rad/s would only be able to store a maximum of 1 rotation per second
	const auto angular_vel_change = tics_quat_scale( rigid_body.an_imp_div_sq_dst, 1.0f/rigid_body.mass );
	assert(angular_vel_change.x == angular_vel_change.x); // check for NaN (invalid input imulse?)
	rigid_body.angular_velocity = tics_quat_mul(angular_vel_change, rigid_body.angular_velocity);

	// apply velocities to transform
	transform->position = tics_vec3_add(transform->position, tics_vec3_mul_f(rigid_body.velocity, delta));
	const auto rotation_change = tics_quat_scale(rigid_body.angular_velocity, delta * 10.0f);
	transform->rotation = tics_quat_mul(transform->rotation, rotation_change);

	// linear air friction
	const auto lin_fric = 0.2f;
	rigid_body.velocity = tics_vec3_sub(rigid_body.velocity, tics_vec3_mul_f(rigid_body.velocity, lin_fric * delta));
	// angular air friction
	const auto ang_fric = 0.5f;
	// Lerp towards identity for friction
	rigid_body.angular_velocity = tics_quat_lerp(rigid_body.angular_velocity, {0,0,0,1}, ang_fric * delta);

	// reset impulses
	rigid_body.impulse = {0,0,0};
	rigid_body.an_imp_div_sq_dst = {0,0,0,1};
}

void World::update(const float delta) {
	static std::vector<std::chrono::nanoseconds> dynamics_times;
	static std::vector<std::chrono::nanoseconds> collision_detection_times;
	static std::vector<std::chrono::nanoseconds> collision_response_times;

	const auto cd_start = std::chrono::high_resolution_clock::now();
	const auto collisions = collision_detection(delta);
	const auto cd_time = std::chrono::high_resolution_clock::now() - cd_start;
	collision_detection_times.push_back(cd_time);

	const auto cr_start = std::chrono::high_resolution_clock::now();
	collision_response(delta, collisions);
	const auto cr_time = std::chrono::high_resolution_clock::now() - cr_start;
	collision_response_times.push_back(cr_time);

	const auto d_start = std::chrono::high_resolution_clock::now();
	// dynamics
	for (auto wp_object : m_objects) {
		if (auto sp_object = wp_object.lock()) {
			const auto rigid_body = dynamic_cast<RigidBody *>(sp_object.get());
			if (!rigid_body) { continue; }
			apply_dynamics(*rigid_body, delta, m_gravity);
		}
	}
	const auto d_time = std::chrono::high_resolution_clock::now() - d_start;
	dynamics_times.push_back(d_time);

	std::chrono::nanoseconds cd_total = 0ns;
	for (const auto &t : collision_detection_times) { cd_total += t; }
	std::chrono::nanoseconds cr_total = 0ns;
	for (const auto &t : collision_response_times) { cr_total += t; }
	std::chrono::nanoseconds d_total = 0ns;
	for (const auto &t : dynamics_times) { d_total += t; }

	static int i = 0;
	if (i%10 == 0) {
		std::cout
			<< "la "
			<< "d: " << d_total / dynamics_times.size() << ", "
			<< "cd: " << cd_total / collision_detection_times.size() << ", "
			<< "cr: " << cr_total / collision_response_times.size()
		<< "\n";
	}
	i++;
}

std::vector<tics::Collision> World::collision_detection(const float delta) {
	std::vector<Collision> collisions;

	for (auto wp_a : m_objects) {
		for (auto wp_b : m_objects) {
			auto sp_a = wp_a.lock();
			auto sp_b = wp_b.lock();
			if (!sp_a || !sp_b) { continue; }

			// break if both pointers point to the same object -> we will only check unique pairs
			if (sp_a == sp_b) { break; }

			// don't test static against static
			const auto sb_a = dynamic_cast<StaticBody *>(sp_a.get());
			const auto sb_b = dynamic_cast<StaticBody *>(sp_b.get());
			if (sb_a && sb_b) { continue; }

			if (sp_a->get_collider().expired() || sp_b->get_collider().expired() ||
				sp_a->get_transform().expired() || sp_b->get_transform().expired()
			) {
				continue;
			}

			auto collision_points = collision_test(
				*(sp_a->get_collider().lock()), *(sp_a->get_transform().lock()),
				*(sp_b->get_collider().lock()), *(sp_b->get_transform().lock())
			);

			if (collision_points.has_collision) {
				collisions.emplace_back(sp_a, sp_b, collision_points);
			}
		}
	}

	return collisions;
}

void World::collision_response(const float delta, const std::vector<tics::Collision> &collisions) {
	for (const auto& collision : collisions) {
		if (m_collision_event) { m_collision_event(collision); }
	}

	for (auto wp_solver : m_solvers) {
		if (auto sp_solver = wp_solver.lock()) {
			sp_solver->solve(collisions, delta);
		}
	}
}

void World::set_gravity(const tics_vec3 gravity) {
	m_gravity = gravity;
}

void World::set_collision_event(const std::function<void(const Collision&)> collision_event) {
	m_collision_event = collision_event;
}
