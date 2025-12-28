#include "tics.h"

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

// test that helped me find a bug in the Terathon library
static void test_terathon_geometric_anti_product() {
	{
		const auto R = Terathon::Motor3D::MakeRotation(
			0.2 * 3.141, Terathon::Normalize(Terathon::Bivector3D(1,0.3,-0.05))
		);
		const auto T = Terathon::Motor3D::MakeTranslation(
			Terathon::Vector3D(0.2, -0.5, 0.1)
		);

		// rotate, then translate

		auto p_rt = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), R);
		p_rt = Terathon::Transform(p_rt, T);

		const auto Q_0 = T * R;
		const auto p_q_0 = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), Q_0);
		assert(Terathon::Magnitude(p_rt - p_q_0) < 0.001);

		auto Q_1 = T * R.v;
		const auto p_q_1 = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), Q_1);
		assert(Terathon::Magnitude(p_rt - p_q_1) < 0.001);

		// translate, then rotate

		auto p_tr = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), T);
		p_tr = Terathon::Transform(p_tr, R);

		auto Q_2 = R * T;
		const auto p_q_2 = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), Q_2);
		assert(Terathon::Magnitude(p_tr - p_q_2) < 0.001);

		auto Q_3 = R.v * T;
		const auto p_q_3 = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), Q_3);
		assert(Terathon::Magnitude(p_tr - p_q_3) < 0.001);
	}
}

static Terathon::Motor3D scale_motor(const Terathon::Motor3D &motor, const float scale) {
	// "scale" operator by delta
	// - implemented as a lerp(identity, motor) + unitize
	// - maybe it could be implemented more optimized
	// - used += because no + operator exists ...
	auto m = Terathon::Motor3D::identity * (1.0 - scale);
	m +=     motor                       *        scale;
	// m.Unitize();
	m.v.Normalize();
	return m;
}

static Terathon::Quaternion scale_quaternion(const Terathon::Quaternion &quaternion, const float scale) {
	// "scale" operator by delta
	// - implemented as a lerp(identity, quaternion) + normalize
	// - maybe it could be implemented more optimized
	auto q = Terathon::Quaternion::identity * (1.0 - scale);
	q +=     quaternion                     *        scale;
	q.Normalize();
	return q;
}

static void apply_dynamics(tics::RigidBody &rigid_body, const float delta, const Terathon::Vector3D &gravity) {
	const auto &transform = rigid_body.get_transform().lock();

	// add gravity
	rigid_body.impulse += rigid_body.mass * delta * rigid_body.gravity_scale * gravity;

	// apply impulses to velocities
	assert(rigid_body.mass != 0.0f);
	// linear
	rigid_body.velocity += rigid_body.impulse / rigid_body.mass;
	// angular
	// NOTE: angular velocity is stored in rad / 0.1s, because a quaternion/rotor
	//       using rad/s would only be able to store a maximum of 1 rotation per second
	const auto angular_vel_change = scale_quaternion( rigid_body.an_imp_div_sq_dst, 1.0f/rigid_body.mass );
	assert(angular_vel_change.x == angular_vel_change.x); // check for NaN (invalid input imulse?)
	rigid_body.angular_velocity = angular_vel_change * rigid_body.angular_velocity;

	// apply velocities to transform
	#ifdef TICS_GA
		const auto translation_change = Terathon::Motor3D::MakeTranslation(rigid_body.velocity * delta);
		const auto rotation_change_rotor = scale_quaternion(rigid_body.angular_velocity, delta * 10.0);
		transform->motor = translation_change * transform->motor * rotation_change_rotor;
	#else
		transform->position += rigid_body.velocity * delta;
		const auto rotation_change = scale_quaternion(rigid_body.angular_velocity, delta * 10.0);
		transform->rotation = transform->rotation * rotation_change;
	#endif

	// linear air friction
	const auto lin_fric = 0.2f;
	rigid_body.velocity -= rigid_body.velocity * (lin_fric * delta);
	// angular air friction
	const auto ang_fric = 0.5f;
	rigid_body.angular_velocity = scale_quaternion(rigid_body.angular_velocity, 1.0 - (ang_fric*delta));

	// reset impulses
	rigid_body.impulse = Terathon::Vector3D(0,0,0);
	rigid_body.an_imp_div_sq_dst = Terathon::Quaternion::identity;
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
#ifdef TICS_GA
			<< "ga "
#else
			<< "la "
#endif
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

void World::set_gravity(const Terathon::Vector3D gravity) {
	m_gravity = gravity;
}

void World::set_collision_event(const std::function<void(const Collision&)> collision_event) {
	m_collision_event = collision_event;
}
