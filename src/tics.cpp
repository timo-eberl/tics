#include "tics_internal.h"
#include <cassert>

extern "C" tics_world* tics_world_create(tics_world_desc desc) {
	tics_world* world = new tics_world();

	world->cpp_world->set_gravity(desc.gravity);

	// Initialize Default Solvers
	auto impulse = std::make_shared<tics::ImpulseSolver>();
	auto non_inter = std::make_shared<tics::NonIntersectionConstraintSolver>();
	// Store them in the wrapper so they don't get deleted
	world->solvers.push_back(impulse);
	world->solvers.push_back(non_inter);
	// Register them with the simulation
	world->cpp_world->add_solver(impulse);
	world->cpp_world->add_solver(non_inter);

	return world;
}

extern "C" void tics_world_destroy(tics_world* world) {
	if (world) {
		// C++ destructors for smart pointers and maps will automatically release memory.
		delete world;
	}
}

extern "C" void tics_world_step(tics_world* world, float delta) {
	assert(world);
	world->cpp_world->update(delta);
}

extern "C" tics_shape_id tics_create_shape(tics_world* world, tics_shape_desc desc) {
	assert(world);

	std::shared_ptr<tics::Collider> collider = nullptr;
	// Translate C struct -> C++ Class
	switch (desc.type) {
	case TICS_SHAPE_SPHERE: {
		auto sph = std::make_shared<tics::SphereCollider>();
		sph->center = desc.data.sphere.center;
		sph->radius = desc.data.sphere.radius;
		collider = sph;
		break;
	}
	case TICS_SHAPE_PLANE: {
		auto pln = std::make_shared<tics::PlaneCollider>();
		pln->normal = desc.data.plane.normal;
		pln->distance = desc.data.plane.distance;
		collider = pln;
		break;
	}
	case TICS_SHAPE_MESH: {
		auto msh = std::make_shared<tics::MeshCollider>();
		// copy data from pointer array
		if (desc.data.mesh.vertices && desc.data.mesh.vertex_count > 0) {
			msh->positions.assign(desc.data.mesh.vertices,
								  desc.data.mesh.vertices + desc.data.mesh.vertex_count);
		}
		collider = msh;
		break;
	}
	default: {
		assert(false); // not implemented
	}
	}

	if (collider) {
		tics_shape_id id = world->next_shape_id();
		world->shapes[id] = collider;
		return id;
	}

	return 0;
}

extern "C" void tics_destroy_shape(tics_world* world, tics_shape_id shape) {
	assert(world);

	// Removing the shared_ptr from the map will decrement the ref count. If a Body is still
	// using it, the Body's shared_ptr will keep it alive until the body is destroyed.
	world->shapes.erase(shape);
}

extern "C" tics_body_id tics_world_add_static_body(tics_world* world, tics_static_desc desc) {
	assert(world);

	// Validate Shape
	auto shape_it = world->shapes.find(desc.shape);
	if (shape_it == world->shapes.end()) {
		return 0; // Invalid Shape ID
	}

	auto transform = std::make_shared<tics::Transform>();
	transform->position = desc.transform.position;
	transform->rotation = desc.transform.rotation;

	auto body = std::make_shared<tics::StaticBody>();
	body->set_collider(shape_it->second);
	body->set_transform(transform);

	body->elasticity = desc.elasticity;

	tics_body_id id = world->next_body_id();
	world->transforms[id] = transform; // Keep transform alive
	world->bodies[id] = body;		   // Keep body alive
	world->cpp_world->add_object(body);

	return id;
}

extern "C" tics_body_id tics_world_add_rigid_body(tics_world* world, tics_rigid_desc desc) {
	assert(world);

	// Validate Shape
	auto shape_it = world->shapes.find(desc.shape);
	if (shape_it == world->shapes.end()) {
		return 0; // Invalid Shape ID
	}

	auto transform = std::make_shared<tics::Transform>();
	transform->position = desc.transform.position;
	transform->rotation = desc.transform.rotation;

	auto body = std::make_shared<tics::RigidBody>();
	body->set_collider(shape_it->second);
	body->set_transform(transform);

	body->velocity = desc.linear_velocity;
	body->angular_velocity = desc.angular_velocity;
	body->mass = desc.mass;
	body->elasticity = desc.elasticity;
	body->gravity_scale = desc.gravity_scale;

	tics_body_id id = world->next_body_id();
	world->transforms[id] = transform; // Keep transform alive
	world->bodies[id] = body;		   // Keep body alive
	world->cpp_world->add_object(body);

	return id;
}

extern "C" void tics_world_remove_body(tics_world* world, tics_body_id id) {
	assert(world);

	auto it = world->bodies.find(id);
	if (it != world->bodies.end()) {
		// Remove from physical world
		world->cpp_world->remove_object(it->second);
		// Remove from our registry (releases shared_ptr)
		world->bodies.erase(it);
		world->transforms.erase(id);
	}
}

extern "C" tics_transform tics_body_get_transform(const tics_world* world, tics_body_id id) {
	assert(world);

	tics_transform result;
	result.position = {0, 0, 0};
	result.rotation = {0, 0, 0, 1};

	auto it = world->bodies.find(id);
	if (it != world->bodies.end()) {
		auto obj = it->second;
		if (auto transform_ptr = obj->get_transform().lock()) {
			// Copy data from C++ object to C struct
			result.position = transform_ptr->position;
			result.rotation = transform_ptr->rotation;
		}
	}

	return result;
}
