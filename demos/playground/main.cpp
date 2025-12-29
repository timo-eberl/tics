#include "visualization.h"
#include <tics.h>
#include <tics_math.h>

// A container to link physics objects to their visual counterparts
struct DynamicObject {
	std::shared_ptr<tics::RigidBody> body;
	std::shared_ptr<tics::Transform> transform;
	std::shared_ptr<tics::MeshCollider> collider;
	std::shared_ptr<ron::MeshNode> visual;
	tics_vec3 original_color;
};

// Container for static objects to maintain ownership
struct StaticObject {
	std::shared_ptr<tics::StaticBody> body;
	std::shared_ptr<tics::Transform> transform;
	std::shared_ptr<tics::MeshCollider> collider;
};

std::shared_ptr<tics::MeshCollider> create_scaled_collider(const std::vector<tics_vec3>& vertices,
														   float scale) {
	auto collider = std::make_shared<tics::MeshCollider>();
	collider->positions.reserve(vertices.size());
	for (const auto& v : vertices) {
		collider->positions.push_back({v.x * scale, v.y * scale, v.z * scale});
	}
	return collider;
}

int main() {
	// Initialize Visualization (Boilerplate hidden in header)
	auto viz = Visualization::initialize(1000, 600, "Physics Playground");

	// Configure Physics World
	tics::World world;
	world.set_gravity({0.0f, -9.81f, 0.0f});

	// Solvers determine how the simulation resolves conflicts.
	// applies forces
	auto impulse_solver = std::make_shared<tics::ImpulseSolver>();
	// fixes intersections
	auto position_solver = std::make_shared<tics::NonIntersectionConstraintSolver>();
	world.add_solver(impulse_solver);
	world.add_solver(position_solver);

	// --- Create Dynamic Objects ---
	std::vector<DynamicObject> spheres;

	// clang-format off
	const tics_vec3 positions[10] = {
		{ 1.2f, 3.6f, 1.2f},
		{ 2.4f, 4.8f, 2.4f},
		{-3.6f, 6.0f, 3.6f},
		{-4.8f, 3.6f, 4.8f},
		{-3.6f, 4.8f, 3.6f},
		{-1.2f, 6.0f,-1.2f},
		{-2.4f, 3.6f,-2.4f},
		{ 3.6f, 4.8f,-3.6f},
		{ 4.8f, 6.0f,-4.8f},
		{ 3.6f, 3.6f,-2.4f}
	};
	// clang-format on
	const float scales[10] = {0.9, 1.3, 2.0, 1.2, 1.3, 0.8, 0.9, 1.3, 1.7, 1.2};
	const float elasticities[10] = {0.9, 0.9, 0.8, 0.85, 0.8, 0.95, 1.0, 0.75, 0.8, 0.95};

	// Load collision mesh data (smooth sphere for better physics performance)
	const auto sphere_collision_vertices =
		viz.import_objects("models/icosphere_smooth.glb").front().vertices;

	for (int i = 0; i < 21; i++) {
		int idx = i % 10;
		float scale = scales[idx];
		tics_vec3 pos = tics_vec3_add(positions[idx], {0, (float)(i / 10) * 2.0f, 0});
		tics_vec3 color = viz.random_color();

		// Setup Transform
		auto transform = std::make_shared<tics::Transform>();
		transform->position = pos;
		transform->rotation = tics_quat_from_axis_angle(tics_vec3_normalize(pos), 0.1f);

		// Create Visuals (We scale the visual mesh to match the physics radius)
		auto visual_node = viz.create_sphere_node(scale, color);

		auto collider = create_scaled_collider(sphere_collision_vertices, scale);

		// Create Physics Body
		auto rb = std::make_shared<tics::RigidBody>();
		rb->mass = scale * scale * scale * 2.0f;
		rb->elasticity = elasticities[idx];
		rb->set_transform(transform);
		rb->set_collider(collider);

		world.add_object(rb);

		// Store everything to prevent destruction (we have ownership)
		spheres.push_back({rb, transform, collider, visual_node, color});
	}

	// --- Create Static Geometry (Ground) ---
	// "StaticBody" does not move but collides with RigidBodies.
	std::vector<StaticObject> static_objects;

	auto static_meshes = viz.import_objects("models/ground_smooth.glb");
	for (const auto& mesh_data : static_meshes) {
		auto transform = std::make_shared<tics::Transform>();
		transform->position = {mesh_data.position.x, mesh_data.position.y, mesh_data.position.z};

		auto collider = create_scaled_collider(mesh_data.vertices, 1.0f);

		auto sb = std::make_shared<tics::StaticBody>();
		sb->set_transform(transform);
		sb->set_collider(collider);
		world.add_object(sb);

		static_objects.push_back({sb, transform, collider});
	}
	// Add pure visual scenery (non-collidable)
	viz.add_scenery("models/ground.glb");

	// Main Loop
	const float physics_dt = 1.0f / 60.0f; // Fixed time step
	const float time_limit = 20.0f;		   // seconds
	float total_time = 0.0f;

	while (!viz.should_close()) {
		// Physics Step
		if (total_time < time_limit) {
			world.update(physics_dt);
			total_time += physics_dt;
		}

		// Visual Sync (Copy Physics Transform -> Visual Transform)
		for (auto& sphere : spheres) {
			// We can now safely access the transform directly from our storage
			viz.sync_transform(*sphere.transform, sphere.visual);
		}

		viz.update_and_render();
	}

	return 0;
}
