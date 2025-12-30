#include "visualization.h"
#include <tics.h>
#include <tics_math.h>

// A container to link physics objects to their visual counterparts
struct DynamicObject {
	tics_body_id body;
	std::shared_ptr<ron::MeshNode> visual;
};

// Container for static objects to maintain ownership
struct StaticObject {
	tics_body_id body;
};

tics_shape_id create_scaled_shape(tics_world* world, const std::vector<tics_vec3>& vertices,
								  float scale) {
	std::vector<tics_vec3> scaled_verts;
	scaled_verts.reserve(vertices.size());
	for (const auto& v : vertices) {
		scaled_verts.push_back({v.x * scale, v.y * scale, v.z * scale});
	}

	tics_shape_desc desc = {};
	desc.type = TICS_SHAPE_CONVEX;
	desc.data.convex.vertices = scaled_verts.data();
	desc.data.convex.vertex_count = scaled_verts.size();

	return tics_create_shape(world, desc);
}

int main() {
	// Initialize Visualization (Boilerplate hidden in header)
	auto viz = Visualization::initialize(1000, 600, "Physics Playground");

	// Configure Physics World
	tics_world_desc world_desc = {};
	world_desc.gravity = {0.0f, -9.81f, 0.0f};

	tics_world* world = tics_world_create(world_desc);

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
	const float scales[3] = {0.8f, 1.2f, 1.7f};
	const float elasticities[10] = {0.9, 0.9, 0.8, 0.85, 0.8, 0.95, 1.0, 0.75, 0.8, 0.95};

	// Load collision mesh data (smooth sphere for better physics performance)
	const auto sphere_collision_vertices =
		viz.import_objects("models/icosphere_smooth.glb").front().vertices;

	// Pre-create shared colliders for each scale.
	std::vector<tics_shape_id> shared_shapes;
	for (float s : scales) {
		shared_shapes.push_back(create_scaled_shape(world, sphere_collision_vertices, s));
	}

	for (int i = 0; i < 21; i++) {
		int idx = i % 10;
		int scale_idx = i % 3; // Cycle through the 3 scales
		float scale = scales[scale_idx];

		tics_vec3 pos = tics_vec3_add(positions[idx], {0, (float)(i / 10) * 2.0f, 0});
		tics_vec3 color = viz.random_color();

		// Create Visuals (We scale the visual mesh to match the physics radius)
		auto visual_node = viz.create_sphere_node(scale, color);

		// Create Physics Body
		tics_rigid_body_desc desc = {};
		desc.transform.position = pos;
		desc.transform.rotation = tics_quat_from_axis_angle(tics_vec3_normalize(pos), 0.1f);
		desc.shape = shared_shapes[scale_idx];
		desc.mass = scale * scale * scale * 2.0f;
		desc.elasticity = elasticities[idx];
		desc.gravity_scale = 1.0f;
		desc.linear_velocity = {0, 0, 0};
		desc.angular_velocity = tics_quat_identity();

		tics_body_id body_id = tics_world_add_rigid_body(world, desc);

		spheres.push_back({body_id, visual_node});
	}

	// --- Create Static Geometry (Ground) ---
	// "StaticBody" does not move but collides with RigidBodies.
	std::vector<StaticObject> static_objects;

	auto static_meshes = viz.import_objects("models/ground_smooth.glb");
	for (const auto& mesh_data : static_meshes) {
		tics_shape_id shape_id = create_scaled_shape(world, mesh_data.vertices, 1.0f);

		tics_static_body_desc desc = {};
		desc.transform.position = {mesh_data.position.x, mesh_data.position.y,
								   mesh_data.position.z};
		desc.transform.rotation = tics_quat_identity();
		desc.shape = shape_id;
		desc.elasticity = 0.8f;

		tics_body_id body_id = tics_world_add_static_body(world, desc);

		static_objects.push_back({body_id});
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
			tics_world_step(world, physics_dt);
			total_time += physics_dt;
		}

		// Visual Sync (Copy Physics Transform -> Visual Transform)
		for (auto& sphere : spheres) {
			tics_transform t = tics_body_get_transform(world, sphere.body);
			viz.sync_transform(t, sphere.visual);
		}

		viz.update_and_render();
	}

	tics_world_destroy(world);

	return 0;
}
