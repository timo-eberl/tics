#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>

#include "game_loop.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/mat4x4.hpp>
#include <ron.h>
#include <tics.h>
#include <tics_math.h>

using namespace std::chrono_literals;

auto accumulated_render_time = 0ns;
unsigned int accumulated_render_time_count = 0;
auto accumulated_physics_time = 0ns;
unsigned int accumulated_physics_time_count = 0;

struct Sphere {
	const std::shared_ptr<tics::RigidBody> rigid_body;
	const std::shared_ptr<tics::Transform> transform;
	const std::shared_ptr<tics::MeshCollider> collider;
	std::shared_ptr<ron::MeshNode> mesh_node;
	glm::vec3 color;
};

struct StaticObject {
	const std::shared_ptr<tics::StaticBody> static_body;
	const std::shared_ptr<tics::Transform> transform;
	const std::shared_ptr<tics::MeshCollider> collider;
};

glm::mat4 transform_to_model_matrix(const tics::Transform& transform) {
	const auto transl_mat =
		glm::translate(glm::identity<glm::mat4>(),
					   glm::vec3(transform.position.x, transform.position.y, transform.position.z));

	tics_mat4 t_rot_mat = tics_mat4_from_quat(transform.rotation);
	auto rot_mat = glm::make_mat4(t_rot_mat.m);

	return transl_mat * rot_mat;
}

ron::DirectionalLight create_generic_light() {
	ron::DirectionalLight light = {};

	light.use_custom_shadow_target_world_position = true;
	light.custom_shadow_target_world_position = glm::vec3(0.0f, 0.0f, 0.0f);
	light.world_direction = glm::normalize(glm::vec3(0.1f, 1.0f, 0.0f));
	light.shadow.enabled = true;
	light.shadow.map_size = glm::uvec2(2048);
	light.shadow.bias = 0.05f;
	light.shadow.far = 30.0f;
	light.shadow.frustum_size = 50.0f;

	return light;
}

Sphere create_sphere(tics_vec3 position, tics_vec3 velocity, tics_quat angular_velocity,
					 const ron::Scene& scene, glm::vec3 color = glm::vec3(1.0), float scale = 1.0f,
					 float elasticity = 0.9f) {
	Sphere sphere = Sphere({
		std::make_shared<tics::RigidBody>(),
		std::make_shared<tics::Transform>(),
		std::make_shared<tics::MeshCollider>(),
		ron::gltf::import("models/icosphere.glb").get_mesh_nodes().front(),
	});

	sphere.rigid_body->set_collider(sphere.collider);
	sphere.rigid_body->set_transform(sphere.transform);
	sphere.rigid_body->mass = scale * scale * scale * 2.0;
	sphere.rigid_body->velocity = velocity;
	sphere.rigid_body->angular_velocity = angular_velocity;
	sphere.rigid_body->elasticity = elasticity;
	sphere.color = color;
	sphere.transform->position = position;

	// apply scale to visual mesh
	for (auto& section : sphere.mesh_node->get_mesh()->sections) {
		for (auto& position : section.geometry->positions) {
			position *= scale;
		}
	}
	// copy the default material and modify it
	const auto material = std::make_shared<ron::Material>(*scene.default_material);
	material->uniforms["albedo_color"] = ron::make_uniform(glm::vec4(color, 1.0));
	// copy the mesh node to set its color
	const auto cloned_mesh_node =
		std::make_shared<ron::MeshNode>(std::make_shared<ron::Mesh>(*sphere.mesh_node->get_mesh()),
										transform_to_model_matrix(*sphere.transform));
	cloned_mesh_node->get_mesh()->sections.front().material = material;
	sphere.mesh_node = cloned_mesh_node;

	const auto geometry = ron::gltf::import("models/icosphere_smooth.glb")
							  .get_mesh_nodes()
							  .front()
							  ->get_mesh()
							  ->sections.front()
							  .geometry;
	// apply scale to collision mesh
	for (auto& position : geometry->positions) {
		position *= scale;
	}
	// copy positions to MeshCollider
	for (const auto& vertex_pos : geometry->positions) {
		sphere.collider->positions.push_back({vertex_pos.x, vertex_pos.y, vertex_pos.z});
	}

	return sphere;
}

std::shared_ptr<std::vector<StaticObject>> create_static_objects(const std::string& gltf_path) {
	auto objects = std::make_shared<std::vector<StaticObject>>();
	const auto mesh_nodes = ron::gltf::import(gltf_path).get_mesh_nodes();

	for (auto& mesh_node : mesh_nodes) {
		StaticObject static_object{
			std::make_shared<tics::StaticBody>(),
			std::make_shared<tics::Transform>(),
			std::make_shared<tics::MeshCollider>(),
		};

		const auto center = mesh_node->get_model_matrix() * glm::vec4(0, 0, 0, 1);
		static_object.transform->position = {center.x, center.y, center.z};

		const auto ground_geometry = mesh_node->get_mesh()->sections.front().geometry;
		for (const auto& vertex_pos : ground_geometry->positions) {
			static_object.collider->positions.push_back({vertex_pos.x, vertex_pos.y, vertex_pos.z});
		}
		static_object.static_body->set_collider(static_object.collider);
		static_object.static_body->set_transform(static_object.transform);

		objects->emplace_back(static_object);
	}

	return objects;
}

struct AreaTrigger {
	const std::shared_ptr<tics::CollisionArea> area;
	const std::shared_ptr<tics::Transform> transform;
	const std::shared_ptr<tics::MeshCollider> collider;
	std::shared_ptr<ron::MeshNode> mesh_node;
};

struct ProgramState {
	std::shared_ptr<std::vector<Sphere>> spheres;
	std::shared_ptr<std::vector<StaticObject>> static_geometry;
	AreaTrigger area_trigger;

	ron::PerspectiveCamera camera;
	ron::CameraViewportControls camera_controls;
	std::shared_ptr<ron::Scene> render_scene;
	std::unique_ptr<ron::OpenGLRenderer> renderer;

	tics::World physics_world;
	std::shared_ptr<tics::ImpulseSolver> impulse_solver;
	std::shared_ptr<tics::NonIntersectionConstraintSolver> position_solver;
	std::shared_ptr<tics::CollisionAreaSolver> collision_area_solver;
};

static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
static ProgramState initialize(GLFWwindow* window);
static void process(GLFWwindow* window, ProgramState& state);
static void render(GLFWwindow* window, ProgramState& state);

const int res_x = 1000;
const int res_y = 600;

int main() {
	glfwInit();

	// tell GLFW we are using OpenGL 4.6
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
	// tell GLFW we want to use the core-profile -> no backwards-compatible features
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(res_x, res_y, "Physics Playground", NULL, NULL);
	if (window == NULL) {
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	// glfwSwapInterval(0); // request to disable vsync

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	{ // this scope ensures that "state" is destroyed before the opengl context is destroyed
	  // (glfwTerminate)
		ProgramState state = initialize(window);
		glfwSetWindowUserPointer(window, static_cast<void*>(&state));

		GameLoop game_loop(
			[window, &state](const float delta, const float fixed_delay) { render(window, state); },
			[window, &state](const float delta) { process(window, state); }, 1.0f / 60.0f, 0.0f,
			1.0f / 60.0f);

		auto last_title_update_time_point = std::chrono::high_resolution_clock::now();

		while (!glfwWindowShouldClose(window)) {
			game_loop.update();

			// measure performance
			static const auto update_interval = 250ms;
			const auto now = std::chrono::high_resolution_clock::now();
			if (now - last_title_update_time_point >= update_interval) {
				last_title_update_time_point = now;

				const auto avg_render_time = std::chrono::duration<double>(
					accumulated_render_time / accumulated_render_time_count);
				const double avg_render_time_ms = avg_render_time.count() * 1000.0;

				accumulated_render_time = 0ns;
				accumulated_render_time_count = 0;

				const auto avg_physics_time = std::chrono::duration<double>(
					accumulated_physics_time / accumulated_physics_time_count);
				const double avg_physics_time_ms = avg_physics_time.count() * 1000.0;

				accumulated_physics_time = 0ns;
				accumulated_physics_time_count = 0;

				std::stringstream title_stream;
				title_stream << "Physics Playground"
							 << " - Render Time: " << std::fixed
							 << std::setprecision(2) // specify decimal places
							 << avg_render_time_ms
							 << "ms"
							 // << " - Render FPS: "
							 // << static_cast<int>(1000.0 / avg_render_time_ms)
							 << " - Phyiscs Time: " << std::fixed
							 << std::setprecision(2) // specify decimal places
							 << avg_physics_time_ms << "ms"
					// << " - Physics FPS: "
					// << static_cast<int>(1000.0 / avg_physics_time_ms)
					;
				glfwSetWindowTitle(window, +title_stream.str().c_str());
			}
		}
	}

	glfwTerminate();
	return 0;
}

ProgramState initialize(GLFWwindow* window) {
	// renderer setup
	auto scene = std::make_shared<ron::Scene>();
	scene->set_directional_light(create_generic_light());
	auto renderer = std::make_unique<ron::OpenGLRenderer>(res_x, res_y);
	renderer->set_clear_color(glm::vec4(0.1, 0.1, 0.1, 1.0));
	// renderer->render_axes = true;
	// renderer->render_grid = true;
	// camera setup
	const auto initial_camera_rotation = glm::vec2(glm::radians(-24.2f), glm::radians(63.6f));
	auto camera_controls = ron::CameraViewportControls(initial_camera_rotation);
	camera_controls.set_target(glm::vec3(0.0f, 0.0f, 0.0f));

	tics::World physics_world;
	physics_world.set_gravity({0.0f, -9.81f, 0.0f});

	auto spheres = std::make_shared<std::vector<Sphere>>();

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

	// create some spheres
	for (size_t i = 0; i < 21; i++) {
		tics_vec3 pos = tics_vec3_add(positions[i % 10], {0, (float)(i / 10) * 2.0f, 0});
		auto sphere = create_sphere(
			pos, {0, 0, 0}, // velocity
			tics_quat_from_axis_angle(tics_vec3_normalize(positions[i % 10]), 0.1f), *scene,
			glm::vec3( // color
				static_cast<double>(std::rand()) / RAND_MAX,
				static_cast<double>(std::rand()) / RAND_MAX,
				static_cast<double>(std::rand()) / RAND_MAX),
			scales[i % 10], elasticities[i % 10]);
		spheres->emplace_back(sphere);
		physics_world.add_object(sphere.rigid_body);
		scene->add(sphere.mesh_node);
	}

	// add static geometry
	auto static_geometry = create_static_objects("models/ground_smooth.glb");
	for (const auto& static_object : *static_geometry) {
		physics_world.add_object(static_object.static_body);
		scene->add(ron::gltf::import("models/ground.glb"));
	}

	// area trigger that turns objects red that are inside it
	AreaTrigger area_trigger = {std::make_shared<tics::CollisionArea>(),
								std::make_shared<tics::Transform>(),
								std::make_shared<tics::MeshCollider>(),
								ron::gltf::import("models/rectangle.glb").get_mesh_nodes().front()};
	area_trigger.area->set_collider(area_trigger.collider);
	area_trigger.area->set_transform(area_trigger.transform);
	area_trigger.area->on_collision_enter = [spheres, scene](const auto& other,
															 const auto& collision_data) {
		std::cout << "collision_data: normal: {" << collision_data.normal.x << ","
				  << collision_data.normal.y << "," << collision_data.normal.z << "}"
				  << " depth: " << collision_data.depth << "\n";
		const auto debug_sphere = create_sphere(collision_data.a, {0, 0, 0}, tics_quat_identity(),
												*scene, {1, 1, 0}, 0.1);
		scene->add(debug_sphere.mesh_node);
		for (const auto& sphere : *spheres) {
			if (sphere.rigid_body == other.lock()) {
				sphere.mesh_node->get_mesh()->sections.front().material->uniforms["albedo_color"] =
					ron::make_uniform(glm::vec4(glm::vec3(1.0, 0.1, 0.1), 1.0));
			}
		}
	};
	area_trigger.area->on_collision_exit = [spheres](const auto& other) {
		for (const auto& sphere : *spheres) {
			if (sphere.rigid_body == other.lock()) {
				sphere.mesh_node->get_mesh()->sections.front().material->uniforms["albedo_color"] =
					ron::make_uniform(glm::vec4(sphere.color, 1.0));
			}
		}
	};
	// apply scale to mesh
	for (auto& section : area_trigger.mesh_node->get_mesh()->sections) {
		for (auto& position : section.geometry->positions) {
			position *= glm::vec3(2.0, 1.0, 2.0);
		}
	}

	area_trigger.transform->position = {-2.0f, 2.0f, 0.0f};
	area_trigger.mesh_node->set_model_matrix(transform_to_model_matrix(*area_trigger.transform));
	const auto area_trigger_geometry =
		area_trigger.mesh_node->get_mesh()->sections.front().geometry;
	// copy positions to MeshCollider
	for (const auto& vertex_pos : area_trigger_geometry->positions) {
		area_trigger.collider->positions.push_back({vertex_pos.x, vertex_pos.y, vertex_pos.z});
	}
	// physics_world.add_object(area_trigger.area);
	// scene->add(area_trigger.mesh_node);

	// applies forces
	auto impulse_solver = std::make_shared<tics::ImpulseSolver>();
	// fixes intersections
	auto position_solver = std::make_shared<tics::NonIntersectionConstraintSolver>();
	// alerts collision areas
	auto collision_area_solver = std::make_shared<tics::CollisionAreaSolver>();
	physics_world.add_solver(impulse_solver);
	physics_world.add_solver(position_solver);
	physics_world.add_solver(collision_area_solver);

	renderer->preload(*scene);

	glfwSetScrollCallback(window, scroll_callback);

	return ProgramState{
		spheres,
		static_geometry,
		area_trigger,
		ron::PerspectiveCamera(60.0f, (float)(res_x) / (float)(res_y), 0.1f, 1000.0f),
		camera_controls,
		scene,
		std::move(renderer),
		physics_world,
		impulse_solver,
		position_solver,
		collision_area_solver};
}

void process(GLFWwindow* window, ProgramState& state) {
	glfwPollEvents();

	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, true);
		return;
	}

	auto start_time_point = std::chrono::high_resolution_clock::now();

	const float time_scale = 1.0f;
	const float delta = time_scale / 60.0f;

	const float time_limit = 20.0f; // seconds
	static float total_time = 0;

	if (total_time < time_limit) {
		state.physics_world.update(delta);
		total_time += delta;
	}

	const auto physics_time = std::chrono::high_resolution_clock::now() - start_time_point;
	++accumulated_physics_time_count;
	accumulated_physics_time += physics_time;

	for (const auto& sphere : *state.spheres) {
		sphere.mesh_node->set_model_matrix(transform_to_model_matrix(*sphere.transform));
	}

	state.camera_controls.update(*window, state.camera);
}

void render(GLFWwindow* window, ProgramState& state) {
	auto start_time_point = std::chrono::high_resolution_clock::now();

	state.renderer->render(*(state.render_scene), state.camera);

	const auto render_time = std::chrono::high_resolution_clock::now() - start_time_point;
	++accumulated_render_time_count;
	accumulated_render_time += render_time;

	glfwSwapBuffers(window);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
	auto state = static_cast<ProgramState*>(glfwGetWindowUserPointer(window));
	assert(state);

	state->renderer->resolution = glm::uvec2(width, height);

	state->camera.set_aspect_ratio(static_cast<float>(width) / static_cast<float>(height));
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
	auto state = static_cast<ProgramState*>(glfwGetWindowUserPointer(window));

	if (state) {
		if (yoffset == 1.0) {
			state->camera_controls.scroll_callback(ron::CameraViewportControls::UP);
		} else if (yoffset == -1.0) {
			state->camera_controls.scroll_callback(ron::CameraViewportControls::DOWN);
		}
	}
}
