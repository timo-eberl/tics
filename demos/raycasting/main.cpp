#include <cmath>
#include <iostream>
#include <chrono>
#include <sstream>

#include "game_loop.h"
#include "utils.h"

#include <tics.h>
#include <ron.h>
#include <glm/gtx/string_cast.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <TSVector3D.h>
#include <TSMatrix3D.h>
#include <TSMatrix4D.h>
#include <TSMotor3D.h>

using namespace std::chrono_literals;

static const auto pga = true;

auto accumulated_render_time = 0ns;
unsigned int accumulated_render_time_count = 0;
auto accumulated_physics_time = 0ns;
unsigned int accumulated_physics_time_count = 0;

struct AreaTrigger {
	const std::shared_ptr<tics::CollisionArea> area;
	const std::shared_ptr<tics::Transform> transform;
	const std::shared_ptr<tics::MeshCollider> collider;
	std::shared_ptr<ron::MeshNode> mesh_node;
};

struct RaycastTarget {
	const std::shared_ptr<tics::MeshCollider> collider;
	std::shared_ptr<ron::MeshNode> mesh_node;
};

struct ProgramState {
	RaycastTarget raycast_target;
	RaycastTarget raycast_target_2;

	ron::PerspectiveCamera camera;
	ron::CameraViewportControls camera_controls;
	ron::Scene render_scene;
	std::unique_ptr<ron::OpenGLRenderer> renderer;

	tics::World physics_world;
};

static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
static ProgramState initialize(GLFWwindow* window);
static void process(GLFWwindow* window, ProgramState& state);
static void render(GLFWwindow* window, ProgramState& state);

int main() {
	glfwInit();

	// tell GLFW we are using OpenGL 4.6
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
	// tell GLFW we want to use the core-profile -> no backwards-compatible features
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(1280, 720, "Physics Playground", NULL, NULL);
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

	{ // this scope ensures that "state" is destroyed before the opengl context is destroyed (glfwTerminate)
		ProgramState state = initialize(window);
		glfwSetWindowUserPointer(window, static_cast<void *>(&state));

		GameLoop game_loop(
			[window, &state](const float delta, const float fixed_delay) { render(window, state); },
			[window, &state](const float delta) { process(window, state); },
			1.0f/60.0f, 0.0f, 1.0f/60.0f
		);

		auto last_title_update_time_point = std::chrono::high_resolution_clock::now();

		while (!glfwWindowShouldClose(window)) {
			game_loop.update();

			// measure performance
			static const auto update_interval = 250ms;
			const auto now = std::chrono::high_resolution_clock::now();
			if (now - last_title_update_time_point >= update_interval) {
				last_title_update_time_point = now;

				const auto avg_render_time = std::chrono::duration<double>(
					accumulated_render_time / accumulated_render_time_count
				);
				const double avg_render_time_ms = avg_render_time.count() * 1000.0;

				accumulated_render_time = 0ns;
				accumulated_render_time_count = 0;

				const auto avg_physics_time = std::chrono::duration<double>(
					accumulated_physics_time / accumulated_physics_time_count
				);
				const double avg_physics_time_ms = avg_physics_time.count() * 1000.0;

				accumulated_physics_time = 0ns;
				accumulated_physics_time_count = 0;

				std::stringstream title_stream;
				title_stream
					<< "Physics Playground"
					<< " - Render Time: "
					<< std::fixed << std::setprecision(2) // specify decimal places
					<< avg_render_time_ms << "ms"
					// << " - Render FPS: "
					// << static_cast<int>(1000.0 / avg_render_time_ms)
					<< " - Phyiscs Time: "
					<< std::fixed << std::setprecision(2) // specify decimal places
					<< avg_physics_time_ms << "ms"
					// << " - Physics FPS: "
					// << static_cast<int>(1000.0 / avg_physics_time_ms)
					;
				glfwSetWindowTitle(window,  + title_stream.str().c_str());
			}
		}
	}

	glfwTerminate();
	return 0;
}

ProgramState initialize(GLFWwindow* window) {
	// renderer setup
	auto scene = ron::Scene();
	scene.set_directional_light(create_generic_light());
	auto renderer = std::make_unique<ron::OpenGLRenderer>(1280, 720);
	renderer->render_axes = true;
	renderer->render_grid = true;
	renderer->set_clear_color(glm::vec4(0.1, 0.1, 0.1, 1.0));
	// camera setup
	const auto initial_camera_rotation = glm::vec2(glm::radians(-24.2f), glm::radians(63.6f));
	auto camera_controls = ron::CameraViewportControls(initial_camera_rotation);
	camera_controls.set_target(glm::vec3(0.0f, 0.0f, 0.0f));

	tics::World physics_world;
	physics_world.set_gravity(Terathon::Vector3D(0.0, -10.0, 0.0));

	// raycasting setup - first target
	RaycastTarget raycast_target = {
		std::make_shared<tics::MeshCollider>(),
		ron::gltf::import("models/suzanne_high_res.glb").get_mesh_nodes().front()
	};
	const auto raycast_target_geometry = raycast_target.mesh_node->get_mesh()->sections.front().geometry;
	// copy positions and inidices to MeshCollider
	raycast_target.collider->indices = raycast_target_geometry->indices;
	for (const auto &vertex_pos : raycast_target_geometry->positions) {
		raycast_target.collider->positions.push_back(Terathon::Vector3D(vertex_pos.x, vertex_pos.y, vertex_pos.z));
	}
	raycast_target.collider->edges = std::vector<Terathon::Line3D>(raycast_target.collider->indices.size() / 3);
	for (size_t triangle_index = 0; triangle_index < raycast_target.collider->indices.size() / 3; triangle_index++) {
		auto a = Terathon::Point3D(raycast_target.collider->positions.at(raycast_target.collider->indices.at(triangle_index * 3 + 0)));
		auto b = Terathon::Point3D(raycast_target.collider->positions.at(raycast_target.collider->indices.at(triangle_index * 3 + 1)));
		auto c = Terathon::Point3D(raycast_target.collider->positions.at(raycast_target.collider->indices.at(triangle_index * 3 + 2)));
		b -= a; c -= a;
		const auto e_bc = Terathon::Wedge(b, c);
		raycast_target.collider->edges.at(triangle_index) = e_bc;
	}
	scene.add(raycast_target.mesh_node);

	// raycasting setup - second target
	RaycastTarget raycast_target_2 = {
		std::make_shared<tics::MeshCollider>(),
		ron::gltf::import("models/triangle_1.glb").get_mesh_nodes().front()
	};
	const auto raycast_target_2_geometry = raycast_target_2.mesh_node->get_mesh()->sections.front().geometry;
	// copy positions and inidices to MeshCollider
	raycast_target_2.collider->indices = raycast_target_2_geometry->indices;
	for (const auto &vertex_pos : raycast_target_2_geometry->positions) {
		raycast_target_2.collider->positions.push_back(Terathon::Vector3D(vertex_pos.x, vertex_pos.y, vertex_pos.z));
	}
	// scene.add(raycast_target_2.mesh_node);

	renderer->preload(scene);

	glfwSetScrollCallback(window, scroll_callback);

	return ProgramState {
		raycast_target,
		raycast_target_2,
		ron::PerspectiveCamera(60.0f, 1280.0f/720.0f, 0.1f, 1000.0f),
		camera_controls,
		scene,
		std::move(renderer),
		physics_world,
	};
}

void process(GLFWwindow* window, ProgramState& state) {
	glfwPollEvents();

	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, true);
		return;
	}

	auto start_time_point = std::chrono::high_resolution_clock::now();

	static std::vector<std::chrono::nanoseconds> raycast_times;

	// const auto time_since_start = std::chrono::high_resolution_clock::now() - state.start_time_point;
	// const auto time_since_start_s = time_since_start.count() / 1000000000.0;
	// const auto cos_time = cos( 2 * time_since_start_s );

	const auto target_glm = state.camera_controls.get_target();
	
	auto glm_model_mat = state.raycast_target.mesh_node->get_model_matrix();
	// glm_model_mat[3][0] = target_glm.x + cos_time * 0.5;
	// glm_model_mat[3][1] = target_glm.y + cos_time * 0.5;
	// glm_model_mat[3][2] = target_glm.z;
	// state.raycast_target.mesh_node->set_model_matrix(glm_model_mat);
	const auto model_mat = Terathon::Transform3D(
		glm_model_mat[0][0], glm_model_mat[1][0], glm_model_mat[2][0], glm_model_mat[3][0],
		glm_model_mat[0][1], glm_model_mat[1][1], glm_model_mat[2][1], glm_model_mat[3][1],
		glm_model_mat[0][2], glm_model_mat[1][2], glm_model_mat[2][2], glm_model_mat[3][2]
	);
	auto model_motor = Terathon::Motor3D();
	model_motor.SetTransformMatrix(model_mat);

	auto glm_model_mat_2 = state.raycast_target_2.mesh_node->get_model_matrix();
	// glm_model_mat_2[3][0] = target_glm.x + cos_time * 0.5;
	// glm_model_mat_2[3][1] = target_glm.y + cos_time * 0.5;
	// glm_model_mat_2[3][2] = target_glm.z;
	// state.raycast_target_2.mesh_node->set_model_matrix(glm_model_mat_2);
	const auto model_mat_2 = Terathon::Transform3D(
		glm_model_mat_2[0][0], glm_model_mat_2[1][0], glm_model_mat_2[2][0], glm_model_mat_2[3][0],
		glm_model_mat_2[0][1], glm_model_mat_2[1][1], glm_model_mat_2[2][1], glm_model_mat_2[3][1],
		glm_model_mat_2[0][2], glm_model_mat_2[1][2], glm_model_mat_2[2][2], glm_model_mat_2[3][2]
	);
	auto model_motor_2 = Terathon::Motor3D();
	model_motor_2.SetTransformMatrix(model_mat_2);

	const auto cam_model_matrix = state.camera.get_model_matrix();
	const auto cam_pos_glm = cam_model_matrix * glm::vec4(0,0,0,1);
	const auto cam_pos = Terathon::Point3D(cam_pos_glm.x, cam_pos_glm.y, cam_pos_glm.z);

	// intersect only one triangle test
	{
		// auto target = Terathon::Point3D(target_glm.x, target_glm.y, target_glm.z);

		// const auto direction = Terathon::Normalize( target - Terathon::Point3D(cam_pos_glm.x, cam_pos_glm.y, cam_pos_glm.z) );
		// target = cam_pos + direction;

		// const bool pga_raycast_1_hit = tics::pga_raycast(model_motor, *state.raycast_target.collider, cam_pos, target);
		// const bool pga_raycast_2_hit = tics::pga_raycast(model_motor_2, *state.raycast_target_2.collider, cam_pos, target);

		// const bool raycast_1_hit = tics::raycast(model_mat, *state.raycast_target.collider, cam_pos, direction);
		// const bool raycast_2_hit = tics::raycast(model_mat_2, *state.raycast_target_2.collider, cam_pos, direction);

		// std::cout << "    raycast_hit: " << raycast_1_hit << ", " << raycast_2_hit << std::endl;
		// // assert(raycast_1_hit || raycast_2_hit);
		// static int raycast_fail = 0;
		// if (!(raycast_1_hit || raycast_2_hit)) {
		// 	raycast_fail++;
		// }
		// std::cout << "pga_raycast_hit: " << pga_raycast_1_hit << ", " << pga_raycast_2_hit << std::endl;
		// // assert(pga_raycast_1_hit || pga_raycast_2_hit);
		// static int pga_raycast_fail = 0;
		// if (!(pga_raycast_1_hit || pga_raycast_2_hit)) {
		// 	pga_raycast_fail++;
		// }
		// std::cout << "***** conventional " << raycast_fail << ":" << pga_raycast_fail << " pga *****" << std::endl;
	}

	// performance measuring
	if (pga) {
		// without transform:
		// suzanne.glb: ~ 0.50 ms
		// suzanne_high_res.glb: : ~ 90 ms

		// with transform:
		// suzanne.glb: ~ 1.21 ms (+0.71 ms)
		// suzanne_high_res.glb: : ~ 274 ms (+184 ms)

		// new test suzanne_high_res.glb, no transform, no camera movement, release build: 1787933ns, 1766558ns
		// without rendering: 1239489ns
		// windows: 2095463ns
		const auto target_pos_glm = cam_model_matrix * glm::vec4(0,0,-1,1);
		const auto direction_glm = target_pos_glm - cam_pos_glm;
		const auto direction = Terathon::Vector3D(direction_glm.x, direction_glm.y, direction_glm.z);

		const auto ga_start_time_point = std::chrono::high_resolution_clock::now();
		const bool ga_raycast_hit = tics::pga_raycast(
			*state.raycast_target.collider, cam_pos + Terathon::Vector3D(0,1,0), direction
		);
		const auto ray_cast_time = std::chrono::high_resolution_clock::now() - ga_start_time_point;
		raycast_times.push_back(ray_cast_time);
		std::cout << "ga_raycast_hit: " << ga_raycast_hit << "\n";
	} else {
		// without transform:
		// suzanne.glb: ~ 0.33 ms
		// suzanne_high_res.glb: ~ 53 ms

		// with transform:
		// suzanne.glb: ~ 0.75 ms (+0.42 ms)
		// suzanne_high_res.glb: : ~ 140 ms (+87 ms)

		// new test suzanne_high_res.glb, no transform, no camera movement, release build: 1669995ns, 1666801ns
		// without rendering: 1033106ns
		// windows: 1781122ns
		const auto target_pos_glm = cam_model_matrix * glm::vec4(0,0,-1,1);
		const auto direction_glm = target_pos_glm - cam_pos_glm;
		const auto direction = Terathon::Vector3D(direction_glm.x, direction_glm.y, direction_glm.z);

		const auto la_start_time_point = std::chrono::high_resolution_clock::now();
		const bool la_raycast_hit = tics::raycast(
			*state.raycast_target.collider, cam_pos + Terathon::Vector3D(0,1,0), direction
		);
		const auto ray_cast_time = std::chrono::high_resolution_clock::now() - la_start_time_point;
		raycast_times.push_back(ray_cast_time);
		std::cout << "la_raycast_hit: " << la_raycast_hit << "\n";
	}
	std::chrono::nanoseconds total = 0ns;
	for (const auto &t : raycast_times) {
		total += t;
	}
	std::cout << "avg time: " << total / raycast_times.size() << "\n";

	float time_scale = 1.0f;
	state.physics_world.update(time_scale/60.0f);

	const auto physics_time = std::chrono::high_resolution_clock::now() - start_time_point;
	++accumulated_physics_time_count;
	accumulated_physics_time += physics_time;

	state.camera_controls.update(*window, state.camera);
}

void render(GLFWwindow* window, ProgramState& state) {
	auto start_time_point = std::chrono::high_resolution_clock::now();

	state.renderer->render(state.render_scene, state.camera);

	const auto render_time = std::chrono::high_resolution_clock::now() - start_time_point;
	++accumulated_render_time_count;
	accumulated_render_time += render_time;

	glfwSwapBuffers(window);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
	auto state = static_cast<ProgramState *>(glfwGetWindowUserPointer(window));
	assert(state);

	state->renderer->resolution = glm::uvec2(width, height);

	state->camera.set_aspect_ratio(static_cast<float>(width) / static_cast<float>(height));
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
	auto state = static_cast<ProgramState *>(glfwGetWindowUserPointer(window));

	if (state) {
		if (yoffset == 1.0) {
			state->camera_controls.scroll_callback(ron::CameraViewportControls::UP);
		}
		else if (yoffset == -1.0) {
			state->camera_controls.scroll_callback(ron::CameraViewportControls::DOWN);
		}
	}
}
