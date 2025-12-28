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
	physics_world.set_gravity(Terathon::Vector3D(0.0, -0.01, 0.0));

	auto spheres = std::make_shared<std::vector<Sphere>>();

	// initial sphere
	// auto sphere_1 = create_sphere(
	// 	Terathon::Vector3D(0.0, 1.5, 0.25), Terathon::Vector3D(0.0, 8.0, 0.0),
	// 	Terathon::Quaternion::identity,
	// 	*scene, glm::vec3(0.2, 0.2, 0.8)
	// );
// #ifdef TICS_GA
// 	sphere_1.transform->motor = sphere_1.transform->motor * Terathon::Quaternion::MakeRotationZ(3.141 * 0.05);
// #else
// 	sphere_1.transform->rotation = Terathon::Quaternion::MakeRotationZ(3.141 * 0.05);
// #endif
// 	spheres->emplace_back(sphere_1);
// 	physics_world.add_object(sphere_1.rigid_body);
// 	scene->add(sphere_1.mesh_node);

	const Terathon::Vector3D positions [10] = {
		Terathon::Vector3D( 1,1+2, 1) * 1.2f,
		Terathon::Vector3D( 2,2+2, 2) * 1.2f,
		Terathon::Vector3D(-3,3+2, 3) * 1.2f,
		Terathon::Vector3D(-4,1+2, 4) * 1.2f,
		Terathon::Vector3D(-3,2+2, 3) * 1.2f,
		Terathon::Vector3D(-1,3+2,-1) * 1.2f,
		Terathon::Vector3D(-2,1+2,-2) * 1.2f,
		Terathon::Vector3D( 3,2+2,-3) * 1.2f,
		Terathon::Vector3D( 4,3+2,-4) * 1.2f,
		Terathon::Vector3D( 3,1+2,-2) * 1.2f
	};

	const float scales [10] = {
		0.9,
		1.3,
		2.0,
		1.2,
		1.3,
		0.8,
		0.9,
		1.3,
		1.7,
		1.2
	};

	const float elasticities [10] = {
		0.9,
		0.9,
		0.8,
		0.85,
		0.8,
		0.95,
		1.0,
		0.75,
		0.8,
		0.95
	};
	// windows compiler version \MSVC\14.37.32822

	// 10x simplex 10s -- ga d: 2032ns, cd: 90884ns, cr: 5033ns
	// 10x simplex 10s -- la d: 1966ns, cd: 81693ns, cr: 4067ns

	// 20xSimplex10sLA: la d: 3266ns, cd: 196279ns, cr: 6618ns
	// 20xSimplex10sGA: ga d: 2840ns, cd: 224019ns, cr: 7744ns
	// 21xSimplex10sLA: la d: 3458ns, cd: 213139ns, cr: 6959ns
	// 21xSimplex10sGA: ga d: 2760ns, cd: 225415ns, cr: 8026ns
	// 21xIcosphere10sLA: la d: 3285ns, cd: 287990ns, cr: 7634ns
	// 21xIcosphere10sGA: ga d: 2872ns, cd: 313519ns, cr: 9549ns
	// 100xIcosphere10sGA: ga d: 8836ns, cd: 3017671ns, cr: 46996ns
	// 100xIcosphere10sLA: la d: 11055ns, cd: 3074072ns, cr: 44397ns
	// 100xIcosphere10sLA_CDGA: la d: 10370ns, cd: 2785935ns, cr: 41888ns
	// 							la d: 11387ns, cd: 3032901ns, cr: 45025ns
	// 							la d: 11439ns, cd: 3011314ns, cr: 44895ns
	// 100xIcosphereHighres10sGA: 	ga d: 9473ns, cd: 4459003ns, cr: 52146ns
	// 								ga d: 9371ns, cd: 4302813ns, cr: 50911ns
	// 100xIcosphereHighres10sLA: 	la d: 11205ns, cd: 4415956ns, cr: 42248ns
	// 21xIcosphereHighres20sNoCrGA: 	ga d: 2939ns, cd: 368144ns, cr: 30ns
	// 							debug	ga d: 33487ns, cd: 4234190ns, cr: 25ns
	// 21xIcosphereHighres20sNoCrLA: 	la d: 3455ns, cd: 361200ns, cr: 30ns
	// 							debug	la d: 27099ns, cd: 3980621ns, cr: 24ns
	// 21xIcosphereHighres20sNoCr2GA: 	ga d: 2743ns, cd: 459380ns, cr: 27ns
	// 						windows		ga d: 2929ns, cd: 531053ns, cr: 24ns
	// 						windows		ga d: 2769ns, cd: 496519ns, cr: 23ns
	// 21xIcosphereHighres20sNoCr2LA: 	la d: 3335ns, cd: 495508ns, cr: 29ns
	// 						windows		la d: 2375ns, cd: 465945ns, cr: 22ns
	// 						windows		la d: 2421ns, cd: 525230ns, cr: 23ns
	//						windows 	la d: 2498ns, cd: 479107ns, cr: 23ns
	// RaycastMissLowresLA: 		avg time:   11416ns
	// 					windows 	avg time:   14805ns
	// RaycastMissLowresGA: 		avg time:   11941ns
	// 					windows 	avg time:   15924ns
	// RaycastMissLowresGAPreEdge: 	avg time:   12118ns
	// 					windows 	avg time:   14374ns

	// --- RELEVANT TESTS BELOW ---

	// RaycastHitLA: 				avg time: 1042900ns
	// 					windows 	avg time: 1175602ns
	// RaycastHitGA: 				avg time: 1237085ns
	// 					windows 	avg time: 1381974ns
	// RaycastHitGAPreEdge: 		avg time: 1209924ns
	// 					windows 	avg time: 1273892ns

	// Dyn300x20sLA					:	la d: 35896ns, cd: 29ns, cr: 26ns
	// 									la d: 35969ns, cd: 29ns, cr: 25ns
	// 						windows		la d: 47859ns, cd: 22ns, cr: 23ns
	// Dyn300x20sGA					:	ga d: 40653ns, cd: 31ns, cr: 26ns
	// 									ga d: 42067ns, cd: 30ns, cr: 25ns
	// 						windows		ga d: 48709ns, cd: 23ns, cr: 22ns

	// 21xIcosphereHighres20sGA: 	ga d: 2770ns, cd: 404380ns, cr: 11554ns
	// 								ga d: 2797ns, cd: 418571ns, cr: 11653ns
	// 								ga d: 2819ns, cd: 422945ns, cr: 11691ns
	//					windows		ga d: 2557ns, cd: 336512ns, cr: 9572ns
	// 21xIcosphereHighres20sLA: 	la d: 3320ns, cd: 414524ns, cr: 9496ns
	// 								la d: 3233ns, cd: 408939ns, cr: 9569ns
	// 								la d: 3231ns, cd: 407854ns, cr: 9293ns
	//					windows		la d: 2228ns, cd: 336911ns, cr: 7229ns
	//					windows		la d: 2288ns, cd: 341886ns, cr: 7338ns

	// create some spheres
	for (size_t i = 0; i < 300; i++) {
		auto sphere = create_sphere(
			positions[i % 10] + Terathon::Vector3D(0,i/10,0)*2.0f,
			Terathon::Vector3D(0,0,0), // velocity
			Terathon::Quaternion::MakeRotation(0.1, !Terathon::Normalize(positions[i%10])),
			*scene,
			glm::vec3( // color
				static_cast<double>(std::rand()) / RAND_MAX,
				static_cast<double>(std::rand()) / RAND_MAX,
				static_cast<double>(std::rand()) / RAND_MAX
			),
			scales[i % 10],
			elasticities[i % 10]
		);
		spheres->emplace_back(sphere);
		physics_world.add_object(sphere.rigid_body);
		scene->add(sphere.mesh_node);
	}

	// add static geometry
	auto static_geometry = create_static_objects("models/ground_smooth.glb");
	for (const auto &static_object : *static_geometry) {
		physics_world.add_object(static_object.static_body);
		scene->add(ron::gltf::import("models/ground.glb"));
	}

	// area trigger that turns objects red that are inside it
	AreaTrigger area_trigger = {
		std::make_shared<tics::CollisionArea>(),
		std::make_shared<tics::Transform>(),
		std::make_shared<tics::MeshCollider>(),
		ron::gltf::import("models/rectangle.glb").get_mesh_nodes().front()
	};
	area_trigger.area->set_collider(area_trigger.collider);
	area_trigger.area->set_transform(area_trigger.transform);
	area_trigger.area->on_collision_enter = [spheres, scene](const auto &other, const auto &collision_data) {
		std::cout << "collision_data: normal: {"
			<< collision_data.normal.x << "," << collision_data.normal.y << "," << collision_data.normal.z << "}"
			<< " depth: " << collision_data.depth << "\n";
		const auto debug_sphere = create_sphere(
			collision_data.a, Terathon::Vector3D(0,0,0),
			Terathon::Quaternion(1), *scene, {1,1,0}, 0.1
		);
		scene->add(debug_sphere.mesh_node);
		for (const auto &sphere : *spheres) {
			if (sphere.rigid_body == other.lock()) {
				sphere.mesh_node->get_mesh()->sections.front().material->uniforms["albedo_color"]
					= ron::make_uniform(glm::vec4(glm::vec3(1.0, 0.1, 0.1), 1.0));
			}
		}
	};
	area_trigger.area->on_collision_exit = [spheres](const auto &other) {
		for (const auto &sphere : *spheres) {
			if (sphere.rigid_body == other.lock()) {
				sphere.mesh_node->get_mesh()->sections.front().material->uniforms["albedo_color"]
					= ron::make_uniform(glm::vec4(sphere.color, 1.0));
			}
		}
	};
	// apply scale to mesh
	for (auto &section : area_trigger.mesh_node->get_mesh()->sections) {
		for (auto &position : section.geometry->positions) {
			position *= glm::vec3(2.0, 1.0, 2.0);
		}
	}

#ifdef TICS_GA
	area_trigger.transform->motor = Terathon::Motor3D::MakeTranslation(Terathon::Vector3D(-2.0, 2.0, 0.0));
#else
	area_trigger.transform->position = Terathon::Vector3D(-2.0, 2.0, 0.0);
#endif
	area_trigger.mesh_node->set_model_matrix(transform_to_model_matrix(*area_trigger.transform));
	const auto area_trigger_geometry = area_trigger.mesh_node->get_mesh()->sections.front().geometry;
	// copy positions and inidices to MeshCollider
	area_trigger.collider->indices = area_trigger_geometry->indices;
	for (const auto &vertex_pos : area_trigger_geometry->positions) {
		area_trigger.collider->positions.push_back(Terathon::Vector3D(vertex_pos.x, vertex_pos.y, vertex_pos.z));
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
	// physics_world.add_solver(collision_area_solver);

	renderer->preload(*scene);

	glfwSetScrollCallback(window, scroll_callback);

	return ProgramState {
		spheres,
		static_geometry,
		area_trigger,
		ron::PerspectiveCamera(60.0f, (float)(res_x)/(float)(res_y), 0.1f, 1000.0f),
		camera_controls,
		scene,
		std::move(renderer),
		physics_world,
		impulse_solver,
		position_solver,
		collision_area_solver
	};
}

void process(GLFWwindow* window, ProgramState& state) {
	glfwPollEvents();

	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, true);
		return;
	}

	auto start_time_point = std::chrono::high_resolution_clock::now();

	// add random impulses for dynamics benchmark
	for (auto &sphere : *state.spheres) {
		sphere.rigid_body->an_imp_div_sq_dst = Terathon::Quaternion::MakeRotation(
			(static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * 0.1f,
			Terathon::Normalize(Terathon::Bivector3D(
				static_cast<float>(std::rand()) / RAND_MAX,
				static_cast<float>(std::rand()) / RAND_MAX,
				static_cast<float>(std::rand()) / RAND_MAX
			))
		);
		sphere.rigid_body->impulse = Terathon::Vector3D(
			(static_cast<float>(std::rand()) / RAND_MAX) * 2.0f - 1.0f,
			(static_cast<float>(std::rand()) / RAND_MAX) * 2.0f - 1.0f,
			(static_cast<float>(std::rand()) / RAND_MAX) * 2.0f - 1.0f
		) * 0.3f;
	}

	// create spheres with random size and color
	// if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT == GLFW_PRESS)) {
	// 	auto sphere = create_sphere(
	// 		Terathon::Vector3D( // position
	// 			(0.5-static_cast<double>(std::rand()) / RAND_MAX)*1.0,
	// 			(0.5-static_cast<double>(std::rand()) / RAND_MAX) + 3.0,
	// 			(0.5-static_cast<double>(std::rand()) / RAND_MAX)*1.0
	// 		),
	// 		Terathon::Vector3D( // velocity
	// 			(0.5-static_cast<double>(std::rand()) / RAND_MAX)*5,
	// 			(    static_cast<double>(std::rand()) / RAND_MAX)*5,
	// 			(0.5-static_cast<double>(std::rand()) / RAND_MAX)*5
	// 		),
	// 		Terathon::Quaternion::identity, // angular velocity
	// 		// Terathon::Quaternion::MakeRotation(
	// 		// 	//angle
	// 		// 	3.141 * 2 * static_cast<float>(std::rand()) / RAND_MAX * 0.2,
	// 		// 	// axis
	// 		// 	!Terathon::Normalize(Terathon::Vector3D(
	// 		// 		static_cast<float>(std::rand()) / RAND_MAX,
	// 		// 		static_cast<float>(std::rand()) / RAND_MAX,
	// 		// 		static_cast<float>(std::rand()) / RAND_MAX
	// 		// 	))
	// 		// ),
	// 		*(state.render_scene),
	// 		glm::vec3( // color
	// 			static_cast<double>(std::rand()) / RAND_MAX,
	// 			static_cast<double>(std::rand()) / RAND_MAX,
	// 			static_cast<double>(std::rand()) / RAND_MAX
	// 		),
	// 		// scale
	// 		0.5 + 0.5 * static_cast<double>(std::rand()) / RAND_MAX,
	// 		0.8 + 0.2 * static_cast<double>(std::rand()) / RAND_MAX
	// 	);

	// 	state.spheres->emplace_back(sphere);
	// 	state.physics_world.add_object(sphere.rigid_body);
	// 	state.render_scene->add(sphere.mesh_node);
	// }


	const float time_scale = 1.0f;
	const float delta = time_scale/60.0f;

	const float time_limit = 20.0f; // seconds
	static float total_time = 0;

	if (total_time < time_limit) {
		state.physics_world.update(delta);
		total_time += delta;
	}

	const auto physics_time = std::chrono::high_resolution_clock::now() - start_time_point;
	++accumulated_physics_time_count;
	accumulated_physics_time += physics_time;

	for (const auto &sphere : *state.spheres) {
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
