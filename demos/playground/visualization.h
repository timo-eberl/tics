#pragma once

#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

// clang-format off
#include <glad/glad.h>
#include <GLFW/glfw3.h> // include glfw after glad
// clang-format on
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <ron.h>
#include <tics.h> // For Transform mapping

// Helper struct for imported mesh data (visuals + collider vertices)
struct ImportedMesh {
	std::shared_ptr<ron::MeshNode> node;
	std::vector<tics_vec3> vertices;
	tics_vec3 position; // Initial position from GLTF
};

class Visualization {
  public:
	static Visualization initialize(int width, int height, const char* title) {
		Visualization viz;

		glfwInit();
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

		viz.window = glfwCreateWindow(width, height, title, NULL, NULL);
		if (!viz.window) {
			std::cerr << "Failed to create GLFW window" << std::endl;
			glfwTerminate();
			exit(-1);
		}
		glfwMakeContextCurrent(viz.window);
		gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

		glfwSetWindowUserPointer(viz.window, &viz);
		glfwSetFramebufferSizeCallback(viz.window, framebuffer_size_callback);
		glfwSetScrollCallback(viz.window, scroll_callback);

		// Renderer Setup
		viz.scene = std::make_shared<ron::Scene>();
		viz.setup_light();

		viz.renderer = std::make_unique<ron::OpenGLRenderer>(width, height);
		viz.renderer->set_clear_color(glm::vec4(0.1, 0.1, 0.1, 1.0));

		// Camera Setup
		viz.camera = ron::PerspectiveCamera(60.0f, (float)width / (float)height, 0.1f, 1000.0f);
		viz.camera_controls =
			ron::CameraViewportControls(glm::vec2(glm::radians(-24.2f), glm::radians(63.6f)));
		viz.camera_controls.set_target(glm::vec3(0.0f));

		return viz;
	}

	bool should_close() {
		return glfwWindowShouldClose(window) || glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS;
	}

	void update_and_render() {
		glfwPollEvents();
		camera_controls.update(*window, camera);
		renderer->render(*scene, camera);
		glfwSwapBuffers(window);
	}

	// --- Visual Helpers ---

	std::shared_ptr<ron::MeshNode> create_sphere_node(float radius, tics_vec3 color) {
		// Import generic sphere
		auto node = ron::gltf::import("models/icosphere.glb").get_mesh_nodes().front();

		// Scale visuals
		for (auto& section : node->get_mesh()->sections) {
			for (auto& pos : section.geometry->positions)
				pos *= radius;
		}

		// Create unique material
		auto material = std::make_shared<ron::Material>(*scene->default_material);
		material->uniforms["albedo_color"] =
			ron::make_uniform(glm::vec4(color.x, color.y, color.z, 1.0));

		// Clone mesh node to apply material
		auto cloned_node = std::make_shared<ron::MeshNode>(
			std::make_shared<ron::Mesh>(*node->get_mesh()), glm::mat4(1.0f));
		cloned_node->get_mesh()->sections.front().material = material;

		scene->add(cloned_node);
		return cloned_node;
	}

	std::vector<ImportedMesh> import_objects(const std::string& filepath) {
		printf("Importing %s...\n", filepath.c_str());
		std::vector<ImportedMesh> result;
		auto nodes = ron::gltf::import(filepath).get_mesh_nodes();

		for (auto& node : nodes) {
			ImportedMesh item;
			item.node = node; // Use the imported node directly

			// Extract position from GLTF matrix
			auto model_mat = node->get_model_matrix();
			glm::vec4 center = model_mat * glm::vec4(0, 0, 0, 1);
			item.position = {center.x, center.y, center.z};

			// Extract vertices for collision
			const auto& geometry = node->get_mesh()->sections.front().geometry;
			for (const auto& pos : geometry->positions) {
				item.vertices.push_back({pos.x, pos.y, pos.z});
			}
			result.push_back(item);
		}
		return result;
	}

	void add_scenery(const std::string& filepath) { scene->add(ron::gltf::import(filepath)); }

	void add_node(std::shared_ptr<ron::MeshNode> node) { scene->add(node); }

	void set_node_position(std::shared_ptr<ron::MeshNode> node, tics_vec3 pos) {
		node->set_model_matrix(
			glm::translate(glm::mat4(1.0f), glm::vec3(pos.x, pos.y, pos.z)));
	}

	// Syncs a visual mesh to a physics transform
	void sync_transform(const tics_transform& t_trans, std::shared_ptr<ron::MeshNode> v_node) {
		glm::mat4 trans = glm::translate(
			glm::mat4(1.0f), glm::vec3(t_trans.position.x, t_trans.position.y, t_trans.position.z));

		// Convert quaternion
		tics_quat q = t_trans.rotation;
		glm::quat glm_q(q.w, q.x, q.y, q.z);
		glm::mat4 rot = glm::mat4_cast(glm_q);

		v_node->set_model_matrix(trans * rot);
	}

	void set_node_color(std::shared_ptr<ron::MeshNode> node, tics_vec3 color) {
		node->get_mesh()->sections.front().material->uniforms["albedo_color"] =
			ron::make_uniform(glm::vec4(color.x, color.y, color.z, 1.0));
	}

	tics_vec3 random_color() {
		return {(float)rand() / (float)RAND_MAX, (float)rand() / (float)RAND_MAX,
				(float)rand() / (float)RAND_MAX};
	}

  private:
	Visualization() = default;

	GLFWwindow* window = nullptr;
	std::shared_ptr<ron::Scene> scene;
	std::unique_ptr<ron::OpenGLRenderer> renderer;
	ron::PerspectiveCamera camera;
	ron::CameraViewportControls camera_controls;

	void setup_light() {
		ron::DirectionalLight light = {};
		light.use_custom_shadow_target_world_position = true;
		light.world_direction = glm::normalize(glm::vec3(0.1f, 1.0f, 0.0f));
		light.shadow.enabled = true;
		light.shadow.map_size = glm::uvec2(2048);
		light.shadow.bias = 0.05f;
		light.shadow.far = 30.0f;
		light.shadow.frustum_size = 50.0f;
		scene->set_directional_light(light);
	}

	static void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
		auto* viz = (Visualization*)glfwGetWindowUserPointer(window);
		viz->renderer->resolution = glm::uvec2(width, height);
		viz->camera.set_aspect_ratio((float)width / (float)height);
	}

	static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
		auto* viz = (Visualization*)glfwGetWindowUserPointer(window);
		if (yoffset == 1.0) viz->camera_controls.scroll_callback(ron::CameraViewportControls::UP);
		else if (yoffset == -1.0)
			viz->camera_controls.scroll_callback(ron::CameraViewportControls::DOWN);
	}
};
