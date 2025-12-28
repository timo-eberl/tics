#pragma once

#include <tics.h>
#include <ron.h>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <TSVector3D.h>
#include <TSMatrix3D.h>
#include <TSMatrix4D.h>
#include <TSMotor3D.h>

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

glm::mat4 transform_to_model_matrix(const tics::Transform &transform) {
#ifdef TICS_GA
	const auto t_mat = transform.motor.GetTransformMatrix();
	float m00 = t_mat(0,0); float m01 = t_mat(0,1); float m02 = t_mat(0,2); float m03 = t_mat(0,3);
	float m10 = t_mat(1,0); float m11 = t_mat(1,1); float m12 = t_mat(1,2); float m13 = t_mat(1,3);
	float m20 = t_mat(2,0); float m21 = t_mat(2,1); float m22 = t_mat(2,2); float m23 = t_mat(2,3);
	float m30 = t_mat(3,0); float m31 = t_mat(3,1); float m32 = t_mat(3,2); float m33 = t_mat(3,3);
	float aaa[16];
	aaa[   0] = m00; aaa[   1] = m10; aaa[   2] = m20; aaa[   3] = m30;
	aaa[ 4+0] = m01; aaa[ 4+1] = m11; aaa[ 4+2] = m21; aaa[ 4+3] = m31;
	aaa[ 8+0] = m02; aaa[ 8+1] = m12; aaa[ 8+2] = m22; aaa[ 8+3] = m32;
	aaa[12+0] = m03; aaa[12+1] = m13; aaa[12+2] = m23; aaa[12+3] = m33;

	return glm::make_mat4(aaa);
#else
	const auto transl_mat = glm::translate(glm::identity<glm::mat4>(), glm::vec3(
		transform.position.x, transform.position.y, transform.position.z
	));

	const auto t_rot_mat = transform.rotation.GetRotationMatrix();
	float m00 = t_rot_mat(0,0); float m01 = t_rot_mat(0,1); float m02 = t_rot_mat(0,2);
	float m10 = t_rot_mat(1,0); float m11 = t_rot_mat(1,1); float m12 = t_rot_mat(1,2);
	float m20 = t_rot_mat(2,0); float m21 = t_rot_mat(2,1); float m22 = t_rot_mat(2,2);
	float aaa[16];
	aaa[   0] = m00; aaa[   1] = m10; aaa[   2] = m20; aaa[   3] = 0;
	aaa[ 4+0] = m01; aaa[ 4+1] = m11; aaa[ 4+2] = m21; aaa[ 4+3] = 0;
	aaa[ 8+0] = m02; aaa[ 8+1] = m12; aaa[ 8+2] = m22; aaa[ 8+3] = 0;
	aaa[12+0] =   0; aaa[12+1] =   0; aaa[12+2] =   0; aaa[12+3] = 1;
	auto rot_mat = glm::make_mat4(aaa);

	return transl_mat * rot_mat;
#endif
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

Sphere create_sphere(
	Terathon::Vector3D position, Terathon::Vector3D velocity, Terathon::Quaternion angular_velocity,
	const ron::Scene &scene,
	glm::vec3 color = glm::vec3(1.0), float scale = 1.0f, float elasticity = 0.9f
) {
	Sphere sphere = Sphere({
		std::make_shared<tics::RigidBody>(),
		std::make_shared<tics::Transform>(),
		std::make_shared<tics::MeshCollider>(),
		ron::gltf::import("models/cube.glb").get_mesh_nodes().front(),
	});

	sphere.rigid_body->set_collider(sphere.collider);
	sphere.rigid_body->set_transform(sphere.transform);
	sphere.rigid_body->mass = scale*scale*scale * 2.0;
	sphere.rigid_body->velocity = velocity;
	sphere.rigid_body->angular_velocity = angular_velocity;
	sphere.rigid_body->elasticity = elasticity;
	sphere.color = color;
#ifdef TICS_GA
	sphere.transform->motor = Terathon::Motor3D::MakeTranslation(position);
#else
	sphere.transform->position = position;
#endif

	// apply scale to visual mesh
	for (auto &section : sphere.mesh_node->get_mesh()->sections) {
		for (auto &position : section.geometry->positions) {
			position *= scale;
		}
	}
	// copy the default material and modify it
	const auto material = std::make_shared<ron::Material>(*scene.default_material);
	material->uniforms["albedo_color"] = ron::make_uniform(glm::vec4(color, 1.0));
	// copy the mesh node to set its color
	const auto cloned_mesh_node = std::make_shared<ron::MeshNode>(
		std::make_shared<ron::Mesh>(*sphere.mesh_node->get_mesh()),
		transform_to_model_matrix(*sphere.transform)
	);
	cloned_mesh_node->get_mesh()->sections.front().material = material;
	sphere.mesh_node = cloned_mesh_node;

	const auto geometry = ron::gltf::import("models/icosphere_smooth.glb").get_mesh_nodes()
		.front()->get_mesh()->sections.front().geometry;
	// apply scale to collision mesh
	for (auto &position : geometry->positions) {
		position *= scale;
	}
	// copy positions and inidices to MeshCollider
	sphere.collider->indices = geometry->indices;
	for (const auto &vertex_pos : geometry->positions) {
		sphere.collider->positions.push_back(Terathon::Vector3D(vertex_pos.x, vertex_pos.y, vertex_pos.z));
	}

	return sphere;
}

std::shared_ptr<std::vector<StaticObject>> create_static_objects(const std::string& gltf_path) {
	auto objects = std::make_shared<std::vector<StaticObject>>();
	const auto mesh_nodes = ron::gltf::import(gltf_path).get_mesh_nodes();

	for (auto &mesh_node : mesh_nodes) {
		StaticObject static_object {
			std::make_shared<tics::StaticBody>(),
			std::make_shared<tics::Transform>(),
			std::make_shared<tics::MeshCollider>(),
		};

		const auto center = mesh_node->get_model_matrix() * glm::vec4(0,0,0,1);
#ifdef TICS_GA
	static_object.transform->motor = Terathon::Motor3D::MakeTranslation(Terathon::Vector3D(center.x,center.y,center.z));
#else
	static_object.transform->position = Terathon::Vector3D(center.x,center.y,center.z);
#endif

		const auto ground_geometry = mesh_node->get_mesh()->sections.front().geometry;
		static_object.collider->indices = ground_geometry->indices;
		for (const auto &vertex_pos : ground_geometry->positions) {
			static_object.collider->positions.push_back(Terathon::Vector3D(vertex_pos.x, vertex_pos.y, vertex_pos.z));
		}
		static_object.static_body->set_collider(static_object.collider);
		static_object.static_body->set_transform(static_object.transform);

		objects->emplace_back(static_object);
	}

	return objects;
}
