#pragma once

#include <functional>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include "tics_math.h"

namespace tics {

struct Transform {
	tics_vec3 position = {0, 0, 0};
	tics_quat rotation = {0, 0, 0, 1};
	tics_vec3 get_position() const { return position; }
	tics_quat get_rotation() const { return rotation; }
};

enum ColliderType {
	SPHERE,
	PLANE,
	MESH,
};

struct Collider {
	ColliderType type;
};

struct SphereCollider : Collider {
	SphereCollider() { type = SPHERE; };
	tics_vec3 center = {0, 0, 0};
	float radius = 1.0f;
};

struct PlaneCollider : Collider {
	PlaneCollider() { type = PLANE; };
	tics_vec3 normal = {0, 1, 0};
	float distance = 0.0f;
};

struct MeshCollider : Collider {
	MeshCollider() { type = MESH; };
	std::vector<tics_vec3> positions = {};
};

struct CollisionPoints {
	// a and b are the points where each shape penetrates the other most
	tics_vec3 a;
	tics_vec3 b;
	tics_vec3 normal; // penetration vector direction
	float depth;	  // penetration vector length
	bool has_collision = false;
};

CollisionPoints collision_test(const Collider& a, const Transform& at, const Collider& b,
							   const Transform& bt);

struct Collision;

class ICollisionObject {
  public:
	virtual ~ICollisionObject() = default;

	virtual void set_collider(const std::weak_ptr<Collider> collider) = 0;
	virtual std::weak_ptr<Collider> get_collider() const = 0;

	virtual void set_transform(const std::weak_ptr<Transform> transform) = 0;
	virtual std::weak_ptr<Transform> get_transform() const = 0;
};

// A physics body that is not moved by physics simulation. RigidBodies can collide with it.
// When moved manually, it doesn't affect objects in its path.
class StaticBody : public ICollisionObject {
  public:
	virtual ~StaticBody() = default;
	virtual void set_collider(const std::weak_ptr<Collider> collider) override;
	virtual std::weak_ptr<Collider> get_collider() const override;
	virtual void set_transform(const std::weak_ptr<Transform> transform) override;
	virtual std::weak_ptr<Transform> get_transform() const override;

	float elasticity = 0.8f; // [0;1]
  private:
	std::weak_ptr<Collider> m_collider;
	std::weak_ptr<Transform> m_transform;
};

// A physics body that is moved by physics simulation.
class RigidBody : public ICollisionObject {
  public:
	virtual ~RigidBody() = default;
	virtual void set_collider(const std::weak_ptr<Collider> collider) override;
	virtual std::weak_ptr<Collider> get_collider() const override;
	virtual void set_transform(const std::weak_ptr<Transform> transform) override;
	virtual std::weak_ptr<Transform> get_transform() const override;

	tics_vec3 velocity = {0, 0, 0};
	// !! unit: rad / 0.01 s !!
	tics_quat angular_velocity = {0, 0, 0, 1};

	// accumulated, applied and reset every frame
	// an impulse is an instantaneous change in momentum
	tics_vec3 impulse = {0, 0, 0};
	// angular impulse (instantaneous change in angular momentum) divided by square distance to the
	// application pos
	tics_quat an_imp_div_sq_dst = {0, 0, 0, 1};

	float mass = 1.0f;
	float elasticity = 0.9f; // [0;1]
	float gravity_scale = 1.0f;

  private:
	std::weak_ptr<Collider> m_collider;
	std::weak_ptr<Transform> m_transform;
};

// A region that detects other CollisionAreas, RigidBodies and StaticBodies entering or exiting it
class CollisionArea : public ICollisionObject {
  public:
	virtual ~CollisionArea() = default;
	virtual void set_collider(const std::weak_ptr<Collider> collider) override;
	virtual std::weak_ptr<Collider> get_collider() const override;
	virtual void set_transform(const std::weak_ptr<Transform> transform) override;
	virtual std::weak_ptr<Transform> get_transform() const override;

	std::function<void(const std::weak_ptr<ICollisionObject> other, CollisionPoints collision_data)>
		on_collision_enter;
	std::function<void(const std::weak_ptr<ICollisionObject> other)> on_collision_exit;

  private:
	std::weak_ptr<Collider> m_collider;
	std::weak_ptr<Transform> m_transform;
};

struct Collision {
	const std::weak_ptr<ICollisionObject> a;
	const std::weak_ptr<ICollisionObject> b;
	const CollisionPoints points;
};

class ISolver {
  public:
	virtual ~ISolver(){};

	virtual void solve(const std::vector<Collision>& collisions, float delta) = 0;
};

class World {
  public:
	void add_object(const std::weak_ptr<ICollisionObject> object);
	void remove_object(const std::weak_ptr<ICollisionObject> object);

	void add_solver(const std::weak_ptr<ISolver> solver);
	void remove_solver(const std::weak_ptr<ISolver> solver);

	void update(const float delta);

	std::vector<Collision> collision_detection(const float delta);
	void collision_response(const float delta, const std::vector<Collision>& collisions);

	void set_gravity(const tics_vec3 gravity);
	void set_collision_event(const std::function<void(const Collision&)> collision_event);

  private:
	std::vector<std::weak_ptr<ICollisionObject>> m_objects;
	std::vector<std::weak_ptr<ISolver>> m_solvers;
	tics_vec3 m_gravity = {0.0, -9.81, 0.0};
	std::function<void(const Collision&)> m_collision_event;
};

class ImpulseSolver : public ISolver {
  public:
	~ImpulseSolver(){};

	virtual void solve(const std::vector<Collision>& collisions, float delta) override;
};

class NonIntersectionConstraintSolver : public ISolver {
  public:
	~NonIntersectionConstraintSolver(){};

	virtual void solve(const std::vector<Collision>& collisions, float delta) override;
};

struct ObjectAndCollisionData {
	std::weak_ptr<ICollisionObject> object;
	CollisionPoints collision_points;
	bool collision_points_swapped;
};

class CollisionAreaSolver : public ISolver {
  public:
	~CollisionAreaSolver(){};

	virtual void solve(const std::vector<Collision>& collisions, float delta) override;

  private:
	typedef std::map<CollisionArea*, std::vector<ObjectAndCollisionData>> AreasCollisionRecord;

	AreasCollisionRecord m_areas_collision_record = {};
};

} // namespace tics
