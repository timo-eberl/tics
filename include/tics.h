#pragma once

#include <vector>
#include <unordered_map>
#include <map>
#include <memory>
#include <functional>

#include <TSVector3D.h>
#include <TSMatrix3D.h>
#include <TSMatrix4D.h>
#include <TSMotor3D.h>

namespace tics {

#ifdef TICS_GA
#else
#endif

struct Transform {
	#ifdef TICS_GA
		Terathon::Motor3D motor = Terathon::Motor3D::identity;
		Terathon::Point3D get_position() const { return motor.GetPosition(); }
		Terathon::Quaternion get_rotation() const { return motor.v; }
	#else
		Terathon::Vector3D position = Terathon::Vector3D(0,0,0);
		Terathon::Quaternion rotation = Terathon::Quaternion::identity;
		Terathon::Vector3D get_position() const { return position; }
		Terathon::Quaternion get_rotation() const { return rotation; }
	#endif
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
	Terathon::Vector3D center = Terathon::Vector3D(0, 0, 0);
	float radius = 1.0f;
};

struct PlaneCollider : Collider {
	PlaneCollider() { type = PLANE; };
	Terathon::Vector3D normal = Terathon::Vector3D(0, 1, 0);
	float distance = 0.0f;
};

struct MeshCollider : Collider {
	MeshCollider() { type = MESH; };
	std::vector<Terathon::Vector3D> positions = {};
	std::vector<uint32_t> indices = {};
	std::vector<Terathon::Line3D> edges = {};
};

struct eafds {
	std::vector<Terathon::Vector3D> positions = {};
	std::vector<uint32_t> indices = {};
	std::vector<Terathon::Line3D> edges = {};
};

struct gasfdas {
	std::vector<Terathon::Vector3D> positions = {};
	std::vector<uint32_t> indices = {};
};

static void kdfak() {
	sizeof(eafds);
	sizeof(gasfdas);
}

bool pga_raycast(const MeshCollider &mesh_collider, const Terathon::Point3D ray_start, const Terathon::Vector3D direction);

bool raycast(const MeshCollider &mesh_collider, const Terathon::Vector3D ray_start, const Terathon::Vector3D direction);

struct CollisionPoints {
	// a and b are the points where each shape penetrates the other most
	// a is the point on shape a that is farthest in shape b
	// the penetration vector points from a to b / from shape b to shape a
	Terathon::Vector3D a;
	Terathon::Vector3D b;
	Terathon::Vector3D normal; // penetration vector direction
	float depth; // penetration vector length
	bool has_collision = false;
};

CollisionPoints collision_test(
	const Collider& a, const Transform& at,
	const Collider& b, const Transform& bt
);

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

	Terathon::Motor3D motor_velocity = Terathon::Motor3D::identity;

	Terathon::Vector3D velocity = Terathon::Vector3D(0,0,0);
	// !! unit: rad / 0.01 s !!
	Terathon::Quaternion angular_velocity = Terathon::Quaternion::identity;

	// accumulated, applied and reset every frame
	// an impulse is an instantaneous change in momentum
	Terathon::Vector3D impulse = Terathon::Vector3D(0,0,0);
	// angular impulse (instantaneous change in angular momentum) divided by square distance to the application pos
	// !! unit: kg*rad / 0.01 s !!
	Terathon::Quaternion an_imp_div_sq_dst = Terathon::Quaternion::identity;

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

	std::function<void(const std::weak_ptr<ICollisionObject> other, CollisionPoints collision_data)> on_collision_enter;
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
	virtual ~ISolver() {};

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
	void collision_response(const float delta, const std::vector<Collision> &collisions);

	void set_gravity(const Terathon::Vector3D gravity);
	void set_collision_event(const std::function<void(const Collision&)> collision_event);
private:
	std::vector<std::weak_ptr<ICollisionObject>> m_objects;
	std::vector<std::weak_ptr<ISolver>> m_solvers;
	Terathon::Vector3D m_gravity = Terathon::Vector3D(0.0, -9.81, 0.0);
	std::function<void(const Collision&)> m_collision_event;
};

class ImpulseSolver : public ISolver {
public:
	~ImpulseSolver() {};

	virtual void solve(const std::vector<Collision>& collisions, float delta) override;
};

class NonIntersectionConstraintSolver : public ISolver {
public:
	~NonIntersectionConstraintSolver() {};

	virtual void solve(const std::vector<Collision>& collisions, float delta) override;
};

struct ObjectAndCollisionData {
	std::weak_ptr<ICollisionObject> object;
	CollisionPoints collision_points;
	bool collision_points_swapped;
};

class CollisionAreaSolver : public ISolver {
public:
	~CollisionAreaSolver() {};

	virtual void solve(const std::vector<Collision>& collisions, float delta) override;
private:
	typedef std::map< CollisionArea *, std::vector<ObjectAndCollisionData> >
		AreasCollisionRecord;

	AreasCollisionRecord m_areas_collision_record = {};
};

} // tics
