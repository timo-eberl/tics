#include "tics.h"

using tics::RigidBody;
using tics::Collider;
using tics::Transform;

void RigidBody::set_collider(const std::weak_ptr<Collider> collider) {
	m_collider = collider;
}

std::weak_ptr<Collider> RigidBody::get_collider() const {
	return m_collider;
}

void RigidBody::set_transform(const std::weak_ptr<Transform> transform) {
	m_transform = transform;
}

std::weak_ptr<Transform> RigidBody::get_transform() const {
	return m_transform;
}
