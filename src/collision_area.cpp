#include "tics.h"

using tics::CollisionArea;
using tics::Collider;
using tics::Transform;

void CollisionArea::set_collider(const std::weak_ptr<Collider> collider) {
	m_collider = collider;
}

std::weak_ptr<Collider> CollisionArea::get_collider() const {
	return m_collider;
}

void CollisionArea::set_transform(const std::weak_ptr<Transform> transform) {
	m_transform = transform;
}

std::weak_ptr<Transform> CollisionArea::get_transform() const {
	return m_transform;
}
