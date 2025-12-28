#include "tics.h"

using tics::StaticBody;
using tics::Collider;
using tics::Transform;

void StaticBody::set_collider(const std::weak_ptr<Collider> collider) {
	m_collider = collider;
}

std::weak_ptr<Collider> StaticBody::get_collider() const {
	return m_collider;
}

void StaticBody::set_transform(const std::weak_ptr<Transform> transform) {
	m_transform = transform;
}

std::weak_ptr<Transform> StaticBody::get_transform() const {
	return m_transform;
}
