#ifndef TICS_INTERNAL_H
#define TICS_INTERNAL_H

#include "tics.h"
#include "tics_old.h"

#include <unordered_map>
#include <vector>

struct tics_world {
	std::unique_ptr<tics::World> cpp_world;

	// Resource Management:
	// We hold shared_ptrs here to ensure the resources stay alive
	// while the C API refers to them via integer IDs.

	// Map: Shape ID -> Old C++ Collider
	std::unordered_map<tics_shape_id, std::shared_ptr<tics::Collider>> shapes;
	// Map: Body ID -> Old C++ Collision Object (Base class)
	std::unordered_map<tics_body_id, std::shared_ptr<tics::ICollisionObject>> bodies;

	// Map: Body ID -> Old C++ Transforms
	// We must own the Transforms because the C++ API bodies only hold weak_ptrs.
	std::unordered_map<tics_body_id, std::shared_ptr<tics::Transform>> transforms;

	// Keep solvers alive
	std::vector<std::shared_ptr<tics::ISolver>> solvers;

	// Initialize to 1 (0 is invalid)
	uint32_t body_id_counter = 1;
	uint32_t next_body_id() { return body_id_counter++; }
	uint32_t shape_id_counter = 1;
	uint32_t next_shape_id() { return shape_id_counter++; }

	tics_world() { cpp_world = std::make_unique<tics::World>(); }
};

#endif // TICS_INTERNAL_H
