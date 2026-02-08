#include "tics.h"

#include <pcg.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// --- Configuration ---
#define ISLAND_COUNT 10 // world size will adjust with this
#define STEPS 600	   // 10 seconds of simulation

// --- Scene Layout Constants ---
#define ISLAND_SPACING_X 300.0f
#define ISLAND_OFFSET_Z 200.0f
#define JITTER_POS 60.0f // Random offset for island centers

// --- Object Dimensions & Counts ---
// 1 Monolith (50m), 49 Crates (5m), 450 Debris (0.5m)
#define MONOLITH_SIZE 50.0f
#define CRATE_SIZE 5.0f
#define DEBRIS_SIZE 0.5f

#define CRATE_COUNT 49
#define DEBRIS_COUNT 450

// Interpenetration depth to ensure collision detection triggers immediately
#define GROUND_EPSILON 0.01f

// --- Hardcoded Geometry ---

// Helper to fill vertex buffer for a box centered at 0
void fill_box_verts(tics_vec3* v, float s) {
	float h = s * 0.5f;
	v[0] = (tics_vec3){-h, -h, -h};
	v[1] = (tics_vec3){h, -h, -h};
	v[2] = (tics_vec3){h, h, -h};
	v[3] = (tics_vec3){-h, h, -h};
	v[4] = (tics_vec3){-h, -h, h};
	v[5] = (tics_vec3){h, -h, h};
	v[6] = (tics_vec3){h, h, h};
	v[7] = (tics_vec3){-h, h, h};
}

// Standard box indices
static const uint32_t box_indices[] = {4, 5, 6, 4, 6, 7, 1, 0, 3, 1, 3, 2, 0, 4, 7, 0, 7, 3,
									   5, 1, 2, 5, 2, 6, 7, 6, 2, 7, 2, 3, 0, 1, 5, 0, 5, 4};

tics_quat quat_axis_angle(float x, float y, float z, float angle) {
	float s = sinf(angle * 0.5f);
	return (tics_quat){x * s, y * s, z * s, cosf(angle * 0.5f)};
}

int main() {
	tics_world* world = tics_world_create(
		(tics_world_desc){.gravity = {0}, .air_friction_linear = 0, .air_friction_angular = 0});

	// Prepare Shape Data
	tics_vec3 verts_mono[8];
	fill_box_verts(verts_mono, MONOLITH_SIZE);
	tics_vec3 verts_crate[8];
	fill_box_verts(verts_crate, CRATE_SIZE);
	tics_vec3 verts_debris[8];
	fill_box_verts(verts_debris, DEBRIS_SIZE);

	// Massive floor plate (100km x 1m x 100km)
	tics_vec3 verts_floor[8];
	float floor_h = (ISLAND_COUNT / 2) * ISLAND_SPACING_X;
	verts_floor[0] = (tics_vec3){-floor_h, -1.0f, -floor_h};
	verts_floor[1] = (tics_vec3){floor_h, -1.0f, -floor_h};
	verts_floor[2] = (tics_vec3){floor_h, 0.0f, -floor_h};
	verts_floor[3] = (tics_vec3){-floor_h, 0.0f, -floor_h};
	verts_floor[4] = (tics_vec3){-floor_h, -1.0f, floor_h};
	verts_floor[5] = (tics_vec3){floor_h, -1.0f, floor_h};
	verts_floor[6] = (tics_vec3){floor_h, 0.0f, floor_h};
	verts_floor[7] = (tics_vec3){-floor_h, 0.0f, floor_h};

	// Register Shapes
	tics_shape_id sh_mono = tics_create_shape(
		world, (tics_shape_desc){.type = TICS_SHAPE_CONVEX, .data.convex = {verts_mono, 8}});
	tics_shape_id sh_crate = tics_create_shape(
		world, (tics_shape_desc){.type = TICS_SHAPE_CONVEX, .data.convex = {verts_crate, 8}});
	tics_shape_id sh_debris = tics_create_shape(
		world, (tics_shape_desc){.type = TICS_SHAPE_CONVEX, .data.convex = {verts_debris, 8}});
	tics_shape_id sh_floor = tics_create_shape(
		world, (tics_shape_desc){.type = TICS_SHAPE_CONVEX, .data.convex = {verts_floor, 8}});

	// Optional Debug Upload
	tics_debug_upload_shape_mesh(sh_mono, verts_mono, box_indices, 36);
	tics_debug_upload_shape_mesh(sh_crate, verts_crate, box_indices, 36);
	tics_debug_upload_shape_mesh(sh_debris, verts_debris, box_indices, 36);
	tics_debug_upload_shape_mesh(sh_floor, verts_floor, box_indices, 36);

	// Create Static Ground
	// Floor top is at Y=0.
	tics_world_add_static_body(
		world,
		(tics_static_body_desc){.transform = {.position = {0, 0, 0}, .rotation = {0, 0, 0, 1}},
								.shape = sh_floor,
								.elasticity = 0.5f});

	// Initialize RNG
	pcg32_random_t rng = {.state = 0x853C49E6748FEA9BULL, .inc = 0xDA3E39CB94B95BDBULL};

	for (int i = 0; i < ISLAND_COUNT; i++) {
		// --- Island Center Calculation ---
		// Highway layout: Alternating +/- Z offset
		float cx = (i / 2) * ISLAND_SPACING_X;
		float cz = (i % 2 == 0) ? ISLAND_OFFSET_Z : -ISLAND_OFFSET_Z;

		// Apply Jitter so they don't align perfectly on grid axes
		cx += rand_range(&rng, -JITTER_POS, JITTER_POS);
		cz += rand_range(&rng, -JITTER_POS, JITTER_POS);

		// --- The Monolith (Tier 1 - 50m) ---
		// Sunk slightly into ground (Y=0) to ensure collision
		// Center Y = 25.0 - Epsilon
		tics_world_add_rigid_body(
			world,
			(tics_rigid_body_desc){
				.shape = sh_mono,
				.transform = {.position = {cx, (MONOLITH_SIZE / 2.0f) - GROUND_EPSILON, cz},
							  // Slight rotation to break AABB alignment
							  .rotation = quat_axis_angle(0, 1, 0, rand_range(&rng, 0, 6.28f))},
				.mass = 10000.0f,
				.elasticity = 0.0f,
				.gravity_scale = 1.0f,
				// Explicitly zero velocity for "Resting" state
				.linear_velocity = {0, 0, 0},
				.angular_velocity = {0, 0, 0}});

		// --- Crates (Tier 2 - 5m) ---
		// Ring around monolith (Radius ~60m)
		float r_crate = 60.0f;
		for (int j = 0; j < CRATE_COUNT; j++) {
			float theta = (j / (float)CRATE_COUNT) * 6.2831f;
			// Jitter angle/radius
			theta += rand_range(&rng, -0.02f, 0.02f);
			float r = r_crate + rand_range(&rng, -10.0f, 10.0f);

			tics_world_add_rigid_body(
				world,
				(tics_rigid_body_desc){
					.shape = sh_crate,
					.transform = {.position = {cx + cosf(theta) * r,
											   (CRATE_SIZE / 2.0f) - GROUND_EPSILON,
											   cz + sinf(theta) * r},
								  .rotation = quat_axis_angle(0, 1, 0, rand_range(&rng, 0, 6.28f))},
					.mass = 100.0f,
					.elasticity = 0.2f,
					.gravity_scale = 1.0f});
		}

		// --- Debris (Tier 3 - 0.5m) ---
		// Wider field (Radius 80-120m)
		// Note: Simple polar distribution guarantees sparsity for small objects
		for (int k = 0; k < DEBRIS_COUNT; k++) {
			float theta = (k / (float)DEBRIS_COUNT) * 6.2831f;
			// High randomness in radius creates a scattered field look
			float r = rand_range(&rng, 80.0f, 120.0f);

			tics_world_add_rigid_body(
				world,
				(tics_rigid_body_desc){
					.shape = sh_debris,
					.transform = {.position = {cx + cosf(theta) * r,
											   (DEBRIS_SIZE / 2.0f) - GROUND_EPSILON,
											   cz + sinf(theta) * r},
								  .rotation = quat_axis_angle(0, 1, 0, rand_range(&rng, 0, 6.28f))},
					.mass = 10.0f,
					.elasticity = 0.5f,
					.gravity_scale = 1.0f});
		}
	}

	// The Traveler
	// A single dynamic object passing through the void between islands.
	// Tests the cost of updating Broad Phase in a mostly static world.
	float world_len = (ISLAND_COUNT / 2) * ISLAND_SPACING_X;
	tics_world_add_rigid_body(
		world, (tics_rigid_body_desc){
				   .shape = sh_crate,
				   .transform = {.position = {-100.0f, 3.0f,
											  0.0f}, // Start behind first island, centered in void
								 .rotation = {0, 0, 0, 1}},
				   .linear_velocity = {100.0f, 0.0f, 0.0f}, // High speed X movement
				   .mass = 50.0f,
				   .gravity_scale = 0.0f});

	// --- Simulation Loop ---
	for (int f = 0; f < STEPS; f++) {
		tics_world_step(world, 1.0f / 60.0f);
	}

	tics_world_destroy(world);

	return 0;
}
