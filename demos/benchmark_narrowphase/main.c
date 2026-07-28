#include "tics.h"

#include <pcg.h>

#include <math.h>
#include <stdio.h>

// Options that are defined through cmake:
// BENCHMARK_PARTICLE_COUNT
// BENCHMARK_STEPS

#define CONTAINER_SIZE 75.0f
#define WALL_THICKNESS 5.0f
#define LIN_VEL 50.0f
#define ANG_VEL 1.0f

static const float SHAPE_RADII[] = {
	0.20f, 0.30f, 0.40f, 0.50f, 0.60f, 0.70f,
	0.25f, 0.35f, 0.45f, 0.55f, 0.65f, 0.75f,
	2.00f
};
#define NUM_SHAPE_RADII (sizeof(SHAPE_RADII) / sizeof(SHAPE_RADII[0]))
#define NUM_SHAPE_VARIATIONS (NUM_SHAPE_RADII * 3)

#define SWAY_AMPLITUDE_Z 30.0f
#define SWAY_FREQUENCY 1.2f

tics_quat quat_axis_angle(float x, float y, float z, float angle) {
	float s = sinf(angle * 0.5f);
	return (tics_quat){x * s, y * s, z * s, cosf(angle * 0.5f)};
}

int main() {
	tics_world_desc world_desc = {0};
	world_desc.gravity = (tics_vec3){0, -10.0, 0};
	// set high angular friction so capsules dont move out of bounds
	world_desc.air_friction_angular = 1;
	tics_world* world = tics_world_create(world_desc);

	// Create a flat plate for the walls. We make it wider than the container to ensure
	// nothing escapes from the corners. The local Z axis represents the thickness.
	tics_shape_id wall_shape = tics_create_box_shape(
		world, (tics_vec3){CONTAINER_SIZE * 0.5 + WALL_THICKNESS,
						   CONTAINER_SIZE * 0.5 + WALL_THICKNESS, WALL_THICKNESS * 0.5f});

	float off = (CONTAINER_SIZE / 2.0f);
	off += WALL_THICKNESS * 0.5f;// make sure the inner face aligns with CONTAINER_SIZE
	struct {
		tics_vec3 p;
		tics_quat r;
	} walls[] = {
		{{0, 0, off}, {0, 0, 0, 1}},                        // Front  (points +Z)
		{{0, 0, -off}, quat_axis_angle(0, 1, 0, 3.14159f)}, // Back   (rotated 180° to point -Z)
		{{-off, 0, 0}, quat_axis_angle(0, 1, 0, -1.5708f)}, // Left   (rotated -90° to point -X)
		{{off, 0, 0}, quat_axis_angle(0, 1, 0, 1.5708f)},   // Right  (rotated +90° to point +X)
		{{0, -off, 0}, quat_axis_angle(1, 0, 0, 1.5708f)},  // Bottom (rotated +90° to point -Y)
		{{0, off, 0}, quat_axis_angle(1, 0, 0, -1.5708f)}   // Top    (rotated -90° to point +Y)
	};

	tics_body_id wall_bodies[6];
	for (int i = 0; i < 6; i++) {
		wall_bodies[i] = tics_world_add_rigid_body(
			world,
			(tics_rigid_body_desc){.transform = {.position = walls[i].p, .rotation = walls[i].r},
								   .shape = wall_shape,
								   .linear_velocity = {0, 0, 0},
								   .angular_velocity = {0, 0, 0},
								   .mass = 0.0f,
								   .elasticity = 0.95f,
								   .gravity_scale = 0.0f});
	}

	// Spawn Particles (Prime Stepper Algorithm)
	int dim = (int)ceil(pow((float)BENCHMARK_PARTICLE_COUNT, 1.0f / 3.0f));
	int total_cells = dim * dim * dim;
	float stride = CONTAINER_SIZE / (float)dim;
	float start = -(CONTAINER_SIZE * 0.5f) + (stride * 0.5f);

	// Prime Step: Visits every cell exactly once in a chaotic order
	// Ensures objects are not ordered perfectly along coordinate axes
	long long prime_step = 100003;
	if (dim % prime_step == 0) prime_step += 2;

	// Initialize RNG (Seed with arbitrary constants)
	pcg32_random_t rng = {.state = 0x853C49E6748FEA9BULL, .inc = 0xDA3E39CB94B95BDBULL};

	tics_shape_id shape_pool[NUM_SHAPE_VARIATIONS];
	for (int i = 0; i < NUM_SHAPE_RADII; i++) {
		float radius = SHAPE_RADII[i];
		shape_pool[i*3+0] = tics_create_sphere_shape(world, (tics_vec3){0, 0, 0}, fmin(radius,0.5));
		shape_pool[i*3+1] = tics_create_capsule_shape(
			world, (tics_vec3){0, -radius+0.3f, 0}, (tics_vec3){0, radius-0.3f, 0}, 0.3f);
		float half_ext = radius / sqrtf(3.0f);
		shape_pool[i*3+2] = tics_create_box_shape(
			world, (tics_vec3){0.4f, 0.4f, half_ext});
	}

	tics_body_id bodies[BENCHMARK_PARTICLE_COUNT];
	for (int i = 0; i < BENCHMARK_PARTICLE_COUNT; i++) {
		// Chaotic index generation (Spatial Position)
		int idx = (int)((i * prime_step) % total_cells);

		tics_vec3 pos = {start + (idx % dim) * stride, start + ((idx / dim) % dim) * stride,
						 start + (idx / (dim * dim)) * stride};

		// Randomized properties using PCG
		// This quaternion is not normalized, but tics will handle it fine
		tics_quat q = quat_axis_angle(rand_range(&rng, -1.0f, 1.0f), rand_range(&rng, -1.0f, 1.0f),
									  rand_range(&rng, -1.0f, 1.0f),
									  rand_range(&rng, 0.0f, 6.2831f));

		// Grab a random index to select a shape from the pre-generated pool
		int shape_idx = (int)rand_range(&rng, 0.0f, NUM_SHAPE_VARIATIONS);
		if (shape_idx >= NUM_SHAPE_VARIATIONS) shape_idx = NUM_SHAPE_VARIATIONS - 1;

		tics_shape_id active_shape = shape_pool[shape_idx];

		bodies[i] = tics_world_add_rigid_body(
			world, (tics_rigid_body_desc){.shape = active_shape,
										  .transform = {.position = pos, .rotation = q},
										  .linear_velocity = {rand_range(&rng, -LIN_VEL, LIN_VEL),
															  rand_range(&rng, -LIN_VEL, LIN_VEL),
															  rand_range(&rng, -LIN_VEL, LIN_VEL)},
										  .angular_velocity = {rand_range(&rng, -ANG_VEL, ANG_VEL),
															   rand_range(&rng, -ANG_VEL, ANG_VEL),
															   rand_range(&rng, -ANG_VEL, ANG_VEL)},
										  .mass = 1.0f,
										  .elasticity = 0.95f,
										  .gravity_scale = 1.0f});
	}

	float delta_time = 1.0f / 60.0f;
	float current_time = 0.0f;

	for (int f = 0; f < BENCHMARK_STEPS; f++) {
		// Calculate the instantaneous velocity for the swaying motion
		// For position x(t) = Amplitude * sin(Frequency * t),
		// the velocity is v(t) = Amplitude * Frequency * cos(Frequency * t)
		float sway_vel_z = SWAY_AMPLITUDE_Z * SWAY_FREQUENCY * cosf(SWAY_FREQUENCY * current_time);
		for (int i = 0; i < 6; i++) {
			tics_body_set_velocity(world, wall_bodies[i], (tics_vec3){0.0f, 0.0f, sway_vel_z});
		}

		tics_world_step(world, delta_time);
		current_time += delta_time;
	}

	tics_world_destroy(world);
	return 0;
}
