#include "tics.h"

#include <pcg.h>

#include <math.h>
#include <stdio.h>
#include <unistd.h> // For usleep

// Options that are defined through cmake:
// BENCHMARK_SPHERE_COUNT
// BENCHMARK_CAPSULE_COUNT
// BENCHMARK_BOX_COUNT
// BENCHMARK_STEPS

#define TOTAL_PARTICLE_COUNT (BENCHMARK_SPHERE_COUNT + BENCHMARK_CAPSULE_COUNT + BENCHMARK_BOX_COUNT)

#define CONTAINER_SIZE_X 300.0f
#define CONTAINER_SIZE_Y 75.0f
#define CONTAINER_SIZE_Z 75.0f

#define WALL_THICKNESS 5.0f
#define LIN_VEL 50.0f
#define ANG_VEL 1.0f

static const float SHAPE_RADII[] = {
	0.20f, 0.30f, 0.40f, 0.50f, 0.60f, 0.70f,
	0.25f, 0.35f, 0.45f, 0.55f, 0.65f, 0.75f,
	2.00f
};
#define NUM_SHAPE_RADII (sizeof(SHAPE_RADII) / sizeof(SHAPE_RADII[0]))

#define SWAY_AMPLITUDE_Z 30.0f
#define SWAY_FREQUENCY 1.2f

static long long gcd(long long a, long long b) {
	while (b != 0) {
		long long t = b;
		b = a % b;
		a = t;
	}
	return a;
}

tics_quat quat_axis_angle(float x, float y, float z, float angle) {
	float s = sinf(angle * 0.5f);
	return (tics_quat){x * s, y * s, z * s, cosf(angle * 0.5f)};
}

int main() {
	tics_world_desc world_desc = {};
	world_desc.gravity = (tics_vec3){0, -10.0f, 0};
	// set high angular friction so capsules dont move out of bounds
	world_desc.air_friction_angular = 1;
	tics_world* world = tics_world_create(world_desc);

	// Create box shapes for each pair of opposing walls. Adding thickness along tangential
	// dimensions ensures the walls overlap at the seams so objects cannot escape through corners.
	tics_shape_id wall_shape_z = tics_create_box_shape(
		world, (tics_vec3){CONTAINER_SIZE_X * 0.5f + WALL_THICKNESS,
						   CONTAINER_SIZE_Y * 0.5f + WALL_THICKNESS, WALL_THICKNESS * 0.5f});

	tics_shape_id wall_shape_x = tics_create_box_shape(
		world, (tics_vec3){CONTAINER_SIZE_Z * 0.5f + WALL_THICKNESS,
						   CONTAINER_SIZE_Y * 0.5f + WALL_THICKNESS, WALL_THICKNESS * 0.5f});

	tics_shape_id wall_shape_y = tics_create_box_shape(
		world, (tics_vec3){CONTAINER_SIZE_X * 0.5f + WALL_THICKNESS,
						   CONTAINER_SIZE_Z * 0.5f + WALL_THICKNESS, WALL_THICKNESS * 0.5f});

	float off_x = (CONTAINER_SIZE_X * 0.5f) + (WALL_THICKNESS * 0.5f);
	float off_y = (CONTAINER_SIZE_Y * 0.5f) + (WALL_THICKNESS * 0.5f);
	float off_z = (CONTAINER_SIZE_Z * 0.5f) + (WALL_THICKNESS * 0.5f);

	struct {
		tics_vec3 p;
		tics_quat r;
		tics_shape_id shape;
	} walls[] = {
		{{0, 0, off_z}, {0, 0, 0, 1}, wall_shape_z},
		{{0, 0, -off_z}, quat_axis_angle(0, 1, 0, 3.14159f), wall_shape_z},
		{{-off_x, 0, 0}, quat_axis_angle(0, 1, 0, -1.5708f), wall_shape_x},
		{{off_x, 0, 0}, quat_axis_angle(0, 1, 0, 1.5708f), wall_shape_x},
		{{0, -off_y, 0}, quat_axis_angle(1, 0, 0, 1.5708f), wall_shape_y},
		{{0, off_y, 0}, quat_axis_angle(1, 0, 0, -1.5708f), wall_shape_y}
	};

	tics_body_id wall_bodies[6];
	for (int i = 0; i < 6; i++) {
		wall_bodies[i] = tics_world_add_rigid_body(
			world,
			(tics_rigid_body_desc){.transform = {.position = walls[i].p, .rotation = walls[i].r},
								   .shape = walls[i].shape,
								   .linear_velocity = {0, 0, 0},
								   .angular_velocity = {0, 0, 0},
								   .mass = 0.0f,
								   .elasticity = 0.95f,
								   .gravity_scale = 0.0f});
	}

	// Spawn Particles (Prime Stepper Algorithm)
	float container_vol = CONTAINER_SIZE_X * CONTAINER_SIZE_Y * CONTAINER_SIZE_Z;
	float cell_stride = cbrtf(container_vol / (float)TOTAL_PARTICLE_COUNT);

	int dim_x = (int)ceilf(CONTAINER_SIZE_X / cell_stride);
	int dim_y = (int)ceilf(CONTAINER_SIZE_Y / cell_stride);
	int dim_z = (int)ceilf(CONTAINER_SIZE_Z / cell_stride);
	if (dim_x * dim_y * dim_z < TOTAL_PARTICLE_COUNT) dim_z++;

	long long total_cells = (long long)dim_x * dim_y * dim_z;

	float stride_x = CONTAINER_SIZE_X / (float)dim_x;
	float stride_y = CONTAINER_SIZE_Y / (float)dim_y;
	float stride_z = CONTAINER_SIZE_Z / (float)dim_z;

	float start_x = -(CONTAINER_SIZE_X * 0.5f) + (stride_x * 0.5f);
	float start_y = -(CONTAINER_SIZE_Y * 0.5f) + (stride_y * 0.5f);
	float start_z = -(CONTAINER_SIZE_Z * 0.5f) + (stride_z * 0.5f);

	// The step and total_cells must be coprime to ensure full period permutation cycle
	long long prime_step = 100003;
	while (gcd(prime_step, total_cells) != 1) {
		prime_step++;
	}

	// Initialize RNG (Seed with arbitrary constants)
	pcg32_random_t rng = {.state = 0x853C49E6748FEA9BULL, .inc = 0xDA3E39CB94B95BDBULL};

	tics_shape_id sphere_pool[NUM_SHAPE_RADII];
	tics_shape_id capsule_pool[NUM_SHAPE_RADII];
	tics_shape_id box_pool[NUM_SHAPE_RADII];

	for (int i = 0; i < NUM_SHAPE_RADII; i++) {
		float radius = SHAPE_RADII[i];
		sphere_pool[i] = tics_create_sphere_shape(
			world, (tics_vec3){0, 0, 0}, fminf(radius, 0.5f));
		capsule_pool[i] = tics_create_capsule_shape(
			world, (tics_vec3){0, -radius + 0.3f, 0}, (tics_vec3){0, radius - 0.3f, 0}, 0.3f);
		float half_ext = radius / sqrtf(3.0f);
		box_pool[i] = tics_create_box_shape(
			world, (tics_vec3){0.4f, 0.4f, half_ext});
	}

	tics_body_id bodies[TOTAL_PARTICLE_COUNT];
	for (int i = 0; i < TOTAL_PARTICLE_COUNT; i++) {
		// Chaotic index generation (Spatial Position)
		long long idx = (i * prime_step) % total_cells;

		int ix = (int)(idx % dim_x);
		int iy = (int)((idx / dim_x) % dim_y);
		int iz = (int)(idx / ((long long)dim_x * dim_y));

		tics_vec3 pos = {
			start_x + (float)ix * stride_x,
			start_y + (float)iy * stride_y,
			start_z + (float)iz * stride_z
		};

		// Randomized properties using PCG
		// This quaternion is not normalized, but tics will handle it fine
		tics_quat q = quat_axis_angle(rand_range(&rng, -1.0f, 1.0f),
									  rand_range(&rng, -1.0f, 1.0f),
									  rand_range(&rng, -1.0f, 1.0f),
									  rand_range(&rng, 0.0f, 6.2831f));

		// Grab a random radius index
		int radius_idx = (int)rand_range(&rng, 0.0f, NUM_SHAPE_RADII);
		if (radius_idx >= NUM_SHAPE_RADII) radius_idx = NUM_SHAPE_RADII - 1;

		// Deterministic shape assignment guarantees exact counts per shape type
		tics_shape_id active_shape;
		if (i < BENCHMARK_SPHERE_COUNT) {
			active_shape = sphere_pool[radius_idx];
		} else if (i < BENCHMARK_SPHERE_COUNT + BENCHMARK_CAPSULE_COUNT) {
			active_shape = capsule_pool[radius_idx];
		} else {
			active_shape = box_pool[radius_idx];
		}

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
