#include "tics.h"

#include <pcg.h>

#include <math.h>
#include <stdio.h>

// Options that can be defined through cmake:
// BENCHMARK_PARTICLE_COUNT
// BENCHMARK_STEPS

#define CONTAINER_SIZE 60.0f
#define LIN_VEL 50.0f
#define ANG_VEL 1.0f
#define SPAWN_STATICS false

#define WALL_THICKNESS 25.0f
// if enabled, use velocity reflection at borders instead of colliders
// #define USE_VIRTUAL_WALLS

// if enabled, use spheres instead of tetrahedrons
#define USE_SPHERES
#define SPHERE_RADIUS 0.49f

#define SWAY_AMPLITUDE 27.0f
#define SWAY_FREQUENCY 1.2f

// Regular Tetrahedron (Radius 0.5, Diameter ~1.0).
static const tics_vec3 tet_verts[] = {{0.471404f, 0.0f, -0.166667f},
									  {-0.235702f, 0.408248f, -0.166667f},
									  {-0.235702f, -0.408248f, -0.166667f},
									  {0.0f, 0.0f, 0.5f}};
static const uint32_t tet_indices[] = {0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3};

// Wall plate (Centered)
#define HS (CONTAINER_SIZE / 2.0 + WALL_THICKNESS)
#define HT (WALL_THICKNESS / 2.0)
static const tics_vec3 wall_verts[] = {{-HS, -HS, -HT}, {HS, -HS, -HT}, {HS, HS, -HT},
									   {-HS, HS, -HT},	{-HS, -HS, HT}, {HS, -HS, HT},
									   {HS, HS, HT},	{-HS, HS, HT}};
static const uint32_t wall_indices[] = {4, 5, 6, 4, 6, 7, 1, 0, 3, 1, 3, 2, 0, 4, 7, 0, 7, 3,
										5, 1, 2, 5, 2, 6, 7, 6, 2, 7, 2, 3, 0, 1, 5, 0, 5, 4};

tics_quat quat_axis_angle(float x, float y, float z, float angle) {
	float s = sinf(angle * 0.5f);
	return (tics_quat){x * s, y * s, z * s, cosf(angle * 0.5f)};
}

int main() {
	// Create Physics World without gravity or friction
	tics_world_desc world_desc = {0};
	world_desc.gravity = (tics_vec3){0, -10.0, 0};
	tics_world* world = tics_world_create(world_desc);

#ifdef USE_SPHERES
	tics_shape_id part_shape = tics_create_sphere_shape(world, (tics_vec3){0}, SPHERE_RADIUS);
#else
	tics_shape_id part_shape = tics_create_convex_shape(world, tet_verts, 4, tet_indices, 12);
#endif

#ifndef USE_VIRTUAL_WALLS

#ifdef USE_SPHERES
	float sphere_wall_radius = 1000.0f;
	tics_shape_id wall_shape = tics_create_sphere_shape(
		world, (tics_vec3){0, 0, sphere_wall_radius - WALL_THICKNESS / 2.0f}, sphere_wall_radius);
#else
	tics_shape_id wall_shape = tics_create_convex_shape(world, wall_verts, 8, wall_indices, 36);
#endif


	float off = (CONTAINER_SIZE / 2.0f) + (WALL_THICKNESS / 2.0f);
	struct {
		tics_vec3 p;
		tics_quat r;
	} walls[] = {
		{{0, 0, off}, {0, 0, 0, 1}},						// Front  (points +Z)
		{{0, 0, -off}, quat_axis_angle(0, 1, 0, 3.14159f)}, // Back   (rotated 180° to point -Z)
		{{-off, 0, 0}, quat_axis_angle(0, 1, 0, -1.5708f)}, // Left   (rotated -90° to point -X)
		{{off, 0, 0}, quat_axis_angle(0, 1, 0, 1.5708f)},	// Right  (rotated +90° to point +X)
		{{0, -off, 0}, quat_axis_angle(1, 0, 0, 1.5708f)},	// Bottom (rotated +90° to point -Y)
		{{0, off, 0}, quat_axis_angle(1, 0, 0, -1.5708f)}	// Top    (rotated -90° to point +Y)
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
								   .elasticity = 0.9f,
								   .gravity_scale = 0.0f});
	}
#endif

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
									  rand_range(&rng, 0.0f, 6.2831f) // 0 to 2pi
		);

		// Make 5% of particles static bodies
		if (SPAWN_STATICS && i % 20 == 0) {
			tics_world_add_static_body(
				world, (tics_static_body_desc){.transform = {.position = pos, .rotation = q},
											   .shape = part_shape,
											   .elasticity = 0.9f});
		}
		else {
			bodies[i] = tics_world_add_rigid_body(
				world,
				(tics_rigid_body_desc){.shape = part_shape,
									   .transform = {.position = pos, .rotation = q},
									   .linear_velocity = {rand_range(&rng, -LIN_VEL, LIN_VEL),
														   rand_range(&rng, -LIN_VEL, LIN_VEL),
														   rand_range(&rng, -LIN_VEL, LIN_VEL)},
									   .angular_velocity = {rand_range(&rng, -ANG_VEL, ANG_VEL),
															rand_range(&rng, -ANG_VEL, ANG_VEL),
															rand_range(&rng, -ANG_VEL, ANG_VEL)},
									   .mass = 1.0f,
									   .elasticity = 0.9f,
									   .gravity_scale = 1.0f});
		}
	}

	float delta_time = 1.0f / 60.0f;
	float current_time = 0.0f;

	for (int f = 0; f < BENCHMARK_STEPS; f++) {
#ifndef USE_VIRTUAL_WALLS
		// Calculate the instantaneous velocity for the swaying motion
		// For position x(t) = Amplitude * sin(Frequency * t),
		// the velocity is v(t) = Amplitude * Frequency * cos(Frequency * t)
		float sway_vel = SWAY_AMPLITUDE * SWAY_FREQUENCY * cosf(SWAY_FREQUENCY * current_time);
		for (int i = 0; i < 6; i++) {
			tics_body_set_velocity(world, wall_bodies[i],
								   (tics_vec3){sway_vel, 0.0f, sway_vel * 0.3f});
		}
#endif

		tics_world_step(world, delta_time);
		current_time += delta_time;

#ifdef USE_VIRTUAL_WALLS
		// Reflect velocities at boundaries instead of using physical walls
		float boundary = CONTAINER_SIZE / 2.0f;
		for (int i = 0; i < BENCHMARK_PARTICLE_COUNT; i++) {
			if (i % 20 == 0) continue; // Skip static bodies

			tics_transform t = tics_body_get_transform(world, bodies[i]);
			tics_vec3 v = tics_body_get_velocity(world, bodies[i]);

			bool reflect = false;
			if ((t.position.x > boundary && v.x > 0) || (t.position.x < -boundary && v.x < 0)) {
				v.x = -v.x;
				reflect = true;
			}
			if ((t.position.y > boundary && v.y > 0) || (t.position.y < -boundary && v.y < 0)) {
				v.y = -v.y;
				reflect = true;
			}
			if ((t.position.z > boundary && v.z > 0) || (t.position.z < -boundary && v.z < 0)) {
				v.z = -v.z;
				reflect = true;
			}
			if (reflect) { tics_body_set_velocity(world, bodies[i], v); }
		}
#endif
	}

	tics_world_destroy(world);
	return 0;
}
