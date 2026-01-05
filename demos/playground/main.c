#include "models_data.h"

#include <raylib.h>
#include <raylib_util.h>
#include <raymath.h>
#include <tics.h>
#include <tics_raylib_bridge.h>

#include <stdlib.h>

#define PHYSICS_TIMESTEP (1.0f / 60.0f)
#define MAX_BODIES 1000
#define DYNAMIC_BODIES 50
// If the delta time exceeds this, the simulation will slow down rather than freeze.
const float MAX_FRAME_TIME = 0.25f;

typedef struct {
	tics_body_id body;
	Model* visual_ref;
	Color color;
} game_entity;

float random_float(float min, float max) {
	return min + (float)rand() / ((float)RAND_MAX / (max - min));
}

int main(void) {
	srand(42);
	InitWindow(1280, 720, "Tics Physics Demo");
	set_window_top_left(0);
	SetTargetFPS(60);

	Camera3D camera = {0};
	camera.position = (Vector3){15.0f, 10.0f, 15.0f};
	camera.target = (Vector3){0.0f, 2.0f, 0.0f};
	camera.up = (Vector3){0.0f, 1.0f, 0.0f};
	camera.fovy = 45.0f;
	camera.projection = CAMERA_PERSPECTIVE;

	// ----------------------------------------------------------------------------------
	// Setup physics and create static geometry
	// ----------------------------------------------------------------------------------
	tics_world* world = tics_world_create((tics_world_desc){.gravity = {0.0f, -9.81f, 0.0f}});

	Model static_models[static_object_count];
	for (int i = 0; i < static_object_count; i++) {
		// Create raylib model and insert mesh data
		static_models[i] = create_raylib_model(
			(float*)ground_vertex_buffers[i], (int)ground_vertex_buffer_sizes[i],
			ground_index_buffers[i], (int)ground_index_buffer_sizes[i]);
		// Convert position and rotation into matrix for rendering
		tics_transform t = {ground_positions[i], ground_rotations[i]};
		static_models[i].transform = to_raylib_matrix(t);

		// Create Physics Body
		tics_shape_desc shape_desc = {.type = TICS_SHAPE_CONVEX,
									  .data.convex.vertices = ground_vertex_buffers[i],
									  .data.convex.vertex_count = ground_vertex_buffer_sizes[i]};
		tics_static_body_desc body_desc = {
			.transform = {.position = ground_positions[i], .rotation = ground_rotations[i]},
			.shape = tics_create_shape(world, shape_desc),
			.elasticity = 0.8f};
		tics_world_add_static_body(world, body_desc);

		// upload debug shape - optional, but nice for debug visualization
		tics_debug_upload_shape_mesh(body_desc.shape, ground_vertex_buffers[i],
									 ground_index_buffers[i], ground_index_buffer_sizes[i]);
	}

	// ----------------------------------------------------------------------------------
	// Create dynamic entities
	// ----------------------------------------------------------------------------------
	game_entity entities[MAX_BODIES];
	int entity_count = 0;

	// Pre-load dynamic models
	Model md_cube = create_raylib_model((float*)cube_Cube_vertices, 24, cube_Cube_indices, 36);
	Model md_sphere = create_raylib_model((float*)icosphere_Icosphere_vertices, 240,
										  icosphere_Icosphere_indices, 240);

	// Pre-create physics shapes
	tics_shape_id sh_cube =
		tics_create_shape(world, (tics_shape_desc){.type = TICS_SHAPE_CONVEX,
												   .data.convex.vertices = cube_Cube_vertices,
												   .data.convex.vertex_count = 24});
	tics_shape_id sh_sphere = tics_create_shape(
		world, (tics_shape_desc){.type = TICS_SHAPE_CONVEX,
								 .data.convex.vertices = icosphere_Icosphere_vertices,
								 .data.convex.vertex_count = 240});

	// upload debug shape - optional, but nice for debug visualization
	tics_debug_upload_shape_mesh(sh_cube, cube_Cube_vertices, cube_Cube_indices,
								 cube_index_buffer_sizes[0]);
	tics_debug_upload_shape_mesh(sh_sphere, icosphere_Icosphere_vertices,
								 icosphere_Icosphere_indices, icosphere_index_buffer_sizes[0]);

	// Spawn randomized dynamic objects
	for (int i = 0; i < DYNAMIC_BODIES; i++) {
		bool is_cube = (rand() % 2 == 0);

		tics_rigid_body_desc desc = {0};
		desc.shape = is_cube ? sh_cube : sh_sphere;
		desc.mass = 1.0f;
		desc.elasticity = 1.0f;
		desc.gravity_scale = 1.0f;
		desc.transform.position =
			(tics_vec3){random_float(-4, 4), random_float(5, 25), random_float(-4, 4)};
		desc.transform.rotation =
			to_tics_quat(QuaternionFromEuler(random_float(0, 360), random_float(0, 360), 0));

		entities[entity_count++] = (game_entity){.body = tics_world_add_rigid_body(world, desc),
												 .visual_ref = is_cube ? &md_cube : &md_sphere,
												 .color = is_cube ? MAROON : GOLD};
	}

	// ----------------------------------------------------------------------------------
	// Main Loop
	// ----------------------------------------------------------------------------------
	float accumulator = 0.0f;

	while (!WindowShouldClose()) {
		float delta = fminf(GetFrameTime(), MAX_FRAME_TIME);

		update_fly_camera(&camera);

		accumulator += delta;
		while (accumulator >= PHYSICS_TIMESTEP) {
			tics_world_step(world, PHYSICS_TIMESTEP);
			accumulator -= PHYSICS_TIMESTEP;
		}

		BeginDrawing();
		{
			ClearBackground(RAYWHITE);
			BeginMode3D(camera);
			{
				// Draw static geometry
				for (int i = 0; i < static_object_count; i++) {
					DrawModel(static_models[i], (Vector3){0}, 1.0f, LIGHTGRAY);
					DrawModelWires(static_models[i], (Vector3){0}, 1.0f, BLACK);
				}

				// Draw dynamic entities
				for (int i = 0; i < entity_count; i++) {
					game_entity* entity = &entities[i];
					// Extract transform from physics
					tics_transform transform = tics_body_get_transform(world, entity->body);
					// Convert to matrix for rendering
					entity->visual_ref->transform = to_raylib_matrix(transform);

					DrawModel(*entity->visual_ref, (Vector3){0}, 1.0f, entity->color);
					DrawModelWires(*entity->visual_ref, (Vector3){0}, 1.0f, BLACK);
				}

				DrawGrid(20, 1.0f);
			}
			EndMode3D();
		}
		EndDrawing();
	}

	// Cleanup
	for (int i = 0; i < static_object_count; i++) {
		UnloadModel(static_models[i]);
	}
	UnloadModel(md_cube);
	UnloadModel(md_sphere);
	tics_world_destroy(world);
	CloseWindow();
	return 0;
}
