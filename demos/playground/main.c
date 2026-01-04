#include "demo_framework.h"
#include "models_data.h"

#include <raylib.h>
#include <raymath.h>
#include <tics.h>

#include <stdlib.h>

#define PHYSICS_TIMESTEP (1.0f / 60.0f)
#define MAX_BODIES 1000
#define DYNAMIC_BODIES 50

typedef struct {
	tics_body_id body;
	Model* visual_ref;
	Color color;
} GameEntity;

int main(void) {
	srand(42);
	InitWindow(1280, 720, "Tics Physics Demo");
	SetTargetFPS(60);

	Camera3D camera = {0};
	camera.position = (Vector3){15.0f, 10.0f, 15.0f};
	camera.target = (Vector3){0.0f, 2.0f, 0.0f};
	camera.up = (Vector3){0.0f, 1.0f, 0.0f};
	camera.fovy = 45.0f;
	camera.projection = CAMERA_PERSPECTIVE;

	// ----------------------------------------------------------------------------------
	// 1. Physics Setup
	// ----------------------------------------------------------------------------------
	tics_world* world = tics_world_create((tics_world_desc){.gravity = {0.0f, -9.81f, 0.0f}});

	// ----------------------------------------------------------------------------------
	// 2. Create Static Ground
	// ----------------------------------------------------------------------------------
	Model ground_models[ground_object_count];

	for (int i = 0; i < ground_object_count; i++) {
		// Create Visual
		ground_models[i] =
			LoadModelFromRaw(ground_vertex_buffers[i], (int)ground_vertex_buffer_sizes[i],
							 ground_index_buffers[i], (int)ground_index_buffer_sizes[i]);

		// Bake static transform into model matrix for simpler drawing
		Matrix matRot = QuaternionToMatrix(ToRaylibQuat(ground_rotations[i]));
		Matrix matTrans =
			MatrixTranslate(ground_positions[i].x, ground_positions[i].y, ground_positions[i].z);
		ground_models[i].transform = MatrixMultiply(matRot, matTrans);

		// Create Physics Body
		tics_shape_desc shape_desc = {.type = TICS_SHAPE_CONVEX,
									  .data.convex.vertices = ground_vertex_buffers[i],
									  .data.convex.vertex_count = ground_vertex_buffer_sizes[i]};

		tics_static_body_desc body_desc = {
			.transform = {.position = ground_positions[i], .rotation = ground_rotations[i]},
			.shape = tics_create_shape(world, shape_desc),
			.elasticity = 0.5f};
		tics_world_add_static_body(world, body_desc);
	}

	// ----------------------------------------------------------------------------------
	// 3. Create Dynamic Entities
	// ----------------------------------------------------------------------------------
	GameEntity entities[MAX_BODIES];
	int entity_count = 0;

	// Pre-load dynamic models
	Model md_cube = LoadModelFromRaw(cube_Cube_vertices, 24, cube_Cube_indices, 36);
	Model md_sphere =
		LoadModelFromRaw(icosphere_Icosphere_vertices, 240, icosphere_Icosphere_indices, 240);

	// Pre-create shapes
	tics_shape_id sh_cube =
		tics_create_shape(world, (tics_shape_desc){.type = TICS_SHAPE_CONVEX,
												   .data.convex.vertices = cube_Cube_vertices,
												   .data.convex.vertex_count = 24});
	tics_shape_id sh_sphere = tics_create_shape(
		world, (tics_shape_desc){.type = TICS_SHAPE_CONVEX,
								 .data.convex.vertices = icosphere_Icosphere_vertices,
								 .data.convex.vertex_count = 240});

	for (int i = 0; i < DYNAMIC_BODIES; i++) {
		bool is_cube = (rand() % 2 == 0);

		tics_rigid_body_desc desc = {0};
		desc.shape = is_cube ? sh_cube : sh_sphere;
		desc.mass = 1.0f;
		desc.elasticity = 0.3f;
		desc.gravity_scale = 1.0f;
		desc.transform.position =
			(tics_vec3){GetRandomFloat(-4, 4), GetRandomFloat(5, 25), GetRandomFloat(-4, 4)};
		desc.transform.rotation =
			ToTicsQuat(QuaternionFromEuler(GetRandomFloat(0, 360), GetRandomFloat(0, 360), 0));

		entities[entity_count++] = (GameEntity){.body = tics_world_add_rigid_body(world, desc),
												.visual_ref = is_cube ? &md_cube : &md_sphere,
												.color = is_cube ? MAROON : GOLD};
	}

	// ----------------------------------------------------------------------------------
	// 4. Main Loop
	// ----------------------------------------------------------------------------------
	float accumulator = 0.0f;

	while (!WindowShouldClose()) {
		float dt = GetFrameTime();
		UpdateFlyCamera(&camera);

		accumulator += dt;
		while (accumulator >= PHYSICS_TIMESTEP) {
			tics_world_step(world, PHYSICS_TIMESTEP);
			accumulator -= PHYSICS_TIMESTEP;
		}

		BeginDrawing();
		ClearBackground(RAYWHITE);
		BeginMode3D(camera);

		// Draw Ground
		for (int i = 0; i < ground_object_count; i++) {
			DrawModel(ground_models[i], (Vector3){0}, 1.0f, LIGHTGRAY);
			DrawModelWires(ground_models[i], (Vector3){0}, 1.0f, BLACK);
		}

		// Draw Dynamic
		for (int i = 0; i < entity_count; i++) {
			GameEntity* e = &entities[i];
			tics_transform t = tics_body_get_transform(world, e->body);

			Matrix matRot = QuaternionToMatrix(ToRaylibQuat(t.rotation));
			Matrix matTrans = MatrixTranslate(t.position.x, t.position.y, t.position.z);
			e->visual_ref->transform = MatrixMultiply(matRot, matTrans);

			DrawModel(*e->visual_ref, (Vector3){0}, 1.0f, e->color);
			DrawModelWires(*e->visual_ref, (Vector3){0}, 1.0f, BLACK);
		}

		DrawGrid(20, 1.0f);
		EndMode3D();
		EndDrawing();
	}

	// Cleanup
	for (int i = 0; i < ground_object_count; i++)
		UnloadModel(ground_models[i]);
	UnloadModel(md_cube);
	UnloadModel(md_sphere);
	tics_world_destroy(world);
	CloseWindow();
	return 0;
}
