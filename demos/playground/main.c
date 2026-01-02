#include <raylib.h>
#include <raymath.h>
#include <tics.h>

#include "demo_framework.h"
#include <stdlib.h>
#include <time.h>

#define PHYSICS_TIMESTEP (1.0f / 60.0f)
#define MAX_BODIES 1000
#define DYNAMIC_BODIES 200

// Link physics body to visual model
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
	// 1. Load Assets (Scenes)
	// ----------------------------------------------------------------------------------

	// The ground file contains multiple static meshes
	DemoScene scene_ground = LoadDemoScene("assets/models/ground.glb");

	// These files contain single objects for spawning
	DemoScene scene_cube = LoadDemoScene("assets/models/cube.glb");
	DemoScene scene_sphere = LoadDemoScene("assets/models/icosphere.glb");

	// ----------------------------------------------------------------------------------
	// 2. Setup Physics World
	// ----------------------------------------------------------------------------------
	tics_world_desc world_desc = {.gravity = {0.0f, -9.81f, 0.0f}};
	tics_world* world = tics_world_create(world_desc);

	GameEntity entities[MAX_BODIES];
	int entity_count = 0;

	// ----------------------------------------------------------------------------------
	// 3. Create Static Ground (Iterate all objects in ground file)
	// ----------------------------------------------------------------------------------
	for (int i = 0; i < scene_ground.count; i++) {
		DemoObject* obj = &scene_ground.objects[i];

		// Create Shape from vertices
		tics_shape_desc shape_desc = {.type = TICS_SHAPE_CONVEX,
									  .data.convex.vertices = obj->raw_vertices,
									  .data.convex.vertex_count = obj->vertex_count};
		tics_shape_id shape = tics_create_shape(world, shape_desc);

		// Create Static Body using the GLTF Transform
		tics_static_body_desc body_desc = {0};

		// Extract pos/rot from the default transform matrix
		Vector3 pos = {obj->default_transform.m12, obj->default_transform.m13,
					   obj->default_transform.m14};
		Quaternion rot = QuaternionFromMatrix(obj->default_transform);

		body_desc.transform.position = ToTicsVec(pos);
		body_desc.transform.rotation = ToTicsQuat(rot);
		body_desc.shape = shape;
		body_desc.elasticity = 0.5f;

		entities[entity_count++] =
			(GameEntity){.body = tics_world_add_static_body(world, body_desc),
						 .visual_ref = &obj->model,
						 .color = LIGHTGRAY};
	}

	// ----------------------------------------------------------------------------------
	// 4. Create Shapes for Dynamic Objects
	// ----------------------------------------------------------------------------------
	tics_shape_id shape_cube = 0;
	if (scene_cube.count > 0) {
		tics_shape_desc desc = {.type = TICS_SHAPE_CONVEX,
								.data.convex.vertices = scene_cube.objects[0].raw_vertices,
								.data.convex.vertex_count = scene_cube.objects[0].vertex_count};
		shape_cube = tics_create_shape(world, desc);
	}

	tics_shape_id shape_sphere = 0;
	if (scene_sphere.count > 0) {
		tics_shape_desc desc = {.type = TICS_SHAPE_CONVEX,
								.data.convex.vertices = scene_sphere.objects[0].raw_vertices,
								.data.convex.vertex_count = scene_sphere.objects[0].vertex_count};
		shape_sphere = tics_create_shape(world, desc);
	}

	// ----------------------------------------------------------------------------------
	// 5. Spawn Dynamic Bodies
	// ----------------------------------------------------------------------------------
	for (int i = 0; i < DYNAMIC_BODIES; i++) {
		bool is_cube = (rand() % 2 == 0);

		tics_rigid_body_desc desc = {0};
		desc.shape = is_cube ? shape_cube : shape_sphere;
		desc.mass = 1.0f;
		desc.elasticity = 0.3f;
		desc.gravity_scale = 1.0f;

		desc.transform.position.x = GetRandomFloat(-4.0f, 4.0f);
		desc.transform.position.y = GetRandomFloat(5.0f, 25.0f);
		desc.transform.position.z = GetRandomFloat(-4.0f, 4.0f);

		Quaternion q = QuaternionFromEuler(GetRandomFloat(0, 360), GetRandomFloat(0, 360), 0);
		desc.transform.rotation = ToTicsQuat(q);

		entities[entity_count++] = (GameEntity){
			.body = tics_world_add_rigid_body(world, desc),
			.visual_ref = is_cube ? &scene_cube.objects[0].model : &scene_sphere.objects[0].model,
			.color = is_cube ? MAROON : GOLD};
	}

	// ----------------------------------------------------------------------------------
	// 6. Main Loop
	// ----------------------------------------------------------------------------------
	float accumulator = 0.0f;

	while (!WindowShouldClose()) {
		float dt = GetFrameTime();
		UpdateFlyCamera(&camera);

		// Fixed Physics Step
		accumulator += dt;
		while (accumulator >= PHYSICS_TIMESTEP) {
			tics_world_step(world, PHYSICS_TIMESTEP);
			accumulator -= PHYSICS_TIMESTEP;
		}

		BeginDrawing();
		ClearBackground(RAYWHITE);
		BeginMode3D(camera);

		for (int i = 0; i < entity_count; i++) {
			GameEntity* e = &entities[i];

			// 1. Get Transform
			tics_transform t = tics_body_get_transform(world, e->body);

			// 2. Build Matrix
			Matrix matRot = QuaternionToMatrix(ToRaylibQuat(t.rotation));
			Matrix matTrans = MatrixTranslate(t.position.x, t.position.y, t.position.z);

			// 3. Apply & Draw
			e->visual_ref->transform = MatrixMultiply(matRot, matTrans);

			DrawModel(*e->visual_ref, (Vector3){0, 0, 0}, 1.0f, e->color);
			DrawModelWires(*e->visual_ref, (Vector3){0, 0, 0}, 1.0f, BLACK);
		}

		DrawGrid(20, 1.0f);

		EndMode3D();
		DrawFPS(10, 10);
		EndDrawing();
	}

	UnloadDemoScene(scene_ground);
	UnloadDemoScene(scene_cube);
	UnloadDemoScene(scene_sphere);
	tics_world_destroy(world);
	CloseWindow();
	return 0;
}
