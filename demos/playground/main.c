#include <raylib.h>
#include <raymath.h>
#include <tics.h>

#include "demo_framework.h"
#include <stdlib.h>
#include <time.h>

#define PHYSICS_TIMESTEP (1.0f / 60.0f)
#define MAX_BODIES 1000
#define DYNAMIC_BODIES 50

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

tics_vec3 Cube_position = {0.0f, -1.56309247f, 0.0f};
tics_quat Cube_rotation = {-0.160142764f, -0.0377949476f, 0.00494773826f, 0.986357689f};
tics_vec3 Cube_vertices[] = {{-10.0f,-1.0f,0.397667885f},{-10.0f,-1.0f,0.397667885f},{-10.0f,-1.0f,0.397667885f},{-10.0f,1.0f,5.10979223f},{-10.0f,1.0f,5.10979223f},{-10.0f,1.0f,5.10979223f},{-10.0f,-1.0f,-5.18561935f},{-10.0f,-1.0f,-5.18561935f},{-10.0f,-1.0f,-5.18561935f},{-10.0f,1.0f,-10.0f},{-10.0f,1.0f,-10.0f},{-10.0f,1.0f,-10.0f},{10.0f,-1.0f,0.397667885f},{10.0f,-1.0f,0.397667885f},{10.0f,-1.0f,0.397667885f},{10.0f,1.0f,5.10979223f},{10.0f,1.0f,5.10979223f},{10.0f,1.0f,5.10979223f},{10.0f,-1.0f,-5.18561935f},{10.0f,-1.0f,-5.18561935f},{10.0f,-1.0f,-5.18561935f},{10.0f,1.0f,-10.0f},{10.0f,1.0f,-10.0f},{10.0f,1.0f,-10.0f}};
uint32_t Cube_indices[] = {2,5,11,2,11,8,6,10,22,6,22,18,20,23,17,20,17,14,12,16,4,12,4,0,7,19,13,7,13,1,21,9,3,21,3,15};

tics_vec3 Cube_001_position = {-6.01640129f, 0.711428285f, -0.136563301f};
tics_quat Cube_001_rotation = {0.0f, 0.0f, 0.0f, 1.0f};
tics_vec3 Cube_001_vertices[] = {{-0.609095573f,-1.60268593f,3.1590867f},{-0.609095573f,-1.60268593f,3.1590867f},{-0.609095573f,-1.60268593f,3.1590867f},{-2.01954007f,-1.30192542f,1.20222282f},{-2.01954007f,-1.30192542f,1.20222282f},{-2.01954007f,-1.30192542f,1.20222282f},{3.54449987f,-1.60336137f,1.03430617f},{3.54449987f,-1.60336137f,1.03430617f},{3.54449987f,-1.60336137f,1.03430617f},{2.37938213f,-1.3026402f,-1.0480547f},{2.37938213f,-1.3026402f,-1.0480547f},{2.37938213f,-1.3026402f,-1.0480547f},{-1.55633211f,-0.192116499f,2.18952775f},{-1.55633211f,-0.192116499f,2.18952775f},{-1.55633211f,-0.192116499f,2.18952775f},{-3.06322551f,1.24753737f,-0.838821292f},{-3.06322551f,1.24753737f,-0.838821292f},{-3.06322551f,1.24753737f,-0.838821292f},{2.59726381f,-0.192791462f,0.0647473931f},{2.59726381f,-0.192791462f,0.0647473931f},{2.59726381f,-0.192791462f,0.0647473931f},{1.33569622f,1.2468226f,-3.08909798f},{1.33569622f,1.2468226f,-3.08909798f},{1.33569622f,1.2468226f,-3.08909798f}};
uint32_t Cube_001_indices[] = {1,3,10,1,10,8,7,9,22,7,22,20,18,21,15,18,15,12,14,17,5,14,5,2,6,19,13,6,13,0,23,11,4,23,4,16};

tics_vec3 Cylinder_position = {-0.229404926f, -0.414651692f, 10.015131f};
tics_quat Cylinder_rotation = {0.0f, 0.412432969f, 0.0f, 0.910987973f};
tics_vec3 Cylinder_vertices[] = {{11.6240387f,-1.97155488f,6.08377695f},{11.6240387f,-1.97155488f,6.08377695f},{11.6240387f,-1.97155488f,6.08377695f},{-4.87579536f,-1.28030491f,-12.2553043f},{-4.87579536f,-1.28030491f,-12.2553043f},{-4.87579536f,-1.28030491f,-12.2553043f},{11.8529892f,-0.231068566f,7.1999402f},{11.8529892f,-0.231068566f,7.1999402f},{-4.64684486f,0.460181803f,-11.1391401f},{-4.64684486f,0.460181803f,-11.1391401f},{10.3960247f,1.45581496f,9.23770142f},{10.3960247f,1.45581496f,9.23770142f},{-6.10381508f,2.14706564f,-9.10138035f},{-6.10381508f,2.14706564f,-9.10138035f},{7.93487072f,2.29978561f,11.2435713f},{7.93487072f,2.29978561f,11.2435713f},{-8.5649662f,2.99103498f,-7.09551239f},{-8.5649662f,2.99103498f,-7.09551239f},{5.62113094f,1.90593803f,12.2789755f},{5.62113094f,1.90593803f,12.2789755f},{-10.878706f,2.59718776f,-6.06010437f},{-10.878706f,2.59718776f,-6.06010437f},{4.53743172f,0.458559215f,11.8594437f},{4.53743172f,0.458559215f,11.8594437f},{-11.9624023f,1.14980936f,-6.47963715f},{-11.9624023f,1.14980936f,-6.47963715f},{5.19084787f,-1.3651067f,10.1812754f},{5.19084787f,-1.3651067f,10.1812754f},{-11.3089886f,-0.673857212f,-8.1578064f},{-11.3089886f,-0.673857212f,-8.1578064f},{7.27563524f,-2.71174574f,8.02970409f},{7.27563524f,-2.71174574f,8.02970409f},{-9.22420311f,-2.02049589f,-10.3093786f},{-9.22420311f,-2.02049589f,-10.3093786f},{9.81630135f,-2.95125103f,6.41147661f},{9.81630135f,-2.95125103f,6.41147661f},{-6.68353462f,-2.26000118f,-11.9276066f},{-6.68353462f,-2.26000118f,-11.9276066f}};
uint32_t Cylinder_indices[] = {2,4,8,2,8,7,7,8,12,7,12,10,10,12,16,10,16,14,14,16,21,14,21,19,19,21,25,19,25,23,23,25,29,23,29,27,27,29,32,27,32,31,13,9,5,5,37,33,33,28,24,24,20,17,17,13,5,5,33,24,5,24,17,31,32,36,31,36,34,34,36,3,34,3,1,35,0,6,6,11,15,15,18,22,22,26,30,30,35,6,6,15,22,6,22,30};

tics_vec3 Cube_002_position = {1.67985928f, -0.454480141f, -12.538805f};
tics_quat Cube_002_rotation = {0.0f, 0.0f, 0.0f, 1.0f};
tics_vec3 Cube_002_vertices[] = {{-12.3660221f,0.0663195848f,1.91464114f},{-12.3660221f,0.0663195848f,1.91464114f},{-12.3660221f,0.0663195848f,1.91464114f},{-12.3660221f,1.73507595f,0.812256336f},{-12.3660221f,1.73507595f,0.812256336f},{-12.3660221f,1.73507595f,0.812256336f},{-12.3660221f,-1.73507595f,-0.812256336f},{-12.3660221f,-1.73507595f,-0.812256336f},{-12.3660221f,-1.73507595f,-0.812256336f},{-12.3660221f,-0.0663195848f,-1.91464114f},{-12.3660221f,-0.0663195848f,-1.91464114f},{-12.3660221f,-0.0663195848f,-1.91464114f},{9.37313461f,0.0663195848f,1.91464114f},{9.37313461f,0.0663195848f,1.91464114f},{9.37313461f,0.0663195848f,1.91464114f},{9.37313461f,1.73507595f,0.812256336f},{9.37313461f,1.73507595f,0.812256336f},{9.37313461f,1.73507595f,0.812256336f},{9.37313461f,-1.73507595f,-0.812256336f},{9.37313461f,-1.73507595f,-0.812256336f},{9.37313461f,-1.73507595f,-0.812256336f},{9.37313461f,-0.0663195848f,-1.91464114f},{9.37313461f,-0.0663195848f,-1.91464114f},{9.37313461f,-0.0663195848f,-1.91464114f}};
uint32_t Cube_002_indices[] = {2,5,11,2,11,8,6,10,22,6,22,18,20,23,17,20,17,14,12,15,3,12,3,0,7,19,13,7,13,1,21,9,4,21,4,16};

tics_vec3 Cube_003_position = {11.1434612f, 0.798618913f, 0.0f};
tics_quat Cube_003_rotation = {0.0f, 0.0f, 0.0f, 1.0f};
tics_vec3 Cube_003_vertices[] = {{0.488510489f,-1.3271625f,10.8777838f},{0.488510489f,-1.3271625f,10.8777838f},{0.488510489f,-1.3271625f,10.8777838f},{-1.32716107f,-0.488510847f,10.8777838f},{-1.32716107f,-0.488510847f,10.8777838f},{-1.32716107f,-0.488510847f,10.8777838f},{0.488510489f,-1.3271625f,-10.8777838f},{0.488510489f,-1.3271625f,-10.8777838f},{0.488510489f,-1.3271625f,-10.8777838f},{-1.32716107f,-0.488510847f,-10.8777838f},{-1.32716107f,-0.488510847f,-10.8777838f},{-1.32716107f,-0.488510847f,-10.8777838f},{1.32716227f,0.488509059f,10.8777838f},{1.32716227f,0.488509059f,10.8777838f},{1.32716227f,0.488509059f,10.8777838f},{-0.488509417f,1.32716072f,10.8777838f},{-0.488509417f,1.32716072f,10.8777838f},{-0.488509417f,1.32716072f,10.8777838f},{1.32716227f,0.488509059f,-10.8777838f},{1.32716227f,0.488509059f,-10.8777838f},{1.32716227f,0.488509059f,-10.8777838f},{-0.488509417f,1.32716072f,-10.8777838f},{-0.488509417f,1.32716072f,-10.8777838f},{-0.488509417f,1.32716072f,-10.8777838f}};
uint32_t Cube_003_indices[] = {2,4,10,2,10,8,6,9,21,6,21,18,19,22,16,19,16,13,12,15,3,12,3,0,7,20,14,7,14,1,23,11,5,23,5,17};

tics_vec3 Cube_004_position = {4.60718822f, 0.711428285f, 3.51234961f};
tics_quat Cube_004_rotation = {0.0f, 0.0f, 0.0f, 1.0f};
tics_vec3 Cube_004_vertices[] = {{-1.0f,-1.0f,1.0f},{-1.0f,-1.0f,1.0f},{-1.0f,-1.0f,1.0f},{-1.0f,1.0f,2.69728708f},{-1.0f,1.0f,2.69728708f},{-1.0f,1.0f,2.69728708f},{-1.0f,-1.0f,-1.0f},{-1.0f,-1.0f,-1.0f},{-1.0f,-1.0f,-1.0f},{-1.0f,1.0f,-2.24379158f},{-1.0f,1.0f,-2.24379158f},{-1.0f,1.0f,-2.24379158f},{1.0f,-1.0f,1.0f},{1.0f,-1.0f,1.0f},{1.0f,-1.0f,1.0f},{2.30432653f,1.91450787f,2.69728708f},{2.30432653f,1.91450787f,2.69728708f},{2.30432653f,1.91450787f,2.69728708f},{1.0f,-1.0f,-1.0f},{1.0f,-1.0f,-1.0f},{1.0f,-1.0f,-1.0f},{2.30432653f,1.91450787f,-2.24379158f},{2.30432653f,1.91450787f,-2.24379158f},{2.30432653f,1.91450787f,-2.24379158f}};
uint32_t Cube_004_indices[] = {2,5,11,2,11,8,7,9,21,7,21,19,20,22,16,20,16,14,13,15,3,13,3,1,6,18,12,6,12,0,23,10,4,23,4,17};

tics_vec3 Cube_005_position = {-11.4606342f, 0.557328463f, 0.0f};
tics_quat Cube_005_rotation = {0.0f, 0.0f, 0.0f, 1.0f};
tics_vec3 Cube_005_vertices[] = {{-1.0f,-1.0f,10.8777838f},{-1.0f,-1.0f,10.8777838f},{-1.0f,-1.0f,10.8777838f},{-1.0f,1.0f,10.8777838f},{-1.0f,1.0f,10.8777838f},{-1.0f,1.0f,10.8777838f},{-1.0f,-1.0f,-10.8777838f},{-1.0f,-1.0f,-10.8777838f},{-1.0f,-1.0f,-10.8777838f},{-1.0f,1.0f,-10.8777838f},{-1.0f,1.0f,-10.8777838f},{-1.0f,1.0f,-10.8777838f},{1.0f,-1.0f,10.8777838f},{1.0f,-1.0f,10.8777838f},{1.0f,-1.0f,10.8777838f},{1.0f,1.0f,10.8777838f},{1.0f,1.0f,10.8777838f},{1.0f,1.0f,10.8777838f},{1.0f,-1.0f,-10.8777838f},{1.0f,-1.0f,-10.8777838f},{1.0f,-1.0f,-10.8777838f},{1.0f,1.0f,-10.8777838f},{1.0f,1.0f,-10.8777838f},{1.0f,1.0f,-10.8777838f}};
uint32_t Cube_005_indices[] = {2,5,11,2,11,8,6,9,21,6,21,18,20,23,17,20,17,14,12,15,3,12,3,0,7,19,13,7,13,1,22,10,4,22,4,16};

tics_vec3 Cube_006_position = {2.43740845f, 0.711428285f, -5.45913649f};
tics_quat Cube_006_rotation = {0.0f, 0.0f, 0.0f, 1.0f};
tics_vec3 Cube_006_vertices[] = {{3.39109278f,-0.128046662f,2.1091485f},{3.39109278f,-0.128046662f,2.1091485f},{3.39109278f,-0.128046662f,2.1091485f},{2.2079618f,0.990971625f,1.69702506f},{2.2079618f,0.990971625f,1.69702506f},{2.2079618f,0.990971625f,1.69702506f},{-0.255073726f,-3.0367837f,2.00066543f},{-0.255073726f,-3.0367837f,2.00066543f},{-0.255073726f,-3.0367837f,2.00066543f},{-1.65356028f,-1.12544203f,1.58213425f},{-1.65356028f,-1.12544203f,1.58213425f},{-1.65356028f,-1.12544203f,1.58213425f},{2.30411768f,1.39391088f,-2.16509748f},{2.30411768f,1.39391088f,-2.16509748f},{2.30411768f,1.39391088f,-2.16509748f},{0.908842564f,2.71869779f,-0.964205861f},{0.908842564f,2.71869779f,-0.964205861f},{0.908842564f,2.71869779f,-0.964205861f},{-1.34204888f,-1.51482701f,-2.27358055f},{-1.34204888f,-1.51482701f,-2.27358055f},{-1.34204888f,-1.51482701f,-2.27358055f},{-2.95267868f,0.602283835f,-1.07909679f},{-2.95267868f,0.602283835f,-1.07909679f},{-2.95267868f,0.602283835f,-1.07909679f}};
uint32_t Cube_006_indices[] = {2,4,9,2,9,7,8,11,23,8,23,20,19,21,16,19,16,14,13,15,3,13,3,1,6,18,12,6,12,0,22,10,5,22,5,17};

tics_vec3 positions[] = {{0.0f,-1.56309247f,0.0f},{-6.01640129f,0.711428285f,-0.136563301f},{-0.229404926f,-0.414651692f,10.015131f},{1.67985928f,-0.454480141f,-12.538805f},{11.1434612f,0.798618913f,0.0f},{4.60718822f,0.711428285f,3.51234961f},{-11.4606342f,0.557328463f,0.0f},{2.43740845f,0.711428285f,-5.45913649f}};
tics_quat rotations[] = {{-0.160142764f,-0.0377949476f,0.00494773826f,0.986357689f},{0.0f,0.0f,0.0f,1.0f},{0.0f,0.412432969f,0.0f,0.910987973f},{0.0f,0.0f,0.0f,1.0f},{0.0f,0.0f,0.0f,1.0f},{0.0f,0.0f,0.0f,1.0f},{0.0f,0.0f,0.0f,1.0f},{0.0f,0.0f,0.0f,1.0f}};
tics_vec3* vertex_buffers[] = {Cube_vertices,Cube_001_vertices,Cylinder_vertices,Cube_002_vertices,Cube_003_vertices,Cube_004_vertices,Cube_005_vertices,Cube_006_vertices};
size_t vertex_buffer_sizes[] = {24,24,38,24,24,24,24,24};
uint32_t* index_buffers[] = {Cube_indices,Cube_001_indices,Cylinder_indices,Cube_002_indices,Cube_003_indices,Cube_004_indices,Cube_005_indices,Cube_006_indices};
size_t index_buffer_sizes[] = {36,36,96,36,36,36,36,36};

	// ----------------------------------------------------------------------------------
	// 3. Create Static Ground (Iterate all objects in ground file)
	// ----------------------------------------------------------------------------------
	for (int i = 0; i < scene_ground.count; i++) {
		DemoObject* obj = &scene_ground.objects[i];

		// Create Shape from vertices
		tics_shape_desc shape_desc = {.type = TICS_SHAPE_CONVEX,
									  .data.convex.vertices = vertex_buffers[i],
									  .data.convex.vertex_count = vertex_buffer_sizes[i]};
		tics_shape_id shape = tics_create_shape(world, shape_desc);

		// Create Static Body using the GLTF Transform
		tics_static_body_desc body_desc = {0};

		// Extract pos/rot from the default transform matrix
		Vector3 pos = {obj->default_transform.m12, obj->default_transform.m13,
					   obj->default_transform.m14};
		Quaternion rot = QuaternionFromMatrix(obj->default_transform);

		body_desc.transform.position = positions[i];
		body_desc.transform.rotation = rotations[i];
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
