#include "models_data.h"
#include "sokol_util.h"
#include "tics.h"

#include <stdio.h>
#include <stdlib.h>

#define SOKOL_IMPL
#if defined(__EMSCRIPTEN__)
#define SOKOL_GLES3
#elif defined(_WIN32)
#define SOKOL_D3D11
#elif defined(__APPLE__)
#define SOKOL_METAL
#else
#define SOKOL_GLCORE
#endif

#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_glue.h"

#define PHYSICS_TIMESTEP (1.0f / 60.0f)
#define DYNAMIC_BODIES 2000
#define SPAWN_INTERVAL 0.01f
#define MAX_FRAME_TIME 0.25f
#define MAX_STATIC_GROUNDS 64

// --- SHADERS (Flat Shading via Derivatives) ----------------------------------
#if defined(__EMSCRIPTEN__) || defined(__ANDROID__)
#define SHADER_PREFIX "#version 300 es\n"
#else
#define SHADER_PREFIX "#version 330\n"
#endif

static const char* vs_source = SHADER_PREFIX "uniform mat4 mvp;\n"
											 "uniform mat4 mv;\n"
											 "layout(location=0) in vec3 position;\n"
											 "out vec3 v_pos;\n"
											 "void main() {\n"
											 "  gl_Position = mvp * vec4(position, 1.0);\n"
											 "  v_pos = (mv * vec4(position, 1.0)).xyz;\n"
											 "}\n";

static const char* fs_source =
	SHADER_PREFIX "precision highp float;\n"
				  "uniform vec4 color;\n"
				  "in vec3 v_pos;\n"
				  "out vec4 frag_color;\n"
				  "void main() {\n"
				  "  vec3 normal = normalize(cross(dFdx(v_pos), dFdy(v_pos)));\n"
				  "  vec3 light_dir = normalize(-v_pos);\n"
				  "  float n_dot_l = max(dot(normal, light_dir), 0.2);\n"
				  "  frag_color = vec4(color.rgb * n_dot_l, color.a);\n"
				  "}\n";

// --- SHADER PARAMS -----------------------------------------------------------
typedef struct {
	su_mat4 mvp;
	su_mat4 mv;
} vs_params_t;

typedef struct {
	float color[4];
} fs_params_t;

// --- APP STATE ---------------------------------------------------------------
typedef struct {
	tics_body_id body;
	int shape_idx; // 0 = cube, 1 = sphere
	float color[4];
} game_entity;

static struct {
	tics_world* world;
	float accumulator;

	su_input input;
	su_camera camera;

	sg_pass_action pass_action;
	sg_pipeline pip;

	sg_bindings bind_grounds[MAX_STATIC_GROUNDS];
	sg_bindings bind_cube;
	sg_bindings bind_sphere;

	game_entity entities[DYNAMIC_BODIES];
	int entity_count;

	tics_shape_id sh_cube;
	tics_shape_id sh_sphere;
	float spawn_timer;
} state;

// --- UTILS -------------------------------------------------------------------
float random_float(float min, float max) {
	return min + (float)rand() / ((float)RAND_MAX / (max - min));
}

static su_mat4 mat4_from_tics(tics_transform t) {
	float x = t.rotation.x, y = t.rotation.y, z = t.rotation.z, w = t.rotation.w;
	float x2 = x + x, y2 = y + y, z2 = z + z;
	float xx = x * x2, xy = x * y2, xz = x * z2;
	float yy = y * y2, yz = y * z2, zz = z * z2;
	float wx = w * x2, wy = w * y2, wz = w * z2;

	su_mat4 m = {0};
	m.m[0] = 1.0f - (yy + zz);
	m.m[1] = xy + wz;
	m.m[2] = xz - wy;
	m.m[4] = xy - wz;
	m.m[5] = 1.0f - (xx + zz);
	m.m[6] = yz + wx;
	m.m[8] = xz + wy;
	m.m[9] = yz - wx;
	m.m[10] = 1.0f - (xx + yy);
	m.m[12] = t.position.x;
	m.m[13] = t.position.y;
	m.m[14] = t.position.z;
	m.m[15] = 1.0f;
	return m;
}

static sg_buffer make_vbuf(const float* data, size_t count) {
	return sg_make_buffer(&(sg_buffer_desc){.usage = {.vertex_buffer = true, .immutable = true},
											.data = (sg_range){data, count * 3 * sizeof(float)}});
}

static sg_buffer make_ibuf(const uint32_t* data, size_t count) {
	return sg_make_buffer(&(sg_buffer_desc){.usage = {.index_buffer = true, .immutable = true},
											.data = (sg_range){data, count * sizeof(uint32_t)}});
}

static void spawn_entity(void) {
	if (state.entity_count >= DYNAMIC_BODIES) return;

	bool is_cube = (rand() % 2 == 0);
	float rand_ang = random_float(0, 3.1415f * 2.0f);
	su_vec3 axis =
		su_vec3_normalize((su_vec3){random_float(-1, 1), random_float(-1, 1), random_float(-1, 1)});
	tics_quat start_rot = {axis.x * sinf(rand_ang / 2), axis.y * sinf(rand_ang / 2),
						   axis.z * sinf(rand_ang / 2), cosf(rand_ang / 2)};

	tics_body_id body = tics_world_add_rigid_body(
		state.world,
		(tics_rigid_body_desc){.shape = is_cube ? state.sh_cube : state.sh_sphere,
							   .mass = 3.0f,
							   .elasticity = 1.0f,
							   .gravity_scale = 1.0f,
							   .transform = {.position = {random_float(-4, 4), random_float(5, 25),
														  random_float(-4, 4)},
											 .rotation = start_rot}});

	state.entities[state.entity_count] = (game_entity){
		.body = body,
		.shape_idx = is_cube ? 0 : 1,
		.color = {is_cube ? 0.74f : 1.0f, is_cube ? 0.16f : 0.79f, is_cube ? 0.25f : 0.0f, 1.0f}};
	state.entity_count++;
}

static void init(void) {
	srand(42);

	// Setup Graphics & State
	sg_setup(&(sg_desc){.environment = sglue_environment()});
	state.pass_action = (sg_pass_action){
		.colors[0] = {.load_action = SG_LOADACTION_CLEAR,
					  .clear_value = {0.96f, 0.96f, 0.96f, 1.0f}} // RayWhite
	};

	state.camera.position = (su_vec3){30.0f, 20.0f, 30.0f};
	state.camera.target = (su_vec3){0.0f, 2.0f, 0.0f};
	state.camera.up = (su_vec3){0.0f, 1.0f, 0.0f};

	// Setup Shaders
	sg_shader shd = sg_make_shader(&(sg_shader_desc){
		.vertex_func.source = vs_source,
		.fragment_func.source = fs_source,
		.uniform_blocks[0] =
			{.stage = SG_SHADERSTAGE_VERTEX,
			 .size = sizeof(vs_params_t),
			 .glsl_uniforms = {[0] = {.type = SG_UNIFORMTYPE_MAT4, .glsl_name = "mvp"},
							   [1] = {.type = SG_UNIFORMTYPE_MAT4, .glsl_name = "mv"}}},
		.uniform_blocks[1] = {
			.stage = SG_SHADERSTAGE_FRAGMENT,
			.size = sizeof(fs_params_t),
			.glsl_uniforms[0] = {.type = SG_UNIFORMTYPE_FLOAT4, .glsl_name = "color"}}});

	state.pip = sg_make_pipeline(&(sg_pipeline_desc){
		.shader = shd,
		.layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT3,
		.index_type = SG_INDEXTYPE_UINT32,
		.depth = {.compare = SG_COMPAREFUNC_LESS_EQUAL, .write_enabled = true},
		.cull_mode = SG_CULLMODE_BACK,
		.face_winding = SG_FACEWINDING_CCW,
	});

	// Setup Physics World
	state.world = tics_world_create((tics_world_desc){
		.gravity = {0.0f, -9.81f, 0.0f}, .air_friction_linear = 0.05, .air_friction_angular = 0.2});

	// Setup Static Geometry
	int ground_count =
		static_object_count < MAX_STATIC_GROUNDS ? static_object_count : MAX_STATIC_GROUNDS;
	for (int i = 0; i < ground_count; i++) {
		state.bind_grounds[i].vertex_buffers[0] =
			make_vbuf((float*)ground_vertex_buffers[i], ground_vertex_buffer_sizes[i]);
		state.bind_grounds[i].index_buffer =
			make_ibuf(ground_index_buffers[i], ground_index_buffer_sizes[i]);

		tics_shape_id shape = tics_create_shape(
			state.world,
			(tics_shape_desc){.type = TICS_SHAPE_CONVEX,
							  .data.convex.vertices = ground_vertex_buffers[i],
							  .data.convex.vertex_count = ground_vertex_buffer_sizes[i]});

		tics_debug_upload_shape_mesh(shape, ground_vertex_buffers[i], ground_index_buffers[i],
									 ground_index_buffer_sizes[i]);

		tics_world_add_static_body(
			state.world, (tics_static_body_desc){.transform = {.position = ground_positions[i],
															   .rotation = ground_rotations[i]},
												 .shape = shape,
												 .elasticity = 0.5f});
	}

	// Setup Dynamic Entities
	state.bind_cube.vertex_buffers[0] = make_vbuf((float*)cube_Cube_vertices, 24);
	state.bind_cube.index_buffer = make_ibuf(cube_Cube_indices, 36);

	state.bind_sphere.vertex_buffers[0] = make_vbuf((float*)icosphere_Icosphere_vertices, 240);
	state.bind_sphere.index_buffer = make_ibuf(icosphere_Icosphere_indices, 240);

	state.sh_cube =
		tics_create_shape(state.world, (tics_shape_desc){.type = TICS_SHAPE_CONVEX,
														 .data.convex.vertices = cube_Cube_vertices,
														 .data.convex.vertex_count = 24});
	state.sh_sphere = tics_create_shape(
		state.world, (tics_shape_desc){.type = TICS_SHAPE_CONVEX,
									   .data.convex.vertices = icosphere_Icosphere_vertices,
									   .data.convex.vertex_count = 240});

	tics_debug_upload_shape_mesh(state.sh_cube, cube_Cube_vertices, cube_Cube_indices,
								 cube_index_buffer_sizes[0]);
	tics_debug_upload_shape_mesh(state.sh_sphere, icosphere_Icosphere_vertices,
								 icosphere_Icosphere_indices, icosphere_index_buffer_sizes[0]);
}

static void frame(void) {
	float dt = (float)sapp_frame_duration();
	float clamped_dt = dt > MAX_FRAME_TIME ? MAX_FRAME_TIME : dt;

	// Camera
	su_camera_navigate(&state.camera, &state.input, clamped_dt);

	// Physics Step
	state.accumulator += clamped_dt;
	while (state.accumulator >= PHYSICS_TIMESTEP) {
		tics_world_step(state.world, PHYSICS_TIMESTEP);
		state.accumulator -= PHYSICS_TIMESTEP;
	}

	// Object Spawning
	state.spawn_timer += clamped_dt;
	if (state.spawn_timer > SPAWN_INTERVAL) { // Spawns 10 objects per second
		spawn_entity();
		state.spawn_timer = 0.0f;
	}

	// Object Deletion
	for (int i = 0; i < state.entity_count;) {
		tics_transform tf = tics_body_get_transform(state.world, state.entities[i].body);

		if (tf.position.y < -50.0f) {
			tics_world_remove_body(state.world, state.entities[i].body);

			// Remove from the array by swapping with the last element
			state.entities[i] = state.entities[state.entity_count - 1];
			state.entity_count--;
		}
		else { i++; }
	}

	// Rendering
	// Matrix Math
	float w = (float)sapp_width();
	float h = (float)sapp_height();
	su_mat4 proj = su_mat4_persp(45.0f * (3.14159f / 180.0f), w / h, 0.1f, 1000.0f);
	su_mat4 view = su_mat4_look_at(state.camera.position, state.camera.target, state.camera.up);
	sg_begin_pass(&(sg_pass){.action = state.pass_action, .swapchain = sglue_swapchain()});
	sg_apply_pipeline(state.pip);
	// Statics
	fs_params_t fs_static = {.color = {0.78f, 0.78f, 0.78f, 1.0f}}; // Light Gray
	int ground_count =
		static_object_count < MAX_STATIC_GROUNDS ? static_object_count : MAX_STATIC_GROUNDS;
	for (int i = 0; i < ground_count; i++) {
		su_mat4 model = mat4_from_tics((tics_transform){ground_positions[i], ground_rotations[i]});
		vs_params_t vs_static = {.mvp = su_mat4_mul(proj, su_mat4_mul(view, model)),
								 .mv = su_mat4_mul(view, model)};

		sg_apply_bindings(&state.bind_grounds[i]);
		sg_apply_uniforms(0, &SG_RANGE(vs_static)); // Slot 0: vs_params_t
		sg_apply_uniforms(1, &SG_RANGE(fs_static)); // Slot 1: fs_params_t
		sg_draw(0, ground_index_buffer_sizes[i], 1);
	}
	// Dynamics
	for (int i = 0; i < state.entity_count; i++) {
		game_entity* ent = &state.entities[i];
		tics_transform tf = tics_body_get_transform(state.world, ent->body);

		su_mat4 model = mat4_from_tics(tf);
		vs_params_t vs_dyn = {.mvp = su_mat4_mul(proj, su_mat4_mul(view, model)),
							  .mv = su_mat4_mul(view, model)};
		fs_params_t fs_dyn = {
			.color = {ent->color[0], ent->color[1], ent->color[2], ent->color[3]}};

		sg_apply_bindings(ent->shape_idx == 0 ? &state.bind_cube : &state.bind_sphere);
		sg_apply_uniforms(0, &SG_RANGE(vs_dyn)); // Slot 0: vs_params_t
		sg_apply_uniforms(1, &SG_RANGE(fs_dyn)); // Slot 1: fs_params_t
		sg_draw(0, ent->shape_idx == 0 ? 36 : 240, 1);
	}

	sg_end_pass();
	sg_commit();

	su_input_end_frame(&state.input);
}

static void event(const sapp_event* ev) {
	su_input_update(&state.input, ev);
}

static void cleanup(void) {
	tics_world_destroy(state.world);
	sg_shutdown();
}

sapp_desc sokol_main(int argc, char* argv[]) {
	(void)argc;
	(void)argv;
	return (sapp_desc){.init_cb = init,
					   .frame_cb = frame,
					   .cleanup_cb = cleanup,
					   .event_cb = event,
					   .width = 1280,
					   .height = 614,
					   .window_title = "Tics Physics Demo",
					   .icon.sokol_default = true};
}
