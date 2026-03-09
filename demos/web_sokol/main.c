#include "tics.h"
#include <math.h>
#include <stdio.h>

// --- SOKOL SETUP -------------------------------------------------------------
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

// --- 3D MATH -----------------------------------------------------------------
typedef struct {
	float m[16];
} mat4_t;

static mat4_t mat4_mul(mat4_t a, mat4_t b) {
	mat4_t res = {0};
	for (int c = 0; c < 4; c++) {
		for (int r = 0; r < 4; r++) {
			res.m[c * 4 + r] = a.m[0 * 4 + r] * b.m[c * 4 + 0] + a.m[1 * 4 + r] * b.m[c * 4 + 1] +
							   a.m[2 * 4 + r] * b.m[c * 4 + 2] + a.m[3 * 4 + r] * b.m[c * 4 + 3];
		}
	}
	return res;
}

static mat4_t mat4_persp(float fov_y, float aspect, float n, float f) {
	mat4_t m = {0};
	float t = tanf(fov_y * 0.5f);
	m.m[0] = 1.0f / (t * aspect);
	m.m[5] = 1.0f / t;
	m.m[10] = -(f + n) / (f - n);
	m.m[11] = -1.0f;
	m.m[14] = -(2.0f * f * n) / (f - n);
	return m;
}

static mat4_t mat4_from_tics(tics_transform t) {
	float x = t.rotation.x, y = t.rotation.y, z = t.rotation.z, w = t.rotation.w;
	float x2 = x + x, y2 = y + y, z2 = z + z;
	float xx = x * x2, xy = x * y2, xz = x * z2;
	float yy = y * y2, yz = y * z2, zz = z * z2;
	float wx = w * x2, wy = w * y2, wz = w * z2;

	mat4_t m = {0};
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

// --- GEOMETRY & SHADERS ------------------------------------------------------

// 3D Cube: Positions + Colors
static const float cube_vertices[] = {
	-1.0, -1.0, 1.0,  1.0,	0.0,  0.0,	1.0,  1.0,	-1.0, 1.0,	1.0,  0.0,	0.0,  1.0,	1.0,  1.0,
	1.0,  1.0,	0.0,  0.0,	1.0,  -1.0, 1.0,  1.0,	1.0,  0.0,	0.0,  1.0,	-1.0, -1.0, -1.0, 0.0,
	1.0,  0.0,	1.0,  -1.0, 1.0,  -1.0, 0.0,  1.0,	0.0,  1.0,	1.0,  1.0,	-1.0, 0.0,	1.0,  0.0,
	1.0,  1.0,	-1.0, -1.0, 0.0,  1.0,	0.0,  1.0,	1.0,  -1.0, -1.0, 0.0,	0.0,  1.0,	1.0,  1.0,
	1.0,  -1.0, 0.0,  0.0,	1.0,  1.0,	1.0,  1.0,	1.0,  0.0,	0.0,  1.0,	1.0,  1.0,	-1.0, 1.0,
	0.0,  0.0,	1.0,  1.0,	-1.0, -1.0, -1.0, 1.0,	1.0,  0.0,	1.0,  -1.0, -1.0, 1.0,	1.0,  1.0,
	0.0,  1.0,	-1.0, 1.0,	1.0,  1.0,	1.0,  0.0,	1.0,  -1.0, 1.0,  -1.0, 1.0,  1.0,	0.0,  1.0,
	-1.0, 1.0,	-1.0, 0.0,	1.0,  1.0,	1.0,  -1.0, 1.0,  1.0,	0.0,  1.0,	1.0,  1.0,	1.0,  1.0,
	1.0,  0.0,	1.0,  1.0,	1.0,  1.0,	1.0,  -1.0, 0.0,  1.0,	1.0,  1.0,	-1.0, -1.0, -1.0, 1.0,
	0.0,  1.0,	1.0,  1.0,	-1.0, -1.0, 1.0,  0.0,	1.0,  1.0,	1.0,  -1.0, 1.0,  1.0,	0.0,  1.0,
	1.0,  -1.0, -1.0, 1.0,	1.0,  0.0,	1.0,  1.0,
};

static const uint16_t cube_indices[] = {0,	1,	2,	0,	2,	3,	4,	5,	6,	4,	6,	7,
										8,	9,	10, 8,	10, 11, 12, 13, 14, 12, 14, 15,
										16, 17, 18, 16, 18, 19, 20, 21, 22, 20, 22, 23};

#if defined(__EMSCRIPTEN__) || defined(__ANDROID__)
#define SHADER_PREFIX "#version 300 es\n"
#else
#define SHADER_PREFIX "#version 330\n"
#endif

static const char* vs_source = SHADER_PREFIX "uniform mat4 mvp;\n"
											 "layout(location=0) in vec4 position;\n"
											 "layout(location=1) in vec4 color0;\n"
											 "out vec4 color;\n"
											 "void main() {\n"
											 "  gl_Position = mvp * position;\n"
											 "  color = color0;\n"
											 "}\n";

static const char* fs_source = SHADER_PREFIX "precision mediump float;\n"
											 "in vec4 color;\n"
											 "out vec4 frag_color;\n"
											 "void main() {\n"
											 "  frag_color = color;\n"
											 "}\n";

// --- APP STATE ---------------------------------------------------------------
static struct {
	tics_world* world;
	tics_body_id falling_box;

	sg_pipeline pip;
	sg_bindings bind;
	sg_pass_action pass_action;
} state;

// --- APP LIFECYCLE -----------------------------------------------------------

static void init(void) {
	// 1. Setup Graphics
	sg_setup(&(sg_desc){.environment = sglue_environment()});

	state.pass_action = (sg_pass_action){
		.colors[0] = {.load_action = SG_LOADACTION_CLEAR, .clear_value = {0.1f, 0.1f, 0.1f, 1.0f}}};

	// 2. Setup Geometry Buffers
	state.bind.vertex_buffers[0] =
		sg_make_buffer(&(sg_buffer_desc){.usage = {.vertex_buffer = true, .immutable = true},
										 .size = sizeof(cube_vertices),
										 .data = SG_RANGE(cube_vertices)});

	state.bind.index_buffer =
		sg_make_buffer(&(sg_buffer_desc){.usage = {.index_buffer = true, .immutable = true},
										 .size = sizeof(cube_indices),
										 .data = SG_RANGE(cube_indices)});

	// 3. Setup Shader & Pipeline
	sg_shader shd = sg_make_shader(
		&(sg_shader_desc){.vertex_func.source = vs_source,
						  .fragment_func.source = fs_source,
						  .uniform_blocks[0] = {.stage = SG_SHADERSTAGE_VERTEX,
												.size = sizeof(mat4_t),
												.glsl_uniforms[0] = {.type = SG_UNIFORMTYPE_MAT4,
																	 .glsl_name = "mvp"}}});

	state.pip = sg_make_pipeline(
		&(sg_pipeline_desc){.shader = shd,
							.layout = {.attrs = {[0] = {.format = SG_VERTEXFORMAT_FLOAT3},
												 [1] = {.format = SG_VERTEXFORMAT_FLOAT4}}},
							.index_type = SG_INDEXTYPE_UINT16,
							.depth = {.compare = SG_COMPAREFUNC_LESS_EQUAL, .write_enabled = true},
							.cull_mode = SG_CULLMODE_BACK});

	// 4. Setup Tics Physics
	state.world = tics_world_create((tics_world_desc){.gravity = {0, -9.81f, 0}});

	tics_shape_id box_shape = tics_create_shape(
		state.world, (tics_shape_desc){.type = TICS_SHAPE_SPHERE,
									   .data.sphere = {.center = {0}, .radius = 1.0f}});

	state.falling_box = tics_world_add_rigid_body(
		state.world,
		(tics_rigid_body_desc){.transform = {.position = {0, 15.0f, 0}, .rotation = {0, 0, 0, 1}},
							   .shape = box_shape,
							   .mass = 1.0f,
							   .angular_velocity = {2.0f, 1.5f, 0.5f}});
}

static void frame(void) {
	// 1. Step Physics
	tics_world_step(state.world, 1.0f / 60.0f);

	// 2. Setup Camera Matrices
	float w = (float)sapp_width();
	float h = (float)sapp_height();
	mat4_t proj = mat4_persp(60.0f * (3.14159265f / 180.0f), w / h, 0.01f, 100.0f);

	// Move camera slightly up and back
	mat4_t view = {.m = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, -5.0f, -25.0f, 1}};
	mat4_t view_proj = mat4_mul(proj, view);

	// 3. Render Sequence
	sg_begin_pass(&(sg_pass){.action = state.pass_action, .swapchain = sglue_swapchain()});

	sg_apply_pipeline(state.pip);
	sg_apply_bindings(&state.bind);

	// Get current physics transform and update MVP matrix
	tics_transform box_tf = tics_body_get_transform(state.world, state.falling_box);
	mat4_t model = mat4_from_tics(box_tf);
	mat4_t mvp = mat4_mul(view_proj, model);

	sg_apply_uniforms(0, &SG_RANGE(mvp));
	sg_draw(0, 36, 1);

	sg_end_pass();
	sg_commit();
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
					   .width = 800,
					   .height = 600,
					   .window_title = "Tics Physics Demo"};
}
