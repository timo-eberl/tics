#include "blick_os.h"
#include "blick_protocol.h"
#include "raylib_util.h"
#include "shaders.h"

#include <raymath.h>
#include <rlgl.h>

#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const int SPHERE_RESOLUTION = 64;
const int SPHERE_WIRE_RINGS = 5;
const int SPHERE_WIRE_SLICES = 8;

// For sphere rendering we will use raw OpenGL, because raylib does not support efficient line
// drawing. This should be safe since glDrawElements exists on all backends supported by Raylib.

// forward declare glDrawElements
#if defined(__cplusplus)
extern "C" {
#endif
void glDrawElements(unsigned int mode, int count, unsigned int type, const void* indices);
#if defined(__cplusplus)
}
#endif
// Define constants if missing
#ifndef GL_LINES
#define GL_LINES 0x0001
#endif
#ifndef GL_UNSIGNED_SHORT
#define GL_UNSIGNED_SHORT 0x1403
#endif

static Mesh sphere_mesh = {0};
static Mesh sphere_wire_mesh = {0};
static Material sphere_material = {0};
static Material sphere_wire_material = {0};
static Mesh cylinder_mesh = {0};
static Mesh cylinder_wire_mesh = {0};
static Mesh cube_mesh = {0};
static Mesh cube_wire_mesh = {0};
static int view_pos_loc = -1;

static Mesh gen_sphere_wires(float radius, int rings, int slices, int segments);
static Mesh gen_cylinder_wires(float radius, float height, int slices);
static Mesh gen_cube_wires(void);

static void init_graphics_resources() {
	sphere_mesh = GenMeshSphere(1.0f, SPHERE_RESOLUTION, SPHERE_RESOLUTION);
	sphere_wire_mesh =
		gen_sphere_wires(1.0f, SPHERE_WIRE_RINGS, SPHERE_WIRE_SLICES, SPHERE_RESOLUTION);

	cylinder_mesh = GenMeshCylinder(1.0f, 1.0f, SPHERE_RESOLUTION);
	cylinder_wire_mesh = gen_cylinder_wires(1.0f, 1.0f, SPHERE_WIRE_SLICES);

	cube_mesh = GenMeshCube(1.0f, 1.0f, 1.0f);
	cube_wire_mesh = gen_cube_wires();

	Shader shader = LoadShaderFromMemory(VS_CODE, FS_CODE);
	Shader unlit_shader = LoadShaderFromMemory(VS_WIRE_CODE, FS_WIRE_CODE);

	// Get uniform location for camera position
	view_pos_loc = GetShaderLocation(shader, "viewPos");

	sphere_wire_material = LoadMaterialDefault();
	sphere_wire_material.shader = unlit_shader; // Assign the unlit shader

	sphere_material = LoadMaterialDefault();
	sphere_material.shader = shader; // Assign our custom shader
}

static void cleanup_graphics_resources() {
	UnloadShader(sphere_material.shader);
	UnloadShader(sphere_wire_material.shader);
	UnloadMesh(sphere_mesh);
	UnloadMesh(sphere_wire_mesh);
	UnloadMesh(cylinder_mesh);
	UnloadMesh(cylinder_wire_mesh);
	UnloadMesh(cube_mesh);
	UnloadMesh(cube_wire_mesh);
}

static void draw_mesh_lines(Mesh mesh, Material material, Matrix transform) {
	rlEnableShader(material.shader.id);

	// Send Matrices
	Matrix matModel = transform;
	Matrix matView = rlGetMatrixModelview();
	Matrix matModelView = MatrixMultiply(matModel, matView);
	Matrix matProjection = rlGetMatrixProjection();
	Matrix matMvp = MatrixMultiply(matModelView, matProjection);

	rlSetUniformMatrix(material.shader.locs[SHADER_LOC_MATRIX_MVP], matMvp);
	rlSetUniformMatrix(material.shader.locs[SHADER_LOC_MATRIX_MODEL], matModel);

	// Bind Color
	Color color = material.maps[MATERIAL_MAP_DIFFUSE].color;
	float c[4] = {color.r / 255.0f, color.g / 255.0f, color.b / 255.0f, color.a / 255.0f};
	rlSetUniform(material.shader.locs[SHADER_LOC_COLOR_DIFFUSE], c, SHADER_UNIFORM_VEC4, 1);

	// Draw
	rlEnableVertexArray(mesh.vaoId);
	// 2 indices per line. mesh.triangleCount held our "edge count"
	// We use direct GL calls via rlgl to force LINES mode
	glDrawElements(GL_LINES, mesh.triangleCount * 2, GL_UNSIGNED_SHORT, 0);
	rlDisableVertexArray();

	rlDisableShader();
}

static Mesh gen_sphere_wires(float radius, int rings, int slices, int segments) {
	Mesh mesh = {0};

	// 1. Calculate counts
	// Meridians: 'slices/2' full circles. Each circle has 'segments' edges.
	// Latitudes: 'rings' circles. Each has 'segments' edges.
	int num_meridians = slices / 2;
	int total_edges = (num_meridians * segments) + (rings * segments);

	mesh.vertexCount = (num_meridians * segments) +
					   (rings * segments); // We duplicate verts for simplicity in this logic
	mesh.triangleCount =
		total_edges; // In lines mode, 'triangleCount' is often used as 'primitive count' or ignored

	// Allocate (Using malloc directly or Raylib's MemAlloc)
	mesh.vertices = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
	mesh.indices = (unsigned short*)MemAlloc(total_edges * 2 * sizeof(unsigned short));

	int vIndex = 0;
	int iIndex = 0;

	// --- Generate Meridians (Vertical Circles) ---
	for (int i = 0; i < num_meridians; i++) {
		float theta = (PI * i) / num_meridians;
		int baseVert = vIndex;

		for (int k = 0; k < segments; k++) {
			float alpha = (2.0f * PI * k) / segments;

			// Vertex Pos
			mesh.vertices[vIndex * 3 + 0] = radius * cosf(alpha) * cosf(theta);
			mesh.vertices[vIndex * 3 + 1] = radius * sinf(alpha);
			mesh.vertices[vIndex * 3 + 2] = -radius * cosf(alpha) * sinf(theta);

			// Indices (Connect k to k+1)
			mesh.indices[iIndex++] = vIndex;
			mesh.indices[iIndex++] = (k == segments - 1) ? baseVert : (vIndex + 1);

			vIndex++;
		}
	}

	// --- Generate Latitudes (Horizontal Rings) ---
	for (int i = 0; i < rings; i++) {
		float phi = PI * (float)(i + 1) / (rings + 1);
		float y = radius * cosf(phi);
		float r = radius * sinf(phi);
		int baseVert = vIndex;

		for (int k = 0; k < segments; k++) {
			float alpha = (2.0f * PI * k) / segments;

			// Vertex Pos
			mesh.vertices[vIndex * 3 + 0] = r * cosf(alpha);
			mesh.vertices[vIndex * 3 + 1] = y;
			mesh.vertices[vIndex * 3 + 2] = r * sinf(alpha);

			// Indices
			mesh.indices[iIndex++] = vIndex;
			mesh.indices[iIndex++] = (k == segments - 1) ? baseVert : (vIndex + 1);

			vIndex++;
		}
	}

	UploadMesh(&mesh, false); // Upload to GPU
	return mesh;
}

static Mesh gen_cylinder_wires(float radius, float height, int slices) {
	Mesh mesh = {0};
	int total_edges = slices * 3; // Top ring, bottom ring, vertical lines
	mesh.vertexCount = slices * 2;
	mesh.triangleCount = total_edges;
	mesh.vertices = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
	mesh.indices = (unsigned short*)MemAlloc(total_edges * 2 * sizeof(unsigned short));

	int iIndex = 0;

	for (int i = 0; i < slices; i++) {
		float angle = (2.0f * PI * i) / slices;
		float cx = radius * cosf(angle);
		float cz = radius * sinf(angle);

		mesh.vertices[(i * 2) * 3 + 0] = cx;
		mesh.vertices[(i * 2) * 3 + 1] = height; // Top ring at 'height'
		mesh.vertices[(i * 2) * 3 + 2] = cz;

		mesh.vertices[(i * 2 + 1) * 3 + 0] = cx;
		mesh.vertices[(i * 2 + 1) * 3 + 1] = 0.0f; // Bottom ring at '0'
		mesh.vertices[(i * 2 + 1) * 3 + 2] = cz;

		int next_i = (i + 1) % slices;

		// Top ring edge
		mesh.indices[iIndex++] = i * 2;
		mesh.indices[iIndex++] = next_i * 2;

		// Bottom ring edge
		mesh.indices[iIndex++] = i * 2 + 1;
		mesh.indices[iIndex++] = next_i * 2 + 1;

		// Vertical edge
		mesh.indices[iIndex++] = i * 2;
		mesh.indices[iIndex++] = i * 2 + 1;
	}

	UploadMesh(&mesh, false);
	return mesh;
}

static Mesh gen_cube_wires(void) {
	Mesh mesh = {0};
	mesh.vertexCount = 8;
	mesh.triangleCount = 12;

	mesh.vertices = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
	mesh.indices = (unsigned short*)MemAlloc(mesh.triangleCount * 2 * sizeof(unsigned short));

	float vertices[8][3] = {
		{-0.5f, -0.5f, -0.5f}, { 0.5f, -0.5f, -0.5f},
		{ 0.5f, -0.5f,  0.5f}, {-0.5f, -0.5f,  0.5f},
		{-0.5f,  0.5f, -0.5f}, { 0.5f,  0.5f, -0.5f},
		{ 0.5f,  0.5f,  0.5f}, {-0.5f,  0.5f,  0.5f}
	};
	memcpy(mesh.vertices, vertices, sizeof(vertices));

	unsigned short indices[24] = {
		0, 1, 1, 2, 2, 3, 3, 0, // Bottom ring
		4, 5, 5, 6, 6, 7, 7, 4, // Top ring
		0, 4, 1, 5, 2, 6, 3, 7  // Vertical pillars
	};
	memcpy(mesh.indices, indices, sizeof(indices));

	UploadMesh(&mesh, false);
	return mesh;
}

static Color shade(Vector3 light_dir, Vector3 normal, Color base_color) {
	float n_dot_l = Vector3DotProduct(normal, light_dir);
	n_dot_l = fabsf(n_dot_l); // use abs so backfaces are shaded too
	float intensity = fmax(pow(n_dot_l, 0.2), 0.7);

	return (Color){(unsigned char)(base_color.r * intensity),
				   (unsigned char)(base_color.g * intensity),
				   (unsigned char)(base_color.b * intensity), base_color.a};
}

// Calculates face normal of a triangle and applies view-dependent lighting
static Color shade_triangle(Vector3 v0, Vector3 v1, Vector3 v2, Vector3 cam_pos, Color base_color) {
	Vector3 edge1 = Vector3Subtract(v1, v0);
	Vector3 edge2 = Vector3Subtract(v2, v0);
	Vector3 normal = Vector3Normalize(Vector3CrossProduct(edge1, edge2));
	Vector3 light_dir = Vector3Normalize(Vector3Subtract(cam_pos, v0)); // approximation

	return shade(light_dir, normal, base_color);
}

static Color unpack_color(uint32_t c) {
	// Unpack Color: 0xAABBGGRR
	Color color;
	color.a = (c >> 24) & 0xFF;
	color.b = (c >> 16) & 0xFF;
	color.g = (c >> 8) & 0xFF;
	color.r = (c) & 0xFF;
	return color;
}

static void draw_command(const blick_cmd* cmd, blick_shm_header* shm, Vector3 cam_pos) {
	Color color = unpack_color(cmd->color);

	switch (cmd->type) {

	case BLICK_CMD_LINE: {
		Vector3 start = {cmd->data.line.start.x, cmd->data.line.start.y, cmd->data.line.start.z};
		Vector3 end = {cmd->data.line.end.x, cmd->data.line.end.y, cmd->data.line.end.z};
		DrawLine3D(start, end, color);
	} break;

	case BLICK_CMD_ARROW: {
		Vector3 start = {cmd->data.arrow.start.x, cmd->data.arrow.start.y, cmd->data.arrow.start.z};
		Vector3 end = {cmd->data.arrow.end.x, cmd->data.arrow.end.y, cmd->data.arrow.end.z};

		// Draw the shaft (Exact line from start to end)
		DrawLine3D(start, end, color);

		// Draw the tip (Cone)
		Vector3 diff = Vector3Subtract(end, start);
		float length = Vector3Length(diff);

		if (length > 0.0001f) {
			// Heuristic: Head is 0.4 units long, but clamped to 20% of length for small arrows.
			// This ensures the arrow tip doesn't swallow the whole arrow when it's tiny.
			float head_len = fminf(0.4f, length * 0.2f);
			float head_radius = head_len * 0.4f;

			// Backtrack from 'end' to find the base of the cone
			Vector3 dir = Vector3Scale(diff, 1.0f / length);
			Vector3 head_base = Vector3Subtract(end, Vector3Scale(dir, head_len));

			// Draw Cylinder with 0 end-radius (a Cone). Tip is exactly at 'end'.
			DrawCylinderWiresEx(head_base, end, head_radius, 0.0f, 6, color);
		}
	} break;

	case BLICK_CMD_POINT: {
		Vector3 p = {cmd->data.point.pos.x, cmd->data.point.pos.y, cmd->data.point.pos.z};
		float r = cmd->data.point.radius;

		// Draw a 3D Crosshair centered at the point
		DrawLine3D((Vector3){p.x - r, p.y, p.z}, (Vector3){p.x + r, p.y, p.z}, color);
		DrawLine3D((Vector3){p.x, p.y - r, p.z}, (Vector3){p.x, p.y + r, p.z}, color);
		DrawLine3D((Vector3){p.x, p.y, p.z - r}, (Vector3){p.x, p.y, p.z + r}, color);
	} break;

	case BLICK_CMD_AABB: {
		Vector3 min = {cmd->data.aabb.min.x, cmd->data.aabb.min.y, cmd->data.aabb.min.z};
		Vector3 max = {cmd->data.aabb.max.x, cmd->data.aabb.max.y, cmd->data.aabb.max.z};
		BoundingBox box = {min, max};
		DrawBoundingBox(box, color);
	} break;

	case BLICK_CMD_TRIANGLE: {
		Vector3 a = {cmd->data.triangle.a.x, cmd->data.triangle.a.y, cmd->data.triangle.a.z};
		Vector3 b = {cmd->data.triangle.b.x, cmd->data.triangle.b.y, cmd->data.triangle.b.z};
		Vector3 c = {cmd->data.triangle.c.x, cmd->data.triangle.c.y, cmd->data.triangle.c.z};

		Color shaded = shade_triangle(a, b, c, cam_pos, color);
		DrawTriangle3D(a, b, c, shaded);
	} break;

	case BLICK_CMD_TRANSFORM: {
		Vector3 pos = {cmd->data.transform.pos.x, cmd->data.transform.pos.y,
					   cmd->data.transform.pos.z};
		Quaternion q = {cmd->data.transform.rot.x, cmd->data.transform.rot.y,
						cmd->data.transform.rot.z, cmd->data.transform.rot.w};

		float scale = cmd->data.transform.size;
		Vector3 right = Vector3RotateByQuaternion((Vector3){1.0f, 0.0f, 0.0f}, q);
		Vector3 up = Vector3RotateByQuaternion((Vector3){0.0f, 1.0f, 0.0f}, q);
		Vector3 forward = Vector3RotateByQuaternion((Vector3){0.0f, 0.0f, 1.0f}, q);

		// Draw Axis Gizmo (Red=X, Green=Y, Blue=Z)
		DrawLine3D(pos, Vector3Add(pos, Vector3Scale(right, scale)), RED);
		DrawLine3D(pos, Vector3Add(pos, Vector3Scale(up, scale)), GREEN);
		DrawLine3D(pos, Vector3Add(pos, Vector3Scale(forward, scale)), BLUE);
	} break;

	case BLICK_CMD_SPHERE: {
		float camera_pos[3] = {cam_pos.x, cam_pos.y, cam_pos.z};
		SetShaderValue(sphere_material.shader, view_pos_loc, camera_pos, SHADER_UNIFORM_VEC3);

		Quaternion q = {cmd->data.sphere.rot.x, cmd->data.sphere.rot.y, cmd->data.sphere.rot.z,
						cmd->data.sphere.rot.w};
		Matrix mat = QuaternionToMatrix(q);

		float radius = cmd->data.sphere.radius;
		// Apply scaling (The mesh is radius 1.0, so scale = radius)
		Matrix mat_scale = MatrixScale(radius, radius, radius);
		mat = MatrixMultiply(mat_scale, mat);

		// Inject translation directly into the matrix (Column-Major: m12, m13, m14)
		mat.m12 = cmd->data.sphere.pos.x;
		mat.m13 = cmd->data.sphere.pos.y;
		mat.m14 = cmd->data.sphere.pos.z;

		if (cmd->data.sphere.wireframe) {
			// rlPushMatrix();
			// rlMultMatrixf(MatrixToFloat(mat));
			// draw_sphere_wires_smooth(1.0f, SPHERE_WIRE_RINGS, 8, SPHERE_RESOLUTION, color);
			// rlPopMatrix();

			sphere_wire_material.maps[MATERIAL_MAP_DIFFUSE].color = color;
			// DrawMesh applies the matrix and renders the VBO residing in VRAM.
			draw_mesh_lines(sphere_wire_mesh, sphere_wire_material, mat);
		}
		else {
			sphere_material.maps[MATERIAL_MAP_DIFFUSE].color = color;
			// DrawMesh applies the matrix and renders the VBO residing in VRAM.
			DrawMesh(sphere_mesh, sphere_material, mat);
		}
	} break;

	case BLICK_CMD_CAPSULE: {
		float camera_pos[3] = {cam_pos.x, cam_pos.y, cam_pos.z};
		SetShaderValue(sphere_material.shader, view_pos_loc, camera_pos, SHADER_UNIFORM_VEC3);

		Vector3 p_a = {cmd->data.capsule.p_a.x, cmd->data.capsule.p_a.y, cmd->data.capsule.p_a.z};
		Vector3 p_b = {cmd->data.capsule.p_b.x, cmd->data.capsule.p_b.y, cmd->data.capsule.p_b.z};
		float r = cmd->data.capsule.radius;
		bool wire = cmd->data.capsule.wireframe;

		Material* mat_ptr = wire ? &sphere_wire_material : &sphere_material;
		mat_ptr->maps[MATERIAL_MAP_DIFFUSE].color = color;

		Matrix mat_a = MatrixMultiply(MatrixScale(r, r, r), MatrixTranslate(p_a.x, p_a.y, p_a.z));
		if (wire) draw_mesh_lines(sphere_wire_mesh, *mat_ptr, mat_a);
		else DrawMesh(sphere_mesh, *mat_ptr, mat_a);

		Matrix mat_b = MatrixMultiply(MatrixScale(r, r, r), MatrixTranslate(p_b.x, p_b.y, p_b.z));
		if (wire) draw_mesh_lines(sphere_wire_mesh, *mat_ptr, mat_b);
		else DrawMesh(sphere_mesh, *mat_ptr, mat_b);

		Vector3 diff = Vector3Subtract(p_b, p_a);
		float length = Vector3Length(diff);

		if (length > 0.0001f) {
			Vector3 dir = Vector3Scale(diff, 1.0f / length);
			Vector3 up = {0.0f, 1.0f, 0.0f};
			Vector3 axis = Vector3CrossProduct(up, dir);
			float dot = Vector3DotProduct(up, dir);
			Quaternion q;

			if (dot < -0.9999f) {
				q = QuaternionFromAxisAngle((Vector3){1.0f, 0.0f, 0.0f}, PI);
			} else if (dot > 0.9999f) {
				q = QuaternionIdentity();
			} else {
				q.x = axis.x;
				q.y = axis.y;
				q.z = axis.z;
				q.w = 1.0f + dot;
				q = QuaternionNormalize(q);
			}

			// Cylinder base starts at 0 and ends at Y=1. Scale Y by length, then position at p_a.
			Matrix mat_cyl = MatrixScale(r, length, r);
			mat_cyl = MatrixMultiply(mat_cyl, QuaternionToMatrix(q));
			mat_cyl = MatrixMultiply(mat_cyl, MatrixTranslate(p_a.x, p_a.y, p_a.z));

			if (wire) draw_mesh_lines(cylinder_wire_mesh, *mat_ptr, mat_cyl);
			else DrawMesh(cylinder_mesh, *mat_ptr, mat_cyl);
		}
	} break;

	case BLICK_CMD_OBB: {
		float camera_pos[3] = {cam_pos.x, cam_pos.y, cam_pos.z};
		SetShaderValue(sphere_material.shader, view_pos_loc, camera_pos, SHADER_UNIFORM_VEC3);

		Quaternion q = {cmd->data.obb.rot.x, cmd->data.obb.rot.y, cmd->data.obb.rot.z,
						cmd->data.obb.rot.w};
		Matrix mat = QuaternionToMatrix(q);

		Vector3 ext = {cmd->data.obb.extents.x, cmd->data.obb.extents.y, cmd->data.obb.extents.z};
		Matrix mat_scale = MatrixScale(ext.x, ext.y, ext.z);
		mat = MatrixMultiply(mat_scale, mat);

		mat.m12 = cmd->data.obb.pos.x;
		mat.m13 = cmd->data.obb.pos.y;
		mat.m14 = cmd->data.obb.pos.z;

		if (cmd->data.obb.wireframe) {
			sphere_wire_material.maps[MATERIAL_MAP_DIFFUSE].color = color;
			draw_mesh_lines(cube_wire_mesh, sphere_wire_material, mat);
		} else {
			sphere_material.maps[MATERIAL_MAP_DIFFUSE].color = color;
			DrawMesh(cube_mesh, sphere_material, mat);
		}
	} break;

	case BLICK_CMD_TEXT:
		break; // text is drawn after the 3D pass as an overlay

	case BLICK_CMD_DRAW_MESH: {
		rlPushMatrix();
		Quaternion q = {cmd->data.mesh.rot.x, cmd->data.mesh.rot.y, cmd->data.mesh.rot.z,
						cmd->data.mesh.rot.w};
		Matrix mat = QuaternionToMatrix(q);
		// Inject translation directly into the matrix (Column-Major: m12, m13, m14)
		mat.m12 = cmd->data.mesh.pos.x;
		mat.m13 = cmd->data.mesh.pos.y;
		mat.m14 = cmd->data.mesh.pos.z;
		rlMultMatrixf(MatrixToFloat(mat));

		rlColor4ub(color.r, color.g, color.b, color.a);

		uint32_t offset = cmd->data.mesh.offset;
		uint32_t v_count = cmd->data.mesh.vertex_count;
		float* pool = shm->mesh_pool;

		if (cmd->data.mesh.wireframe) {
			// Simply using RL_LINES on a triangle mesh doesn't work correctly
			// Expand Triangles (3 verts) into Lines (6 verts)
			rlBegin(RL_LINES);
			for (uint32_t v = 0; v < v_count; v += 3) {
				if (v + 2 >= v_count) break;

				float* p0 = &pool[offset + ((v + 0) * 3)];
				float* p1 = &pool[offset + ((v + 1) * 3)];
				float* p2 = &pool[offset + ((v + 2) * 3)];
				// clang-format off
				// Edge 0-1, Edge 1-2, Edge 2-0
				rlVertex3f(p0[0], p0[1], p0[2]); rlVertex3f(p1[0], p1[1], p1[2]);
				rlVertex3f(p1[0], p1[1], p1[2]); rlVertex3f(p2[0], p2[1], p2[2]);
				rlVertex3f(p2[0], p2[1], p2[2]); rlVertex3f(p0[0], p0[1], p0[2]);
				// clang-format on
			}
			rlEnd();
		}
		else {
			// Calculate local camera position for lighting
			Vector3 local_cam_pos = Vector3Transform(cam_pos, MatrixInvert(mat));

			rlBegin(RL_TRIANGLES);
			for (uint32_t v = 0; v < v_count; v += 3) {
				if (v + 2 >= v_count) break;

				float* p0 = &pool[offset + ((v + 0) * 3)];
				float* p1 = &pool[offset + ((v + 1) * 3)];
				float* p2 = &pool[offset + ((v + 2) * 3)];
				Vector3 v0 = {p0[0], p0[1], p0[2]};
				Vector3 v1 = {p1[0], p1[1], p1[2]};
				Vector3 v2 = {p2[0], p2[1], p2[2]};

				// Lighting in local space
				Color shaded = shade_triangle(v0, v1, v2, local_cam_pos, color);
				rlColor4ub(shaded.r, shaded.g, shaded.b, shaded.a);

				rlVertex3f(v0.x, v0.y, v0.z);
				rlVertex3f(v1.x, v1.y, v1.z);
				rlVertex3f(v2.x, v2.y, v2.z);
			}
			rlEnd();
		}

		rlPopMatrix();
	} break;

	default:
		break;
	}
}

static void draw_text_bordered(const char* text, int x, int y, int size, Color color) {
	float rn = color.r / 255.0f;
	float gn = color.g / 255.0f;
	float bn = color.b / 255.0f;
	// Linearize color (approximate sRGB by squaring) + Luminance Calculation
	float lum = 0.2126f * (rn * rn) + 0.7152f * (gn * gn) + 0.0722f * (bn * bn);
	Color outline_color = (lum > 0.1f) ? BLACK : WHITE;
	// Draw outline by offsetting text multiple times
	for (int u = -1; u <= 1; u++) {
		for (int v = -1; v <= 1; v++) {
			DrawText(text, x + u, y + v, size, outline_color);
		}
	}
	DrawText(text, x, y, size, color);
}

typedef struct {
	Vector2 screen_end;
	float z;
	Color color;
	const char* label;
} gizmo_axis;

static void draw_camera_orientation_gizmo(Camera3D camera, int center_x, int center_y, float size) {
	Matrix view = GetCameraMatrix(camera);
	Vector3 origin = Vector3Transform((Vector3){0.0f, 0.0f, 0.0f}, view);

	Vector3 dirs[3] = {
		Vector3Subtract(Vector3Transform((Vector3){1.0f, 0.0f, 0.0f}, view), origin),
		Vector3Subtract(Vector3Transform((Vector3){0.0f, 1.0f, 0.0f}, view), origin),
		Vector3Subtract(Vector3Transform((Vector3){0.0f, 0.0f, 1.0f}, view), origin)
	};

	gizmo_axis axes[3] = {
		{{center_x + dirs[0].x * size, center_y - dirs[0].y * size}, dirs[0].z, RED, "x"},
		{{center_x + dirs[1].x * size, center_y - dirs[1].y * size}, dirs[1].z, GREEN, "y"},
		{{center_x + dirs[2].x * size, center_y - dirs[2].y * size}, dirs[2].z, BLUE, "z"}
	};

	// Sort and draw back-to-front based on view space Z depth.
	// In standard OpenGL view space, a more negative Z means further away from the camera.
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2 - i; j++) {
			if (axes[j].z > axes[j + 1].z) {
				gizmo_axis temp = axes[j];
				axes[j] = axes[j + 1];
				axes[j + 1] = temp;
			}
		}
	}

	for (int i = 0; i < 3; i++) {
		DrawLineEx((Vector2){center_x, center_y}, axes[i].screen_end, 3.0f, axes[i].color);
		draw_text_bordered(axes[i].label, (int)axes[i].screen_end.x - 5,
						   (int)axes[i].screen_end.y - 10, 20, RAYWHITE);
	}
}

int main(void) {
	printf("[BLICK VIEWER] Waiting for shared memory connection...\n");
	blick_shm_header* shm = NULL;
	while (!shm) {
		shm = blick_os_viewer_open_shm();
		if (!shm) blick_os_sleep_ms(100);
	}
	printf("[BLICK VIEWER] Connected.\n");

	// Tell raylib to shut up
	SetTraceLogLevel(LOG_NONE);

	SetConfigFlags(FLAG_WINDOW_RESIZABLE);
	InitWindow(1280, 720, "Blick Debug Viewer");

	// Move the debug window to the top-right corner of the current monitor.
	// This prevents it from spawning directly on top of the main simulation window.
#if WIN32
	// On windows we need to move it down a little, otherwise the window handle is outside the
	// desktop area
	set_window_top_right(0, 30);
#else
	set_window_top_right(0, 0);
#endif

	SetTargetFPS(60);

	Camera3D camera = {0};
	camera.position = (Vector3){15.0f, 10.0f, 15.0f};
	camera.target = (Vector3){0.0f, 2.0f, 0.0f};
	camera.up = (Vector3){0.0f, 1.0f, 0.0f};
	camera.fovy = 45.0f;
	camera.projection = CAMERA_PERSPECTIVE;

	// Toggle state for layers 0-9
	bool layer_visible[10];
	for (int i = 0; i < 2; i++) {
		layer_visible[i] = true;
	}
	for (int i = 2; i < 10; i++) {
		layer_visible[i] = false;
	}

	int cull_mode = 0; // 0: Back, 1: Front, 2: None
	bool ui_visible = true;

	init_graphics_resources();

	// Allocate on stack, because its too big for the stack for high CMD counts
	// (above ~130.000 commands on linux)
	blick_buffer* local_buf = malloc(sizeof(blick_buffer));
	if (!local_buf) return 1;

	while (!WindowShouldClose()) {
		// --- DATA SYNC ---
		// Identify the latest frame
		uint32_t idx = atomic_load(&shm->latest_buffer_idx);
		// Claim the buffer so the host doesn't overwrite it
		atomic_store(&shm->reading_idx, idx);
		// Race condition check: Did the host update 'latest' while we were claiming it?
		if (atomic_load(&shm->latest_buffer_idx) != idx) {
			// Race condition detected! Skip this frame and try again next loop
			atomic_store(&shm->reading_idx, 0xFFFFFFFF);
			continue;
		}
		// Copy to local memory
		memcpy(local_buf, &shm->buffers[idx], sizeof(blick_buffer));
		// Release the claim
		atomic_store(&shm->reading_idx, 0xFFFFFFFF);

		update_fly_camera(&camera);

		// Toggle layers 0-9
		for (int i = 0; i < 10; i++) {
			if (IsKeyPressed(KEY_ZERO + i)) layer_visible[i] = !layer_visible[i];
		}

		if (IsKeyPressed(KEY_TAB)) cull_mode = (cull_mode + 1) % 3;
		if (IsKeyPressed(KEY_H)) ui_visible = !ui_visible;

		switch (cull_mode) {
		case 0:
			rlEnableBackfaceCulling();
			rlSetCullFace(RL_CULL_FACE_BACK);
			break;
		case 1:
			rlEnableBackfaceCulling();
			rlSetCullFace(RL_CULL_FACE_FRONT);
			break;
		case 2:
			rlDisableBackfaceCulling();
			break;
		}
		BeginDrawing();
		{
			ClearBackground((Color){30, 30, 30, 255});

			// Pass 1: Opaque Objects (Alpha == 255)
			rlEnableDepthMask();
			BeginMode3D(camera);
			for (uint32_t i = 0; i < local_buf->count; i++) {
				blick_cmd* cmd = &local_buf->cmds[i];
				// Skip if layer is hidden
				if (cmd->layer < 10 && !layer_visible[cmd->layer]) continue;

				// Check Alpha (High byte of 0xAABBGGRR)
				// If < 255, it's transparent, skip it for this pass
				if ((cmd->color >> 24) < 255) continue;

				draw_command(cmd, shm, camera.position);
			}
			EndMode3D();

			// Pass 2: Transparent Objects (Alpha < 255)
			// Additive blending + No Z-Write (to allow overlapping "glow")
			rlDisableDepthMask();
			BeginMode3D(camera);
			BeginBlendMode(BLEND_ADDITIVE);
			for (uint32_t i = 0; i < local_buf->count; i++) {
				blick_cmd* cmd = &local_buf->cmds[i];
				if (cmd->layer < 10 && !layer_visible[cmd->layer]) continue;

				// If == 255, it's opaque, skip it for this pass
				if ((cmd->color >> 24) == 255) continue;

				draw_command(cmd, shm, camera.position);
			}
			// Restore standard render state
			EndBlendMode();
			EndMode3D();
			rlEnableDepthMask();

			// Fake 3D text drawn in 2D
			for (uint32_t i = 0; i < local_buf->count; i++) {
				blick_cmd* cmd = &local_buf->cmds[i];
				if (cmd->layer < 10 && !layer_visible[cmd->layer]) continue;

				if (cmd->type == BLICK_CMD_TEXT) {
					Color color = unpack_color(cmd->color);
					Vector3 pos = {cmd->data.text.pos.x, cmd->data.text.pos.y,
								   cmd->data.text.pos.z};

					Vector3 cam_forward = Vector3Subtract(camera.target, camera.position);
					Vector3 cam_to_text_pos = Vector3Subtract(pos, camera.position);

					// Only draw if the point is in front of the camera
					if (Vector3DotProduct(cam_to_text_pos, cam_forward) > 0.0f) {
						Vector2 screen_pos = GetWorldToScreen(pos, camera);
						draw_text_bordered(cmd->data.text.buffer, (int)screen_pos.x,
										   (int)screen_pos.y - 10, 20, color);
					}
				}
			}

			// Disable culling again, because Text does not get drawn if front face culling is on
			rlDisableBackfaceCulling();

			if (ui_visible) {
				// UI: Layer Status
				int ui_y = GetScreenHeight() - 30;
				int ui_padding = 30;

				draw_text_bordered("[H]", 10, ui_y - ui_padding * 2, 20, YELLOW);
				draw_text_bordered("Toggle UI", 70, ui_y - ui_padding * 2, 20, RAYWHITE);

				const char* cull_names[] = {"Back", "Front", "None"};
				draw_text_bordered("[Tab]", 10, ui_y - ui_padding, 20, YELLOW);
				draw_text_bordered("Culling:", 70, ui_y - ui_padding, 20, RAYWHITE);
				Color cull_color = (cull_mode == 2) ? GRAY : GREEN;
				draw_text_bordered(TextFormat("%s", cull_names[cull_mode]), 160,
								   ui_y - ui_padding, 20, cull_color);

				draw_text_bordered("[0-9]", 10, ui_y, 20, YELLOW);
				draw_text_bordered("Layers:", 70, ui_y, 20, RAYWHITE);
				for (int i = 0; i < 10; i++) {
					Color c = layer_visible[i] ? GREEN : GRAY;
					draw_text_bordered(TextFormat("%d", i), 160 + (i * 20), ui_y, 20, c);
				}

				draw_camera_orientation_gizmo(camera, 60, 60, 40.0f);
			}
		}
		EndDrawing();
	}

	void cleanup_graphics_resources();

	blick_os_viewer_close_shm(shm);
	CloseWindow();

	return 0;
}
