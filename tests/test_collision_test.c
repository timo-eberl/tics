#include "test.h"

#include <tics.h>
#include <tics_internal.h>

// clang-format off
static tics_vec3 cube_vertices[] = {{-0.5f,-0.5f,0.5f},{-0.5f,-0.5f,0.5f},{-0.5f,-0.5f,0.5f},{-0.5f,0.5f,0.5f},{-0.5f,0.5f,0.5f},{-0.5f,0.5f,0.5f},{-0.5f,-0.5f,-0.5f},{-0.5f,-0.5f,-0.5f},{-0.5f,-0.5f,-0.5f},{-0.5f,0.5f,-0.5f},{-0.5f,0.5f,-0.5f},{-0.5f,0.5f,-0.5f},{0.5f,-0.5f,0.5f},{0.5f,-0.5f,0.5f},{0.5f,-0.5f,0.5f},{0.5f,0.5f,0.5f},{0.5f,0.5f,0.5f},{0.5f,0.5f,0.5f},{0.5f,-0.5f,-0.5f},{0.5f,-0.5f,-0.5f},{0.5f,-0.5f,-0.5f},{0.5f,0.5f,-0.5f},{0.5f,0.5f,-0.5f},{0.5f,0.5f,-0.5f}};
static uint32_t cube_indices[] = {2,5,11,2,11,8,6,9,21,6,21,18,20,23,17,20,17,14,12,15,3,12,3,0,7,19,13,7,13,1,22,10,4,22,4,16};
static tics_vec3 pyramid_vertices[] = {{-1, 1, -1}, {1, 1, -1}, {1, 1, 1}, {-1, 1, 1}, {0, -1, 0}};
static uint32_t pyramid_indices[] = {0, 2, 1, 0, 3, 2, 0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4};
static tics_vec3 icosphere_vertices[] = {{0.0f,-1.0f,-0.0f},{0.0f,-1.0f,-0.0f},{0.0f,-1.0f,-0.0f},{0.0f,-1.0f,-0.0f},{0.0f,-1.0f,-0.0f},{0.723607302f,-0.447219521f,0.525725305f},{0.723607302f,-0.447219521f,0.525725305f},{0.723607302f,-0.447219521f,0.525725305f},{0.723607302f,-0.447219521f,0.525725305f},{0.723607302f,-0.447219521f,0.525725305f},{-0.276388019f,-0.447219849f,0.850649238f},{-0.276388019f,-0.447219849f,0.850649238f},{-0.276388019f,-0.447219849f,0.850649238f},{-0.276388019f,-0.447219849f,0.850649238f},{-0.276388019f,-0.447219849f,0.850649238f},{-0.894426227f,-0.447215617f,-0.0f},{-0.894426227f,-0.447215617f,-0.0f},{-0.894426227f,-0.447215617f,-0.0f},{-0.894426227f,-0.447215617f,-0.0f},{-0.894426227f,-0.447215617f,-0.0f},{-0.276388019f,-0.447219849f,-0.850649238f},{-0.276388019f,-0.447219849f,-0.850649238f},{-0.276388019f,-0.447219849f,-0.850649238f},{-0.276388019f,-0.447219849f,-0.850649238f},{-0.276388019f,-0.447219849f,-0.850649238f},{0.723607302f,-0.447219521f,-0.525725305f},{0.723607302f,-0.447219521f,-0.525725305f},{0.723607302f,-0.447219521f,-0.525725305f},{0.723607302f,-0.447219521f,-0.525725305f},{0.723607302f,-0.447219521f,-0.525725305f},{0.276388019f,0.447219849f,0.850649238f},{0.276388019f,0.447219849f,0.850649238f},{0.276388019f,0.447219849f,0.850649238f},{0.276388019f,0.447219849f,0.850649238f},{0.276388019f,0.447219849f,0.850649238f},{-0.723607302f,0.447219521f,0.525725305f},{-0.723607302f,0.447219521f,0.525725305f},{-0.723607302f,0.447219521f,0.525725305f},{-0.723607302f,0.447219521f,0.525725305f},{-0.723607302f,0.447219521f,0.525725305f},{-0.723607302f,0.447219521f,-0.525725305f},{-0.723607302f,0.447219521f,-0.525725305f},{-0.723607302f,0.447219521f,-0.525725305f},{-0.723607302f,0.447219521f,-0.525725305f},{-0.723607302f,0.447219521f,-0.525725305f},{0.276388019f,0.447219849f,-0.850649238f},{0.276388019f,0.447219849f,-0.850649238f},{0.276388019f,0.447219849f,-0.850649238f},{0.276388019f,0.447219849f,-0.850649238f},{0.276388019f,0.447219849f,-0.850649238f},{0.894426227f,0.447215617f,-0.0f},{0.894426227f,0.447215617f,-0.0f},{0.894426227f,0.447215617f,-0.0f},{0.894426227f,0.447215617f,-0.0f},{0.894426227f,0.447215617f,-0.0f},{0.0f,1.0f,-0.0f},{0.0f,1.0f,-0.0f},{0.0f,1.0f,-0.0f},{0.0f,1.0f,-0.0f},{0.0f,1.0f,-0.0f},{-0.162455559f,-0.850654423f,0.499995261f},{-0.162455559f,-0.850654423f,0.499995261f},{-0.162455559f,-0.850654423f,0.499995261f},{-0.162455559f,-0.850654423f,0.499995261f},{-0.162455559f,-0.850654423f,0.499995261f},{-0.162455559f,-0.850654423f,0.499995261f},{0.425322682f,-0.850654185f,0.3090114f},{0.425322682f,-0.850654185f,0.3090114f},{0.425322682f,-0.850654185f,0.3090114f},{0.425322682f,-0.850654185f,0.3090114f},{0.425322682f,-0.850654185f,0.3090114f},{0.425322682f,-0.850654185f,0.3090114f},{0.262868822f,-0.525737643f,0.809011638f},{0.262868822f,-0.525737643f,0.809011638f},{0.262868822f,-0.525737643f,0.809011638f},{0.262868822f,-0.525737643f,0.809011638f},{0.262868822f,-0.525737643f,0.809011638f},{0.262868822f,-0.525737643f,0.809011638f},{0.850647867f,-0.525735915f,-0.0f},{0.850647867f,-0.525735915f,-0.0f},{0.850647867f,-0.525735915f,-0.0f},{0.850647867f,-0.525735915f,-0.0f},{0.850647867f,-0.525735915f,-0.0f},{0.850647867f,-0.525735915f,-0.0f},{0.425322682f,-0.850654185f,-0.3090114f},{0.425322682f,-0.850654185f,-0.3090114f},{0.425322682f,-0.850654185f,-0.3090114f},{0.425322682f,-0.850654185f,-0.3090114f},{0.425322682f,-0.850654185f,-0.3090114f},{0.425322682f,-0.850654185f,-0.3090114f},{-0.525729775f,-0.850651681f,-0.0f},{-0.525729775f,-0.850651681f,-0.0f},{-0.525729775f,-0.850651681f,-0.0f},{-0.525729775f,-0.850651681f,-0.0f},{-0.525729775f,-0.850651681f,-0.0f},{-0.525729775f,-0.850651681f,-0.0f},{-0.688189387f,-0.525736213f,0.49999693f},{-0.688189387f,-0.525736213f,0.49999693f},{-0.688189387f,-0.525736213f,0.49999693f},{-0.688189387f,-0.525736213f,0.49999693f},{-0.688189387f,-0.525736213f,0.49999693f},{-0.688189387f,-0.525736213f,0.49999693f},{-0.162455559f,-0.850654423f,-0.499995261f},{-0.162455559f,-0.850654423f,-0.499995261f},{-0.162455559f,-0.850654423f,-0.499995261f},{-0.162455559f,-0.850654423f,-0.499995261f},{-0.162455559f,-0.850654423f,-0.499995261f},{-0.162455559f,-0.850654423f,-0.499995261f},{-0.688189387f,-0.525736213f,-0.49999693f},{-0.688189387f,-0.525736213f,-0.49999693f},{-0.688189387f,-0.525736213f,-0.49999693f},{-0.688189387f,-0.525736213f,-0.49999693f},{-0.688189387f,-0.525736213f,-0.49999693f},{-0.688189387f,-0.525736213f,-0.49999693f},{0.262868822f,-0.525737643f,-0.809011638f},{0.262868822f,-0.525737643f,-0.809011638f},{0.262868822f,-0.525737643f,-0.809011638f},{0.262868822f,-0.525737643f,-0.809011638f},{0.262868822f,-0.525737643f,-0.809011638f},{0.262868822f,-0.525737643f,-0.809011638f},{0.951057851f,0.0f,0.309012622f},{0.951057851f,0.0f,0.309012622f},{0.951057851f,0.0f,0.309012622f},{0.951057851f,0.0f,0.309012622f},{0.951057851f,0.0f,0.309012622f},{0.951057851f,0.0f,0.309012622f},{0.951057851f,0.0f,-0.309012622f},{0.951057851f,0.0f,-0.309012622f},{0.951057851f,0.0f,-0.309012622f},{0.951057851f,0.0f,-0.309012622f},{0.951057851f,0.0f,-0.309012622f},{0.951057851f,0.0f,-0.309012622f},{0.0f,0.0f,0.99999994f},{0.0f,0.0f,0.99999994f},{0.0f,0.0f,0.99999994f},{0.0f,0.0f,0.99999994f},{0.0f,0.0f,0.99999994f},{0.0f,0.0f,0.99999994f},{0.587785602f,0.0f,0.809016705f},{0.587785602f,0.0f,0.809016705f},{0.587785602f,0.0f,0.809016705f},{0.587785602f,0.0f,0.809016705f},{0.587785602f,0.0f,0.809016705f},{0.587785602f,0.0f,0.809016705f},{-0.951057851f,0.0f,0.309012622f},{-0.951057851f,0.0f,0.309012622f},{-0.951057851f,0.0f,0.309012622f},{-0.951057851f,0.0f,0.309012622f},{-0.951057851f,0.0f,0.309012622f},{-0.951057851f,0.0f,0.309012622f},{-0.587785602f,0.0f,0.809016705f},{-0.587785602f,0.0f,0.809016705f},{-0.587785602f,0.0f,0.809016705f},{-0.587785602f,0.0f,0.809016705f},{-0.587785602f,0.0f,0.809016705f},{-0.587785602f,0.0f,0.809016705f},{-0.587785602f,0.0f,-0.809016705f},{-0.587785602f,0.0f,-0.809016705f},{-0.587785602f,0.0f,-0.809016705f},{-0.587785602f,0.0f,-0.809016705f},{-0.587785602f,0.0f,-0.809016705f},{-0.587785602f,0.0f,-0.809016705f},{-0.951057851f,0.0f,-0.309012622f},{-0.951057851f,0.0f,-0.309012622f},{-0.951057851f,0.0f,-0.309012622f},{-0.951057851f,0.0f,-0.309012622f},{-0.951057851f,0.0f,-0.309012622f},{-0.951057851f,0.0f,-0.309012622f},{0.587785602f,0.0f,-0.809016705f},{0.587785602f,0.0f,-0.809016705f},{0.587785602f,0.0f,-0.809016705f},{0.587785602f,0.0f,-0.809016705f},{0.587785602f,0.0f,-0.809016705f},{0.587785602f,0.0f,-0.809016705f},{0.0f,0.0f,-0.99999994f},{0.0f,0.0f,-0.99999994f},{0.0f,0.0f,-0.99999994f},{0.0f,0.0f,-0.99999994f},{0.0f,0.0f,-0.99999994f},{0.0f,0.0f,-0.99999994f},{0.688189387f,0.525736213f,0.49999693f},{0.688189387f,0.525736213f,0.49999693f},{0.688189387f,0.525736213f,0.49999693f},{0.688189387f,0.525736213f,0.49999693f},{0.688189387f,0.525736213f,0.49999693f},{0.688189387f,0.525736213f,0.49999693f},{-0.262868822f,0.525737643f,0.809011638f},{-0.262868822f,0.525737643f,0.809011638f},{-0.262868822f,0.525737643f,0.809011638f},{-0.262868822f,0.525737643f,0.809011638f},{-0.262868822f,0.525737643f,0.809011638f},{-0.262868822f,0.525737643f,0.809011638f},{-0.850647867f,0.525735915f,-0.0f},{-0.850647867f,0.525735915f,-0.0f},{-0.850647867f,0.525735915f,-0.0f},{-0.850647867f,0.525735915f,-0.0f},{-0.850647867f,0.525735915f,-0.0f},{-0.850647867f,0.525735915f,-0.0f},{-0.262868822f,0.525737643f,-0.809011638f},{-0.262868822f,0.525737643f,-0.809011638f},{-0.262868822f,0.525737643f,-0.809011638f},{-0.262868822f,0.525737643f,-0.809011638f},{-0.262868822f,0.525737643f,-0.809011638f},{-0.262868822f,0.525737643f,-0.809011638f},{0.688189387f,0.525736213f,-0.49999693f},{0.688189387f,0.525736213f,-0.49999693f},{0.688189387f,0.525736213f,-0.49999693f},{0.688189387f,0.525736213f,-0.49999693f},{0.688189387f,0.525736213f,-0.49999693f},{0.688189387f,0.525736213f,-0.49999693f},{0.162455559f,0.850654364f,0.499995261f},{0.162455559f,0.850654364f,0.499995261f},{0.162455559f,0.850654364f,0.499995261f},{0.162455559f,0.850654364f,0.499995261f},{0.162455559f,0.850654364f,0.499995261f},{0.162455559f,0.850654364f,0.499995261f},{0.525729775f,0.850651681f,-0.0f},{0.525729775f,0.850651681f,-0.0f},{0.525729775f,0.850651681f,-0.0f},{0.525729775f,0.850651681f,-0.0f},{0.525729775f,0.850651681f,-0.0f},{0.525729775f,0.850651681f,-0.0f},{-0.425322682f,0.850654185f,0.3090114f},{-0.425322682f,0.850654185f,0.3090114f},{-0.425322682f,0.850654185f,0.3090114f},{-0.425322682f,0.850654185f,0.3090114f},{-0.425322682f,0.850654185f,0.3090114f},{-0.425322682f,0.850654185f,0.3090114f},{-0.425322682f,0.850654185f,-0.3090114f},{-0.425322682f,0.850654185f,-0.3090114f},{-0.425322682f,0.850654185f,-0.3090114f},{-0.425322682f,0.850654185f,-0.3090114f},{-0.425322682f,0.850654185f,-0.3090114f},{-0.425322682f,0.850654185f,-0.3090114f},{0.162455559f,0.850654364f,-0.499995261f},{0.162455559f,0.850654364f,-0.499995261f},{0.162455559f,0.850654364f,-0.499995261f},{0.162455559f,0.850654364f,-0.499995261f},{0.162455559f,0.850654364f,-0.499995261f},{0.162455559f,0.850654364f,-0.499995261f}};
static uint32_t icosphere_indices[] = {0,66,60,7,71,79,3,63,90,4,91,105,1,102,84,9,81,123,10,72,132,17,101,146,24,110,159,26,118,170,8,121,143,12,137,151,19,149,167,22,157,179,28,173,127,32,180,212,35,190,225,42,194,233,45,198,234,51,207,221,217,235,56,219,205,237,204,47,236,238,228,58,239,200,229,202,40,231,230,224,59,232,192,226,193,37,227,222,214,57,223,188,215,186,30,210,211,216,55,213,181,218,183,50,220,128,209,53,126,172,208,171,48,206,177,199,49,178,156,201,158,41,203,165,196,44,166,148,197,147,39,195,152,191,36,150,136,189,135,34,187,141,182,33,142,120,184,122,52,185,169,176,46,168,116,175,114,20,174,161,163,43,160,112,162,113,18,164,145,155,38,144,100,154,98,14,153,134,139,31,133,74,138,76,6,140,125,131,54,124,83,130,82,29,129,87,117,25,85,103,115,104,21,119,106,108,23,107,93,109,95,16,111,94,99,15,92,65,97,64,13,96,80,89,27,78,70,88,68,2,86,62,77,11,61,67,73,69,5,75};
// Wall geometry
// Calculated with CONTAINER_SIZE=25.0, WALL_THICKNESS=10.0 (HS=22.5, HT=5.0)
static const tics_vec3 wall_vertices[] = {
	{-22.5f, -22.5f, -5.0f}, {22.5f, -22.5f, -5.0f}, {22.5f, 22.5f, -5.0f},
	{-22.5f, 22.5f, -5.0f},  {-22.5f, -22.5f, 5.0f}, {22.5f, -22.5f, 5.0f},
	{22.5f, 22.5f, 5.0f},    {-22.5f, 22.5f, 5.0f}
};
static const uint32_t wall_indices[] = {
	4, 5, 6, 4, 6, 7, 1, 0, 3, 1, 3, 2, 0, 4, 7, 0, 7, 3,
	5, 1, 2, 5, 2, 6, 7, 6, 2, 7, 2, 3, 0, 1, 5, 0, 5, 4
};
// clang-format on

typedef struct {
	tics_world* world;
	const shape_data* cube;
	const shape_data* icosphere;
	const shape_data* pyramid;
	const shape_data* analytic_sphere;
	const shape_data* offset_sphere;
	const shape_data* wall;
	const shape_data* capsule;
	const shape_data* huge_sphere;
	const shape_data* small_capsule;
	const shape_data* small_box;
} test_env;

static test_env setup_test_env(void) {
	tics_world_desc world_desc = {.gravity = {0.0f, -9.81f, 0.0f}};
	tics_world* world = tics_world_create(world_desc);

	// Create Cube
	tics_create_convex_shape(
		world, cube_vertices, sizeof(cube_vertices) / sizeof(tics_vec3),
		cube_indices, sizeof(cube_indices) / sizeof(cube_indices[0]));

	// Create Sphere
	tics_create_convex_shape(
		world, icosphere_vertices, sizeof(icosphere_vertices) / sizeof(tics_vec3),
		icosphere_indices, sizeof(icosphere_indices) / sizeof(icosphere_indices[0]));

	// Create Pyramid
	tics_create_convex_shape(
		world, pyramid_vertices, sizeof(pyramid_vertices) / sizeof(tics_vec3),
		pyramid_indices, sizeof(pyramid_indices) / sizeof(pyramid_indices[0]));

	// Create Analytic Sphere
	// Standard sphere with Radius 0.5 centered at origin.
	tics_create_sphere_shape(world, (tics_vec3){0, 0, 0}, 0.5f);

	// Create Offset Analytic Sphere
	// Sphere with Radius 0.5, but the shape center is locally offset by (1,0,0).
	// This is used to test that shape local transforms are respected.
	tics_create_sphere_shape(world, (tics_vec3){1.0f, 0, 0}, 0.5f);

	// Create Wall Plate
	tics_create_convex_shape(
		world, wall_vertices, sizeof(wall_vertices) / sizeof(tics_vec3),
		wall_indices, sizeof(wall_indices) / sizeof(wall_indices[0]));

	// Create Capsule
	tics_create_capsule_shape(world, (tics_vec3){0, -1.0f, 0}, (tics_vec3){0, 1.0f, 0}, 0.5f);

	// Create Huge Sphere
	tics_create_sphere_shape(world, (tics_vec3){0, 0, 0}, 10.0f);

	// Create Small Capsule
	tics_create_capsule_shape(world, (tics_vec3){0, -0.3f, 0}, (tics_vec3){0, 0.3f, 0}, 0.19f);

	// Create Small Box
	tics_create_box_shape(world, (tics_vec3){0.282901645f, 0.282901645f, 0.282901645f});

	return (test_env){
		.world = world,
		// Access internal shape data using hardcoded indices.
		// NOTE: These tests assume the internal storage is linear and matches the creation order.
		.cube = &world->shapes[0],
		.icosphere = &world->shapes[1],
		.pyramid = &world->shapes[2],
		.analytic_sphere = &world->shapes[3],
		.offset_sphere = &world->shapes[4],
		.wall = &world->shapes[5],
		.capsule = &world->shapes[6],
		.huge_sphere = &world->shapes[7],
		.small_capsule = &world->shapes[8],
		.small_box = &world->shapes[9],
	};
}

static void pyramid_test(const shape_data* shape_ptr) {
	// Setup:
	// Pyramid Geometry: Flat Base at Y=1, Sharp Tip at Y=-1.
	// Position B at Y = 1.9.
	// B's Tip World Y = 1.9 + (-1.0) = 0.9.
	// A's Top World Y = 1.0.
	// Expected Depth = 1.0 - 0.9 = 0.1.
	tics_transform tA = {.position = {0, 0, 0}, .rotation = {0, 0, 0, 1}};
	tics_transform tB = {.position = {0, 1.9f, 0}, .rotation = {0, 0, 0, 1}};
	{
		collision_result result = collision_test(shape_ptr, tA, shape_ptr, tB);

		ASSERT_TRUE(result.has_collision);
		ASSERT_FLOAT_APPROX(result.depth, 0.1f);
		// Normal points A -> B (Down)
		ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){0, -1, 0}));
		// Point A is on the surface of A (Y=1.0)
		ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){0, 1.0f, 0}));
		// Point B is the tip of B (Y=0.9)
		ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){0, 0.9f, 0}));
	}
	{
		// swapped transforms -> A and B are swapped and normal is flipped
		collision_result result = collision_test(shape_ptr, tB, shape_ptr, tA);

		ASSERT_TRUE(result.has_collision);
		ASSERT_FLOAT_APPROX(result.depth, 0.1f);
		ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){0, 1, 0}));
		ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){0, 0.9f, 0}));
		ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){0, 1.0f, 0}));
	}
	{
		// rotate A by 180° -> now the tips are intersecting
		tA.rotation = (tics_quat){1, 0, 0, 0};
		collision_result result = collision_test(shape_ptr, tA, shape_ptr, tB);

		// For a vertex vs vertex collision, the correct result is not pointing from one vertex to
		// the other, even though that might seem intuitive at first. The definition of the
		// separation vector states that it is the shortest vector required to separate the shapes.
		// For polygons, there is always a smaller vector than the one from the two vertices.
		ASSERT_TRUE(result.has_collision);
		// the depth must be at most the distance of the vertices
		ASSERT_TRUE(result.depth <= 0.1f);
	}
}

static void edge_edge_crossed_test(const shape_data* cube) {
	// Setup:
	// We arrange two cubes to form a cross (+).
	// Cube A: Rotated 45° around X. Topmost feature is an edge parallel to X.
	// Cube B: Rotated 45° around Z. Bottommost feature is an edge parallel to Z.
	// This configuration eliminates face-face and vertex-face contacts, forcing
	// the solver to resolve the "Edge-Edge" case.

	// Constants for a standard cube (half-extent = 0.5)
	// The highest point of a cube rotated 45° is sqrt(.5^2 + .5^2) = sqrt(0.5).
	const float ext_y = 0.707106781f;
	const float penetration = 0.1f;

	// Position B such that it overlaps A by exactly 0.1
	// A_top_y = ext_y
	// B_bottom_y = B_pos_y - ext_y
	// Overlap = A_top_y - B_bottom_y = 2*ext_y - B_pos_y = 0.1
	// B_pos_y = 2*ext_y - 0.1
	const float b_pos_y = (2.0f * ext_y) - penetration;

	// Quaternion for 45 degrees (PI/4)
	// sin(PI/8) ~= 0.3826834, cos(PI/8) ~= 0.9238795
	const float q_sin = 0.382683432f;
	const float q_cos = 0.923879533f;

	tics_transform tA = {
		.position = {0, 0, 0}, .rotation = {q_sin, 0, 0, q_cos} // Rotated 45° on X
	};

	tics_transform tB = {
		.position = {0, b_pos_y, 0}, .rotation = {0, 0, q_sin, q_cos} // Rotated 45° on Z
	};

	collision_result result = collision_test(cube, tA, cube, tB);

	ASSERT_TRUE(result.has_collision);
	ASSERT_FLOAT_APPROX(result.depth, penetration);

	// Normal points from point A (objects A's top edge) -> point B (object B's bottom edge)
	ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){0, -1, 0}));

	// Verify Contact Points:
	// Point A is the center of its top edge: (0, ext_y, 0)
	// Point B is the center of its bottom edge: (0, b_pos_y - ext_y, 0)
	ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){0, ext_y, 0}));
	ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){0, b_pos_y - ext_y, 0}));
}

static void vertex_edge_test(const shape_data* cube, const shape_data* pyramid) {
	// Setup:
	// Cube A: Rotated 45 degrees around X axis.
	// This places an edge (parallel to X) at the highest point Y = 0.7071...
	// Pyramid B: Tip is at local (0, -1, 0).
	// We position B such that the tip penetrates the cube's top edge by 0.1.

	// Cube half-extent is 0.5. Diagonal to edge is sqrt(0.5^2 + 0.5^2)
	const float ext_y = 0.707106781f;
	const float penetration = 0.1f;

	// Pyramid Position Calculation:
	// Target Tip World Y = Cube Top Y - penetration
	// World Y = Pos Y + Local Tip Y
	// Pos Y = (ext_y - penetration) - (-1.0f)
	const float b_pos_y = (ext_y - penetration) + 1.0f;

	// Quaternion for 45 degrees around X (PI/4)
	// sin(PI/8) ~= 0.3826834, cos(PI/8) ~= 0.9238795
	const float q_sin = 0.382683432f;
	const float q_cos = 0.923879533f;

	tics_transform tA = {
		.position = {0, 0, 0}, .rotation = {q_sin, 0, 0, q_cos} // Rotated 45° X
	};

	tics_transform tB = {.position = {0, b_pos_y, 0}, .rotation = {0, 0, 0, 1}};

	collision_result result = collision_test(cube, tA, pyramid, tB);

	// For a vertex vs edge collision, the correct result is not going through the vertex, even
	// though that might seem intuitive at first. The definition of the separation vector states
	// that it is the shortest vector required to separate the shapes. For polygons, there is always
	// a smaller vector.
	ASSERT_TRUE(result.has_collision);
	// separation vector length must be equal or smaller than the distance from vertex to edge
	ASSERT_TRUE(result.depth <= penetration);
}

static void analytic_sphere_test(const shape_data* sphere) {
	// Setup: Two spheres (Radius 0.5).
	// A: at origin. Surface at Y=0.5.
	// B: at Y=0.9. Surface at Y=0.9 - 0.5 = 0.4.
	// Penetration: 0.5 - 0.4 = 0.1.
	tics_transform tA = {.position = {0, 0, 0}, .rotation = {0, 0, 0, 1}};
	tics_transform tB = {.position = {0, 0.9f, 0}, .rotation = {0, 0, 0, 1}};

	collision_result result = collision_test(sphere, tA, sphere, tB);

	// Important check for optimization:
	// If the solver uses an analytic path, this should be exact.
	ASSERT_TRUE(result.has_collision);
	ASSERT_FLOAT_APPROX(result.depth, 0.1f);
	ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){0, -1, 0}));

	// Contact points:
	// Point on A: (0, 0.5, 0)
	// Point on B: (0, 0.4, 0)
	ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){0, 0.5f, 0}));
	ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){0, 0.4f, 0}));
}

static void rotated_offset_sphere_test(const shape_data* offset_sphere,
									   const shape_data* normal_sphere) {
	// Setup:
	// Shape A (offset_sphere): Radius 0.5, Local Center (1, 0, 0).
	// We rotate Shape A by 90 degrees around Z.
	// Logical Center becomes (0, 1, 0) in world space.
	// Surface A extends up to Y=1.5.
	const float q_sin = 0.70710678f; // sin(45 deg)
	const float q_cos = 0.70710678f; // cos(45 deg)
	tics_transform tA = {
		.position = {0, 0, 0}, .rotation = {0, 0, q_sin, q_cos} // +90 deg Z
	};

	// Shape B (normal_sphere): Radius 0.5, Local Center (0, 0, 0).
	// We place it at (0, 1.9, 0).
	// Surface B extends down to Y=1.4.
	tics_transform tB = {.position = {0, 1.9f, 0}, .rotation = {0, 0, 0, 1}};

	// Overlap calculation:
	// Center A (World) = (0, 1, 0)
	// Center B (World) = (0, 1.9, 0)
	// Distance = 0.9. Radius Sum = 1.0. Depth = 0.1.

	collision_result result = collision_test(offset_sphere, tA, normal_sphere, tB);

	// What this tests:
	// The collision solver must effectively apply (Transform * LocalCenter)
	// to find the world center of the sphere. If it ignores rotation for the
	// center offset, the sphere would be at (1, 0, 0) and miss B entirely.
	ASSERT_TRUE(result.has_collision);
	ASSERT_FLOAT_APPROX(result.depth, 0.1f);
	ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){0, -1, 0}));

	// Contact points:
	// Point A: CenterA(0,1,0) + Radius(0.5)*Up = (0, 1.5, 0)
	// Point B: CenterB(0,1.9,0) + Radius(0.5)*Down = (0, 1.4, 0)
	ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){0, 1.5f, 0}));
	ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){0, 1.4f, 0}));
}

static void capsule_sphere_body_test(const shape_data* capsule, const shape_data* sphere) {
	// Place capsule at origin and sphere slightly overlapping the cylindrical body on the X-axis.
	// The sum of the radii is 1.0. The distance is 0.9.
	tics_transform t_capsule = {.position = {0, 0, 0}, .rotation = {0, 0, 0, 1}};
	tics_transform t_sphere = {.position = {0.9f, 0, 0}, .rotation = {0, 0, 0, 1}};

	{
		collision_result result = collision_test(capsule, t_capsule, sphere, t_sphere);

		ASSERT_TRUE(result.has_collision);
		ASSERT_FLOAT_APPROX(result.depth, 0.1f);
		// Normal points from B to A, pushing the capsule to the left (-X)
		ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){-1.0f, 0, 0}));
		ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){0.5f, 0, 0}));
		ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){0.4f, 0, 0}));
	}
	{
		// Swapped order: Sphere is now Shape A, Capsule is Shape B.
		collision_result result = collision_test(sphere, t_sphere, capsule, t_capsule);

		ASSERT_TRUE(result.has_collision);
		ASSERT_FLOAT_APPROX(result.depth, 0.1f);
		// Normal points from B to A, pushing the sphere to the right (+X)
		ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){1.0f, 0, 0}));
		ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){0.4f, 0, 0}));
		ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){0.5f, 0, 0}));
	}
}

static void capsule_sphere_inside_test(const shape_data* capsule, const shape_data* huge_sphere) {
	// Place huge sphere (radius 10) at origin.
	// Place capsule (radius 0.5) completely inside the sphere at Y=8.
	// The capsule segment is from (0, 7, 0) to (0, 9, 0) in world space.
	// Closest segment point to the sphere center is (0, 7, 0). Distance is 7.0.
	// Sum of radii = 10.0 + 0.5 = 10.5.
	// Penetration depth = 10.5 - 7.0 = 3.5.
	tics_transform t_capsule = {.position = {0, 8.0f, 0}, .rotation = {0, 0, 0, 1}};
	tics_transform t_sphere = {.position = {0, 0, 0}, .rotation = {0, 0, 0, 1}};

	{
		collision_result result = collision_test(capsule, t_capsule, huge_sphere, t_sphere);

		ASSERT_TRUE(result.has_collision);
		ASSERT_FLOAT_APPROX(result.depth, 3.5f);
		// Normal points from B to A, pushing the capsule up (+Y) out of the sphere
		ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){0, 1.0f, 0}));
		// Point A is the deepest point of capsule inside the sphere: bottom tip (0, 6.5, 0)
		ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){0, 6.5f, 0}));
		// Point B is the deepest point of sphere inside the capsule: top edge (0, 10.0, 0)
		ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){0, 10.0f, 0}));
	}
	{
		// Swapped order: Sphere is now Shape A, Capsule is Shape B.
		collision_result result = collision_test(huge_sphere, t_sphere, capsule, t_capsule);

		ASSERT_TRUE(result.has_collision);
		ASSERT_FLOAT_APPROX(result.depth, 3.5f);
		// Normal points from B to A, pushing the sphere down (-Y)
		ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){0, -1.0f, 0}));
		ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){0, 10.0f, 0}));
		ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){0, 6.5f, 0}));
	}
}


// Helper to verify that specific shape configurations do not cause infinite loops
// in GJK or EPA.
static void verify_no_cycling(const shape_data* shape_a, tics_transform t_a,
							  const shape_data* shape_b, tics_transform t_b) {
	// The test passes if this function returns and doesn't loop endlessly.
	TEST_TIMEOUT_BEGIN(1);
	collision_result result = collision_test(shape_a, t_a, shape_b, t_b);
	(void)result;
	TEST_TIMEOUT_END();
}

// TODO look into this more closely
static void degenerate_epa_expansion_test(const shape_data* wall) {
	tics_transform tA = {
		.position = {17.5f, 0.0f, 0.0f},
		.rotation = {0.0f, 0.707108021f, 0.0f, 0.707105458f}
	};
	tics_transform tB = {
		.position = {0.0f, -17.5f, 0.0f},
		.rotation = {0.707108021f, 0.0f, 0.0f, 0.707105458f}
	};

	collision_result result = collision_test(wall, tA, wall, tB);
	ASSERT_TRUE(result.has_collision);
}

void run_collision_test_tests(void) {
	test_env env = setup_test_env();

	// convex hulls can collide in different ways, e.g. one vertex (corner) of a shape intersects
	// with a face of another.

	// vertex vs vertex and vertex vs face
	pyramid_test(env.pyramid);
	// vertex vs edge
	vertex_edge_test(env.cube, env.pyramid);
	// edge vs edge where edges are NOT parallel
	edge_edge_crossed_test(env.cube); // We even test edge-cases (haha!)
	// TODO edge vs edge (parallel)
	// TODO face vs edge
	// TODO face vs face

	// sphere vs sphere
	analytic_sphere_test(env.analytic_sphere);
	rotated_offset_sphere_test(env.offset_sphere, env.analytic_sphere);

	// capsule vs sphere
	capsule_sphere_body_test(env.capsule, env.analytic_sphere);
	capsule_sphere_inside_test(env.capsule, env.huge_sphere);

	// Cycling
	// I ended up in an endless loop in a simulation with those shapes and transforms.

	// GJK reported a collision that was not actually one and EPA was cycling.
	verify_no_cycling(
		env.cube,
		(tics_transform){.position = {-4.71135092f, 0.13406682f, -3.19023204f},
						 .rotation = {0.286603302f, 0.118719958f, 0.878672123f, 0.363123149f}},
		env.icosphere,
		(tics_transform){.position = {-3.22701406f, 0.377987236f, -3.94106793f},
						 .rotation = {-0.427287906f, -0.406753719f, 0.677122593f, 0.440110296f}});

	// GJK was cycling endlessly in the loop that searches for the 4th support point.
	// It was endlessly "rotating" around the origin
	// This happened after fixing the two tests above (in GJK when expanding, I prioritized the face
	// with greatest dot product)
	verify_no_cycling(
		env.cube,
		(tics_transform){.position = {-0.367497623, 0.279514551, 3.76133966},
						 .rotation = {-0.294131428, -0.124484502, -0.24287124, -0.915972233}},
		env.icosphere,
		(tics_transform){.position = {-2.0598464, 0.83966881, 3.97224188},
						 .rotation = {-0.389955401, 0.259868205, 0.880065918, 0.0767269805}});

	verify_no_cycling(
		env.icosphere,
		(tics_transform){.position = {3.59118629, 1.7181555, -8.70375252},
						 .rotation = {0.150770277, 0.173036918, -0.446831852, -0.864690959}},
		env.cube,
		(tics_transform){.position = {2.89894867, 0.134436712, -8.2301302},
						 .rotation = {0.688763201, -0.450365841, 0.0151280379, 0.567937136}});

	// EPA was cycling.
	// This happened after implementing voronoi region checks in GJK (which didn't work quite yet)
	verify_no_cycling(
		env.cube,
		(tics_transform){.position = {6.18676424, 2.59370232, 0.717267215},
						 .rotation = {-0.419851273, -0.422811836, -0.5161798, 0.615234554}},
		env.icosphere,
		(tics_transform){.position = {6.4184494, 2.60607433, -0.901389718},
						 .rotation = {0.649845541, -0.229413226, 0.268408537, 0.673073828}});

	// Edge case in EPA triggered by coplanar faces
	degenerate_epa_expansion_test(env.wall);

	// GJK was cycling.
	// Happened for moving walls in benchmark_broadphase
	verify_no_cycling(
		env.wall,
		(tics_transform){.position = {-12.1937532, 0.0, 2.65312362},
						 .rotation = {0.0, 0.707108021, 0.0, 0.707105458}},
		env.wall,
		(tics_transform){.position = {5.30624723, 17.5, 2.65312362},
						 .rotation = {0.707108021, 0.0, 0.0, 0.707105458}});

	// Previously reported a wrong collision with Voronoi evaluation
	{
		collision_result result = collision_test(
			env.small_capsule,
			(tics_transform){.position = {26.2250042f, -29.617548f, 32.3244514f},
							 .rotation = {-0.439939231f, 0.24404043f, 0.453430086f, -0.735730171f}},
			env.small_box,
			(tics_transform){.position = {25.8232727f, -29.5600929f, 32.5288315f},
							 .rotation = {0.0814626962f, 0.891633272f, 0.42037642f, 0.1470972f}});

		ASSERT_TRUE(result.has_collision);
		ASSERT_FLOAT_APPROX(result.depth, 0.184121f);
		ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){0.771410f, -0.576180f, -0.270078f}));
		ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){26.022343f, -29.533079f, 32.268078f}));
		ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){26.164375f, -29.639166f, 32.218349f}));
	}

	// The following cases had changed results from a previous implementation, but the new results
	// seem to be correct. Their output was visually verified and the penetration vectors seemed to
	// be accurate.
	{
		collision_result result = collision_test(
			env.small_capsule,
			(tics_transform){.position = {24.852102f, -24.815182f, 20.987659f},
							 .rotation = {0.183501f, 0.197979f, -0.119481f, -0.955435f}},
			env.small_box,
			(tics_transform){.position = {25.135647f, -24.707201f, 20.935389f},
							 .rotation = {0.784483f, 0.107370f, 0.610461f, 0.019866f}});

		ASSERT_TRUE(result.has_collision);
		ASSERT_FLOAT_APPROX(result.depth, 0.188543f);
		ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){-0.962059f, -0.099921f, 0.253884f}));
		ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){24.988197f, -24.524965f, 20.820034f}));
		ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){24.806808f, -24.543804f, 20.867903f}));
	}
	{
		collision_result result = collision_test(
			env.small_box,
			(tics_transform){.position = {-2.713785f, -6.535111f, -12.512428f},
							 .rotation = {-0.475887f, -0.662614f, -0.568872f, -0.104200f}},
			env.small_capsule,
			(tics_transform){.position = {-2.821120f, -6.486034f, -11.969286f},
							 .rotation = {-0.124086f, 0.019645f, -0.734432f, -0.666953f}});

		ASSERT_TRUE(result.has_collision);
		ASSERT_FLOAT_APPROX(result.depth, 0.076532f);
		ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){-0.087536f, -0.369438f, -0.925122f}));
		ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){-2.909770f, -6.536714f, -12.063334f}));
		ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){-2.916469f, -6.564988f, -12.134134f}));
	}
	{
		collision_result result = collision_test(
			env.small_box,
			(tics_transform){.position = {-9.754210f, -0.102319f, 11.734973f},
							 .rotation = {-0.597061f, 0.324355f, -0.492304f, 0.544012f}},
			env.small_capsule,
			(tics_transform){.position = {-9.940026f, -0.119196f, 12.115313f},
							 .rotation = {0.684962f, 0.486646f, 0.362394f, -0.403327f}});

		ASSERT_TRUE(result.has_collision);
		ASSERT_FLOAT_APPROX(result.depth, 0.192813f);
		ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){0.148319f, -0.197690f, -0.968979f}));
		ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){-9.652744f, -0.178941f, 12.058095f}));
		ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){-9.624146f, -0.217059f, 11.871264f}));
	}
	{
		collision_result result = collision_test(
			env.small_box,
			(tics_transform){.position = {18.804766f, -27.180012f, -27.706049f},
							 .rotation = {-0.296268f, -0.449414f, 0.229265f, -0.810981f}},
			env.small_capsule,
			(tics_transform){.position = {19.146282f, -27.316242f, -27.718805f},
							 .rotation = {0.057886f, -0.550599f, -0.822911f, -0.127699f}});

		ASSERT_TRUE(result.has_collision);
		ASSERT_FLOAT_APPROX(result.depth, 0.225207f);
		ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){-0.754026f, 0.655964f, 0.034001f}));
		ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){19.195988f, -27.308809f, -27.795370f}));
		ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){19.026176f, -27.161081f, -27.787712f}));
	}
	{
		collision_result result = collision_test(
			env.small_box,
			(tics_transform){.position = {18.934721f, -26.752932f, -26.948402f},
							 .rotation = {-0.266401f, -0.764306f, 0.251981f, -0.530445f}},
			env.small_capsule,
			(tics_transform){.position = {19.063068f, -26.943653f, -26.790159f},
							 .rotation = {0.383450f, -0.093997f, -0.570465f, -0.720208f}});

		ASSERT_TRUE(result.has_collision);
		ASSERT_FLOAT_APPROX(result.depth, 0.402462f);
		ASSERT_VEC3_APPROX(result.normal, ((tics_vec3){0.363772f, 0.669471f, -0.647671f}));
		ASSERT_VEC3_APPROX(result.point_a, ((tics_vec3){18.910431f, -27.081247f, -26.690079f}));
		ASSERT_VEC3_APPROX(result.point_b, ((tics_vec3){19.056835f, -26.811811f, -26.950741f}));
	}
	// This case has multiple acceptable solutions:
	// depth=0.007191, normal=(-0.837791, 0.466698, 0.283370) len=1.000000, pt_a=(-29.384985, -24.549356, 3.422855)
	// depth=0.007198, normal=(-0.772748, 0.134516, 0.620295) len=1.000000, pt_a=(-29.384892, -24.548313, 3.421410)
	{
		collision_result result = collision_test(
			env.small_box,
			(tics_transform){.position = {25.037027, -18.998978, 5.304494},
							 .rotation = {-0.310111, -0.557608, 0.545750, 0.543196}},
			env.small_box,
			(tics_transform){.position = {24.736820, -19.656927, 5.465693},
							 .rotation = {-0.131284, 0.743533, -0.096580, -0.648534}});
		ASSERT_TRUE(result.has_collision);
	}

	tics_world_destroy(env.world);
}
