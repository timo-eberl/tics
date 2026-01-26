// Standard Vertex Shader (Passes data to Fragment Shader)
static const char* VS_CODE =
"#version 330\n"
"in vec3 vertexPosition;\n"
"in vec2 vertexTexCoord;\n"
"in vec3 vertexNormal;\n"
"in vec4 vertexColor;\n"
"uniform mat4 mvp;\n"
"uniform mat4 matModel;\n"
"uniform mat4 matNormal;\n"
"out vec3 fragPosition;\n"
"out vec3 fragNormal;\n"
"out vec2 fragTexCoord;\n"
"void main() {\n"
"	fragPosition = vec3(matModel * vec4(vertexPosition, 1.0));\n"
"	fragNormal = normalize(vec3(matNormal * vec4(vertexNormal, 1.0)));\n"
"	fragTexCoord = vertexTexCoord;\n"
"	gl_Position = mvp * vec4(vertexPosition, 1.0);\n"
"}\n";

// Fragment Shader (matches 'shade' function)
static const char* FS_CODE =
"#version 330\n"
"in vec3 fragPosition;\n"
"in vec3 fragNormal;\n"
"in vec2 fragTexCoord;\n"
"uniform vec4 colDiffuse;\n"
"uniform sampler2D texture0;\n"
"uniform vec3 viewPos;\n" // Camera Position
"out vec4 finalColor;\n"
"void main() {\n"
"	vec3 normal = normalize(fragNormal);\n"
"	vec3 light_dir = normalize(viewPos - fragPosition);\n"
"	// Replicating: n_dot_l = fabsf(Vector3DotProduct(normal, light_dir));\n"
"	float n_dot_l = abs(dot(normal, light_dir));\n"
"	// Replicating: intensity = fmax(pow(n_dot_l, 0.2), 0.7);\n"
"	float intensity = max(pow(n_dot_l, 0.2), 0.7);\n"
"	vec4 base = colDiffuse * texture(texture0, fragTexCoord);\n"
"	finalColor = vec4(base.rgb * intensity, base.a);\n"
"}\n";

// Minimal Vertex Shader: Just projects the position
static const char* VS_WIRE_CODE =
"#version 330\n"
"in vec3 vertexPosition;\n"
"uniform mat4 mvp;\n"
"void main() {\n"
"	gl_Position = mvp * vec4(vertexPosition, 1.0);\n"
"}\n";

// Minimal Fragment Shader: Just outputs the color
static const char* FS_WIRE_CODE =
"#version 330\n"
"uniform vec4 colDiffuse;\n"
"out vec4 finalColor;\n"
"void main() {\n"
"	finalColor = colDiffuse;\n"
"}\n";
