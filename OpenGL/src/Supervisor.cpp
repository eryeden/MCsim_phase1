#include <Supervisor.hpp>

using namespace Space;
using namespace glm;

Supervisor::Supervisor() 
	:world()
{

	Initialize(800, 600);

}

Supervisor::~Supervisor() {
	glfwTerminate();
}

bool Supervisor::Initialize(
	const int & _width_window
	, const int & _height_window
	) 
{
	if (!glfwInit()) {
		fprintf(stderr, "ERROR: could not start GLFW3\n");
		return false;
	}

	int w, h;
	w = 800;
	h = 600;

	window = glfwCreateWindow(w, h, "MCS",
		NULL, NULL);
	if (!window) {
		fprintf(stderr, "ERROR: could not open window with GLFW3\n");
		glfwTerminate();
		return false;
	}

	glfwMakeContextCurrent(window);

	//GLEWを使う場合
#ifdef _MSC_VER
	glewExperimental = GL_TRUE;
	glewInit();
#endif

	const GLubyte* renderer = glGetString(GL_RENDERER);
	const GLubyte* version = glGetString(GL_VERSION);
	printf("Renderer: %s\n", renderer);
	printf("Version: %s\n", version);

	//ワールド初期化
	world.Initialize(window);

	//シェーダーのバインドをする
	world.BindShader(
		"./../../OpenGL/glsl/StandardShading_vs.glsl"
		, "./../../OpenGL/glsl/StandardShading_fs.glsl"
		, "./../../OpenGL/glsl/StandardShading_vs_non_tex.glsl"
		, "./../../OpenGL/glsl/StandardShading_fs_non_tex.glsl"
		);

	world.BindShaderPostProcess(
		"./../../OpenGL/glsl/pp_vs.glsl"
		, "./../../OpenGL/glsl/pp_fs.glsl"
		, "./../../OpenGL/glsl/fxaa_vs2.glsl"
		, "./../../OpenGL/glsl/fxaa_fs2.glsl"
		);

	world.BindShaderShadowMapping(
		"./../../OpenGL/glsl/shadow_mapping_depth_vs.glsl"
		, "./../../OpenGL/glsl/shadow_mapping_depth_fs.glsl"
		, "./../../OpenGL/glsl/shadow_mapping_standard_vs.glsl"
		, "./../../OpenGL/glsl/shadow_mapping_standard_fs.glsl"
		);

	//world.SetPositionLight(vec3(20, 50, 20));
}

//Worldへの参照を返す
const World & Supervisor::GetWorldHundler() {
	return world;
}

void Supervisor::GenerateModel(MC::Core & _mc_core) {
	
}



