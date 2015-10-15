#include <Supervisor.hpp>
#include <Eigen/Dense>


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
	Model mdl_component;
	vec3 position_modelspace;
	vec3 position_worldspace;
	mat4 attitude;
	mat4 attitude_model;
	Eigen::Matrix3d att33;

	position_worldspace = vec3(0.0f);
	attitude_model = rotate(mat4(1.0f), (float)(0.0), vec3(0.0, 0.0, 1.0));

	mdl_component.SetModelPositionWorldSpace(position_worldspace);
	mdl_component.SetModelAttitude(attitude_model);

	space_models.push_back(mdl_component);

	for (auto itr : _mc_core.components) {
		if (itr->id != MC::ID_COMPONENT) {
			//STLコンポーネントでなければ飛ばす
			continue;
		}

		Object obj_componet;

		//アップキャスト
		MC::StLComponent * comp = dynamic_cast<MC::StLComponent *>(itr);
		obj_componet.LoadModel(comp->GetPathToModelFile());

		position_modelspace.x = comp->GetPositionModelspace()(0);
		position_modelspace.y = comp->GetPositionModelspace()(1);
		position_modelspace.z = comp->GetPositionModelspace()(2);

		att33 = comp->GetAttitude();

		attitude = mat4(
			att33(0, 0), att33(1, 0), att33(2, 0), 0.0f
			, att33(0, 1), att33(1, 1), att33(2, 1), 0.0f
			, att33(0, 2), att33(1, 2), att33(2, 2), 0.0f
			, 0.0f, 0.0f, 0.0f, 1.0f
			);

		obj_componet.SetObjectPositionModelSpace(position_modelspace);
		obj_componet.SetObjectAttitude(attitude);
		space_objects.push_back(obj_componet);
	}

	//モデルにオブジェクトを追加
	for (auto itr : space_objects) {
		space_models[0].AddObject(&itr);
	}

	world.AddModel(&space_models[0]);


}



