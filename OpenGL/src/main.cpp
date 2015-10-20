/*
Eigen, GLM, GLI : MIT license
ASSIMP : BSD license
GLFW : zlib/libpng license.
GLEW : Modified BSD License, the Mesa 3-D License (MIT License), and the Khronos License (MIT License)
*/


//Widnowsの場合
#if (_MSC_VER == 1900)	 //Visual Studio 2015
#pragma comment(lib, "glfw3-vc140.lib")
#pragma comment(lib, "assimp-vc140.lib")
#endif

#if (_MSC_VER == 1800)	 //Visual Studio 2013
#pragma comment(lib, "glfw3-vc120")  // glfw3-vc120.lib
#pragma comment(lib, "assimp-vc120.lib")  //assimp-vc120.lib
#endif

#ifdef _MSC_VER
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "GlU32.Lib")
#pragma comment(lib, "glew32.lib")

#include <gl/glew.h>
#define _CRT_SECURE_NO_WARNINGS
#endif

#define GL_GLEXT_PROTOTYPES
//GLEWの拡張を使うには此れ↑が必要（glGenBuffers等）
//Linuxではこれだけで良い　おそらくGLEWのライブラリ不必要
//http://stackoverflow.com/questions/3032386/glgenbuffers-not-defined
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define _USE_MATH_DEFINES
#include <cmath>

#include "shader.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/geometric.hpp>
//using namespace glm;

#include <Supervisor.hpp>
#include <Sim/MC.hpp>
#include <Controller/Controller_test.hpp>
#include <Manager/Manager.hpp>

#include <Eigen/Dense>
using namespace Eigen;




int main() {


	//################### MC SETTINGS ###############################################################

	Vector3d position_body, position_plop0, position_plop1, position_plop2, position_plop3;
	Vector3d position_cog_body, position_cog_plop0, position_cog_plop1, position_cog_plop2, position_cog_plop3;
	Matrix3d att_body, att_plop0, att_plop1, att_plop2, att_plop3;

	double mass_body, mass_plop0, mass_plop1, mass_plop2, mass_plop3;
	Matrix3d j_body, j_plop0, j_plop1, j_plop2, j_plop3;
	Matrix3d jr_plop0, jr_plop1, jr_plop2, jr_plop3;

	double c_t, c_q;

	std::string p2body, p2motor;
	p2body = "./../../OpenGL/models/Body.stl";
	p2motor = "./../../OpenGL/models/bldcm.stl";
	
	std::string p2test = "./../../OpenGL/models/Body.stl";

	c_t = 0.00000008;
	c_q = 0.00000004;

	mass_body = 0.473932;
	position_body = Vector3d::Zero();
	position_cog_body = Vector3d(
		-0.014185
		, -0.000018
		, 6.483679);
	att_body = Matrix3d::Identity();
	j_body = Matrix3d::Zero();
	j_body(0, 0) = 0.005991;
	j_body(1, 1) = 0.005990;
	j_body(2, 2) = 0.011815;

	mass_plop0 = 0.054000;
	position_plop0 = Vector3d(141.42, -141.42, 30.50);
	position_cog_plop0 = Vector3d(0.00, 14.36, 0.00);
	att_plop0 = AngleAxisd(-90.0 * M_PI / 180.0, Vector3d(1.0f, 0.0f, 0.0f));
	j_plop0 = Matrix3d::Zero();
	j_plop0(0, 0) = 0.001104;
	j_plop0(1, 1) = 0.001104;
	j_plop0(2, 2) = 0.002168;
	jr_plop0 = Matrix3d::Zero();
	jr_plop0(0, 0) = 0.000021;
	jr_plop0(1, 1) = 0.000008;
	jr_plop0(2, 2) = 0.000021;

	mass_plop1 = 0.054000;
	position_plop1 = Vector3d(141.42, 141.42, 30.50);
	position_cog_plop1 = Vector3d(0.00, 14.36, 0.00);
	att_plop1 = AngleAxisd(-90.0 * M_PI / 180.0, Vector3d(1.0f, 0.0f, 0.0f));
	j_plop1 = Matrix3d::Zero();
	j_plop1(0, 0) = 0.001104;
	j_plop1(1, 1) = 0.001104;
	j_plop1(2, 2) = 0.002168;
	jr_plop1 = Matrix3d::Zero();
	jr_plop1(0, 0) = 0.000021;
	jr_plop1(1, 1) = 0.000008;
	jr_plop1(2, 2) = 0.000021;

	mass_plop2 = 0.054000;
	position_plop2 = Vector3d(-141.42, 141.42, 30.50);
	position_cog_plop2 = Vector3d(0.00, 14.36, 0.00);
	att_plop2 = AngleAxisd(-90.0 * M_PI / 180.0, Vector3d(1.0f, 0.0f, 0.0f));
	j_plop2 = Matrix3d::Zero();
	j_plop2(0, 0) = 0.001104;
	j_plop2(1, 1) = 0.001104;
	j_plop2(2, 2) = 0.002168;
	jr_plop2 = Matrix3d::Zero();
	jr_plop2(0, 0) = 0.000021;
	jr_plop2(1, 1) = 0.000008;
	jr_plop2(2, 2) = 0.000021;

	mass_plop3 = 0.054000;
	position_plop3 = Vector3d(-141.42, -141.42, 30.50);
	position_cog_plop3 = Vector3d(0.00, 14.36, 0.00);
	att_plop3 = AngleAxisd(-90.0 * M_PI / 180.0, Vector3d(1.0f, 0.0f, 0.0f));
	j_plop3 = Matrix3d::Zero();
	j_plop3(0, 0) = 0.001104;
	j_plop3(1, 1) = 0.001104;
	j_plop3(2, 2) = 0.002168;
	jr_plop3 = Matrix3d::Zero();
	jr_plop3(0, 0) = 0.000021;
	jr_plop3(1, 1) = 0.000008;
	jr_plop3(2, 2) = 0.000021;

	MC::StLComponent body(
		p2body
		, mass_body
		, j_body
		, att_body
		, position_cog_body
		, position_body
		, Vector3d(0.964, 0.714, 0)
		);

	MC::StLMotorPlop plop0(
		c_t
		, c_q
		, jr_plop0
		, p2motor
		, mass_plop0
		, j_plop0
		, att_plop0
		, position_cog_plop0
		, position_plop0
		, Vector3d(0, 0.71, 0.101)
		);

	MC::StLMotorPlop plop1(
		c_t
		, c_q
		, jr_plop1
		, p2motor
		, mass_plop1
		, j_plop1
		, att_plop1
		, position_cog_plop1
		, position_plop1
		, Vector3d(0, 0.71, 0.101)
		);

	MC::StLMotorPlop plop2(
		c_t
		, c_q
		, jr_plop2
		, p2motor
		, mass_plop2
		, j_plop2
		, att_plop2
		, position_cog_plop2
		, position_plop2
		, Vector3d(0, 0.71, 0.101)
		);

	MC::StLMotorPlop plop3(
		c_t
		, c_q
		, jr_plop3
		, p2motor
		, mass_plop3
		, j_plop3
		, att_plop3
		, position_cog_plop3
		, position_plop3
		, Vector3d(0, 0.71, 0.101)
		);

	MC::StLComponent objtest(
		p2body
		, mass_body
		, j_body
		, Matrix3d::Identity()
		, Vector3d(0, 0, 0)
		, Vector3d(0, 0, 0)
		, Vector3d(0, 0.71, 0.101)
		);


	MC::Generator gene;
	gene << &body;
	gene << &plop0;
	gene << &plop1;
	gene << &plop2;
	gene << &plop3;

	//gene << &objtest;

	gene.SetDt(1.0 / 60.0);
	//gene.set_initialstate_vb(Vector3d(0, 0, 100));
	//gene.set_initialstate_wb(Vector3d(0.0, 0.0, 0.0));
	//gene.set_initialstate_xe(Vector3d(0, 0, 100));
	//gene.set_initialstate_phie(Vector3d::Zero());

	gene.SetInitialVelocityBodyspace(Vector3d(0, 0, 0));
	gene.SetInitialAngularVelocityBodyspace(Vector3d(0.0, 0.0, 0.0));
	gene.SetInitialPositionEarthspace(Vector3d(0, 0, 700));
	gene.set_initialstate_phie(Vector3d::Zero());
	gene.SetInitialQuotanion(Vector4d(0, 0, 0, 1));

	//MC::Core core = gene.generate_core();
	MC::Core core = gene.GenerateCore_q();

	//################### MC SETTINGS ###############################################################

	//################### CONTROLLER SETTIGNS #######################################################

	Controller::Controller_test ct(core);
	ct.Initialize();
	ct.Setw(Vector3d(0.0, 0.0, 1));

	//################### CONTROLLER SETTIGNS #######################################################

	////################### INITIALIZE OPENGL ##########################################################
	//Space::Supervisor sv;
	////sv.Initialize(800, 600);
	//sv.GenerateModel(core);
	//sv.GetWorldHandler().SetPositionLight(glm::vec3(0, 50, 0));
	//sv.RenderLoop();
	////################### INITIALIZE OPENGL ##########################################################

	//################### MANAGER SETTINGS ###########################################################

	SimulationManager::Manager mngr(ct, core);

	//################### MANAGER SETTINGS ###########################################################

	//################### LOOP ##################################

	while (!mngr.Update()) {
		;
	}

	//################### LOOP ##################################



}




////Widnowsの場合
//#if (_MSC_VER == 1900)	 //Visual Studio 2015
//#pragma comment(lib, "glfw3-vc140.lib")
//#pragma comment(lib, "assimp-vc140.lib")
//#endif
//
//#if (_MSC_VER == 1800)	 //Visual Studio 2013
//#pragma comment(lib, "glfw3-vc120")  // glfw3-vc120.lib
//#pragma comment(lib, "assimp-vc120.lib")  //assimp-vc120.lib
//#endif
//
//#ifdef _MSC_VER
//#pragma comment(lib, "opengl32.lib")
//#pragma comment(lib, "GlU32.Lib")
//#pragma comment(lib, "glew32.lib")
//
//#include <gl/glew.h>
//#define _CRT_SECURE_NO_WARNINGS
//#endif
//
//#define GL_GLEXT_PROTOTYPES
////GLEWの拡張を使うには此れ↑が必要（glGenBuffers等）
////Linuxではこれだけで良い　おそらくGLEWのライブラリ不必要
////http://stackoverflow.com/questions/3032386/glgenbuffers-not-defined
//#include <GLFW/glfw3.h>
//#include <cstdio>
//#include <cstdlib>
//#include <cstring>
//
//#include "shader.hpp"
//#include <glm/glm.hpp>
//#include <glm/gtc/matrix_transform.hpp>
//#include <glm/geometric.hpp>
//using namespace glm;
//
//#include "World.hpp"
//
//
//void gl_execute2(GLFWwindow * window);
//void _update_fps_counter(GLFWwindow * window);
//
//int main() {
//
//	if (!glfwInit()) {
//		fprintf(stderr, "ERROR: could not start GLFW3\n");
//		return 1;
//	}
//
//	int w, h;
//	w = 800;
//	h = 600;
//
//	GLFWwindow * window = glfwCreateWindow(w, h, "Hello Triangle",
//		NULL, NULL);
//	if (!window) {
//		fprintf(stderr, "ERROR: could not open window with GLFW3\n");
//		glfwTerminate();
//		return 1;
//	}
//
//	glfwMakeContextCurrent(window);
//
//	//GLEWを使う場合
//#ifdef _MSC_VER
//	glewExperimental = GL_TRUE;
//	glewInit();
//#endif
//
//	const GLubyte* renderer = glGetString(GL_RENDERER);
//	const GLubyte* version = glGetString(GL_VERSION);
//	printf("Renderer: %s\n", renderer);
//	printf("Version: %s\n", version);
//
//	gl_execute2(window);
//
//	glfwTerminate();
//	return 0;
//}
//
//
//
//void gl_execute2(GLFWwindow * window) {
//
//
//	Space::World world(window);
//
//
//
//	world.BindShader(
//		"./../../OpenGL/glsl/StandardShading_vs.glsl"
//		, "./../../OpenGL/glsl/StandardShading_fs.glsl"
//		, "./../../OpenGL/glsl/StandardShading_vs_non_tex.glsl"
//		, "./../../OpenGL/glsl/StandardShading_fs_non_tex.glsl"
//		);
//
//	world.BindShaderPostProcess(
//		"./../../OpenGL/glsl/pp_vs.glsl"
//		, "./../../OpenGL/glsl/pp_fs.glsl"
//		, "./../../OpenGL/glsl/fxaa_vs2.glsl"
//		, "./../../OpenGL/glsl/fxaa_fs2.glsl"
//		);
//
//	//world.BindShaderShadowMapping(
//	//	"./../../OpenGL/glsl/shadow_mapping_depth_vs.glsl"
//	//	, "./../../OpenGL/glsl/shadow_mapping_depth_fs.glsl"
//	//	, "./../../OpenGL/glsl/shadow_mapping_vs.glsl"
//	//	, "./../../OpenGL/glsl/shadow_mapping_fs.glsl"
//	//	);
//
////#ifdef USE_SHADOWMAPPING
//	world.BindShaderShadowMapping(
//		"./../../OpenGL/glsl/shadow_mapping_depth_vs.glsl"
//		, "./../../OpenGL/glsl/shadow_mapping_depth_fs.glsl"
//		, "./../../OpenGL/glsl/shadow_mapping_standard_vs.glsl"
//		, "./../../OpenGL/glsl/shadow_mapping_standard_fs.glsl"
//		);
////#endif
//
//	Space::Object obj, obj1, obj2;
//	Space::Object bldcm0, bldcm1, bldcm2, bldcm3, body;
//	Space::Model mdl, mdl1;
//
//	//obj.LoadModel("C:/Users/ery/OneDrive/Documents/Models/Suspension.stl", vec3(0.964, 0.714, 0));
////	obj1.LoadModel("./../../OpenGL/models/t1.stl", vec3(0.964, 0.714, 0));
////	obj2.LoadModel("./../../OpenGL/models/t1.stl", vec3(0, 0.71, 0.101));
//	//obj.LoadModel("./../../OpenGL/models/Asm.stl", vec3(0.964, 0.714, 0));
//	obj1.LoadModel("./../../OpenGL/models/bldcm.stl", vec3(0, 0.71, 0.101));
//	bldcm0.LoadModel("./../../OpenGL/models/bldcm.stl", vec3(0, 0.71, 0.101));
//	bldcm1.LoadModel("./../../OpenGL/models/bldcm.stl", vec3(0, 0.71, 0.101));
//	bldcm2.LoadModel("./../../OpenGL/models/bldcm.stl", vec3(0, 0.71, 0.101));
//	bldcm3.LoadModel("./../../OpenGL/models/bldcm.stl", vec3(0, 0.71, 0.101));
//	body.LoadModel("./../../OpenGL/models/Body.stl", vec3(0.964, 0.714, 0));
//
//	obj2.LoadModel("./../../OpenGL/models/sphere_v1.stl", vec3(0, 0.71, 0.101));
//
//
//
//#define __PI__ 	3.14159265359f
//
//	world.SetPositionLight(vec3(20, 50, 20));
//
//	obj.SetObjectPositionModelSpace(vec3(20.0, 0.0, 0.0));
//	obj.SetObjectAttitude(rotate(mat4(1.0f), (float)(__PI__ / 180.0 * 0), vec3(0.0, 0.0, 1.0)));
//
//
//	obj1.SetObjectPositionModelSpace(Space::Utility::Convert_m_To_in(vec3(141.42, 141.42, 30.5)));
//	obj1.SetObjectAttitude(rotate(mat4(1.0f), (float)(__PI__ / 180.0 * -90.0), vec3(1.0, 0.0, 0.0)));
//
//	obj2.SetObjectPositionModelSpace(Space::Utility::Convert_m_To_in(vec3(0.0, 0.0, 0.0)));
//	obj2.SetObjectAttitude(rotate(mat4(1.0f), (float)(__PI__ / 180.0 * 0.0), vec3(1.0, 0.0, 0.0)));
//
//	body.SetObjectPositionModelSpace(vec3(0.0, 0.0, 0.0));
//	body.SetObjectAttitude(rotate(mat4(1.0f), (float)(__PI__ / 180.0 * 0), vec3(0.0, 0.0, 1.0)));
//
//	bldcm0.SetObjectPositionModelSpace(Space::Utility::Convert_m_To_in(vec3(-141.42, 141.42, 30.5)));
//	bldcm0.SetObjectAttitude(rotate(mat4(1.0f), (float)(__PI__ / 180.0 * -90.0), vec3(1.0, 0.0, 0.0)));
//	bldcm1.SetObjectPositionModelSpace(Space::Utility::Convert_m_To_in(vec3(141.42, 141.42, 30.5)));
//	bldcm1.SetObjectAttitude(rotate(mat4(1.0f), (float)(__PI__ / 180.0 * -90.0), vec3(1.0, 0.0, 0.0)));
//	bldcm2.SetObjectPositionModelSpace(Space::Utility::Convert_m_To_in(vec3(141.42, -141.42, 30.5)));
//	bldcm2.SetObjectAttitude(rotate(mat4(1.0f), (float)(__PI__ / 180.0 * -90.0), vec3(1.0, 0.0, 0.0)));
//	bldcm3.SetObjectPositionModelSpace(Space::Utility::Convert_m_To_in(vec3(-141.42, -141.42, 30.5)));
//	bldcm3.SetObjectAttitude(rotate(mat4(1.0f), (float)(__PI__ / 180.0 * -90.0), vec3(1.0, 0.0, 0.0)));
//
//	//mdl.AddObject(&obj);
//	//mdl.AddObject(&obj1);
//	//mdl.AddObject(&obj2);
//
//	mdl.AddObject(&body);
//	mdl.AddObject(&bldcm0);
//	mdl.AddObject(&bldcm1);
//	mdl.AddObject(&bldcm2);
//	mdl.AddObject(&bldcm3);
//
//	mdl.SetModelPositionWorldSpace(vec3(6.0, 6.0, 6.0));
//	mdl.SetModelAttitude(rotate(mat4(1.0f), (float)(__PI__ / 180.0 * 0.0), vec3(0.0, 0.0, 1.0)));
//
//	//mdl1.AddObject(&obj);
//	//mdl1.AddObject(&obj1);
//	mdl1.AddObject(&obj2);
//	mdl1.SetModelPositionWorldSpace(vec3(0.0, 0.0, 0.1));
//	mdl1.SetModelAttitude(rotate(mat4(1.0f), (float)(__PI__ / 180.0 * 0.0), vec3(0.0, 0.0, 1.0)));
//
//	world.AddModel(&mdl);
//	world.AddModel(&mdl1);
//
//	while (!glfwWindowShouldClose(window)) {
//		_update_fps_counter(window);
//
//		//world.Render();
//
//		world.RenderShadowMapping();
//
//		glfwSwapBuffers(window);
//		glfwPollEvents();
//
//		if (GLFW_PRESS == glfwGetKey(window, GLFW_KEY_ESCAPE)) {
//			glfwSetWindowShouldClose(window, 1);
//		}
//		else if (GLFW_PRESS == glfwGetKey(window, GLFW_KEY_X)) {
//			
//		}
//		else if (GLFW_PRESS == glfwGetKey(window, GLFW_KEY_C)) {
//
//		}
//
//	}
//}
//
//void _update_fps_counter(GLFWwindow * window) {
//	static double previous_seconds = glfwGetTime();
//	static int frame_count;
//	double current_seconds = glfwGetTime();
//	double elapsed_seconds = current_seconds - previous_seconds;
//	if (elapsed_seconds > 0.25) {
//		previous_seconds = current_seconds;
//		double fps = (double)frame_count / elapsed_seconds;
//		char tmp[128];
//		sprintf(tmp, "opengl @ fps: %.2f", fps);
//		glfwSetWindowTitle(window, tmp);
//		frame_count = 0;
//	}
//	frame_count++;
//}
//


