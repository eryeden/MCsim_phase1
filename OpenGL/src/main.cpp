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
	c_q = 0.000000008;

	//c_t = 0.00000000;
	//c_q = 0.00000000;

	mass_body = 0.473932;
	position_body = 0.001f * Vector3d::Zero();
	//position_cog_body = 0.001f * Vector3d(
	//	-0.014185
	//	, -0.000018
	//	, 6.483679);
	position_cog_body = 0.001f * Vector3d(
		0
		, 0
		, 6.483679);
	att_body = Matrix3d::Identity();
	j_body = Matrix3d::Zero();
	j_body(0, 0) = 0.005991;
	j_body(1, 1) = 0.005990;
	j_body(2, 2) = 0.011815;

	mass_plop0 = 0.054000;
	position_plop0 = 0.001f * Vector3d(141, -141, 30.50);
	position_cog_plop0 = 0.001f * Vector3d(0.00, 14.36, 0.00);
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
	position_plop1 = 0.001f * Vector3d(141, 141, 30.50);
	position_cog_plop1 = 0.001f * Vector3d(0.00, 14.36, 0.00);
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
	position_plop2 = 0.001f * Vector3d(-141, 141, 30.50);
	position_cog_plop2 = 0.001f * Vector3d(0.00, 14.36, 0.00);
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
	position_plop3 = 0.001f * Vector3d(-141, -141, 30.50);
	position_cog_plop3 = 0.001f * Vector3d(0.00, 14.36, 0.00);
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
	plop1.is_ccw = false;

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
	plop3.is_ccw = false;

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

	gene.SetDt(1.0f / 180.0f);
	//gene.SetDt(1.0f / 60.0f);
	//gene.set_initialstate_vb(Vector3d(0, 0, 100));
	//gene.set_initialstate_wb(Vector3d(0.0, 0.0, 0.0));
	//gene.set_initialstate_xe(Vector3d(0, 0, 100));
	//gene.set_initialstate_phie(Vector3d::Zero());

	gene.SetInitialVelocityBodyspace(Vector3d(0, 0, 0));
	gene.SetInitialAngularVelocityBodyspace(Vector3d(0.0, 0.0, 1.0));
	gene.SetInitialPositionEarthspace(Vector3d(0, 0, 0.5));
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

