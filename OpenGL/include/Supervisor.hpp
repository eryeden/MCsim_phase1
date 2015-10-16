#ifndef _WORLD_SUPERVISOR_
#define _WORLD_SUPERVISOR_

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

#include "shader.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/geometric.hpp>
//using namespace glm;

#include <World.hpp>
#include <Sim/MC.hpp>

namespace Space {

	class Supervisor {
	public:

		Supervisor();
		~Supervisor();

		bool Initialize(
			const int & _width_window
			, const int & _height_window
			);

		//Worldへの参照を返す
		Space::World & GetWorldHundler();


		void GenerateModel(MC::Core & _mc_core);
		//void SetModelParameter();


		void RenderLoop();

	private:
		//OpenGL関係はManagerがすべて管理する
		Space::World  world;
		//これらはmc_coreにより自動生成される
		std::vector<Space::Model> space_models;
		std::vector<Space::Object> space_objects;

		GLFWwindow * window;

		void update_fps_counter(GLFWwindow * window);

	};


};





#endif	//!_WORLD_SUPERVISOR_