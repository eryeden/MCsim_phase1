#ifndef _CONTROLLER_BASE_
#define _CONTROLLER_BASE_

#include <Eigen/Dense>
#include <vector>
#include <string>
//using namespace Eigen;
#define MC_G 9.80665

#define USE_STL_VECTOR

#include <Sim/MC.hpp>

namespace Controller {

	//これを継承して使う
	//Sim、Controller、Worldをまとめる上位存在が最初にInitializeを呼び、Updateを更新時に呼ぶようにする


	class Base {
	public:
		Base(MC::Core & _mc_core);

		virtual ~Base() = 0;
		
		virtual void Initialize() = 0;
		virtual void Update() = 0;


	private:
		MC::Core & core;

	};
	

};


#endif //_CONTROLLER_BASE_