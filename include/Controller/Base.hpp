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

	//������p�����Ďg��
	//Sim�AController�AWorld���܂Ƃ߂��ʑ��݂��ŏ���Initialize���ĂсAUpdate���X�V���ɌĂԂ悤�ɂ���


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