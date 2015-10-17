#ifndef _MCSMANAGER_MCSMANAGER_
#define _MCSMANAGER_MCSMANAGER_

#include <Eigen/Dense>
#include <vector>
#include <string>
//using namespace Eigen;
#define MC_G 9.80665

#define USE_STL_VECTOR

#include <Sim/MC.hpp>
#include <Controller/Base.hpp>
#include <Supervisor.hpp>


namespace SimulationManager {

	//������p�����Ďg��
	//Sim�AController�AWorld���܂Ƃ߂��ʑ��݂��ŏ���Initialize���ĂсAUpdate���X�V���ɌĂԂ悤�ɂ���


	class Manager {
	public:
		Manager(
			Controller::Base & _controller_base
			, MC::Core & _mc_core
			);

		//virtual ~Manager() = 0;

		bool Update();


	private:

		Controller::Base & controller_base;
		MC::Core & mc_core;
		Space::Supervisor gl_sv;


	protected:


	};


};


#endif //!_MCSMANAGER_MCSMANAGER_