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

	//これを継承して使う
	//Sim、Controller、Worldをまとめる上位存在が最初にInitializeを呼び、Updateを更新時に呼ぶようにする


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