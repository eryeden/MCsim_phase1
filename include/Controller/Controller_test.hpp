#ifndef _CONTROLLER_CONTROLLER_TEST_
#define _CONTROLLER_CONTROLLER_TEST_

#include <Eigen/Dense>
#include <vector>
#include <string>
//using namespace Eigen;
#define MC_G 9.80665

#define USE_STL_VECTOR

#include <Sim/MC.hpp>
#include <Controller/Base.hpp>

namespace Controller {

	class Controller_test : public Controller::Base {
	public:
		Controller_test(MC::Core & _mc_core);

		virtual ~Controller_test() = 0;



	private:

	};


};


#endif //_CONTROLLER_CONTROLLER_TEST_








