#include <Manager/Manager.hpp>

using namespace SimulationManager;

Manager::Manager(
	Controller::Base & _controller_base
	, MC::Core & _mc_core) 
	: controller_base(_controller_base)
	, mc_core(_mc_core)
{
	;
}


void Manager::InitializeOpenGL() {

}

void Manager::GenerateWorld() {

}

