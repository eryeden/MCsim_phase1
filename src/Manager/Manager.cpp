#include <Manager/Manager.hpp>

using namespace SimulationManager;

Manager::Manager(
	Controller::Base & _controller_base
	, MC::Core & _mc_core)

	: controller_base(_controller_base)
	, mc_core(_mc_core)
	, gl_sv()
{

	gl_sv.GenerateModel(mc_core);
	gl_sv.GetWorldHandler().SetPositionLight(
		glm::vec3(0, 50, 0)
		);
}


bool Manager::Update() {

	controller_base.Update();
	//mc_core.update();

	GLFWwindow * window = const_cast<GLFWwindow *>(gl_sv.GetWindowHandler());

	gl_sv.Render();

	glfwPollEvents();
	if (GLFW_PRESS == glfwGetKey(window, GLFW_KEY_ESCAPE)) {
		glfwSetWindowShouldClose(window, 1);
	}
	else if (GLFW_PRESS == glfwGetKey(window, GLFW_KEY_X)) {

	}
	else if (GLFW_PRESS == glfwGetKey(window, GLFW_KEY_C)) {

	}
	return glfwWindowShouldClose(window);
}
