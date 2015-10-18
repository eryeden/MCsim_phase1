#include <Manager/Manager.hpp>
#include <Controller/Controller_test.hpp>

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
		glm::vec3(30, 50, 30)
		);
}


bool Manager::Update() {

	controller_base.Update();
	mc_core.update();

	GLFWwindow * window = const_cast<GLFWwindow *>(gl_sv.GetWindowHandler());


	glm::vec3 position_worldspace;
	position_worldspace.x = mc_core.x(6);
	position_worldspace.y = mc_core.x(7);
	position_worldspace.z = mc_core.x(8);

	//gl_sv.GetModelHandler().SetModelPositionWorldSpace(
	//	Space::Utility::Convert_m_To_in(
	//		Space::Supervisor::ConvertVector3dToVec3(
	//			Eigen::Vector3d(
	//				mc_core.x(6), mc_core.x(7), mc_core.x(8)
	//				)
	//			)
	//		)
	//	);

	gl_sv.GetModelHandler().SetModelPositionWorldSpace(
		Space::Utility::Convert_m_To_in(position_worldspace)
		);

	glm::mat4 att;
	//Eigen::Matrix3d att33 = mc_core.GetAttitudeMatrix();
	Eigen::Matrix3d att33 = dynamic_cast<Controller::Controller_test &>(controller_base).GetDCM();
	att = glm::mat4(
		att33(0, 0), att33(1, 0), att33(2, 0), 0.0f
		, att33(0, 1), att33(1, 1), att33(2, 1), 0.0f
		, att33(0, 2), att33(1, 2), att33(2, 2), 0.0f
		, 0.0f, 0.0f, 0.0f, 1.0f
		);
	gl_sv.GetModelHandler().SetModelAttitude(att);

	//printf("p: %4.3f\t%4.3f\t%4.3f\n", mc_core.x(6), mc_core.x(7), mc_core.x(8));

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
