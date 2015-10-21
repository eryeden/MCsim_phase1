#include <Controller/Controller_PIDeuler.hpp>
#include <iostream>
#include <random>
#include <ctime>

using namespace Controller;
using namespace Eigen;


std::mt19937 rand2(static_cast<unsigned int>(time(nullptr)));
std::uniform_int_distribution<int> dist(-1000, 1000);

Controller_PID_Euler::Controller_PID_Euler(MC::Core & _mc_core, const double & _dt)
	:Base(_mc_core)
	, Kp(0), Ti(0), Td(0)
	, theta_command(0)
	, theta_present(0)
	, theta_previous(0)
	, theta_derivative(0)
	, theta_integral(0)
	, error_present(0)
	, error_previous(0)
	, dt(_dt)
	, p_base(4600)
	, xboxctrlr(1)
{
	Initialize();
}

Controller_PID_Euler::~Controller_PID_Euler() {

}

void Controller_PID_Euler::Initialize() {
	Set_w_m_All(0);
}

void Controller_PID_Euler::Update() {
	Vector3d angle_euler = core.GetEulerinDegrees();



	Matrix<double, 6, 1> ctrlr_state = xboxctrlr.GetSticksTrigers();
	printf("LX:%f\tLY:%f\tRX:%f\tRY:%f\tLT:%f\tRT:%f\n"
		, ctrlr_state(0)
		, ctrlr_state(1)
		, ctrlr_state(2)
		, ctrlr_state(3)
		, ctrlr_state(4)
		, ctrlr_state(5)
		);

	
	double p_base_out = p_base + 90.0 * (ctrlr_state(5) - ctrlr_state(4));
	Command(ctrlr_state(0) * 20.0);



	//Set_w_m_All(4598.2);
	Set_w_m_All(p_base_out);

	theta_previous = theta_present;
	theta_present = angle_euler(0) + dist(rand2) / 100.0f;

	error_previous = error_present;
	error_present = (theta_command - theta_present);

	theta_integral += error_present * dt;
	theta_derivative = ((error_present) - (error_previous)) / dt;

	//double wdiff
	//	= Kp * (1.0 + Td * theta_derivative + (1.0 / Ti) * theta_integral);
	double wdiff
		= Kp * error_present
		+ Kp * Td * theta_derivative
		+ Kp * (1.0/Ti) * theta_integral
		;


	core.mtrplps[0]->w_m -= wdiff;
	core.mtrplps[1]->w_m += wdiff;

	core.mtrplps[2]->w_m += wdiff;
	core.mtrplps[3]->w_m -= wdiff;

	double upper = 7000;
	double lower = 0;
	for (auto itr : core.mtrplps) {
		itr->w_m = (itr->w_m > upper) ? upper : itr->w_m;
		itr->w_m = (itr->w_m < lower) ? lower : itr->w_m;
	}


	//printf("the_pres:%f\twd:%f\tdiv:%f\tintg:%f\n", theta_present, wdiff, theta_derivative, theta_integral);



}

void Controller_PID_Euler::Set_w_m_All(float _wm) {

	for (auto itr : core.mtrplps) {
		itr->w_m = _wm;// +dist(rand2) / 10.0f;
	}

}
