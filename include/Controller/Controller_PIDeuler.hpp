#ifndef _CONTROLLER_CONTROLLER_PIDEULER_
#define _CONTROLLER_CONTROLLER_PIDEULER_

#include <Eigen/Dense>
#include <vector>
#include <string>
//using namespace Eigen;
#define MC_G 9.80665

#define USE_STL_VECTOR

#include <Sim/MC.hpp>
#include <Controller/Base.hpp>
#include <Xbox/XboxController.hpp>

namespace Controller {

	class ControllerPID {

	public:
		ControllerPID(const double & _dt);
		ControllerPID();


		void Initialize();
		void SetDt(const double & _dt);
		double Update(const double & _x_in);

		void SetKp(const double & _kp) { Kp = _kp; }
		void SetTi(const double & _ti) { Ti = _ti; }
		void SetTd(const double & _td) { Td = _td; };
		void SetCoeffs(
			const double & _kp
			, const double & _ti
			, const double & _td) {
			Kp = _kp;
			Ti = _ti;
			Td = _td;
		}

		void Command(const double & _commd) { x_command = _commd; }

		void Output();

	private:
		//PID�Q�C��
		double Kp, Ti, Td;
		double x_command; //�ڕW�l
		double x_present; //���݂̒l
		double x_previous; //�ЂƂO�̒l
		double error_present;
		double error_previous;
		double error_derivative; //�����l
		double error_integral; //�ϕ��n
		double dt; //����X�e�b�v����
		double wdiff;

	};

	class Controller_PID_Euler : public Controller::Base {
	public:
		Controller_PID_Euler(MC::Core & _mc_core, const double &_dt);

		~Controller_PID_Euler();


		//����X�e�b�v���Ԃ̐ݒ���s��
		void Initialize();
		void Update();

		void SetKp(const double & _kp) { Kp = _kp; }
		void SetTi(const double & _ti) { Ti = _ti; }
		void SetTd(const double & _td) { Td = _td; };
		void SetCoeffs(
			const double & _kp
			, const double & _ti
			, const double & _td) {
			Kp = _kp;
			Ti = _ti;
			Td = _td;
		}

		void Command(const double & _commd) { theta_command = _commd; }
		void SetPBase(const double & _pbase) { p_base = _pbase; }

		ControllerPID controller_altitude;
		ControllerPID controller_pitch;
		ControllerPID controller_roll;

	private:

		void Set_w_m_All(float _wm);

		//PID�Q�C��
		double Kp, Ti, Td;
		double theta_command; //�ڕW�l
		double theta_present; //���݂̒l
		double theta_previous; //�ЂƂO�̒l
		double error_present;
		double error_previous;
		double theta_derivative; //�����l
		double theta_integral; //�ϕ��n
		double dt; //����X�e�b�v����
		double p_base;

		XboxController::CXBOXController xboxctrlr;

	};




};





#endif //_CONTROLLER_CONTROLLER_PIDEULER_








