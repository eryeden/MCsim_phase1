/*
The MIT License (MIT)

Copyright (c) 2015 Kazuki Kikuchi

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

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


//Windows�̂�
#ifdef _MSC_VER
#include <Xbox/XboxController.hpp>
#endif

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
		ControllerPID controller_yawrate;

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

	  //Windows�̂�xs
#ifdef _MSC_VER
		XboxController::CXBOXController xboxctrlr;
#endif
	};




};





#endif //_CONTROLLER_CONTROLLER_PIDEULER_








