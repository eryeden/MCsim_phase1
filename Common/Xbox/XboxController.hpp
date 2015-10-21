#ifndef _XBOX_CONTROLLER_H_
#define _XBOX_CONTROLLER_H_

// No MFC
#define WIN32_LEAN_AND_MEAN

// We need the Windows Header and the XInput Header
#include <windows.h>
#include <XInput.h>
#include <Eigen/Dense>

#pragma comment(lib, "XInput.lib")

namespace XboxController {
	// XBOX Controller Class Definition
	class CXBOXController
	{
	private:
		XINPUT_STATE _controllerState;
		int _controllerNum;
	public:
		CXBOXController(int playerNumber);
		XINPUT_STATE GetState();
		bool IsConnected();
		void Vibrate(int leftVal = 0, int rightVal = 0);
		static Eigen::Vector4d GetThumb(const XINPUT_STATE & _xin);
		static Eigen::Vector2d GetTriger(const XINPUT_STATE & _xin);
		Eigen::Matrix<double, 6, 1> GetSticksTrigers();
	};
}

#endif