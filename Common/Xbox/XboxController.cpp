#include <Xbox/XboxController.hpp>
using namespace XboxController;

CXBOXController::CXBOXController(int playerNumber)
{
	// Set the Controller Number
	_controllerNum = playerNumber - 1;
}

XINPUT_STATE CXBOXController::GetState()
{
	// Zeroise the state
	ZeroMemory(&_controllerState, sizeof(XINPUT_STATE));

	// Get the state
	XInputGetState(_controllerNum, &_controllerState);

	return _controllerState;
}

bool CXBOXController::IsConnected()
{
	// Zeroise the state
	ZeroMemory(&_controllerState, sizeof(XINPUT_STATE));

	// Get the state
	DWORD Result = XInputGetState(_controllerNum, &_controllerState);

	if (Result == ERROR_SUCCESS)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void CXBOXController::Vibrate(int leftVal, int rightVal)
{
	// Create a Vibraton State
	XINPUT_VIBRATION Vibration;

	// Zeroise the Vibration
	ZeroMemory(&Vibration, sizeof(XINPUT_VIBRATION));

	// Set the Vibration Values
	Vibration.wLeftMotorSpeed = leftVal;
	Vibration.wRightMotorSpeed = rightVal;

	// Vibrate the controller
	XInputSetState(_controllerNum, &Vibration);
}


Eigen::Vector4d CXBOXController::GetThumb(const XINPUT_STATE & _xin) {
	return Eigen::Vector4d(
		(double)(_xin.Gamepad.sThumbLX) / 65535.0 * 2.0 //-0.5
		, (double)(_xin.Gamepad.sThumbLY) / 65535.0 * 2.0//	-0.5
		, (double)(_xin.Gamepad.sThumbRX) / 65535.0	* 2.0//-0.5
		, (double)(_xin.Gamepad.sThumbRY) / 65535.0	* 2.0//-0.5
		);
}

Eigen::Vector2d CXBOXController::GetTriger(const XINPUT_STATE & _xin) {
	return Eigen::Vector2d(
		(double)(_xin.Gamepad.bLeftTrigger) / 255.0	
		, (double)(_xin.Gamepad.bRightTrigger) / 255.0 
		);

}

Eigen::Matrix<double, 6, 1> CXBOXController::GetSticksTrigers() {
	GetState();
	Eigen::Matrix<double, 6, 1> ret;
	ret.block<4, 1>(0, 0) = GetThumb(_controllerState);
	ret.block<2, 1>(4, 0) = GetTriger(_controllerState);

	return ret;
}