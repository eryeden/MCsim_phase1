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
	Eigen::Vector4d thumb;
	const double th = 0.17;
	thumb = Eigen::Vector4d(
		(double)(_xin.Gamepad.sThumbLX) / 65535.0 * 2.0 //-0.5
		, (double)(_xin.Gamepad.sThumbLY) / 65535.0 * 2.0//	-0.5
		, (double)(_xin.Gamepad.sThumbRX) / 65535.0	* 2.0//-0.5
		, (double)(_xin.Gamepad.sThumbRY) / 65535.0	* 2.0//-0.5
		);

	for (int i = 0; i < 4; ++i) {
		if (thumb(i) > 0.0) {
			thumb(i) = (abs(thumb(i)) < th) ? 0.0 : thumb(i) - th;
		}
		else {
			thumb(i) = (abs(thumb(i)) < th) ? 0.0 : thumb(i) + th;
		}
	}
	
	return thumb;
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
