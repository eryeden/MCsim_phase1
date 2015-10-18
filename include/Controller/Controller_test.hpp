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

		~Controller_test();


		void Initialize();
		void Update();

		void Setw(const Eigen::Vector3d & _w);
		Eigen::Matrix3d GetDCM();

	private:

		void Set_w_m_All(float _wm);

		//クォータニオンテスト うまくいった　微分値を普通に積分すれば回転しているクオータニオンを得られる
		Eigen::Vector3d w_bodyspace;
		Eigen::Vector4d q; //慣性座標系から機体座標系への変換を示すクォータニオン
		Eigen::Vector4d dq; //qの時間微分
		Eigen::Matrix3d DCM; //クォータニオンをDCMで示したもの 慣性座標系から機体座標系への変換を示す
								//よってこれをOpenGLに渡すには転置して逆行列とし、機体座標系から慣性座標系への変換行列にする必要がある

		double dt;

		Eigen::Matrix3d ConvertQtoDCM(const Eigen::Vector4d & _q); //クォータニオンからDCMへの変換
		Eigen::Matrix4d GetOmega(const Eigen::Vector3d & _w);	//クォータニオンとその時間微分をつなぐ行列Ω
		void Normalize(Eigen::Vector4d & _q);	  //クォータニオンの正規化



	};


};


#endif //_CONTROLLER_CONTROLLER_TEST_








