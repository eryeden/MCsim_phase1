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

		//�N�H�[�^�j�I���e�X�g ���܂��������@�����l�𕁒ʂɐϕ�����Ή�]���Ă���N�I�[�^�j�I���𓾂���
		Eigen::Vector3d w_bodyspace;
		Eigen::Vector4d q; //�������W�n����@�̍��W�n�ւ̕ϊ��������N�H�[�^�j�I��
		Eigen::Vector4d dq; //q�̎��Ԕ���
		Eigen::Matrix3d DCM; //�N�H�[�^�j�I����DCM�Ŏ��������� �������W�n����@�̍��W�n�ւ̕ϊ�������
								//����Ă����OpenGL�ɓn���ɂ͓]�u���ċt�s��Ƃ��A�@�̍��W�n���犵�����W�n�ւ̕ϊ��s��ɂ���K�v������

		double dt;

		Eigen::Matrix3d ConvertQtoDCM(const Eigen::Vector4d & _q); //�N�H�[�^�j�I������DCM�ւ̕ϊ�
		Eigen::Matrix4d GetOmega(const Eigen::Vector3d & _w);	//�N�H�[�^�j�I���Ƃ��̎��Ԕ������Ȃ��s��
		void Normalize(Eigen::Vector4d & _q);	  //�N�H�[�^�j�I���̐��K��



	};


};


#endif //_CONTROLLER_CONTROLLER_TEST_








