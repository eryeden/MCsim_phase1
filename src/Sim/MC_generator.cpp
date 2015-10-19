#include <iostream>
#include <Sim/MC.hpp>
#include <cmath>

using namespace Eigen;
using namespace MC;

MC::Generator::Generator()
	:
	vb0(Vector3d::Zero()), wb0(Vector3d::Zero()),
	xe0(Vector3d::Zero()), phie0(Vector3d::Zero()),
	dt(0.001)
{
	;
}

void MC::Generator::Add(MC::Block * _blk) {
	blks.push_back(_blk);


	switch (_blk->id_alternative_function)
	{
	case ID_AF_NONE:
		break;
	case ID_AF_MOTOR_PLOP:
		//�N���X�L���X�g�Ƃ����炵���@�_�E���L���X�g���A�b�v�L���X�g����
		mtrplps.push_back(static_cast<StLMotorPlop *>(_blk));
		break;
	default:
		break;
	}

}

void MC::Generator::operator<<(MC::Block * _block) {
	Add(_block);
}

void MC::Generator::set_initialstate_vb(const Vector3d &tx) {
	vb0 = tx;
}

void MC::Generator::set_initialstate_wb(const Vector3d &tx) {
	wb0 = tx;
}

void MC::Generator::set_initialstate_xe(const Vector3d &tx) {
	xe0 = tx;
}

void MC::Generator::set_initialstate_phie(const Vector3d &tx) {
	phie0 = tx;
}

void MC::Generator::SetDt(double _dt) {
	dt = _dt;
}

void MC::Generator::SetInitialVelocityBodyspace(const Eigen::Vector3d & _v) {
	vb0 = _v;
}
void MC::Generator::SetInitialAngularVelocityBodyspace(const Eigen::Vector3d & _w) {
	wb0 = _w;
}
void MC::Generator::SetInitialPositionEarthspace(const Eigen::Vector3d & _p) {
	xe0 = _p;
}
void MC::Generator::SetInitialQuotanion(const Eigen::Vector4d & _q) {
	q0 = _q;
}


MC::Core MC::Generator::generate_core() {
	if (mtrplps.empty() && blks.empty()) { //�����Ȃ���Ή����Ȃ�Core��Ԃ�
		return Core();
	}
	//�S�̂̊����e���\���A�d�S�������߂�
	//�d�S�Čv�Z
	double m_sum = 0; //�S����
	for (auto itr = blks.begin(); itr != blks.end(); ++itr) {
		m_sum += (*itr)->mass;
	}

	Vector3d cog; //�S�̏d�S
	for (auto itr = blks.begin(); itr != blks.end(); ++itr) {
		cog += (*itr)->c_o_g * (*itr)->mass;
	}
	cog /= m_sum;

	//r��� �\�����ʒu�X�V
	for (auto itr = blks.begin(); itr != blks.end(); ++itr) {
		(*itr)->r = (*itr)->c_o_g - cog;
	}

	//�S�̂̊����e���\�������߂�
	Matrix3d J_sum = Matrix3d::Zero();
	Matrix3d T = Matrix3d::Zero();
	T <<
		0, 1, 1,
		1, 0, 1,
		1, 1, 0;

	for (auto itr = blks.begin(); itr != blks.end(); ++itr) {
		J_sum = (*itr)->J + T * (*itr)->r.dot(((*itr)->r));
	}

	return Core(J_sum, m_sum, dt, mtrplps, blks, vb0, wb0, xe0, phie0);
}

MC::Core MC::Generator::GenerateCore_q() {
	if (mtrplps.empty() && blks.empty()) { //�����Ȃ���Ή����Ȃ�Core��Ԃ�
		return Core();
	}
	//�S�̂̊����e���\���A�d�S�������߂�
	//�d�S�Čv�Z
	double m_sum = 0; //�S����
	for (auto itr = blks.begin(); itr != blks.end(); ++itr) {
		m_sum += (*itr)->mass;
	}

	Vector3d cog = Vector3d::Zero(); //�S�̏d�S
	for (auto itr = blks.begin(); itr != blks.end(); ++itr) {
		cog += (*itr)->c_o_g * (*itr)->mass;
	}
	cog /= m_sum;

	//r��� �\�����ʒu�X�V
	for (auto itr = blks.begin(); itr != blks.end(); ++itr) {
		(*itr)->r = (*itr)->c_o_g - cog;
	}

	//�S�̂̊����e���\�������߂�
	Matrix3d J_sum = Matrix3d::Zero();
	Matrix3d T = Matrix3d::Zero();
	T <<
		0, 1, 1,
		1, 0, 1,
		1, 1, 0;
	Matrix3d addm = Matrix3d::Zero();
	Vector3d mpr;
	Vector3d r2;

	for (auto itr = blks.begin(); itr != blks.end(); ++itr) {
		r2 = Vector3d(
			(*itr)->r(0) * (*itr)->r(0)
			, (*itr)->r(1) * (*itr)->r(1)
			, (*itr)->r(2) * (*itr)->r(2)
			);
		mpr = (*itr)->mass * T * r2;
		addm(0, 0) = mpr(0);
		addm(1, 1) = mpr(1);
		addm(2, 2) = mpr(2);

		//J_sum = (*itr)->J + T * (*itr)->r.dot(((*itr)->r));
		J_sum = (*itr)->J + addm;
	}

	return Core(J_sum, m_sum, dt, mtrplps, blks, vb0, wb0, xe0, q0);
}




