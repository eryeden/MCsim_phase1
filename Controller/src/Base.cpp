#include <iostream>
#include "MC.hpp"
#include <cmath>

using namespace Eigen;
using namespace MC;

MC::Generator::Generator()
	:
	mob(0), mom(0),
	blocks(NULL), motorplops(NULL),
	vb0(Vector3d::Zero()), wb0(Vector3d::Zero()),
	xe0(Vector3d::Zero()), phie0(Vector3d::Zero()),
	dt(0.001)
{
	;
}

void MC::Generator::add(MC::Block *blk){
	blks.push_back(blk);
}

void MC::Generator::add(MC::MotorPlop *mp){
	mtrplps.push_back(mp);
	blks.push_back(mp);
}

void MC::Generator::set_initialstate_vb(Vector3d &tx){
	vb0 = tx;
}

void MC::Generator::set_initialstate_wb(Vector3d &tx){
	wb0 = tx;
}

void MC::Generator::set_initialstate_xe(Vector3d &tx){
	xe0 = tx;
}

void MC::Generator::set_initialstate_phie(Vector3d &tx){
	phie0 = tx;
}

void MC::Generator::set_dt(double _dt){
	dt = _dt;
}


MC::Core MC::Generator::generate_core(){
	if (mtrplps.empty() && blks.empty()){ //何もなければ何もないCoreを返す
		return Core();
	}
	//全体の慣性テンソル、重心等を求める
	//重心再計算
	double m_sum = 0; //全質量
	for (auto itr = blks.begin(); itr != blks.end(); ++itr){
		m_sum += (*itr)->mass;
	}

	Vector3d cog; //全体重心
	for (auto itr = blks.begin(); itr != blks.end(); ++itr){
		cog += (*itr)->c_o_g * (*itr)->mass;
	}
	cog /= m_sum;

	//r代入 構成時位置更新
	for (auto itr = blks.begin(); itr != blks.end(); ++itr){
		(*itr)->r = (*itr)->c_o_g - cog;
	}

	//全体の慣性テンソルを求める
	Matrix3d J_sum = Matrix3d::Zero();
	Matrix3d T = Matrix3d::Zero();
	T << 
		0, 1, 1,
		1, 0, 1,
		1, 1, 0;

	for (auto itr = blks.begin(); itr != blks.end(); ++itr){
		J_sum = (*itr)->J + T * (*itr)->r.dot(((*itr)->r));
	}

	return Core(J_sum, m_sum, dt, mtrplps, blks, vb0, wb0, xe0, phie0);
}




