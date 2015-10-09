#include <iostream>
#include "MC.hpp"
#include <cmath>

using namespace Eigen;
using namespace MC;


Core::Core()
	: 
		x(Vector12d::Zero()),
		x_prev(Vector12d::Zero()),
		u(Vector12d::Zero()),
		m(0),
		J(Matrix3d::Zero()),
		Z(Matrix12d::Zero()),
		k1(Vector12d::Zero()),
		k2(Vector12d::Zero()),
		k3(Vector12d::Zero()),
		k4(Vector12d::Zero()),
		dt(0),
		dt2(0),
		dt6(0),
		motorplops(NULL),
		n_o_m(0)
{
	;
}

Core::Core(
	const Eigen::Matrix3d &tj, const double &tm, const double &tdt,
	MotorPlop* mps, const unsigned int &nom,
	const Eigen::Vector3d &v_b0, const Eigen::Vector3d &w_b0,
	const Eigen::Vector3d &x_e0, const Eigen::Vector3d &phi_e0
	)
	:
		x(Vector12d::Zero()),
		x_prev(Vector12d::Zero()),
		u(Vector12d::Zero()),
		m(0),
		J(Matrix3d::Zero()),
		Z(Matrix12d::Zero()),
		k1(Vector12d::Zero()),
		k2(Vector12d::Zero()),
		k3(Vector12d::Zero()),
		k4(Vector12d::Zero()),
		dt(0),
		dt2(0),
		dt6(0),
		motorplops(NULL),
		n_o_m(0)
{
	J = tj;
	m = tm;
	dt = tdt;
	dt2 = dt * 0.5;
	dt6 = dt / 0.6;

	motorplops = mps;
	n_o_m = nom;

	x.block<3, 1>(0, 0) = v_b0;
	x.block<3, 1>(3, 0) = w_b0;
	x.block<3, 1>(6, 0) = x_e0;
	x.block<3, 1>(9, 0) = phi_e0;
}

Core::Core(
	const Eigen::Matrix3d &tj, const double &tm, const double &tdt,
	const std::vector<MotorPlop*> &mplps, const std::vector<MC::Block*> &blks,
	const Eigen::Vector3d &v_b0, const Eigen::Vector3d &w_b0,
	const Eigen::Vector3d &x_e0, const Eigen::Vector3d &phi_e0
	)
	:
	x(Vector12d::Zero()),
	x_prev(Vector12d::Zero()),
	u(Vector12d::Zero()),
	m(0),
	J(Matrix3d::Zero()),
	Z(Matrix12d::Zero()),
	k1(Vector12d::Zero()),
	k2(Vector12d::Zero()),
	k3(Vector12d::Zero()),
	k4(Vector12d::Zero()),
	dt(0),
	dt2(0),
	dt6(0),
	mtrplps(mplps),
	components(blks)
{
	J = tj;
	m = tm;
	dt = tdt;
	dt2 = dt * 0.5;
	dt6 = dt / 6.0;

	x.block<3, 1>(0, 0) = v_b0;
	x.block<3, 1>(3, 0) = w_b0;
	x.block<3, 1>(6, 0) = x_e0;
	x.block<3, 1>(9, 0) = phi_e0;
}


Matrix12d Core::mk_Z(const Vector12d & tx){
	Matrix12d tZ = MatrixXd::Zero(12, 12);
	tZ.block<3, 3>(0, 0) = mk_sk(tx.block<3, 1>(3, 0));
	tZ.block<3, 3>(3, 3) = mk_B_mat(tx, J);
	tZ.block<3, 3>(6, 0) = mk_E_mat(tx);
	tZ.block<3, 3>(9, 3) = mk_D_mat(tx);
	return tZ;
}

Vector3d Core::mk_u11(const Vector12d &tx){
	double phi, theta, psi; //φ、θ、ψ
	phi = tx(9);
	theta = tx(10);
	psi = tx(11);
	Vector3d u11;
	u11 <<
		MC_G * sin(theta),
		-MC_G * sin(phi) * cos(theta),
		-MC_G * cos(phi) * cos(theta);

	return u11;
}
//機構構成
Vector3d Core::mk_u12(){
	Vector3d u12 = Vector3d::Zero();
#ifndef USE_STL_VECTOR
	for (int i = 0; i < n_o_m; i++){
		u12 += motorplops[i].get_f();
	}
#else
	for (auto itr = mtrplps.begin(); itr != mtrplps.end(); ++itr){
		u12 += (*itr)->get_f();
	}
#endif
	u12 /= m;
	return u12;
}

Vector3d Core::mk_u2(){
	Matrix3d Jb_inv;
	Jb_inv <<
		1.0 / J(0, 0), 0, 0,
		0, 1.0 / J(1, 1), 0,
		0, 0, 1.0 / J(2, 2);

	Vector3d u2 = Vector3d::Zero();

#ifndef USE_STL_VECTOR
	for (int i = 0; i < n_o_m; i++){
		u2 += (motorplops[i].r.cross(motorplops[i].get_f())) + motorplops[i].get_tau();
	}
#else
	for (auto itr = mtrplps.begin(); itr != mtrplps.end(); ++itr){
		u2 +=
			((*itr)->r.cross((*itr)->get_f())) + (*itr)->get_tau();
	}
#endif
	return Jb_inv * u2;
}

Vector12d Core::mk_u(const Vector12d &tx){
	Vector12d u = Vector12d::Zero();
	u.block<3, 1>(0, 0) = mk_u11(tx) + mk_u12();
	u.block<3, 1>(3, 0) = mk_u2();
	return u;
}

Matrix3d Core::mk_sk(const Vector3d &v){
	Matrix3d cp;
	cp <<    0,  v(2), -v(1),
		 -v(2),     0,  v(0),
		  v(1), -v(0),     0;
		 
	return cp;
}

Matrix3d Core::mk_B_mat(const Vector12d &tx, const Matrix3d &Jb){
	Matrix3d Jb_inv;
	Jb_inv << 1.0/Jb(0, 0), 0, 0,
			  0, 1.0/Jb(1, 1), 0,
			  0, 0, 1.0/Jb(2, 2);

	Matrix3d sk_L_sum = Matrix3d::Zero();
#ifndef USE_STL_VECTOR
	for (int i = 0; i < n_o_m; i++){
		sk_L_sum += mk_sk(motorplops[i].get_l());
	}
#else
	for (auto itr = mtrplps.begin(); itr != mtrplps.end(); ++itr){
		sk_L_sum += mk_sk((*itr)->get_l());
	}
#endif
	return Jb_inv * (mk_sk(tx.block<3, 1>(3, 0)) * Jb + sk_L_sum);
}

Matrix3d Core::mk_E_mat(const Vector12d &tx){
	double phi, theta, psi; //φ、θ、ψ
	phi = tx(9);
	theta = tx(10);
	psi = tx(11);

	return 
		  (AngleAxisd(-psi,   Vector3d(0, 0, 1)).matrix())
		* (AngleAxisd(-theta, Vector3d(0, 1, 0)).matrix())
		* (AngleAxisd(-phi,   Vector3d(1, 0, 0)).matrix());
}

Matrix3d Core::mk_D_mat(const Vector12d &tx){
	double phi, theta, psi; //φ、θ、ψ
	phi = tx(9);
	theta = tx(10);
	psi = tx(11);
	Matrix3d D;
	D << 
		1, sin(phi) * tan(theta), cos(phi) * tan(theta),
		0, cos(phi), -sin(phi),
		0, sin(phi) / cos(theta), cos(phi) / cos(theta);
		
	return D;
}


//KR4を用いて1ステップ進める
void Core::update(){
	Vector12d tx = Vector12d::Zero();

	k1 = mk_Z(x) * x + mk_u(x);
	tx = x + dt2 * k1;

	k2 = mk_Z(tx) * tx + mk_u(tx);
	tx = x + dt2 * k2;

	k3 = mk_Z(tx) * tx + mk_u(tx);
	tx = x + dt * k3;

	Z = mk_Z(tx);
	k4 = Z * tx + mk_u(tx);

	x_prev = x;
	x = x + dt6 * (k1 + 2.0 * (k2 + k3) + k4);
}



Vector12d Core::get_state_vector(){
	return x;
}

Matrix12d Core::get_state_matrix(){
	return Z;
}

















