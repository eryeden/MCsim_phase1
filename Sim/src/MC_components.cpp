#include <iostream>
#include "MC.hpp"
#include <cmath>

using namespace Eigen;

MC::Block::Block()
	:
	mass(0),
	J(Matrix3d::Zero()),
	c_o_g(Vector3d::Zero()),
	r(Vector3d::Zero()),
	id(0)
{
	;
}

MC::Block::Block(double m)
	:
	mass(m),
	J(Matrix3d::Zero()),
	c_o_g(Vector3d::Zero()),
	r(Vector3d::Zero())
{
	;
}

MC::Cylinder::Cylinder(double m, double tr, double th)
	:
	Block(m),
	R(tr), h(th)
{
	id = MC::ID_CYLINDER;
	calc_m_o_i();
}

void MC::Cylinder::calc_m_o_i(){
	J(0, 0) = R * R / 4.0 + h * h / 12.0;
	J(1, 1) = R * R / 4.0 + h * h / 12.0;
	J(2, 2) = R * R / 2.0;

	J *= mass;
}

MC::Cuboid::Cuboid(double m, double tw, double th, double td)
	:
	Block(m),
	w(tw), h(th), d(td)
{
	id = MC::ID_CUBOID;
	calc_m_o_i();
}

void MC::Cuboid::calc_m_o_i(){
	J(0, 0) = h * h + d * d;
	J(1, 1) = d * d + w * w;
	J(2, 2) = w * w + h * h;

	J *= mass / 12.0;
}


MC::StLComponent::StLComponent()
	:
	mat_attitude(Matrix3d::Zero()),
	path_to_stl("")
{
	;
}

MC::StLComponent::StLComponent(const std::string &p2stl)
	:
	mat_attitude(Matrix3d::Zero()),
	path_to_stl(p2stl)
{
	;
}

MC::StLComponent::StLComponent(const std::string &p2stl, const Eigen::Matrix3d &_J,
							const Eigen::Vector3d &_c_o_g)
	:
	mat_attitude(Matrix3d::Zero()),
	path_to_stl(p2stl)
{
	J = _J;
	c_o_g = _c_o_g;
}

MC::StLComponent::StLComponent(const std::string &p2stl, const Eigen::Matrix3d &_J,
							const Eigen::Vector3d &_c_o_g, const Eigen::Matrix3d &m_att)
	:
	mat_attitude(m_att),
	path_to_stl(p2stl)
{
	J = _J;
	c_o_g = _c_o_g;
}

void MC::StLComponent::set_attitude(Matrix3d m_att){
	mat_attitude = m_att;
}

void MC::StLComponent::set_path_to_stl(std::string p2stl){
	path_to_stl = p2stl;
}

MC::MotorPlop::MotorPlop(double m, double tR, double th, double tc_t, double tc_q)
	:Cylinder(m , tR, th),
	c_t(tc_t), c_q(tc_q), w_m(0), Jr(Matrix3d::Zero())
{
	calc_m_o_i();
}

Vector3d MC::MotorPlop::get_f(){
	return Vector3d(0, 0, c_t * w_m * w_m);
}
Vector3d MC::MotorPlop::get_tau(){
	return Vector3d(0, 0, -c_q * w_m * w_m);
}
Vector3d MC::MotorPlop::get_l(){
	return (Jr * Vector3d(0, 0, w_m));
}

MC::StLMotorPlop::StLMotorPlop(double &_c_t, double &_c_q, Matrix3d &_Jr)
	:c_t(_c_t), c_q(_c_q), w_m(0), Jr(_Jr),
	StLComponent()
{
	;
}

MC::StLMotorPlop::StLMotorPlop(double &_c_t, double &_c_q, Matrix3d &_Jr,
								const std::string &p2stl, const Eigen::Matrix3d &_J,
								const Eigen::Vector3d &_c_o_g, const Eigen::Matrix3d &m_att)
	:c_t(_c_t), c_q(_c_q), w_m(0), Jr(_Jr),
	StLComponent(p2stl, _J, _c_o_g, m_att)
{
	;
}

Vector3d MC::StLMotorPlop::get_f(){
	return Vector3d(0, 0, c_t * w_m * w_m);
}
Vector3d MC::StLMotorPlop::get_tau(){
	return Vector3d(0, 0, -c_q * w_m * w_m);
}
Vector3d MC::StLMotorPlop::get_l(){
	return (Jr * Vector3d(0, 0, w_m));
}








