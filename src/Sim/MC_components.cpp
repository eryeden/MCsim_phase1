#include <iostream>
#include <Sim/MC.hpp>
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

MC::Block::Block(double _mass)
	:
	mass(_mass),
	J(Matrix3d::Zero()),
	c_o_g(Vector3d::Zero()),
	r(Vector3d::Zero())
{
	;
}

MC::Block::Block(
	const double & _mass
	, const Eigen::Matrix3d & _J
	, const Eigen::Vector3d & _c_o_g
	, const Eigen::Vector3d & _r)
	:mass(_mass)
	, J(_J)
	, c_o_g(_c_o_g)
	, r(_r)
	, id(ID_NONE)
{
	;
}


MC::Block::Block(
	const double & _mass
	, const Eigen::Matrix3d & _J
	, const Eigen::Vector3d & _c_o_g
	, const Eigen::Vector3d & _r
	, const unsigned char & _id)
	:mass(_mass)
	, J(_J)
	, c_o_g(_c_o_g)
	, r(_r)
	, id(_id)
{
	;
}


MC::Cylinder::Cylinder(const double &_mass, const double & _r, const double &_h)
	:
	Block(_mass),
	R(_r), h(_h)
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

MC::Cuboid::Cuboid(const double &_mass, const double &_w, const double &_h, const double &_d)
	:
	Block(_mass),
	w(_w), h(_h), d(_d)
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

MC::MotorPlop::MotorPlop(double tc_t, double tc_q)
	:c_t(tc_t)
	, c_q(tc_q)
	, w_m(0)
	, Jr(Matrix3d::Zero())
{
	;
}

MC::MotorPlop::MotorPlop(
	const double & _c_t, const double & _c_q
	, const double & _w_m, const Eigen::Matrix3d & _Jr)
	:c_t(_c_t)
	, c_q(_c_q)
	, w_m(_w_m)
	, Jr(_Jr)
{
	;
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
	:MotorPlop(_c_t, _c_q, 0, _Jr)
	,StLComponent()
{
	;
}

MC::StLMotorPlop::StLMotorPlop(double &_c_t, double &_c_q, Matrix3d &_Jr,
								const std::string &p2stl, const Eigen::Matrix3d &_J,
								const Eigen::Vector3d &_c_o_g, const Eigen::Matrix3d &m_att)
	: MotorPlop(_c_t, _c_q, 0, _Jr)
	, StLComponent(p2stl, _J, _c_o_g, m_att)
{
	;
}







	 