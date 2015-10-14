#ifndef _MC_
#define _MC_

#include <Eigen/Dense>
#include <vector>
#include <string>
//using namespace Eigen;
#define MC_G 9.80665

#define USE_STL_VECTOR

namespace MC{ // NAMESPACE MC


using Matrix12d = Eigen::Matrix<double, 12, 12>;
using Vector12d = Eigen::Matrix<double, 12, 1>;

static const unsigned char ID_NONE      = 0x00; //�ݒ�Ȃ�
static const unsigned char ID_CYLINDER  = 0x01; //�~��
static const unsigned char ID_CUBOID    = 0x02; //������
static const unsigned char ID_COMPONENT = 0x03; //STL�t�@�C���ɂ��`����w��


class Block{
public:
	Block();
	Block(double m);
	double mass;
	Eigen::Matrix3d J; //�����e���\��
	Eigen::Vector3d c_o_g; //�\�����d�S���W
	Eigen::Vector3d r; //�S�̏d�S����̋���
	virtual void calc_m_o_i() = 0; //�����e���\���̌v�Z
	unsigned char id; //�`��ID
};


class Cylinder : public Block{
public:
	Cylinder(double m, double r, double h);
	double R;
	double h;
	void calc_m_o_i();
};


class Cuboid : public Block{
public:
	Cuboid(double m, double w, double h, double d);
	double w;
	double h;
	double d;
	void calc_m_o_i();
};

//���f���`���STL�t�@�C�����g�p����
class StLComponent : public Block{
public:
	StLComponent();
	StLComponent(const std::string &p2stl);
	StLComponent(const std::string &p2stl, const Eigen::Matrix3d &_J, const Eigen::Vector3d &_c_o_g);
	StLComponent(const std::string &p2stl, const Eigen::Matrix3d &_J, const Eigen::Vector3d &_c_o_g,
				const Eigen::Matrix3d &m_att);

	void set_attitude(Eigen::Matrix3d m_att);
	void set_path_to_stl(std::string p2stl);

private:
	Eigen::Matrix3d mat_attitude;
	std::string path_to_stl;
};


class MotorPlop : public Cylinder{
public:
	MotorPlop(double m, double R, double h, double c_t, double c_q);
	double c_t;
	double c_q;
	double w_m; //���[�^�[�p���x
	Eigen::Matrix3d Jr; //��]�������e���\��
	Eigen::Vector3d get_f(); //�X���X�g
	Eigen::Vector3d get_tau(); //�g���N
	Eigen::Vector3d get_l(); //�p�^����
};

//���[�^�[�`���STL�t�@�C�����g�p����
class StLMotorPlop : public StLComponent{
public:
	StLMotorPlop(double &_c_t, double &_c_q, Eigen::Matrix3d &_Jr);
	StLMotorPlop(double &_c_t, double &_c_q, Eigen::Matrix3d &_Jr,
					const std::string &p2stl, const Eigen::Matrix3d &_J, 
					const Eigen::Vector3d &_c_o_g, const Eigen::Matrix3d &m_att);
	double c_t;
	double c_q;
	double w_m; //���[�^�[�p���x
	Eigen::Matrix3d Jr; //��]�������e���\��
	Eigen::Vector3d get_f(); //�X���X�g
	Eigen::Vector3d get_tau(); //�g���N
	Eigen::Vector3d get_l(); //�p�^����

};

class Core{
public:

	Core();
	Core(
		const Eigen::Matrix3d &tj, const double &m, const double &dt,
		MotorPlop* mps, const unsigned int &nom,
		const Eigen::Vector3d &v_b0, const Eigen::Vector3d &w_b0,
		const Eigen::Vector3d &x_e0, const Eigen::Vector3d &phi_e0
		);

	Core(
		const Eigen::Matrix3d &tj, const double &m, const double &dt,
		const std::vector<MotorPlop*> &mplps, const std::vector<Block*> &blks,
		const Eigen::Vector3d &v_b0, const Eigen::Vector3d &w_b0,
		const Eigen::Vector3d &x_e0, const Eigen::Vector3d &phi_e0
		);

	Vector12d x;
	Vector12d x_prev;
	Vector12d u;
	double m;
	Eigen::Matrix3d J;
	Matrix12d Z;

	Vector12d k1, k2, k3, k4;
	double dt, dt2, dt6;			 //�p�x
	
	MotorPlop* motorplops; //���[�^�[���z��擪�A�h���X
	unsigned int n_o_m;      //���[�^�[��

	std::vector<MC::MotorPlop*> mtrplps; //���[�^�[�^�v���y�����f��
	std::vector<MC::Block*> components; //�@�̍\����
	
	void update(); //1�X�e�b�v�O�i�ϕ��@RK4

	Vector12d get_state_vector(); //��ԃx�N�g��
	Matrix12d get_state_matrix(); //��ԍs��

private:
	Matrix12d mk_Z(const Vector12d &tx);  //��ԃx�N�g���ɂ��Z�𐶐�
	Eigen::Vector3d mk_u11(const Vector12d &tx); //��ԃx�N�g���ɂ��U11�𐶐�
	Eigen::Vector3d mk_u12(); //U12����
	Eigen::Vector3d mk_u2(); //U2����
	Vector12d mk_u(const Vector12d &tx); //U���� U = t[U11 + U12, U2, 0, 0]
	Eigen::Matrix3d mk_sk(const Eigen::Vector3d &v); //�O�σ}�g���N�X
	Eigen::Matrix3d mk_B_mat(const Vector12d &tx, const Eigen::Matrix3d &Jb); //B�}�g���b�N�X�̐���
	Eigen::Matrix3d mk_E_mat(const Vector12d &tx); //��]�s��̐���
	Eigen::Matrix3d mk_D_mat(const Vector12d &tx); //�p���x�p��]�s��p����
};


class Generator{
public:
	Generator();

	void add(MC::Block *blk);
	void add(MC::MotorPlop *mp);

	void set_initialstate_vb(Eigen::Vector3d &tx);
	void set_initialstate_wb(Eigen::Vector3d &tx);
	void set_initialstate_xe(Eigen::Vector3d &tx);
	void set_initialstate_phie(Eigen::Vector3d &tx);

	void set_dt(double dt);
	
	
	MC::Core generate_core();

private:

	MC::Block* blocks; //�\���ޔz��@�擪�A�h���X
	MC::MotorPlop* motorplops; //���[�^�[/�v���y���z��@�擪�A�h���X
	unsigned int mob; //�\���ތ�
	unsigned int mom; //���[�^�[/�v���y����

	Eigen::Vector3d vb0, wb0, xe0, phie0;

	std::vector<MC::MotorPlop*> mtrplps;
	std::vector<MC::Block*> blks;

	double dt;

};


//����



} // NAMESPACE MC

#endif // !_MC_








