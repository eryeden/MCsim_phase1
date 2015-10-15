#ifndef _SIM_MC_
#define _SIM_MC_

#include <Eigen/Dense>
#include <vector>
#include <string>
//using namespace Eigen;
#define MC_G 9.80665

#define USE_STL_VECTOR

namespace MC { // NAMESPACE MC


	using Matrix12d = Eigen::Matrix<double, 12, 12>;
	using Vector12d = Eigen::Matrix<double, 12, 1>;

	static const unsigned char ID_NONE = 0x00; //�ݒ�Ȃ�
	static const unsigned char ID_CYLINDER = 0x01; //�~��
	static const unsigned char ID_CUBOID = 0x02; //������
	static const unsigned char ID_COMPONENT = 0x03; //STL�t�@�C���ɂ��`����w��

	//����@�\�t���R���|�[�l���g
	static const unsigned char ID_AF_NONE = 0x00;
	static const unsigned char ID_AF_MOTOR_PLOP = 0x01;


	class Block {
	public:
		Block();
		Block(double _mass);
		Block(const double & _mass
			, const Eigen::Matrix3d & _J
			, const Eigen::Vector3d & _c_o_g);
		Block(const double & _mass
			, const Eigen::Matrix3d & _J
			, const Eigen::Vector3d & _c_o_g
			, const unsigned char & _id);
		double mass;
		Eigen::Matrix3d J; //�����e���\��
		Eigen::Vector3d c_o_g; //�\�����d�S���W 
							   //�A�Z���u�����̌��_���g�p�����{�p�[�c�̏d�S�ʒu
		Eigen::Vector3d r; //�S�̏d�S����̋����@	//�����Generate���ɐݒ肳���̂ŏ��������ɒl��ݒ肷��K�v���Ȃ�
						//���f���i���ׂẴp�[�c�����킹�����́j�d�S�����_�Ƃ����Ƃ��̖{�p�[�c�̏d�S�ʒu
		//virtual void calc_m_o_i() = 0; //�����e���\���̌v�Z
		unsigned char id; //�`��ID
		unsigned char id_alternative_function; //����@�\ID
	};


	class Cylinder : public Block {
	public:
		Cylinder(const double &_mass, const double & _r, const double &_h);
		double R;
		double h;
		void calc_m_o_i();
	};


	class Cuboid : public Block {
	public:
		Cuboid(const double &_mass, const double &_w, const double &_h, const double &_d);
		double w;
		double h;
		double d;
		void calc_m_o_i();
	};

	//���f���`���STL�t�@�C�����g�p����
	class StLComponent : public Block {
	public:
		StLComponent();
		StLComponent(const std::string &p2stl);
		StLComponent(
			const std::string &p2stl
			, const Eigen::Matrix3d &_J
			, const Eigen::Vector3d &_c_o_g);
		StLComponent(
			const std::string &p2stl
			, const Eigen::Matrix3d &_J
			, const Eigen::Vector3d &_c_o_g
			, const Eigen::Matrix3d &m_att
			);
		//�A�Z���u���f�[�^����ǂޏꍇ�A���̕����炭
		StLComponent(
			const std::string & _path_to_stl
			, const double & _mass
			, const Eigen::Matrix3d & _J
			, const Eigen::Matrix3d & _m_attitude
			, const Eigen::Vector3d & _position_c_o_g_blockspace
			, const Eigen::Vector3d & _position_modelspace
			);

		void set_attitude(Eigen::Matrix3d _m_att);
		void set_path_to_stl(std::string _path_to_stl);

		const std::string & GetPathToModelFile();
		const Eigen::Vector3d & GetPositionModelspace();
		const Eigen::Matrix3d & GetAttitude();

	private:

		Eigen::Vector3d	position_c_o_g_blockspace;
		//�{�u���b�N���W�ɂ�����d�S�ʒu
		Eigen::Vector3d position_modelspace;
		//�A�Z���u����Ԃɂ�����{�u���b�N���W�n�̌��_�ʒu
		Eigen::Matrix3d mat_attitude;
		//�RD���f���t�@�C���ւ̃p�X
		std::string path_to_stl;
	};


	class MotorPlop {
	public:
		MotorPlop(double c_t, double c_q);
		MotorPlop(
			const double & _c_t, const double & _c_q
			, const double & _w_m, const Eigen::Matrix3d & _Jr
			);

		//���f���̏d�S�ʒu�����_�Ƃ������f����Ԃɂ�����R���|�[�l���g�̏d�S�ʒu
		virtual Eigen::Vector3d GetCOGPositionCOGModelspace() = 0;

		double c_t;
		double c_q;
		double w_m; //���[�^�[�p���x
		Eigen::Matrix3d Jr; //��]�������e���\��
		Eigen::Vector3d get_f(); //�X���X�g
		Eigen::Vector3d get_tau(); //�g���N
		Eigen::Vector3d get_l(); //�p�^����
	};

	//�V�����_�[�^�̃��[�^�[/�v���y��
	class CylinderMotorPlop
		: public Cylinder
		, public MotorPlop
	{
		//���f���̏d�S�ʒu�����_�Ƃ������f����Ԃɂ�����R���|�[�l���g�̏d�S�ʒu
		Eigen::Vector3d GetCOGPositionCOGModelspace();
	};

	//���[�^�[�`���STL�t�@�C�����g�p����
	class StLMotorPlop
		: public StLComponent
		, public MotorPlop
	{
	public:
		StLMotorPlop(double &_c_t, double &_c_q, Eigen::Matrix3d &_Jr);
		StLMotorPlop(
			double &_c_t, double &_c_q, Eigen::Matrix3d &_Jr
			, const std::string &p2stl, const Eigen::Matrix3d &_J
			, const Eigen::Vector3d &_c_o_g, const Eigen::Matrix3d &m_att);
		StLMotorPlop(
			const double &_c_t
			, const double &_c_q
			, const Eigen::Matrix3d &_Jr
			, const std::string & _path_to_stl
			, const double & _mass
			, const Eigen::Matrix3d & _J
			, const Eigen::Matrix3d & _m_attitude
			, const Eigen::Vector3d & _position_c_o_g_blockspace
			, const Eigen::Vector3d & _position_modelspace
			);
		//���f���̏d�S�ʒu�����_�Ƃ������f����Ԃɂ�����R���|�[�l���g�̏d�S�ʒu
		Eigen::Vector3d GetCOGPositionCOGModelspace();


	};

	class Core {
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


	class Generator {
	public:
		Generator();

		void set_initialstate_vb(Eigen::Vector3d &tx);
		void set_initialstate_wb(Eigen::Vector3d &tx);
		void set_initialstate_xe(Eigen::Vector3d &tx);
		void set_initialstate_phie(Eigen::Vector3d &tx);

		void SetDt(double dt);


		MC::Core generate_core();

		void Add(MC::Block * _blk);
		void operator<<(MC::Block * _block);

	private:

		Eigen::Vector3d vb0, wb0, xe0, phie0;

		std::vector<MC::MotorPlop*> mtrplps;
		std::vector<MC::Block*> blks;

		std::vector<MC::StLComponent *> components_model;
		std::vector<MC::MotorPlop *> motor_plops;

		double dt;
	};





} // NAMESPACE MC

#endif // !_SIM_MC_








