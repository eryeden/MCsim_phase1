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

static const unsigned char ID_NONE      = 0x00; //設定なし
static const unsigned char ID_CYLINDER  = 0x01; //円柱
static const unsigned char ID_CUBOID    = 0x02; //直方体
static const unsigned char ID_COMPONENT = 0x03; //STLファイルにより形状を指定


class Block{
public:
	Block();
	Block(double m);
	double mass;
	Eigen::Matrix3d J; //慣性テンソル
	Eigen::Vector3d c_o_g; //構成時重心座標
	Eigen::Vector3d r; //全体重心からの距離
	virtual void calc_m_o_i() = 0; //慣性テンソルの計算
	unsigned char id; //形状ID
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

//モデル描画にSTLファイルを使用する
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
	double w_m; //ローター角速度
	Eigen::Matrix3d Jr; //回転部慣性テンソル
	Eigen::Vector3d get_f(); //スラスト
	Eigen::Vector3d get_tau(); //トルク
	Eigen::Vector3d get_l(); //角運動量
};

//モーター描画にSTLファイルを使用する
class StLMotorPlop : public StLComponent{
public:
	StLMotorPlop(double &_c_t, double &_c_q, Eigen::Matrix3d &_Jr);
	StLMotorPlop(double &_c_t, double &_c_q, Eigen::Matrix3d &_Jr,
					const std::string &p2stl, const Eigen::Matrix3d &_J, 
					const Eigen::Vector3d &_c_o_g, const Eigen::Matrix3d &m_att);
	double c_t;
	double c_q;
	double w_m; //ローター角速度
	Eigen::Matrix3d Jr; //回転部慣性テンソル
	Eigen::Vector3d get_f(); //スラスト
	Eigen::Vector3d get_tau(); //トルク
	Eigen::Vector3d get_l(); //角運動量

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
	double dt, dt2, dt6;
	
	MotorPlop* motorplops; //モーター部配列先頭アドレス
	unsigned int n_o_m;      //モーター個数

	std::vector<MC::MotorPlop*> mtrplps; //モーター／プロペラモデル
	std::vector<MC::Block*> components; //機体構造材
	
	void update(); //1ステップ前進積分　RK4

	Vector12d get_state_vector(); //状態ベクトル
	Matrix12d get_state_matrix(); //状態行列

private:
	Matrix12d mk_Z(const Vector12d &tx);  //状態ベクトルによりZを生成
	Eigen::Vector3d mk_u11(const Vector12d &tx); //状態ベクトルによりU11を生成
	Eigen::Vector3d mk_u12(); //U12生成
	Eigen::Vector3d mk_u2(); //U2生成
	Vector12d mk_u(const Vector12d &tx); //U生成 U = t[U11 + U12, U2, 0, 0]
	Eigen::Matrix3d mk_sk(const Eigen::Vector3d &v); //外積マトリクス
	Eigen::Matrix3d mk_B_mat(const Vector12d &tx, const Eigen::Matrix3d &Jb); //Bマトリックスの生成
	Eigen::Matrix3d mk_E_mat(const Vector12d &tx); //回転行列の生成
	Eigen::Matrix3d mk_D_mat(const Vector12d &tx); //角速度用回転行例角生成
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

	MC::Block* blocks; //構造材配列　先頭アドレス
	MC::MotorPlop* motorplops; //モーター/プロペラ配列　先頭アドレス
	unsigned int mob; //構造材個数
	unsigned int mom; //モーター/プロペラ個数

	Eigen::Vector3d vb0, wb0, xe0, phie0;

	std::vector<MC::MotorPlop*> mtrplps;
	std::vector<MC::Block*> blks;

	double dt;

};





} // NAMESPACE MC

#endif // !_MC_








