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

	static const unsigned char ID_NONE = 0x00; //設定なし
	static const unsigned char ID_CYLINDER = 0x01; //円柱
	static const unsigned char ID_CUBOID = 0x02; //直方体
	static const unsigned char ID_COMPONENT = 0x03; //STLファイルにより形状を指定

	//特殊機能付きコンポーネント
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
		Eigen::Matrix3d J; //慣性テンソル
		Eigen::Vector3d c_o_g; //構成時重心座標 
							   //アセンブリ時の原点を使用した本パーツの重心位置
		Eigen::Vector3d r; //全体重心からの距離　	//これはGenerate時に設定されるので初期化時に値を設定する必要がない
						//モデル（すべてのパーツをあわせたもの）重心を原点としたときの本パーツの重心位置
		//virtual void calc_m_o_i() = 0; //慣性テンソルの計算
		unsigned char id; //形状ID
		unsigned char id_alternative_function; //特殊機能ID
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

	//モデル描画にSTLファイルを使用する
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
		//アセンブリデータから読む場合、この方がらく
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
		//本ブロック座標における重心位置
		Eigen::Vector3d position_modelspace;
		//アセンブリ空間における本ブロック座標系の原点位置
		Eigen::Matrix3d mat_attitude;
		//３Dモデルファイルへのパス
		std::string path_to_stl;
	};


	class MotorPlop {
	public:
		MotorPlop(double c_t, double c_q);
		MotorPlop(
			const double & _c_t, const double & _c_q
			, const double & _w_m, const Eigen::Matrix3d & _Jr
			);

		//モデルの重心位置を原点としたモデル空間におけるコンポーネントの重心位置
		virtual Eigen::Vector3d GetCOGPositionCOGModelspace() = 0;

		double c_t;
		double c_q;
		double w_m; //ローター角速度
		Eigen::Matrix3d Jr; //回転部慣性テンソル
		Eigen::Vector3d get_f(); //スラスト
		Eigen::Vector3d get_tau(); //トルク
		Eigen::Vector3d get_l(); //角運動量
	};

	//シリンダー型のモーター/プロペラ
	class CylinderMotorPlop
		: public Cylinder
		, public MotorPlop
	{
		//モデルの重心位置を原点としたモデル空間におけるコンポーネントの重心位置
		Eigen::Vector3d GetCOGPositionCOGModelspace();
	};

	//モーター描画にSTLファイルを使用する
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
		//モデルの重心位置を原点としたモデル空間におけるコンポーネントの重心位置
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
		double dt, dt2, dt6;			 //頻度

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








