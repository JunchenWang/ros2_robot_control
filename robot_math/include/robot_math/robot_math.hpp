#ifndef ROBOT_MATH__ROBOT_MATH_HPP_
#define ROBOT_MATH__ROBOT_MATH_HPP_

#include "robot_math/visibility_control.h"
#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>
namespace Eigen
{
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
	typedef Eigen::Vector<double, 6> Vector6d;
	typedef Eigen::Matrix<double, 6, 7> Matrix6x7d;
	typedef Eigen::Matrix<double, 7, 6> Matrix7x6d;
	typedef Eigen::Matrix<double, 7, 7> Matrix7d;
	typedef Eigen::Vector<double, 7> Vector7d;
}

namespace robot_math
{
	struct Robot
	{
		int dof;
		std::vector<double> mass; // link mass
		std::vector<std::array<double, 9>> inertia; // 3x3 inertia matrix
		std::vector<std::array<double, 6>> A; // screw axis
		std::vector<std::array<double, 16>> M; // 4x4 tform 
		std::vector<std::array<double, 3>> com; // center of mass
		double ME[16]; // ME is read from urdf
		double TCP[16]; // TCP can be freely set to increase flexibility, default is identity
		double gravity[3]; // [0, 0, -9.8] if the robot is vertically installed 
	};
	    void rcm_Jacobian(const Robot *robot, const std::vector<double> &q, const std::vector<double> &dq_array,
                      const Eigen::Vector3d &p1_F, const Eigen::Vector3d &p2_F, const Eigen::Vector3d &rcm,
                      const Eigen::MatrixXd &Jb, const Eigen::MatrixXd &dJb, const Eigen::Matrix4d &T, const Eigen::Matrix4d &dT,
                      Eigen::MatrixXd &J, Eigen::MatrixXd &dJ, Eigen::Vector3d &x);
	Robot urdf_to_robot(const std::string &description, std::vector<std::string> &joint_names, std::string &link_name, std::string &base_link);
	void print_robot(const Robot &robot);
    // pose: first three are position
	Eigen::Matrix4d pose_to_tform(const std::vector<double> &pose);
	std::vector<double> tform_to_pose(const Eigen::Matrix4d &T);
	std::vector<double> quaternion_pose_to_rv_pose(const std::vector<double> &q_pose);
	std::vector<double> rv_pose_to_quaternion_pose(const std::vector<double> &rv_pose);
	std::vector<double> tform_to_quaternion_pose(const Eigen::Matrix4d &T);// w, x, y, z
	Eigen::Matrix4d quaternion_pose_to_tform(const std::vector<double> &pose);
	Eigen::Matrix3d so_w(const Eigen::Vector3d &w);
	Eigen::Matrix4d se_twist(const Eigen::Vector6d &V);
	Eigen::Vector7d normalize_twist(const Eigen::Vector6d &V);
	Eigen::Matrix3d exp_r(const Eigen::Vector3d &r);
	Eigen::Vector3d logR(const Eigen::Matrix3d &R);
	Eigen::Matrix4d exp_twist(const Eigen::Vector6d &twist);
	Eigen::Vector6d logT(const Eigen::Matrix4d &T);
	Eigen::Matrix4d inv_tform(const Eigen::Matrix4d &T);
	Eigen::Matrix6d adjoint_T(const Eigen::Matrix4d &T);
	Eigen::Matrix6d adjoint_V(const Eigen::Vector6d &V);
	Eigen::Matrix4d make_tform(const Eigen::Matrix3d &R, const Eigen::Vector3d &t);
	Eigen::MatrixXd pinv(const Eigen::MatrixXd &matrix, double tol = 1e-9);
	Eigen::Vector3d cond_matrix(const Eigen::MatrixXd &A);

	void derivative_tform_inv(const Eigen::Matrix4d &T, const Eigen::Matrix4d &dT, Eigen::Matrix4d & dinvT, Eigen::Matrix4d & invT);
	void derivative_adjoint_T(const Eigen::Matrix4d &T, const Eigen::Matrix4d &dT, Eigen::Matrix6d &dAdT, Eigen::Matrix6d &AdT);

    // damping least squares Ax = b with constraint on ||x||
	Eigen::MatrixXd damping_least_square(const Eigen::MatrixXd &A, const Eigen::MatrixXd &b, double con_threshold = 1e6, double lambda = 0.1);

	void jacobian_matrix(const Robot *robot, const std::vector<double> &q, Eigen::MatrixXd &J, Eigen::Matrix4d &T);
	void jacobian_matrix_all(const Robot *robot, const std::vector<double> &q, std::shared_ptr<double[]> &J);
	//void inverse_kin_general(const Robot *robot, Eigen::Matrix4d Td, const std::vector<double> &qref, const double tol[2], std::vector<double> &q, double *flag);
	void forward_kinematics(const Robot *robot, const std::vector<double> &q, Eigen::Matrix4d &T);

	Eigen::MatrixXd J_sharp(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M); // X x 6
	Eigen::MatrixXd d_J_sharp(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dJ, const Eigen::MatrixXd &dM);
	Eigen::MatrixXd d_J_sharp_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dJ, const Eigen::MatrixXd &dM, const Eigen::MatrixXd &X);
	Eigen::MatrixXd J_sharp_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &X, double c = 1000, double lambda = 5e-2); // X x
	Eigen::MatrixXd J_sharp_T_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &X, double c = 1000, double lambda = 5e-2);
	Eigen::MatrixXd A_x(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M);
	Eigen::MatrixXd A_x_inv(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M);
	Eigen::MatrixXd A_x_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &X);
	Eigen::VectorXd null_proj(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::VectorXd &v, double c = 1000, double lambda = 0.1);
	Eigen::MatrixXd null_z(const Eigen::MatrixXd &J); // n x r
	Eigen::MatrixXd d_null_z(const Eigen::MatrixXd &J, const Eigen::MatrixXd &dJ);
	Eigen::MatrixXd z_sharp(const Eigen::MatrixXd &Z, const Eigen::MatrixXd &M); // r x n
	Eigen::MatrixXd d_z_sharp(const Eigen::MatrixXd &Z, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dZ, const Eigen::MatrixXd &dM);
	Eigen::MatrixXd Mu_x_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dJ, const Eigen::MatrixXd &C, const Eigen::MatrixXd &X);
	Eigen::MatrixXd Mu_x(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dJ, const Eigen::MatrixXd &C);
	Eigen::MatrixXd A_v(const Eigen::MatrixXd &Z, const Eigen::MatrixXd &M);
	Eigen::MatrixXd Mu_v(const Eigen::MatrixXd &Z, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dZ, const Eigen::MatrixXd &dM, const Eigen::MatrixXd &C);
	Eigen::MatrixXd Mu_xv(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dJ, const Eigen::MatrixXd &Z, const Eigen::MatrixXd &C);
	Eigen::MatrixXd Mu_vx(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dZ, const Eigen::MatrixXd &dM, const Eigen::MatrixXd &Z, const Eigen::MatrixXd &C);
	Eigen::MatrixXd Mu_vx_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &Z, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dZ, const Eigen::MatrixXd &dM, const Eigen::MatrixXd &C, const Eigen::MatrixXd &X);

	void cal_motion_error(const Eigen::Matrix4d &T, const Eigen::Matrix4d &T_d,
						  const Eigen::Vector3d &v, const Eigen::Vector3d &w,
						  const Eigen::Vector3d &v_d, const Eigen::Vector3d &w_d,
						  const Eigen::Vector3d &a_d, const Eigen::Vector3d &alpha_d,
						  Eigen::Vector6d &xe, Eigen::Vector6d &dxe, Eigen::Vector6d &ddx_d);

	void body_twist_to_spatial(const double T[16], const double Vb[6], const double dVb[16], double Vs[6], double dVs[6]);
	void spatial_twist_to_body(const double T[16], const double Vs[6], double const dVs[16], double Vb[6], double dVb[6]);
	void swap_order(double *buf, int n);
	
    // return the force without gravity in the sensor frame
	Eigen::Vector6d get_ext_force(const std::vector<double>& force, 
	                   double mass, 
					   const std::vector<double>& offset, 
					   const std::vector<double>& cog, 
					   const Eigen::Matrix4d & T_sensor, 
					   const Eigen::Matrix4d & T_robot);

	
	Eigen::Vector6d twist_estimate(const Eigen::Matrix4d &Td, const Eigen::Matrix4d &Td_pre, double dt);

	Eigen::Matrix6d spatial_inertia_matrix(const Eigen::Matrix3d &I, double m, const Eigen::Vector3d &com);
	
	Robot load_robot(const char *filename);
    
    Eigen::VectorXd get_ext_torque(const Robot *robot, const std::vector<double> &q, const Eigen::MatrixXd& fext);

    Eigen::MatrixXd mass_matrix(const Robot *robot, const std::vector<double> &q);
	
	Eigen::VectorXd inverse_dynamics(const Robot *robot, const std::vector<double> &q,
	                                 const std::vector<double> &dq, const std::vector<double> &ddq, 
									 const Eigen::MatrixXd& fext);

	void m_c_g_matrix(const Robot *robot, const std::vector<double> &q,
					  const std::vector<double> &dq, Eigen::MatrixXd &M,
					  Eigen::MatrixXd &C, Eigen::VectorXd &g, Eigen::MatrixXd &Jb, Eigen::MatrixXd &dJb,
					  Eigen::MatrixXd &dM, Eigen::Matrix4d &dTb, Eigen::Matrix4d &Tb);


	void derivative_jacobian_matrix(const Robot *robot, const std::vector<double> &q, const std::vector<double> &dq,
									Eigen::MatrixXd &dJ, Eigen::MatrixXd &J, Eigen::Matrix4d &dT, Eigen::Matrix4d &T);

	void admittance_control(const Robot *robot, const Eigen::Matrix4d &Tcp, const Eigen::Matrix4d &Td, const Eigen::Vector6d &Vd,
							const Eigen::Matrix3d &Mp, const Eigen::Matrix3d &Bp, const Eigen::Matrix3d &Kp,
							const Eigen::Matrix3d &Mr, const Eigen::Matrix3d &Br, const Eigen::Matrix3d &Kr,
							const std::vector<double> &q, const std::vector<double> &qd, const Eigen::Vector6d &F, double dt,
							Eigen::Vector3d &re, Eigen::Vector3d &pe, Eigen::Vector3d &red, Eigen::Vector3d &ped, bool flag = false, double *Tcmd = nullptr);

	void admittance_error_cal(const Robot *robot, const Eigen::Matrix4d &Tcp, const Eigen::Matrix4d &Td, const Eigen::Vector6d &Vd,
							  const std::vector<double> &q, const std::vector<double> &qd, Eigen::Vector3d &re, Eigen::Vector3d &pe, Eigen::Vector3d &red, Eigen::Vector3d &ped, bool flag);

	// pure interaction force to the envrionment expressed in TCP frame (negative if for force drag)	
	Eigen::Vector6d gravity_and_inertia_compensation(const Robot &robot, const Eigen::Matrix4d &Tcp, const Eigen::Matrix4d &Tsensor, const std::vector<double> &q, const std::vector<double> &dq,
												  const std::vector<double> &ddq, const double *rawForce, double mass, const double offset[6], const double cog[3], const Eigen::Matrix3d &mI, double scale = 1.0);

} // namespace robot_math

#endif // ROBOT_MATH__ROBOT_MATH_HPP_
