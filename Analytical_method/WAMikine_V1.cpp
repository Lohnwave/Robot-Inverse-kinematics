/******************************************************************************\
* Copyright (C) 2019 . 江西省智能信息系统重点实验室, All rights reserved.		*
* Version: 1.0																	*
* Last Revised: 2019-12-07														*
* Editor: Luozu																	*
* 基于几何法的带连杆偏移的七自由度冗余机械臂逆运动学求解器						*
\******************************************************************************/
#include "WAMikine.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
using namespace std;
using namespace Eigen;

namespace Ikine{
	//MatrixXd T01, T12, T23, T34, T45, T56, T67, T07;
	/* Barrett WAM D-H parmeters: m  rad */
	extern double DH_alpha[] = { -M_PI / 2, M_PI / 2, -M_PI / 2, M_PI / 2, -M_PI / 2, M_PI / 2, 0 };
	extern double DH_a[] = { 0, 0, 0.045, -0.045, 0, 0, 0 };
	extern double DH_d[] = { 0, 0, 0.55, 0, 0.3, 0, 0.06 };
	extern double theta_L[] = { -2.6, -2, -2.8, -0.9, -4.76, -1.6, -3 };
	extern double theta_U[] = { 2.6, 2, 2.8, 3.1, 1.24, 1.6, 3 };
	void DHmatrix(double alpha, double a, double d, double &theta, MatrixXd &T) {
		// INPUT: alpha, a, d Denavit Hartenberg parameters, and theta, the rotation angle of a wam link (standard notation).
		// OUTPUT: T, the D-H matrix (MatrixXd) calculated with those parameters and theta.
		T.resize(4, 4);
		T.fill(0.0);
		T(0, 0) = cos(theta);
		T(0, 1) = -sin(theta) * cos(alpha);
		T(0, 2) = sin(theta) * sin(alpha);
		T(0, 3) = a * cos(theta);
		T(1, 0) = sin(theta);
		T(1, 1) = cos(theta) * cos(alpha);
		T(1, 2) = -cos(theta) * sin(alpha);
		T(1, 3) = a * sin(theta);
		T(2, 1) = sin(alpha);
		T(2, 2) = cos(alpha);
		T(2, 3) = d;
		T(3, 3) = 1;
	}
	MatrixXd inline DHmatrix(double alpha, double a, double d, double theta){
		Matrix4d T;
		T(0, 0) = cos(theta);
		T(0, 1) = -sin(theta) * cos(alpha);
		T(0, 2) = sin(theta) * sin(alpha);
		T(0, 3) = a * cos(theta);
		T(1, 0) = sin(theta);
		T(1, 1) = cos(theta) * cos(alpha);
		T(1, 2) = -cos(theta) * sin(alpha);
		T(1, 3) = a * sin(theta);
		T(2, 0) = 0;
		T(2, 1) = sin(alpha);
		T(2, 2) = cos(alpha);
		T(2, 3) = d;
		T(3, 0) = 0;
		T(3, 1) = 0;
		T(3, 2) = 0;
		T(3, 3) = 1;
		return T;
	}

	void Fkine(double angles[], Vector3d& pos, Quaterniond& ori)
	{
		Matrix4d _0_T_1 = DHmatrix(DH_alpha[0], DH_a[0], DH_d[0], angles[0]);
		Matrix4d _1_T_2 = DHmatrix(DH_alpha[1], DH_a[1], DH_d[1], angles[1]);
		Matrix4d _2_T_3 = DHmatrix(DH_alpha[2], DH_a[2], DH_d[2], angles[2]);
		Matrix4d _3_T_4 = DHmatrix(DH_alpha[3], DH_a[3], DH_d[3], angles[3]);
		Matrix4d _4_T_5 = DHmatrix(DH_alpha[4], DH_a[4], DH_d[4], angles[4]);
		Matrix4d _5_T_6 = DHmatrix(DH_alpha[5], DH_a[5], DH_d[5], angles[5]);
		Matrix4d _6_T_7 = DHmatrix(DH_alpha[6], DH_a[6], DH_d[6], angles[6]);

		Matrix4d H = _0_T_1 * _1_T_2 * _2_T_3 * _3_T_4 * _4_T_5 * _5_T_6 * _6_T_7;
		// 齐次变换矩阵的第四列前三项元素分别为Position: x,y,z
		pos[0] = H(0, 3);
		pos[1] = H(1, 3);
		pos[2] = H(2, 3);
		// 齐次变换矩阵的左上角方块 00-33 为3x3旋转矩阵 rotation_matrix
		Matrix3d R = H.block(0, 0, 3, 3);
		ori = Quaterniond(R); // rotation_matrix to quaternion 
	}

	bool solve(Vector3d position, Quaterniond orientation, double phi, double theta[])
	{
		orientation.normalize();
		Matrix3d orientation_mx = orientation.toRotationMatrix();
		Vector3d TRz = orientation_mx.col(2);
		Vector3d W_dir = TRz * 0.06;
		Vector3d W = position - W_dir; // wrist position
		double dsq = W.dot(W);
		double d = sqrt(dsq);
		double L1sq = 0.55 * 0.55 + 0.045 * 0.045;
		double L1 = sqrt(L1sq);
		double L2sq = 0.3 * 0.3 + 0.045 * 0.045;
		double L2 = sqrt(L2sq);
		if (d > L1 + L2)
		{
			return false;
		}
		// E(C_rot)
		Matrix3d Rnorm = Quaterniond::FromTwoVectors(Vector3d::UnitZ(), W / d).toRotationMatrix();
		// double beta1 = asin(L2 * sin(beta2) / L1);
		double beta1 = acos((dsq + L1sq - L2sq) / (2 * d * L1));
		double beta2 = acos((dsq + L2sq - L1sq) / (2 * d * L2));
		double R = L1 * sin(beta1);
		Vector3d C(R * cos(phi), R * sin(phi), L1 * cos(beta1));
		Vector3d E = Rnorm * C;
		// L
		// Vector3d WE = E - W;
		// Vector3d WE_n = WE.normalized();
		Vector3d E_n = E.normalized();
		Vector3d E_c_W = E.cross(W);
		Vector3d L = E_n * (0.55 * 0.55 / L1) + E.cross(E_c_W).normalized() * (0.55 * 0.045 / L1);
		// theta 1
		theta[0] = atan2(L.y(), L.x());
		// theta 2
		theta[1] = acos(L.z() / 0.55);

		Vector3d joint3_up = L.cross(Vector3d::UnitZ()).cross(L);
		Vector3d LE = E - L;
		Vector3d LE_n = LE / 0.045;

		// theta 3
		theta[2] = acos(joint3_up.normalized().dot(LE_n));
		if (L.dot(LE_n.cross(joint3_up)) > 0)
		{
			theta[2] = -theta[2];
		}
		theta[2] -= M_PI;

		// double angle_BAU_J = M_PI_2 - atan(0.045 / 0.3);
		// double angle_OAL_J = M_PI_2 - atan(0.045 / 0.55);
		// double theta_U = M_PI_2 + beta2 - angle_BAU_J;
		// double theta_L = M_PI_2 + beta1 - angle_OAL_J;
		// double theta_U = beta2 + atan(0.045 / 0.3);
		// double theta_L = beta1 + atan(0.045 / 0.55);

		//theta 4
		theta[3] = beta2 + atan(0.045 / 0.3) + beta1 + atan(0.045 / 0.55);

		// Vector3d U = W + WE_n * (0.3 * 0.3 / L2) + E_c_W.cross(WE).normalized() * (0.3 * 0.045 / L2);

		Matrix4d _0_T_1 = DHmatrix(DH_alpha[0], DH_a[0], DH_d[0], theta[0]);
		Matrix4d _1_T_2 = DHmatrix(DH_alpha[1], DH_a[1], DH_d[1], theta[1]);
		Matrix4d _2_T_3 = DHmatrix(DH_alpha[2], DH_a[2], DH_d[2], theta[2]);
		Matrix4d _3_T_4 = DHmatrix(DH_alpha[3], DH_a[3], DH_d[3], theta[3]);

		Matrix4d _0_H_4 = _0_T_1 * _1_T_2 * _2_T_3 * _3_T_4;
		Matrix3d R_04 = _0_H_4.block(0, 0, 3, 3);

		Vector3d p_tool = R_04.transpose() * W_dir;

		// theta 5
		theta[4] = atan2(p_tool.y(), p_tool.x());
		// theta 6
		theta[5] = M_PI_2 - atan2(p_tool.z(), sqrt(p_tool.x() * p_tool.x() + p_tool.y() * p_tool.y()));

		Matrix4d _4_T_5 = DHmatrix(DH_alpha[4], DH_a[4], DH_d[4], theta[4]);
		Matrix4d _5_T_6 = DHmatrix(DH_alpha[5], DH_a[5], DH_d[5], theta[5]);

		Matrix4d _0_H_6 = _0_H_4 * _4_T_5 * _5_T_6;
		Vector3d _0_H_6_R_x(_0_H_6(0, 0), _0_H_6(1, 0), _0_H_6(2, 0));

		Vector3d TRx = orientation_mx.col(0);

		// theta 7
		theta[6] = acos(_0_H_6_R_x.dot(TRx));
		if (W_dir.dot(_0_H_6_R_x.cross(TRx)) < 0)
		{
			theta[6] = -theta[6];
		}

		for (int i = 0; i < 7; i++)
		{
			while (theta[i] < theta_L[i])
				theta[i] += (M_PI + M_PI);
			while (theta[i] > theta_U[i])
				theta[i] -= (M_PI + M_PI);
		}

		// TODO : 8 more solutions

		return true;
	}

	bool isValid(double angles[])
	{
		for (int i = 0; i < 7; i++)
		{
			if (angles[i] < theta_L[i] || angles[i] > theta_U[i])
			{
				return false;
			}
		}
		return true;
	}
	bool solve_IK(Eigen::Vector3d position, Eigen::Quaterniond orientation, 
		std::vector<double> currentjoints, std::vector<double>& joints){
		joints.resize(7);
		double minDiff = 1e10;
		bool found = false;
		for (size_t i = 0; i < 360; i++)
		{
			double phi = i / 180.0 * M_PI;
			double angles[7];
			bool has_solution = solve(position, orientation, phi, angles);
			if (has_solution && isValid(angles))
			{
				found = true;
				double diff = 0;
				for (size_t j = 0; j < 7; j++)
				{
					diff += abs(currentjoints[j] - angles[j]);
				}
				if (diff < minDiff)
				{
					for (size_t k = 0; k < 7; k++)
					{
						joints[k] = angles[k];
					}
					minDiff = diff;
				}
			}
		}
		return found;
	}
	//void InitiRobot() {
	//	/*		alpha		a		d		theta		T	*/
	//	DHmatrix(-PI / 2,	0.,		0.,		qoptim(0), T01);
	//	DHmatrix(PI / 2,	0.,		0.,		qoptim(1), T12);
	//	DHmatrix(-PI / 2,	0.045,	0.55,	qoptim(2), T23);
	//	DHmatrix(PI / 2,	-0.045, 0.,		qoptim(3), T34);
	//	DHmatrix(-PI / 2,	0.,		0.3,	qoptim(4), T45);
	//	DHmatrix(PI / 2,	0.,		0.,		qoptim(5), T56);
	//	DHmatrix(0.,		0.,		0.06,	qoptim(6), T67);
	//	T07 = T01 * T12 * T23 * T34 * T45 * T56 * T67;
	//}
}
