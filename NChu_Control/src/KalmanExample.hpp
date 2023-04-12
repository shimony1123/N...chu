#pragma once
#include "ArduinoEigen.h"

// struct KalmanFilter{
//     //システム変数
//     Eigen::MatrixXf A;
//     Eigen::MatrixXf B;
//     Eigen::MatrixXf C;
//     Eigen::MatrixXf Q;
//     Eigen::MatrixXf R;
//     //状態量の行列
//     Eigen::MatrixXf P;
//     Eigen::VectorXf x;
//     Eigen::VectorXf y;

//     void setmatrix(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf Q, 
//     Eigen::MatrixXf R, Eigen::MatrixXf P, Eigen::VectorXf x);

//     void update(Eigen::VectorXf y);
//     Eigen::VectorXf get_x(){
//         return x;
//     }
// };

struct KalmanFilter {
	// システムを表す変数
 	int n, m, r;
 	Eigen::MatrixXf A;
 	Eigen::MatrixXf B;
 	Eigen::MatrixXf C;
 	Eigen::MatrixXf Q;
 	Eigen::MatrixXf R;
    Eigen::MatrixXf F;
    Eigen::MatrixXf H;
 	
 	// 更新していく状態量
 	Eigen::MatrixXf P;
 	Eigen::VectorXf x;
 	
 	void setmatrix(Eigen::MatrixXf B, Eigen::MatrixXf Q, Eigen::MatrixXf R,  Eigen::MatrixXf P, Eigen::VectorXf x);
 		
 	void update(Eigen::VectorXf y, float dt, float gyrox, float gyroy, float gyroz);
 	Eigen::VectorXf get_x(float roll, float pitch, float yaw) {return x;}
 	Eigen::VectorXf f(Eigen::VectorXf x,float gyrox, float gyroy, float gyroz); // 状態方程式のf。クォータニオンの時間微分を返す。
 	Eigen::VectorXf h(Eigen::VectorXf x); // 観測方程式のh
 	Eigen::MatrixXf Jf(float dt); // fのJacobian
 	Eigen::MatrixXf Jh(); // hのJacobian
 };