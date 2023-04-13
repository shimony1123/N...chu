#pragma once
#include "ArduinoEigen.h"

//拡張カルマンフィルタ
struct KalmanFilter{
	// システムを表す変数
 	//Eigen::MatrixXf A;
 	Eigen::MatrixXf B; //4*4の単位行列
 	//Eigen::MatrixXf C;
 	Eigen::MatrixXf Q; //vの共分散。要チューニング
 	Eigen::MatrixXf R; //wの共分散。要チューニング
    Eigen::MatrixXf F;
    Eigen::MatrixXf H;
 	
 	// 更新していく状態量
 	Eigen::MatrixXf P; //誤差共分散の予測値
 	Eigen::VectorXf x;
	Eigen::VectorXf mag_calib; //地磁気の初期値
 	
 	void update(Eigen::VectorXf y, Eigen::VectorXf gyro, Eigen::VectorXf acc, Eigen::VectorXf mag, float dt);
 	Eigen::VectorXf get_x(Eigen::VectorXf euler);
 	Eigen::VectorXf f(Eigen::VectorXf x, Eigen::VectorXf euler); // 状態方程式のf。クォータニオンの時間微分を返す。
 	Eigen::VectorXf h(Eigen::VectorXf x); // 観測方程式のh
 	Eigen::MatrixXf Jf(float dt); // fのJacobian
 	Eigen::MatrixXf Jh(); // hのJacobian
 };

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