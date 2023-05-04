#pragma once
#include "ArduinoEigen.h"

//拡張カルマンフィルタ
struct KalmanFilter{
	Eigen::Vector3f gyro;
	Eigen::Vector3f acc;
	Eigen::Vector3f mag;
	Eigen::Vector3f euler;
	Eigen::Vector3f f_euler;

	// システムを表す変数
 	//Eigen::MatrixXf A;
 	Eigen::Matrix4f B;//4*4の単位行列
 	//Eigen::MatrixXf C;
 	Eigen::Matrix4f Q; //vの共分散。要チューニング
 	Eigen::Matrix4f R; //wの共分散。要チューニング
    Eigen::Matrix4f F;
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(6,4);
 	
 	// 更新していく状態量
 	Eigen::Matrix4f P; //誤差共分散の予測値。setupで初期値の設定が必要。
 	Eigen::Vector4f x; //状態量として今回はクォータニオンを取っている。setupで初期値の設定が必要。

	//初期値
	Eigen::Vector4f mag_calib; //地磁気の初期値。setupでPmodから取る。

	//重力加速度(定数)
	const float g = 9.80665;
 	
 	void update(Eigen::Vector3f gyro, Eigen::Vector3f acc, Eigen::Vector3f mag, float dt);
 	Eigen::Vector4f get_x(Eigen::Vector3f euler);
 	Eigen::Vector4f f(Eigen::VectorXf x, Eigen::VectorXf euler); // 状態方程式のf。クォータニオンの時間微分を返す。
 	Eigen::VectorXf h(Eigen::VectorXf x); // 観測方程式のh
 	Eigen::Matrix4f Jf(Eigen::VectorXf x, Eigen::VectorXf gyro); // fのJacobian
 	Eigen::MatrixXf Jh(Eigen::VectorXf x); // hのJacobian
	void filtered_euler();
 };