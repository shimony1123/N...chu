#pragma once
#include "KalmanExample.hpp"
#include "ArduinoEigen.h"

//以下は拡張カルマンフィルタ
void KalmanFilter::setmatrix(Eigen::MatrixXf B_in,Eigen::MatrixXf Q_in, Eigen::MatrixXf R_in, 
Eigen::MatrixXf P_in, Eigen::VectorXf x_in) {
  	B = B_in;
  	Q = Q_in;
  	R = R_in;
  	x = x_in;
  	P = P_in;
	}

void KalmanFilter::update(Eigen::VectorXf y, float dt,float gyrox,float gyroy,float gyroz) {
	x += f(x,gyrox,gyroy,gyroz);
  	F = Jf(dt);
  	P = F * P * F.transpose() + B * Q * B.transpose();
  	H = Jh();
  	Eigen::MatrixXf G = P * H.transpose() * (H * P * H.transpose() + R).inverse();
  	x += G * (y - h(x));
  	P -= G*H*P;
	}

Eigen::VectorXf KalmanFilter::get_x(float roll, float pitch, float yaw){
	float phi, theta, psi;
	float q0, q1, q2, q3;
	phi = roll;
	theta = pitch;
	psi = yaw;

	q0 = cos(phi/2.0f)*cos(theta/2.0f)*cos(psi/2.0f)+sin(phi/2.0f)*sin(theta/2.0f)*sin(psi/2.0f);
	q1 = sin(phi/2.0f)*cos(theta/2.0f)*cos(psi/2.0f)-cos(phi/2.0f)*sin(theta/2.0f)*sin(psi/2.0f);
	q2 = cos(phi/2.0f)*sin(theta/2.0f)*cos(psi/2.0f)+sin(phi/2.0f)*cos(theta/2.0f)*sin(psi/2.0f);
	q3 = cos(phi/2.0f)*cos(theta/2.0f)*sin(psi/2.0f)-sin(phi/2.0f)*sin(theta/2.0f)*cos(psi/2.0f);

	x << q0,q1,q2,q3;
	return x;
}

Eigen::VectorXf f(Eigen::VectorXf x, float gyrox, float gyroy, float gyroz){
	Eigen::Matrix4f a;
	Eigen::MatrixXf omega(4,1);
	omega << 0,gyrox,gyroy,gyroz;
	a << x(3),-x(2),x(1),x(0),
	     x(2),x(3),-x(0),x(1),
		 -x(1),x(0),x(3),x(2),
		 -x(0),-x(1),-x(2),x(3);

	Eigen::VectorXf xdot;
	xdot = 1.0/2*a*omega;//xdotがクォータニオンの微分
	return xdot;
}

Eigen::VectorXf h(Eigen::VectorXf x){
	float roll;
	float pitch;
	float yaw;

	roll = atan2(2.0f*(x(0)*x(1)+x(2)*x(3)), (x(0)*x(0)-x(1)*x(1)-x(2)*x(2)+x(3)*x(3)));
	pitch = asin(2.0f*(x(0)*x(2)-x(1)*x(3)));
	yaw = atan2(2.0f*(x(0)*x(3)+x(1)*x(2)), x(0)*x(0)+x(1)*x(1)-x(2)*x(2)-x(3)*x(3));

	Eigen::VectorXf y;
	y << roll,pitch,yaw;
	return y;
}

 //線形kf
// //値を代入する関数
// void KalmanFilter::setmatrix(Eigen::MatrixXf A_in, Eigen::MatrixXf B_in, Eigen::MatrixXf C_in,
// Eigen::MatrixXf Q_in, Eigen::MatrixXf R_in, Eigen::MatrixXf P_in, Eigen::VectorXf x_in){
// 	//nは状態変数の数、mは観測変数の次元、rは
//     A = A_in;
//  	B = B_in;
//  	C = C_in;
//  	Q = Q_in;
//  	R = R_in;
//  	x = x_in;
//  	P = P_in;
// }

// void KalmanFilter::update(Eigen::VectorXf y){
//     x = A*x;
//  	P = A*P*A.transpose() + B*Q*B.transpose();
//  	Eigen::MatrixXf G = P*C.transpose() * (C*P*C.transpose() + R).inverse();
//  	x += G*(y - C*x);
//  	P = P - G*C*P;
// }
