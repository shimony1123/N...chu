#include "KalmanExample.hpp"
#include "ArduinoEigen.h"

//以下は拡張カルマンフィルタ

void KalmanFilter::update(Eigen::Vector3f gyro, Eigen::Vector3f acc, Eigen::Vector3f mag, float dt) {
	//行列のチューニング
	Q << 1.0, 0, 0, 0,
	     0, 1.0, 0, 0,
		 0, 0, 1.0, 0,
		 0, 0, 0, 1.0;

	R << 1.0, 0, 0, 0,
	     0, 1.0, 0, 0,
		 0, 0, 1.0, 0,
		 0, 0, 0, 1.0;
	float gain = 1.0;
	R = gain*R;

	Eigen::VectorXf y = Eigen::VectorXf::Zero(6);
	y << acc(0),acc(1),acc(2),mag(0),mag(1),mag(2);

	x += f(x,gyro)*dt;
  	F = Jf(x,gyro)*dt;
  	P = F * P * F.transpose() + B * Q * B.transpose();
  	H = Jh(x);
  	Eigen::MatrixXf G = P * H.transpose() * (H * P * H.transpose() + R).inverse();
  	x += G * (y - h(x));
  	P -= G * H * P;
	}

Eigen::Vector4f KalmanFilter::get_x(Eigen::VectorXf euler){
	float phi, theta, psi;
	float q0, q1, q2, q3;
	phi = euler(0);
	theta = euler(1);
	psi = euler(2);

	q0 = cos(phi/2.0f)*cos(theta/2.0f)*cos(psi/2.0f)+sin(phi/2.0f)*sin(theta/2.0f)*sin(psi/2.0f);
	q1 = sin(phi/2.0f)*cos(theta/2.0f)*cos(psi/2.0f)-cos(phi/2.0f)*sin(theta/2.0f)*sin(psi/2.0f);
	q2 = cos(phi/2.0f)*sin(theta/2.0f)*cos(psi/2.0f)+sin(phi/2.0f)*cos(theta/2.0f)*sin(psi/2.0f);
	q3 = cos(phi/2.0f)*cos(theta/2.0f)*sin(psi/2.0f)-sin(phi/2.0f)*sin(theta/2.0f)*cos(psi/2.0f);

	x << q0,q1,q2,q3;
	return x;
}

Eigen::Vector4f KalmanFilter::f(Eigen::VectorXf x, Eigen::VectorXf gyro){
	Eigen::Matrix4f a;
	Eigen::MatrixXf omega(4,1);
	omega << 0,gyro(0),gyro(1),gyro(2);
	a << x(3),-x(2),x(1),x(0),
	     x(2),x(3),-x(0),x(1),
		 -x(1),x(0),x(3),x(2),
		 -x(0),-x(1),-x(2),x(3);

	Eigen::VectorXf xdot;
	xdot = 1.0/2*a*omega;//xdotがクォータニオンの微分
	return xdot;
}

Eigen::VectorXf KalmanFilter::h(Eigen::VectorXf x){
	float q0,q1,q2,q3;
	q0 = x(0);
	q1 = x(1);
	q2 = x(2);
	q3 = x(3);

	//ここでのRは回転行列
	Eigen::Matrix3f R;
	// 対角成分
	R(0, 0) = q0*q0 + q1*q1 - q2*q2 - q3*q3;
	R(1, 1) = q0*q0 - q1*q1 + q2*q2 - q3*q3;
	R(2, 2) = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	// 対角成分以外
	R(0, 1) = 2.0f*(q1*q2 - q0*q3);
	R(1, 0) = 2.0f*(q1*q2 + q0*q3);
	R(0, 2) = 2.0f*(q1*q3 + q0*q2);
	R(2, 0) = 2.0f*(q1*q3 - q0*q2);
	R(2, 1) = 2.0f*(q2*q3 + q0*q1);
	R(1, 2) = 2.0f*(q2*q3 - q0*q1);
	Eigen::MatrixXf R_total = Eigen::MatrixXf::Zero(6,6);
	R_total << R,0,
	           0,R;

	Eigen::VectorXf earth = Eigen::VectorXf::Zero(6);
	earth << 0, 0, g, mag_calib(0), mag_calib(1), mag_calib(2);

	return R_total*earth;
	//返り値は観測量で、[ax,ay,az,mx,my,mz]
}

Eigen::Matrix4f KalmanFilter::Jf(Eigen::VectorXf x, Eigen::VectorXf gyro){
	Eigen::Matrix4f F; //Fはf(x)のJacobian
	F << gyro(3),gyro(2),-gyro(1),0,
	     -gyro(2),gyro(3),0,gyro(1),
		 gyro(1),0,gyro(3),gyro(2),
		 0,-gyro(1),-gyro(2),gyro(3);

	return F;
}

Eigen::MatrixXf KalmanFilter::Jh(Eigen::VectorXf x){
	float q0,q1,q2,q3,mx,my,mz;
	q0 = x(0);
	q1 = x(1);
	q2 = x(2);
	q3 = x(3);
	mx = mag_calib(0);
	my = mag_calib(1);
	mz = mag_calib(2);

	Eigen::MatrixXf H = Eigen::MatrixXf::Zero(6,4); //Hはh(x)のJacobian
	H << 2*g*q2, 2*g*q3, 2*g*q0, 2*g*q1,
	     -2*g*q1, -2*g*q0, 2*g*q3, 2*g*q2,
		 2*g*q0, -2*g*q1, -2*g*q2, 2*g*q3,
		 2*mx*q0-2*my*q3+2*mz*q2, 2*mx*q1+2*my*q2+2*mz*q3, -2*mx*q2+2*my*q1+2*mz*q0, -2*mx*q3-2*my*q0+2*mz*q1,
		 2*mx*q3+2*my*q0-2*mz*q1, 2*mx*q2-2*my*q1-2*mz*q0, 2*mx*q1+2*my*q2+2*mz*q3, 2*mx*q0-2*my*q3+2*mz*q2,
		 -2*mx*q2+2*my*q1+2*mz*q0, 2*mx*q3+2*my*q0-2*mz*q1, -2*mx*q0+2*my*q3-2*mz*q2, 2*mx*q1+2*my*q2+2*mz*q3;

	return H;
}

void KalmanFilter::filtered_euler(Eigen::Vector3f filtered_euler){
	float phi, theta, psi;
	float q0, q1, q2, q3 = x(0), x(1), x(2), x(3);
	phi = atan2(2.0f*(q0*q1+q2*q3), q0*q0-q1*q1-q2*q2+q3*q3);
	theta = asin(2.0f*(q0*q2-q1*q3));
	psi = atan2(2.0f*(q0*q3+q1*q2), q0*q0+q1*q1-q2*q2-q3*q3);
	Serial.print("phi : "); Serial.println(phi);
	Serial.print("theta : "); Serial.println(theta);
	Serial.print("psi : "); Serial.println(psi);

	filtered_euler << phi,theta,psi;
}