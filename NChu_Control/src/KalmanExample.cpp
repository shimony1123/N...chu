#include "KalmanExample.hpp"

//値を代入する関数
void KalmanFilter::setmatrix(Eigen::MatrixXf A_in, Eigen::MatrixXf B_in, Eigen::MatrixXf C_in,
Eigen::MatrixXf Q_in, Eigen::MatrixXf R_in, Eigen::MatrixXf P_in, Eigen::VectorXf x_in){
	//nは状態変数の数、mは観測変数の次元、rは
    A = A_in;
 	B = B_in;
 	C = C_in;
 	Q = Q_in;
 	R = R_in;
 	x = x_in;
 	P = P_in;
}

void KalmanFilter::update(Eigen::VectorXf y){
    x = A*x;
 	P = A*P*A.transpose() + B*Q*B.transpose();
 	Eigen::MatrixXf G = P*C.transpose() * (C*P*C.transpose() + R).inverse();
 	x += G*(y - C*x);
 	P = P - G*C*P;
}