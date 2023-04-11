#include "KalmanExample.hpp"

//値を代入する関数
KalmanFilter::KalmanFilter(int n_in, int m_in, int r_in, Eigen::MatrixXf A_in, Eigen::MatrixXf B_in, Eigen::MatrixXf C_in,
Eigen::MatrixXf Q_in, Eigen::MatrixXf R_in, Eigen::MatrixXf P_in, Eigen::VectorXf x_in){
    n = n_in;
    m = m_in;
    r = r_in;
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