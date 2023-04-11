#pragma once
#include "ArduinoEigen.h"

struct KalmanFilter{
    //システム変数
    Eigen::MatrixXf A;
    Eigen::MatrixXf B;
    Eigen::MatrixXf C;
    Eigen::MatrixXf Q;
    Eigen::MatrixXf R;
    //状態量の行列
    Eigen::MatrixXf P;
    Eigen::VectorXf x;
    Eigen::VectorXf y;

    void setmatrix(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf Q, 
    Eigen::MatrixXf R, Eigen::MatrixXf P, Eigen::VectorXf x);

    void update(Eigen::VectorXf y);
    Eigen::VectorXf get_x(){
        return x;
    }
};