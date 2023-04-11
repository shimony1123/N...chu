#pragma once
#include "ArduinoEigen.h"

struct KalmanFilter{
    //システム変数
    int n;
    int m;
    int r;
    Eigen::MatrixXf A;
    Eigen::MatrixXf B;
    Eigen::MatrixXf C;
    Eigen::MatrixXf Q;
    Eigen::MatrixXf R;
    //状態量の行列
    Eigen::MatrixXf P;
    Eigen::VectorXf x;
    Eigen::VectorXf y;

    KalmanFilter(int n, int m, int r, Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf Q, 
    Eigen::MatrixXf R, Eigen::MatrixXf P, Eigen::VectorXf x);

    void update(Eigen::VectorXf y);
    Eigen::VectorXf get_x(){
        return x;
    }
};