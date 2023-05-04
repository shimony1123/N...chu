#include <Arduino.h>
#include <Wire.h>
#include "SparkFunLSM9DS1.h"
#include "KalmanFilter.hpp"

#define PRINT_CALCULATED
#define PRINT_SPEED 250
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

LSM9DS1 imu;
static unsigned long lastPrint;
float dt; //刻み幅
Eigen::MatrixXf P_ini = Eigen::MatrixXf::Zero(4, 4); //Pの初期値

KalmanFilter kalmanfilter;

void inputGyro(LSM9DS1 &imu, Eigen::VectorXf gyro); //gyroの値を更新
void inputAccel(LSM9DS1 &imu, Eigen::VectorXf acc); //加速度の値を更新
void inputMag(LSM9DS1 &imu, Eigen::VectorXf mag); //地磁気の値を更新
void printAttitude(LSM9DS1 &imu, Eigen::VectorXf acc, Eigen::VectorXf mag, Eigen::VectorXf euler); //オイラー角の値を更新

void calib(){
  //地磁気calibration
  if (imu.magAvailable()){
      imu.readMag();
      }
  kalmanfilter.mag_calib << imu.calcMag(imu.mx),imu.calcMag(imu.my),imu.calcMag(imu.mz);

  //行列Pの初期化
  P_ini << 1.0, 0, 0, 0,
           0, 1.0, 0, 0,
           0, 0, 1.0, 0,
           0, 0, 0, 1.0;
  kalmanfilter.P = P_ini;

  //状態量x(クォータニオン)の初期化
  kalmanfilter.x << 1.0, 0, 0, 0; //オイラー角が全て0[deg]に対応

  dt = 0.01;
}

void setup(){
  Serial.begin(115200);
  Wire.begin();

  //成功判定
  if(!imu.begin(LSM9DS1_AG_ADDR(1), LSM9DS1_M_ADDR(1), Wire))
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    }
  else
  {
    Serial.println("Succeeded to communicate with LSM9DS1.");
    //成功したらcalibration
    //calib();
  }
}

void loop(){
  //PModでデータを得るところ
  if (imu.gyroAvailable()){
    imu.readGyro();
  }

  if ( imu.accelAvailable() ){
    imu.readAccel();
  }

  if ( imu.magAvailable() ){
    imu.readMag();
  }

  if ((lastPrint + PRINT_SPEED) < millis()){
    printAttitude(imu, kalmanfilter.acc, kalmanfilter.mag, kalmanfilter.euler);
    
    // kalmanfilter.update(kalmanfilter.gyro, kalmanfilter.acc, kalmanfilter.mag, dt);
    // kalmanfilter.filtered_euler();
    // //filtered_eulerがカルマンフィルタを通したあとの値。

    lastPrint = millis(); // Update lastPrint time
  }

  //カルマンフィルタ部分

}