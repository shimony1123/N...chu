#include <Arduino.h>
#include <Wire.h>
#include "SparkFunLSM9DS1.h"
#include "KalmanExample.hpp"

#define PRINT_CALCULATED
#define PRINT_SPEED 250
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

LSM9DS1 imu;
static unsigned long lastPrint;
Eigen::VectorXf gyro;
Eigen::VectorXf acc;
Eigen::VectorXf mag;
Eigen::VectorXf euler;
Eigen::MatrixXf P_ini; //Pの初期値

KalmanFilter kalmanfilter;

void printGyro(LSM9DS1 &imu, Eigen::VectorXf gyro); //gyroの値を更新
void printAccel(LSM9DS1 &imu, Eigen::VectorXf acc); //加速度の値を更新
void printMag(LSM9DS1 &imu, Eigen::VectorXf mag); //地磁気の値を更新
void printAttitude(LSM9DS1 &imu, Eigen::VectorXf acc, Eigen::VectorXf mag, Eigen::VectorXf euler); //オイラー角の値を更新

void calib(){
  //地磁気calibration
  if (imu.magAvailable()){
      imu.readMag();
      }
  kalmanfilter.mag_calib << imu.calcMag(imu.mx),imu.calcMag(imu.my),imu.calcMag(imu.mz);

  //行列Pの初期化
  /*P_ini << 代入*/
  kalmanfilter.P = P_ini;

  //状態量x(クォータニオン)の初期化
  kalmanfilter.x << 1.0, 0, 0, 0; //オイラー角が全て0[deg]に対応
}

void setup(){
  Serial.begin(115200);
  Wire.begin();
  //kalmanfilter.setmatrix(B_in, Q_in, R_in, P_in, x_in);

  //ここでA_in, B_in, C_in, Q_in, R_in, P_in, x_inそれぞれを初期化

  //成功判定
  if(!imu.begin(LSM9DS1_AG_ADDR(1), LSM9DS1_M_ADDR(1), Wire))
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    }
  else
  {
    Serial.println("Succeeded to communicate with LSM9DS1.");
    //成功したらcalibration
    calib();
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
    printGyro(imu,gyro);  // Print "G: gx, gy, gz"
    printAccel(imu,acc); // Print "A: ax, ay, az"
    printMag(imu,mag);   // Print "M: mx, my, mz"
    printAttitude(imu, acc, mag, euler);
    Serial.println();

    lastPrint = millis(); // Update lastPrint time
  }

  //カルマンフィルタ部分

}