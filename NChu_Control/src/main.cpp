#include <Arduino.h>
#include <Wire.h>
#include "SparkFunLSM9DS1.h"
#include "KalmanExample.hpp"

#define PRINT_CALCULATED
#define PRINT_SPEED 250
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

LSM9DS1 imu;
static unsigned long lastPrint;
Eigen::MatrixXf A_in;
Eigen::MatrixXf B_in;
Eigen::MatrixXf C_in;
Eigen::MatrixXf Q_in;
Eigen::MatrixXf R_in;
Eigen::MatrixXf P_in;
Eigen::VectorXf x_in;

KalmanFilter kalmanfilter;

void printGyro(LSM9DS1 &imu);
void printAccel(LSM9DS1 &imu);
void printMag(LSM9DS1 &imu);
void printAttitude(LSM9DS1 &imu, float ax, float ay, float az, float mx, float my, float mz);

void setup(){
  Serial.begin(115200);
  Wire.begin();
  kalmanfilter.setmatrix(A_in, B_in, C_in, Q_in, R_in, P_in, x_in);

  //ここでそれぞれの行列を初期化
  

  //成功判定
  if(!imu.begin(LSM9DS1_AG_ADDR(1), LSM9DS1_M_ADDR(1), Wire)){
    Serial.println("Failed to communicate with LSM9DS1.");
    }
  else{
    Serial.println("Succeeded to communicate with LSM9DS1.");
    }
}

void loop() {
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
    printGyro(imu);  // Print "G: gx, gy, gz"
    printAccel(imu); // Print "A: ax, ay, az"
    printMag(imu);   // Print "M: mx, my, mz"
    printAttitude(imu, imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
    Serial.println();

    lastPrint = millis(); // Update lastPrint time
  }

  //カルマンフィルタ部分

}