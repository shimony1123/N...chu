#include <Arduino.h>
#include <Wire.h>
#include "ArduinoEigen.h"
#include "SparkFunLSM9DS1.h"

#define PRINT_CALCULATED
#define PRINT_SPEED 250
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

//static unsigned long lastPrint;

void inputGyro(LSM9DS1 &imu, Eigen::VectorXf gyro){
  gyro(0) = imu.calcGyro(imu.gx);
  gyro(1) = imu.calcGyro(imu.gy);
  gyro(2) = imu.calcGyro(imu.gz);
  Serial.println("gyro's input succeeded");

// Serial.print("G: ");
// #ifdef PRINT_CALCULATED
//   Serial.print(gyro(0), 2);
//   Serial.print(", ");
//   Serial.print(gyro(1), 2);
//   Serial.print(", ");
//   Serial.print(gyro(2), 2);
//   Serial.println(" deg/s");
// #elif defined PRINT_RAW
//   Serial.print(imu.gx);
//   Serial.print(", ");
//   Serial.print(imu.gy);
//   Serial.print(", ");
//   Serial.println(imu.gz);
// #endif
}

void inputAccel(LSM9DS1 &imu, Eigen::VectorXf acc){
  acc(0) = imu.calcAccel(imu.ax);
  acc(1) = imu.calcAccel(imu.ay);
  acc(2) = imu.calcAccel(imu.az);
  Serial.println("acc's input succeeded");

//   Serial.print("A: ");
// #ifdef PRINT_CALCULATED
//   Serial.print(acc(0), 2);
//   Serial.print(", ");
//   Serial.print(acc(1), 2);
//   Serial.print(", ");
//   Serial.print(acc(2), 2);
//   Serial.println(" g");
// #elif defined PRINT_RAW
//   Serial.print(imu.ax);
//   Serial.print(", ");
//   Serial.print(imu.ay);
//   Serial.print(", ");
//   Serial.println(imu.az);
// #endif
}

void inputMag(LSM9DS1 &imu, Eigen::VectorXf mag){
  mag(0) = imu.calcMag(imu.mx);
  mag(1) = imu.calcMag(imu.my);
  mag(2) = imu.calcMag(imu.mz);
  Serial.println("mag's input succeeded");

  Serial.print("M: ");
  #ifdef PRINT_CALCULATED
  Serial.print(mag(0), 2);
  Serial.print(", ");
  Serial.print(mag(1), 2);
  Serial.print(", ");
  Serial.print(mag(2), 2);
  Serial.println(" gauss");
  #elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
  #endif
}

void printAttitude(LSM9DS1 &imu, Eigen::VectorXf acc, Eigen::VectorXf mag, Eigen::VectorXf euler){
  //inputAccel
  acc(0) = imu.calcAccel(imu.ax);
  acc(1) = imu.calcAccel(imu.ay);
  acc(2) = imu.calcAccel(imu.az);
  Serial.println("acc's input succeeded");

  //inputMag
  mag(0) = imu.calcMag(imu.mx);
  mag(1) = imu.calcMag(imu.my);
  mag(2) = imu.calcMag(imu.mz);
  Serial.println("mag's input succeeded");
  Serial.print("M: ");
  #ifdef PRINT_CALCULATED
  Serial.print(mag(0), 2);
  Serial.print(", ");
  Serial.print(mag(1), 2);
  Serial.print(", ");
  Serial.print(mag(2), 2);
  Serial.println(" gauss");
  #elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
  #endif


  Serial.println(mag(1));
  euler(0) = atan2(acc(1), acc(2)); //roll
  euler(1) = atan2(-acc(0), sqrt(acc(1) * acc(1) + acc(2) * acc(2))); //pitch

  if (mag(1) == 0.0f){
    euler(2) = (mag(0) < 0) ? PI : 0;
  }
  else{
    euler(2) = atan2(mag(0), mag(1)); //yaw
  }

  euler(2) -= DECLINATION * PI / 180;

  if (euler(2) > PI){
    euler(2) -= (2 * PI);
  }
  else if (euler(2) < -PI){
    euler(2) += (2 * PI);
  }

  // Convert everything from radians to degrees:
  euler(0) *= 180.0 / PI;
  euler(1) *= 180.0 / PI;
  euler(2) *= 180.0 / PI;

  Serial.print("Roll, Pitch: ");
  Serial.print(euler(0), 2);
  Serial.print(", ");
  Serial.println(euler(1), 2);
  Serial.print("Yaw: "); Serial.println(euler(2), 2);
}