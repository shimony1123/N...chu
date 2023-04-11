#include <Arduino.h>
#include <Wire.h>
#include "SparkFunLSM9DS1.h"

#define PRINT_CALCULATED
#define PRINT_SPEED 250
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

LSM9DS1 imu;
static unsigned long lastPrint;

void printGyro();
void printAccel();
void printMag();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);

void setup(){
  Serial.begin(115200);
  Wire.begin();

  //成功判定
  if(!imu.begin(LSM9DS1_AG_ADDR(1), LSM9DS1_M_ADDR(1), Wire)){
    Serial.println("Failed to communicate with LSM9DS1.");
    }
  else{
    Serial.println("Succeeded to communicate with LSM9DS1.");
    }
}

void loop() {
  // Update the sensor values whenever new data is available
  if (imu.gyroAvailable())
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro();  // Print "G: gx, gy, gz"
    printAccel(); // Print "A: ax, ay, az"
    printMag();   // Print "M: mx, my, mz"
    printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
    Serial.println();

    lastPrint = millis(); // Update lastPrint time
  }
}