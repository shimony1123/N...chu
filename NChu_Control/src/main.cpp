#include <Arduino.h>
#include <Wire.h>
#include <sbus.h>
#include "SparkFunLSM9DS1.h"
#include "KalmanFilter.hpp"
#include "servo.hpp"

#define PRINT_CALCULATED
#define PRINT_SPEED 250
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.
#define RX_PIN 16

//HardwareSerial Serial1(0);
bfs::SbusRx sbus(&Serial1,RX_PIN,-1,true);//Sbusで送信することはないので、TXピン番号は-1としている。
bfs::SbusData data;

//コンストラクタで代入する値
uint8_t channel_aileron_in = 0;
uint8_t channel_elevator_in = 2;
int freq_in = 100; //周波数=1/周期
uint8_t bit_num_in = 16; // 解像度
int Pin_in_aileron = 15; //左ピン番号
int Pin_in_elevator = 17; //右ピン番号
float duty_ratio_aileron_in = 0.11; // duty比×解像度。ここにroll_controlを代入。
float duty_ratio_elevator_in = 0.19; //ここにpitch_controlを代入。
static unsigned long lastPrint;
float dt; //刻み幅
float power_left;
float power_right;
Eigen::MatrixXf P_ini = Eigen::MatrixXf::Zero(4, 4); //Pの初期値

KalmanFilter kalmanfilter;
LSM9DS1 imu;
servo servo_aileron(channel_aileron_in, freq_in, bit_num_in, Pin_in_aileron, duty_ratio_aileron_in);
servo servo_elevator(channel_elevator_in, freq_in, bit_num_in, Pin_in_elevator, duty_ratio_elevator_in);

void inputGyro(LSM9DS1 &imu, Eigen::Vector3f &gyro); //gyroの値を更新
void inputAccel(LSM9DS1 &imu, Eigen::Vector3f &acc); //加速度の値を更新
void inputMag(LSM9DS1 &imu, Eigen::Vector3f &mag); //地磁気の値を更新
void printAttitude(LSM9DS1 &imu, Eigen::Vector3f &acc, Eigen::Vector3f &mag, Eigen::Vector3f &euler); //オイラー角の値を更新

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
  sbus.Begin();
  //なんでWireとsbusで大文字小文字のルールが違うんだ！！

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
    inputGyro(imu,kalmanfilter.gyro);
    printAttitude(imu, kalmanfilter.acc, kalmanfilter.mag, kalmanfilter.euler);
    
    kalmanfilter.update(kalmanfilter.gyro, kalmanfilter.acc, kalmanfilter.mag, dt);
    kalmanfilter.filtered_euler();
    // //filtered_eulerがカルマンフィルタを通したあとの値。

    lastPrint = millis(); // Update lastPrint time
  }

  // //servo部分
  // ledcWrite(servo_aileron.channel,servo_aileron.duty);
  // ledcWrite(servo_elevator.channel,servo_elevator.duty);

  // //sbus部分
  // if (sbus.Read()){
  //   data = sbus.data();//データを読み込む
  //   for (int i=0; i<3; i++){
  //     //各チャネルに対応する値を表示
  //     Serial.print("ch");
  //     Serial.print(i);
  //     Serial.print(": ");
  //     Serial.print(data.ch[i]);
  //     Serial.print(i==2? "\n" : " ");
  //     //maxは1680,minは368。0が1011。
  //   }
  // }
}