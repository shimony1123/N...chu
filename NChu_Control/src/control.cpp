#include "ArduinoEigen.h"
#include "ArduinoEigen.h"

float roll_control(Eigen::Vector3f euler,float channel1){
    //a_objはエルロンの目標値。max:30°, min:-30°
    float duty_roll; //出力するPWMのduty比
    float g_roll = 0.01; //このP制御のゲインをチューニングする。

    float channel_min = 368;
    float channel_max = 1680;
    float roll_min = -30.0;
    float roll_max = 30.0;
    float roll_obj = roll_min + (roll_max - roll_min)/(channel_max - channel_min) * (channel1 - channel_min);

    float diff_roll = euler(0) - roll_obj;
    duty_roll = g_roll*diff_roll;
}

float pitch_roll(Eigen::Vector3f euler,float channel2){

}