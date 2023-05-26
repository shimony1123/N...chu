#include "ArduinoEigen.h"

 Eigen::Vector2f P_control(Eigen::Vector3f euler,float channel1,float channel2){
    //引数eulerにはカルマンフィルタを通した後のオイラー角が入る。
    float duty_roll; //出力するPWMのduty比
    float duty_pitch; //出力するPWMのduty比
    
    float g_roll = 0.01; //rollのP制御のゲイン。要チューニング。
    float g_pitch = 0.01; //pitchのP制御のゲイン。要チューニング。

    float channel_min = 368;
    float channel_max = 1680;

    float roll_min = -30.0;
    float roll_max = 30.0;
    float pitch_min = -30.0;
    float pitch_max = 30.0;

    float roll_obj = roll_min + (roll_max - roll_min)/(channel_max - channel_min) * (channel1 - channel_min);
    float pitch_obj = pitch_min + (pitch_max - pitch_min)/(channel_max - channel_min) * (channel2 - channel_min);

    float diff_roll = euler(0) - roll_obj;
    float diff_pitch = euler(1) - pitch_obj;

    Eigen::Vector2f duty; //duty[0]がrollのduty比、duty[1]がpitchのduty比
    duty[0] = g_roll*diff_roll;
    duty[1] = g_pitch*diff_pitch;

    return duty;
}