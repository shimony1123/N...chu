#pragma once
#include <Arduino.h>

struct servo{
    uint8_t channel;
    int freq;//周波数=1/周期
    uint8_t bit_num;//解像度。今回は65536。
    int Pin;//ピン番号
    float duty_ratio;//PWM生成時に引数とするのはduty = duty_ratio*resolution(すなわちHIGHにするカウントの最大値)であることに留意。
    float duty_ratio_max;
    float duty_ratio_min;
    int duty;
    int resolution;

    //constructor
    servo(uint8_t channel_in, int freq_in, uint8_t bit_num_in, int Pin_in, float duty_ratio_in);

    //duty比を計算する関数
    //ここではduty比がduty_ratio_minのときにpower = 0, duty_ratio_maxのときにpower = 10であるとする。
    float calculate_duty_ratio(int power, float duty_ratio_max, float duty_ratio_min);
};