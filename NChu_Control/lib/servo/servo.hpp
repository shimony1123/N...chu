#pragma once
#include <Arduino.h>

struct servo{
    uint8_t channel;
    int freq;//周波数=1/周期
    uint8_t bit_num;//resolutionが2の何乗か
    int Pin;//ピン番号
    float duty_ratio;//duty比
    float duty_ratio_max;
    float duty_ratio_min;
    int duty;//解像度×duty比。PWM生成時に引数とするのはこれ。
    int resolution;//解像度

    //constructor
    servo(uint8_t channel_in, int freq_in, uint8_t bit_num_in, int Pin_in, float duty_ratio_in);

    //duty比を計算する関数
    //ここではduty比がduty_ratio_minのときにpower = 0, duty_ratio_maxのときにpower = 10であるとする。
    float calculate_duty_ratio(int power, float duty_ratio_max, float duty_ratio_min);
};