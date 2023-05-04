#pragma once
#include <sbus.h>
#include <Wire.h>

struct servo{
    float e_max;
    float e_min;
    float duty_max;
    float duty_min;

    int channelS;
    int freq;
    int resolution;
    int Pin;
    int duty;

    //constructor
    servo(int channel_in,int freq_in,int resolution_in,int Pin_in,int duty_in);
};