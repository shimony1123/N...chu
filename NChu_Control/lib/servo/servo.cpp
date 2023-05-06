#include "servo.hpp"

//constructor
servo::servo(int channel_in, int freq_in, int resolution_in, int Pin_in, int duty_ratio_in){
    channel = channel_in;
    freq = freq_in;
    resolution = resolution_in;
    Pin = Pin_in;
    duty_ratio = duty_ratio_in;
    duty = duty_ratio*resolution;
    duty_ratio_max = 0.2;
    duty_ratio_min = 0.1;

    ledcSetup(channel,freq,resolution);
    ledcAttachPin(Pin,channel);
}

float servo::calculate_duty_ratio(int power, float duty_ratio_max, float duty_ratio_min){
    float margin = duty_ratio_max -duty_ratio_min;
    float DR = margin/(10.0f-0.0f) * (power-0.0f)+ duty_ratio_min;
    //duty比制限
    if (DR<duty_ratio_min){
        DR = duty_ratio_min;
    }
    else if (DR>duty_ratio_max){
        DR = duty_ratio_max;
    }

    return DR;
}