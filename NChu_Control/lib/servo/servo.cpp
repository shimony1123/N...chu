#include "servo.hpp"
#include <cmath>

//constructor
servo::servo(uint8_t channel_in, int freq_in, uint8_t bit_num_in, int Pin_in, float duty_ratio_in){
    channel = channel_in;
    freq = freq_in;
    bit_num = bit_num_in;
    Pin = Pin_in;
    duty_ratio = duty_ratio_in;
    duty_ratio_max = 0.2;
    duty_ratio_min = 0.1;
    
    resolution = pow(2,bit_num_in);
    duty = duty_ratio_in*resolution;

    ledcSetup(channel,freq,bit_num);
    ledcAttachPin(Pin,channel);
}

float servo::calculate_duty_ratio(int power, float duty_ratio_max, float duty_ratio_min){
    float margin = duty_ratio_max -duty_ratio_min;
    float DR = margin/(10.0f-0.0f) * (power-0.0f) + duty_ratio_min;
    //duty比制限
    if (DR<duty_ratio_min){
        DR = duty_ratio_min;
    }
    else if (DR>duty_ratio_max){
        DR = duty_ratio_max;
    }

    return DR;
}