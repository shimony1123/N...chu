#include <Arduino.h>
#include "servo.hpp"

//constructor
servo::servo(int channel_in,int freq_in,int resolution_in,int Pin_in,int duty_in){
    channel = channel_in;
    freq = freq_in;
    resolution = resolution_in;
    Pin = Pin_in;
    duty = duty_in;
    ledcSetup(channel, freq, resolution);
    ledcAttachPin(Pin, channel);
}