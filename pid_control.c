#include "pid_control.h"

static float prev_altitude = 0;
static float I = 0;
const float Kp = 0.7;
const float Ki = 0.01;
const float Kd = 0.7;
const float MAX_DUTY = 98.00;
const float MIN_DUTY = 2.00;
float delta_t = 10/480;

float PIDUpdate(float setpoint, float current_altitude) {

    float error = setpoint - current_altitude;
    float P = Kp * error;
    //float dI = Ki * error * delta_t;
    //float D = Kd * (prev_altitude - current_altitude) / (delta_t);
    float duty_cycle = P; //+ (I + dI) + D;
    //if (duty_cycle > MAX_DUTY) {
    //    duty_cycle = MAX_DUTY;
    //} else if (duty_cycle < MIN_DUTY) {
    //    duty_cycle = MIN_DUTY;
    //}
    //I = (I + dI);
    /*
    if (I > 100) {
        I = 100;
    } else if (I < 0) {
        I = 0;
    }
    */
    //prev_altitude = current_altitude;

    if (duty_cycle >= MAX_DUTY) {
        return MAX_DUTY;
    } else if (duty_cycle <= MIN_DUTY) {
        return MIN_DUTY;
    } else {
        return duty_cycle;
    }
}
