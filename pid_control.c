#include "pid_control.h"

static float prev_altitude = 0;
static float I = 0;
const float Kp = 0.2;
const float Ki = 0;
const float Kd = 0;
const float MAX_DUTY = 98.00;
const float MIN_DUTY = 2.00;

float PIDUpdate(float setpoint, float current_altitude) {
    float delta_t = 0.002083;
    float error = setpoint - current_altitude;
    float P = Kp * error;
    float dI = Ki * error * delta_t;
    float D = Kd * (prev_altitude - current_altitude) / delta_t;
    float gain = P + (I + dI) + D;

    I = (I + dI);
    if (I > 100) {
        I = 100;
    } else if (I < 0) {
        I = 0;
    }
    prev_altitude = current_altitude;

    if (gain > MAX_DUTY) {
        gain = MAX_DUTY;
    } else if (gain < MIN_DUTY) {
        gain = MIN_DUTY;
    }
    return gain;
}
