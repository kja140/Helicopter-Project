#include "pid_control.h"
#include "pwm_management.h"

/*
static float prev_altitude = 0;
static float I = 0;
const float Kp = 0.7;
const float Ki = 0.01;
const float Kd = 0.7;

*/
const float MAX_DUTY = 98.00;
const float MIN_DUTY = 2.00;
const float MAX_YAW_DUTY = 5.00;
const float MIN_YAW_DUTY = -5.00;
const float delta_t = 0.0208333333333333333333333333333; // 10/480
typedef struct {
    const float Kp, Ki, Kd;
    float I, previous_position;
    int8_t duty_cycle;
} PidStruct;

static PidStruct pidStructAlt = {7, 2, 0.05, 0, 0, 0};

void PIDUpdateAlt(int16_t setpoint, int16_t current_altitude) {
    float error = setpoint - current_altitude;
    float P = pidStructAlt.Kp * error;
    float dI = pidStructAlt.Ki * error * delta_t;
    float D = pidStructAlt.Kd * (pidStructAlt.previous_position - current_altitude) / delta_t;
    uint16_t duty_cycle = P + (pidStructAlt.I + dI) + D;

    pidStructAlt.I = (pidStructAlt.I + dI);
    pidStructAlt.previous_position = current_altitude;

    if (duty_cycle >= MAX_DUTY) {
        duty_cycle = MAX_DUTY;
    } else if (duty_cycle <= MIN_DUTY) {
        duty_cycle = MIN_DUTY;
    }

    pidStructAlt.duty_cycle = duty_cycle;
    setPWM_Main_DC (pidStructAlt.duty_cycle);
}

static PidStruct pidStructYaw = {0.1, 0.1, 0, 0, 0, 0};

void PIDUpdateYaw(int16_t set_orientation, int16_t current_orientation) {
    float error = set_orientation - current_orientation;
    float P = pidStructYaw.Kp * error;
    float dI = pidStructYaw.Ki * error * delta_t;
    float D = pidStructYaw.Kd * (pidStructYaw.previous_position - current_orientation) / delta_t;
    int16_t duty_cycle = P + (pidStructYaw.I + dI) + D;

    pidStructYaw.I = (pidStructYaw.I + dI);
    pidStructYaw.previous_position = current_orientation;

    if (duty_cycle >= MAX_YAW_DUTY) {
        duty_cycle = MAX_YAW_DUTY;
    } else if (duty_cycle <= MIN_YAW_DUTY) {
        duty_cycle = MIN_YAW_DUTY;
    }

    pidStructYaw.duty_cycle = duty_cycle;
    setPWM_Tail_DC (pidStructYaw.duty_cycle);
}
