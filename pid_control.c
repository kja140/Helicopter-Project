#include "pid_control.h"
#include "pwm_management.h"


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
    int8_t control = P + (pidStructAlt.I + dI) + D;

    pidStructAlt.I = (pidStructAlt.I + dI);
    pidStructAlt.previous_position = current_altitude;

    if (pidStructAlt.I > MAX_DUTY)  pidStructAlt.I = MAX_DUTY;
    if (pidStructAlt.I < MIN_DUTY)  pidStructAlt.I = MIN_DUTY;

    if (control > MAX_DUTY) control = MAX_DUTY;
    if (control < MIN_DUTY) control = MIN_DUTY;

    pidStructAlt.duty_cycle = control;
    setPWM_Main_DC (pidStructAlt.duty_cycle);
}

static PidStruct pidStructYaw = {5, 1, 1, 0, 0, 0};

void PIDUpdateYaw(int16_t set_orientation, int16_t current_orientation) {
    // Calculate shortest path error
    float error = set_orientation - current_orientation;
    if (error > 180) {
        error -= 360;
    } else if (error < -180) {
        error += 360;
    }

    float P = pidStructYaw.Kp * error;
    float dI = pidStructYaw.Ki * error * delta_t;
    float D = pidStructYaw.Kd * (current_orientation - pidStructYaw.previous_position) / delta_t;
    int16_t control = P + (pidStructYaw.I + dI) + D;

    pidStructYaw.I += dI;
    // Prevent integral windup
    if (pidStructYaw.I > MAX_DUTY) pidStructYaw.I = MAX_DUTY;
    if (pidStructYaw.I < MIN_DUTY) pidStructYaw.I = MIN_DUTY;

    pidStructYaw.previous_position = current_orientation;

    if (control > MAX_DUTY) control = MAX_DUTY;
    if (control < MIN_DUTY) control = MIN_DUTY;

    pidStructYaw.duty_cycle = control;
    setPWM_Tail_DC(pidStructYaw.duty_cycle);
}
