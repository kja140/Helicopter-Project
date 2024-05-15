#ifndef PID_CONTROL_H_
#define PID_CONTROL_H_

#include <stdint.h>

void PIDUpdateAlt(int16_t setpoint, int16_t current_altitude);
void PIDUpdateYaw(int16_t set_orientation, int16_t current_orientation);

#endif // PID_CONTROL_H_
