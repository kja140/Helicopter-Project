// yaw_management.h
#ifndef YAW_MANAGEMENT_H_
#define YAW_MANAGEMENT_H_

#include <stdint.h>
#include <stdbool.h>


#define PREVIOUS_STATE_DEFAULT 0x50
#define YAW_DEFAULT 0

void initYaw(void);
int16_t getYaw(void);
int16_t calculateYawDegrees(int16_t yaw);
void YawIntHandler(void);

#endif // YAW_MANAGEMENT_H_
