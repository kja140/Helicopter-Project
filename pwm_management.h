// pwm_management.h
#ifndef PWM_CONTROL_H_
#define PWM_CONTROL_H_

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/pwm.h"

#define PWM_MAIN_PERIPH_PWM  SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_RATE_HZ 250 // PWM frequency
#define PWM_DIVIDER_CODE SYSCTL_PWMDIV_2 // PWM clock pre-scaler (1/2)
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_BASE        PWM0_BASE



void initPWM (void);
void setPWM (uint32_t u32Freq, uint32_t u32Duty, int is_main_rotor);


#endif // PWM_CONTROL_H_
