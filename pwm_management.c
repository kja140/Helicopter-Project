// pwm_control.c
#include "pwm_management.h"

#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"





// PWM configuration
//#define PWM_MAIN_RATE_HZ 100 // PWM frequency
#define PWM_START_DC 40 // PWM duty cycle (0%)
#define PWM_DIVIDER 2
static uint32_t ui32Period;

#define PWM_TAIL_RATE_HZ 200 // PWM frequency
#define PWM_TAIL_DC 40 // PWM duty cycle (40%)

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7


#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5

#define PWM_TAIL_BASE        PWM1_BASE
#define PWM_TAIL_GEN         PWM_GEN_2
#define PWM_TAIL_OUTNUM      PWM_OUT_5
#define PWM_TAIL_OUTBIT      PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM  SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN    GPIO_PIN_1

static int8_t duty_cycle_main;
static int8_t duty_cycle_tail;



void
initPWM (void)
{
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
    //last_update_time = current_time_nano;
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);


    ui32Period = SysCtlClockGet() / PWM_DIVIDER / PWM_MAIN_RATE_HZ;

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);

    // Set the initial PWM parameters
    setPWM_Main_DC (PWM_START_DC);

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);




    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);


    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);

    // Set the initial PWM parameters
    setPWM_Tail_DC (PWM_TAIL_DC);

    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);

}

/********************************************************
 * Function to set the freq, duty cycle of M0PWM7
 ********************************************************/
void
setPWM_Main_DC (int8_t duty_cycle)
{
    duty_cycle_main = duty_cycle;
    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM, ui32Period * duty_cycle / 100);
}
    /*else {
        // Calculate the PWM period corresponding to the freq.
        uint32_t ui32Period =
            SysCtlClockGet() / PWM_DIVIDER / ui32Freq;

        PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
        PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM,
            ui32Period * PWM_TAIL_DC / 100);
    }
    */


void
setPWM_Tail_DC (int8_t duty_cycle)
{
    duty_cycle_tail = duty_cycle;
    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM, ui32Period * PWM_TAIL_DC / 100);
}


int8_t
getPWM_Main_DC (void) {
    return duty_cycle_main;
}

int8_t
getPWM_Tail_DC (void) {
    return duty_cycle_tail;
}
