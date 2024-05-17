//*****************************************************************************
//
// ADCdemo1.c - Simple interrupt driven program which samples with AIN0
//
// Author:  P.J. Bones  UCECE
// Last modified:   8.2.2018
//
//*****************************************************************************
// Based on the 'convert' series from 2016
//*****************************************************************************
//#include <stdint.h>
//#include <stdbool.h>
//#include "inc/hw_types.h"
//#include "driverlib/pwm.h"
//#include "driverlib/gpio.h"
//#include "driverlib/sysctl.h"
//#include "driverlib/systick.h"
//#include "driverlib/interrupt.h"
//#include "driverlib/debug.h"
//#include "utils/ustdlib.h"
#include "circBufT.h"
//#include "OrbitOLED/OrbitOLEDInterface.h"
#include "OrbitOLED/lib_OrbitOled/OrbitOled.h"
//#include "driverlib/pin_map.h" //Needed for pin configure

#include "inc/hw_memmap.h"
#include "buttons4.h"
#include "driverlib/adc.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"


#include "display_management.h"
#include "yaw_management.h"
#include "pwm_management.h"
#include "pid_control.h"
#include "switch.h"
#include "adc_management.h"
#include "adc_management.h"
#include "uart.h"


//*****************************************************************************
// Constants
//*****************************************************************************


#define SAMPLE_RATE_HZ 480
#define MIN_ALTITUDE 0
#define MAX_ALTITUDE 100
#define BUF_SIZE 60
//#define DESIRED_ALTITUDE 80



//*****************************************************************************
// Global variables
//*****************************************************************************
static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)

volatile bool PIDFlag = 0;
volatile bool ADCFlag = 0;
//static int32_t current_time_nano;
//static int32_t last_update_time;

volatile int8_t pid_flag = 0;
static volatile int8_t counter = 0;
int8_t set_altitude;
int32_t set_orientation = 0;
volatile int16_t percentagePower;
uint8_t slowTick = 0;
//*****************************************************************************
//
// The interrupt handler for the for SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{

    static uint8_t tickCount = 0;
    static int8_t counter = 0;
    const uint8_t ticksPerSlow = SAMPLE_RATE_HZ / SLOWTICK_RATE_HZ;
    // Initiate a conversion
    if (counter >= 10) {
        pid_flag = 1;
        counter = 0;
    } else
        counter++;

    ADCProcessorTrigger(ADC0_BASE, 3);
    //
    // Poll the buttons
    updateButtons();
    //
    if (tickCount >= ticksPerSlow)
    {                       // Signal a slow tick
        slowTick = 1;
        tickCount = 0;
    } else {
        tickCount++;
    }
}

//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display
//*****************************************************************************
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);

    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}





int main(void)
{
    IntMasterDisable();
    uint16_t adcMean;

    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // 20MHz
    SysCtlPWMClockSet(PWM_DIVIDER_CODE); //PWM Clock rate 10MHz

    // As a precaution, make sure that the peripherals used are reset
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO); // Used for PWM output
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);  // Main Rotor PWM
    SysCtlPeripheralReset (UP_BUT_PERIPH);        // UP button GPIO
    SysCtlPeripheralReset (DOWN_BUT_PERIPH);      // DOWN button GPIO

    initClock ();
    initButtons ();  // Initialises 4 pushbuttons (UP, DOWN, LEFT, RIGHT) and sw1 and sw2
    initDisplay ();
    slider_init();
    while (sw1_is_up ()) {
        displaySetupScreen();
    }
    OrbitOledClear();
    initPWM ();
    initADC ();
    initYaw ();
    initCircBuf (&g_inBuffer, BUF_SIZE);
    initialiseUSB_UART ();

    bool landed = 1;
    bool landing = 0;
    bool altCalibrated = 1;

    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);


    //uint8_t displayState = 0;
    uint16_t helicopterLandedAltitude = 0;


    set_altitude = 0;
    //
    // Enable interrupts to the processor.
    IntMasterEnable();
    SysCtlDelay (SysCtlClockGet() / 24);
    while (1) //each volt is 1,241 units
    {



        adcMean = getADCAverage();
        percentagePower = ((helicopterLandedAltitude - adcMean)  * 100) / 1241;

        if (altCalibrated == 1) {
            helicopterLandedAltitude = adcMean;
            altCalibrated = 0;
            displayLandedScreen ();
        }

        //if (landed == 1) {
        //    displayLandedScreen ();
        //}


        if (checkButton(DOWN) == PUSHED) {
            if (set_altitude - 10 < MIN_ALTITUDE)
                set_altitude = MIN_ALTITUDE;
            else
                set_altitude -= 10;
        }

        if (checkButton(UP) == PUSHED) {
            if (set_altitude + 10 > MAX_ALTITUDE)
                set_altitude = MAX_ALTITUDE;
            else
                set_altitude += 10;
        }

        if (checkButton(RIGHT) == PUSHED) {
            if (set_orientation + 15 >= 180)
                set_orientation -= 345;

            else
                set_orientation += 15;
        }

        if (checkButton(LEFT) == PUSHED) {
            if (set_orientation - 15 <= -180)
                set_orientation += 345;
            else
                set_orientation -= 15;
        }
        if (slowTick == 1) {
              slowTick = false;
              // Form and send a status message to the console
              char statusStr[100];
              usprintf(statusStr, "Altitude%2d | Desired Altitude%3d | Yaw%4d | Desired Yaw%4d Main Duty%4d | Tail Duty%4d \r\n", percentagePower, set_altitude, getYaw(), set_orientation, getPWM_Main_DC(), getPWM_Tail_DC());
              UARTSend (statusStr);
        }

        if (landed == 0)
            displayYaw_Altitude_PWMMain_PWMTail(percentagePower);

        //if (sw1_changed () == 0)


        if ((sw1_is_up () == 1) && landing == 0) {
            landed = 0;
            if (pid_flag == 1) {
                pid_flag = 0;
                if (getCalibratedStatus() == 0) {
                    OrbitOledClear();
                    displayCalibratingScreen();
                    do {
                        setPWM_Tail_DC(30);
                        //setPWM_Main_DC(50);
                        //PIDUpdateAlt(1, percentagePower);
                        //SysCtlDelay(2 * SysCtlClockGet());
                    } while (getCalibratedStatus() == 0);
                    OrbitOledClear();
                } else {
                    PIDUpdateYaw(set_orientation, calculateYawDegrees(getYaw()) / 10);
                }
                PIDUpdateAlt(set_altitude, percentagePower);
            }
        } else if (sw1_is_up () == 0) {
            OrbitOledClear();
            if (percentagePower > 1) {
                landing = 1;
                displayLandedingScreen(percentagePower);
                PIDUpdateYaw(0, calculateYawDegrees(getYaw()) / 10);
                PIDUpdateAlt(0, percentagePower);
            } else if (percentagePower < 1) {
                landing = 0;
                landed = 1;
                displayLandedScreen ();
            }
        }

        if (sw2_changed ())
            SysCtlReset ();

    }
}
