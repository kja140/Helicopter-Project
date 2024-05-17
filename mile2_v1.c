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

#include "driverlib/timer.h"
#include "inc/hw_ints.h"

#include "display_management.h"
#include "yaw_management.h"
#include "pwm_management.h"
#include "pid_control.h"
#include "adc_management.h"
#include "uart.h"


//*****************************************************************************
// Constants
//*****************************************************************************


#define SAMPLE_RATE_HZ 480
#define DESIRED_ALTITUDE 80



//*****************************************************************************
// Global variables
//*****************************************************************************
static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)

volatile bool PIDFlag = 0;
volatile bool ADCFlag = 0;
//static int32_t current_time_nano;
//static int32_t last_update_time;

volatile int8_t pid_flag = 0;

int32_t duty_cycle = 2;
volatile int16_t percentagePower;

#define BUF_SIZE 60

static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static volatile int8_t counter;

//*****************************************************************************
//
// The interrupt handler for the for SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    static uint8_t tickCount = 0;
    const uint8_t ticksPerSlow = SYSTICK_RATE_HZ / SLOWTICK_RATE_HZ;
    // Initiate a conversion
    //
    ADCProcessorTrigger(ADC0_BASE, 3);
    //
    // Poll the buttons
    updateButtons();
    //
    if (++tickCount >= ticksPerSlow)
    {                       // Signal a slow tick
        tickCount = 0;
        slowTick = true;
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




void PIDIntHandler (void) {
    PIDFlag = 1;
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    PIDUpdateAlt(DESIRED_ALTITUDE, percentagePower);
    int16_t yaw = getYaw();
    PIDUpdateYaw(0, yaw);
}

void initPID(void) {

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1)) {
    }

    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, 2000000); //20MHZ clock so interrupt frequency here is 20 MHZ / 20 000 ticks gives 1kHz (updates every 1ms)
    TimerIntRegister(TIMER1_BASE, TIMER_A, PIDIntHandler);
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER1_BASE, TIMER_A);
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
    while (sw1_is_up ()) {
        displaySetupScreen();
    }
    OrbitOledClear();
    initPWM ();
    initADC ();
    initYaw ();
    initPID ();
    initCircBuf (&g_inBuffer, BUF_SIZE);
    initialiseUSB_UART ();
    bool landed = 1;
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);


    uint8_t displayState = 0;
    uint16_t helicopterLandedAltitude = 0;



    //
    // Enable interrupts to the processor.
    IntMasterEnable();
    SysCtlDelay (SysCtlClockGet() / 24);
    while (1) //each volt is 1,241 units
    {



        adcMean = getADCAverage();
        percentagePower = ((helicopterLandedAltitude - adcMean)  * 100) / 1241;

        if (landed == 1) {
            helicopterLandedAltitude = adcMean;
            //helicopterMaxAltitude = (adcMean - 1241);
            landed = 0;
        }





        if ((checkButton (LEFT) == PUSHED)) {
            landed = 1;
        }





        if (slowTick)
        {
            slowTick = false;
            // Form and send a status message to the console
            usprintf(statusStr, "Altitude%2d | Desired Altitude%2d \r\n", percentagePower, DESIRED_ALTITUDE);
            UARTSend (statusStr);
        }

        //if (sw1_changed())
            // the DOWN position of the switch should indicate that the helicopter is either landed or in the process of landing.
            // Changing the slider switch from DOWN to UP when the helicopter is landed should cause the helicopter to take off.


        if ((checkButton (UP) == PUSHED)) {
            if (displayState < 2) {
                displayState++;
            } else {
                displayState = 0;
            }
            OrbitOledClear();
        }

        displayYaw_Altitude_PWMMain_PWMTail(percentagePower);

        if (displayState == 0) {

            displayPercentageVal(percentagePower);
        } else if (displayState == 1) {
            displayMeanAndYaw(adcMean, helicopterLandedAltitude);
        }

        SysCtlDelay (SysCtlClockGet() / 60);  // Update display at ~ 20 Hz
    }
}
