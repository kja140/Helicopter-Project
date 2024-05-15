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
//#include "adc_management.h"


//*****************************************************************************
// Constants
//*****************************************************************************


#define SAMPLE_RATE_HZ 480



//*****************************************************************************
// Global variables
//*****************************************************************************
static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)

//static int32_t current_time_nano;
//static int32_t last_update_time;

volatile int8_t pid_flag = 0;

#define BUF_SIZE 60

static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static volatile counter;

//*****************************************************************************
//
// The interrupt handler for the for SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //current_time_nano += 2083333;
    if (counter >= 10) {
        pid_flag = 1;
        counter = 0;
    } else
        counter++;

    // Initiate a conversion
    //
    ADCProcessorTrigger(ADC0_BASE, 3);
    //
    // Poll the buttons
    updateButtons();
    //
}

//*****************************************************************************
//
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//
//*****************************************************************************
void
ADCIntHandler(void)
{
    uint32_t ulValue;

    //
    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);
    //
    // Place it in the circular buffer (advancing write index)
    writeCircBuf (&g_inBuffer, ulValue);
    //
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);
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
void
initADC (void)
{
    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
                             ADC_CTL_END);

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);

    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);
}

uint16_t getADCAverage(void) {
    uint32_t sum = 0;
    uint16_t i;
    for (i = 0; i < BUF_SIZE; i++)
        sum += readCircBuf(&g_inBuffer);
    return (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;
}


int main(void)
{
    uint16_t adcMean;
    int32_t percentagePower;
    int32_t duty_cycle = 2;

    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // 20MHz
    SysCtlPWMClockSet(PWM_DIVIDER_CODE); //PWM Clock rate 10MHz

    // As a precaution, make sure that the peripherals used are reset
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO); // Used for PWM output
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);  // Main Rotor PWM
    SysCtlPeripheralReset (UP_BUT_PERIPH);        // UP button GPIO
    SysCtlPeripheralReset (DOWN_BUT_PERIPH);      // DOWN button GPIO

    initButtons ();  // Initialises 4 pushbuttons (UP, DOWN, LEFT, RIGHT)
    initPWM ();
    initClock ();
    initADC ();
    initYaw ();
    initDisplay ();
    initCircBuf (&g_inBuffer, BUF_SIZE);
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



        if ((checkButton (LEFT) == PUSHED)) {
            landed = 1;
        }

        if (landed == 1) {
            helicopterLandedAltitude = adcMean;
            //helicopterMaxAltitude = (adcMean - 1241);
            landed = 0;
        }

        percentagePower = ((helicopterLandedAltitude - adcMean)  * 100) / 1241;
        //if (pid_flag == 1) {
        //    pid_flag = 0;
            duty_cycle = PIDUpdate(100, percentagePower);
            setPWM (PWM_MAIN_RATE_HZ, duty_cycle, 1);
        //}


        if ((checkButton (UP) == PUSHED)) {
            if (displayState < 2) {
                displayState++;
            } else {
                displayState = 0;
            }
            OrbitOledClear();
        }

        if (displayState == 0) {

            displayPercentageVal(percentagePower);
        } else if (displayState == 1) {
            displayMeanAndYaw(adcMean, helicopterLandedAltitude);
        }

        SysCtlDelay (SysCtlClockGet() / 60);  // Update display at ~ 20 Hz
    }
}
/*


volatile int8_t pid_flag = 0;

//*****************************************************************************
//
// The interrupt handler for the for SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //current_time_nano += 2083333;
    pid_flag = 1;
    // Initiate a conversion
    //
    ADCProcessorTrigger(ADC0_BASE, 3);
    //
    // Poll the buttons
    updateButtons();
    //
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

int main(void) {
    uint16_t adcMean;
    int32_t percentagePower;
    int32_t duty_cycle = 2;
    float gain = 0;

    initClock();
    initADC();
    initYaw();
    initDisplay();
    initButtons();
    initPWM();


    // Enable interrupts to the processor.
    IntMasterEnable();
    SysCtlDelay (SysCtlClockGet() / 24);
    while (1) //each volt is 1,241 units
    {
        //
        // Background task: calculate the (approximate) mean of the values in the
        // circular buffer and display it, together with the sample number.


        //if (pid_flag == 1) {
            pid_flag = 0;
            gain = PIDUpdate(80, percentagePower);
            duty_cycle *= gain;
            setPWM (PWM_MAIN_RATE_HZ, duty_cycle, 1);
        //}

        adcMean = getADCAverage();

        uint8_t displayState = 0;
        uint16_t helicopterLandedAltitude = 0;

        if (checkButton (LEFT) == PUSHED) {
            helicopterLandedAltitude = adcMean;
        }

        if ((checkButton (UP) == PUSHED)) {
            if (displayState < 2) {
                displayState++;
            } else {
                displayState = 0;
            }
            OrbitOledClear();
        }

        if (displayState == 0) {
            percentagePower = ((helicopterLandedAltitude - adcMean)  * 100) / 1241;
            displayPercentageVal(percentagePower);
        } else if (displayState == 1) {
            displayMeanAndYaw(adcMean, helicopterLandedAltitude);
        }

        SysCtlDelay (SysCtlClockGet() / 60);  // Update display at ~ 20 Hz
    }
}
*/
