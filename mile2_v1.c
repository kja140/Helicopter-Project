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

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "circBufT.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "OrbitOLED/lib_OrbitOled/OrbitOled.h"

#include "buttons4.h"
#include "driverlib/pin_map.h" //Needed for pin configure

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 60
#define SAMPLE_RATE_HZ 480
//#define PREVIOUS_STATE_DEFAULT 0x50
#define YAW_DEFAULT 0

//*****************************************************************************
// Global variables
//*****************************************************************************
static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;    // Counter for the interrupts
static volatile int16_t yaw;
static volatile int8_t previousState;
static int32_t current_time_nano;
static int32_t last_update_time;
static float prev_altitude = 0;
static float I = 0;
const float Kp = 0.2;
const float Ki = 0;
const float Kd = 0;
volatile int8_t pid_flag = 0;




//pwm:
/**********************************************************
 * Generates a single PWM signal on Tiva board pin J4-05 =
 * PC5 (M0PWM7).  This is the same PWM output as the
 * helicopter main rotor.
 **********************************************************/

/**********************************************************
 * Constants
 **********************************************************/
// PWM configuration
#define PWM_MAIN_RATE_HZ 100 // PWM frequency
#define PWM_START_DC 0 // PWM duty cycle (10%)
#define PWM_DIVIDER_CODE SYSCTL_PWMDIV_2 // PWM clock pre-scaler (1/2)
#define PWM_DIVIDER 2

#define PWM_TAIL_RATE_HZ 200 // PWM frequency
#define PWM_TAIL_DC 40 // PWM duty cycle (10%)

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_BASE        PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM  SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
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

#define MAX_DUTY   98.00
#define MIN_DUTY   2.00


/*******************************************
 *      Local prototypes
 *******************************************/
void initialisePWM (void);
void setPWM (uint32_t u32Freq, uint32_t u32Duty, int is_main_rotor);


void
initialisePWM (void)
{
    last_update_time = current_time_nano;
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    uint32_t ui32MainPeriod = SysCtlClockGet() / PWM_DIVIDER / PWM_MAIN_RATE_HZ;

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32MainPeriod);

    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM, ui32MainPeriod * PWM_START_DC / 100);
    // Set the initial PWM parameters
    //setPWM (PWM_MAIN_RATE_HZ, PWM_MAIN_DC, 1);

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);




    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    uint32_t ui32TailPeriod = SysCtlClockGet() / PWM_DIVIDER / PWM_TAIL_RATE_HZ;

    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32TailPeriod);

    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM, ui32TailPeriod * PWM_TAIL_DC / 100);
    // Set the initial PWM parameters
    setPWM (PWM_TAIL_RATE_HZ, PWM_TAIL_DC, 0);

    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
}

/********************************************************
 * Function to set the freq, duty cycle of M0PWM7
 ********************************************************/
void
setPWM (uint32_t ui32Freq, uint32_t duty_cycle, int is_main_rotor)
{
    if (is_main_rotor) {
        // Calculate the PWM period corresponding to the freq.
        uint32_t ui32Period =
            SysCtlClockGet() / PWM_DIVIDER / ui32Freq;

        PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
        PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM,
            ui32Period * duty_cycle / 100);
    } else {
        // Calculate the PWM period corresponding to the freq.
        uint32_t ui32Period =
            SysCtlClockGet() / PWM_DIVIDER / ui32Freq;

        PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
        PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM,
            ui32Period * PWM_TAIL_DC / 100);
    }
}



float PIDUpdate (float setpoint, float current_altitude) {

    float delta_t = 0.002083;
            //(current_time_nano*1000000) - last_update_time;
    //(last_update_time/1000000) = current_time_nano;

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

//:pwm

//*****************************************************************************
//
// The interrupt handler for the for SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    current_time_nano += 2083333;
    pid_flag = 1;
    // Initiate a conversion
    //
    ADCProcessorTrigger(ADC0_BASE, 3);
    g_ulSampCnt++;
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
//
// The handler for the yaw interrupt.
//
//*****************************************************************************
void
YawIntHandler(void)
{
    int32_t gpioPinStates;

    gpioPinStates = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
 //   gpioYawReference = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4);
//0x0002
    int8_t yawSignalB = (gpioPinStates & 0x02) >> 1; // Isolate bit 0
    int8_t yawSignalA = (gpioPinStates & 0x01); // Isolate bit 1


    if (previousState == 0x50) {
        previousState = gpioPinStates;
    }
switch (previousState) {
    case 0:
        if (yawSignalA == 0 && yawSignalB == 1) {
            yaw++;
            previousState = 0b01;
        } else if (yawSignalA == 1 && yawSignalB == 0){
            yaw--;
            previousState = 0b10;
        }
        break;
    case 1:
        if (yawSignalA == 1 && yawSignalB == 1) {
            yaw++;
            previousState = 0b11;
        } else if (yawSignalA == 0 && yawSignalB == 0) {
            yaw--;
            previousState = 0b00;
        }
        break;
    case 3:
        if (yawSignalA == 1 && yawSignalB == 0) {
            yaw++;
            previousState = 0b10;
        } else if (yawSignalA == 0 && yawSignalB == 1){
            yaw--;
            previousState = 0b01;
        }
        break;
    case 2:
        if (yawSignalA == 0 && yawSignalB == 0) {
            yaw++;
            previousState = 0b00;
        } else if (yawSignalA == 1 && yawSignalB == 1) {
            yaw--;
            previousState = 0b11;
        }
        break;
    default:
        break;
}
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
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

void
initDisplay (void)
{
    // intialise the Orbit OLED display
    OLEDInitialise ();
}


void
initYaw(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
    }
    GPIOIntRegister(GPIO_PORTB_BASE, YawIntHandler);
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

}

int16_t calculateYawDegrees(int16_t yaw) {
    int16_t yawd = ((yaw * 3600) / 448) % 3600;
    if (yawd > 1800) {
        yawd -= 3600;
    } else if (yawd < -1792) {
        yawd += 3600;
    }
    return yawd;
}

//*****************************************************************************
//
// Function to display the mean ADC value (10-bit value, note) and sample count.
//
//*****************************************************************************
void
displayMeanVal(uint16_t meanVal, uint32_t helicopterLandedAltitude)
{
    char string[17];  // 16 characters across the display

    OLEDStringDraw("Altitude and Yaw", 0, 0);

    // Display the mean ADC value
    usnprintf(string, sizeof(string), "MeanADC=%4d  ", meanVal);
    OLEDStringDraw(string, 0, 1);

    // Display the helicopter landed altitude
    usnprintf(string, sizeof(string), "LandedADC=%5d  ", helicopterLandedAltitude);
    OLEDStringDraw(string, 0, 2);

    // Display the yaw

    int16_t yawd = calculateYawDegrees(yaw);
    int16_t yawdwhole = yawd / 10;
    int16_t yawddec = abs(yawd % 10);

    usnprintf(string, sizeof(string), "Yaw=%d.%d deg   ", yawdwhole, yawddec);
    OLEDStringDraw(string, 0, 4);
}



void
displayPercentageVal(int32_t perVal)  //displayMeanVal(uint16_t meanVal, uint32_t count)
{
    char string[17];  // 16 characters across the display

    OLEDStringDraw ("Altitude Percentage", 0, 0);

    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf (string, sizeof(string), "Altitude = %4d%%  ", perVal);
    // Update line on display.
    OLEDStringDraw (string, 0, 1);
}




int main(void)
{
    uint16_t i;
    int32_t sum;
    uint16_t adcMean;
    int32_t percentagePower;
    int32_t duty_cycle = 2;
    int32_t gain = 0;


    //pwm:
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // As a precaution, make sure that the peripherals used are reset
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO); // Used for PWM output
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);  // Main Rotor PWM
    SysCtlPeripheralReset (UP_BUT_PERIPH);        // UP button GPIO
    SysCtlPeripheralReset (DOWN_BUT_PERIPH);      // DOWN button GPIO

    initButtons ();  // Initialises 4 pushbuttons (UP, DOWN, LEFT, RIGHT)
    initialisePWM ();
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL |
    SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPWMClockSet(PWM_DIVIDER_CODE);

    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);


    // Initialisation is complete, so turn on the output.
    //:pwm

    previousState = 0x50;
    yaw = YAW_DEFAULT;

    initClock ();
    initADC ();
    initYaw ();
    initDisplay ();
    initCircBuf (&g_inBuffer, BUF_SIZE);
    bool landed = 1;

    uint8_t displayState = 0;
    uint16_t helicopterLandedAltitude = 0;



    //
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

        //setPWM (PWM_TAIL_RATE_HZ, PWM_TAIL_DC, 0);

        sum = 0;
        for (i = 0; i < BUF_SIZE; i++)
            sum = sum + readCircBuf (&g_inBuffer);
        // Calculate and display the rounded mean of the buffer contents
        adcMean = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;


        if ((checkButton (LEFT) == PUSHED)) {
            landed = 1;
        }

        if (landed == 1) {
            helicopterLandedAltitude = adcMean;
            //helicopterMaxAltitude = (adcMean - 1241);
            landed = 0;
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
            displayMeanVal(adcMean, helicopterLandedAltitude);
        }

        SysCtlDelay (SysCtlClockGet() / 60);  // Update display at ~ 20 Hz
    }
}
