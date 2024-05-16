// yaw_management.c
#include "yaw_management.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

static volatile int16_t yaw = YAW_DEFAULT;
static volatile int8_t previousState = PREVIOUS_STATE_DEFAULT;

static int16_t yawReference;
static int32_t gpioYawReference;
static volatile int8_t calibrated = 0;


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

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOIntRegister(GPIO_PORTC_BASE, YawRefIntHandler);
    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_4);


}

int32_t getYaw(void) {
    return yaw;
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
// The handler for the yaw interrupt.
//
//*****************************************************************************
void
YawIntHandler(void)
{
    int32_t gpioPinStates;

    gpioPinStates = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);


//0x0002
    int8_t yawSignalB = (gpioPinStates & 0x02) >> 1; // Isolate bit 0
    int8_t yawSignalA = (gpioPinStates & 0x01); // Isolate bit 1


    if (previousState == PREVIOUS_STATE_DEFAULT) {
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
int8_t getCalibratedStatus (void){
    return calibrated;
}


void YawRefIntHandler (void) {
    gpioYawReference = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4);
    yawReference = (gpioYawReference & 0x04) >> 3;
    if (!calibrated) {
        if (yawReference == 0) {
            calibrated = 1;
            yaw = 0;
        }
    }
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4);

}
