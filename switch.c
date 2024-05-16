#include "switch.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"

bool sw1_prev = false;
bool sw2_prev = false;


void
slider_init(void) {
    // SW1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

    // SW2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
}

bool
sw1_is_up (void)
{
    return GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7) == GPIO_PIN_7;
}

bool
sw2_is_up (void)
{
    return GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6;
}

bool
sw1_changed (void)
{
    bool current = sw1_is_up();
    bool changed = sw1_prev != current;
    sw1_prev = current;
    return changed;
}

bool
sw2_changed (void)
{
    bool current = sw2_is_up();
    bool changed = sw2_prev != current;
    sw2_prev = current;
    return changed;
}
