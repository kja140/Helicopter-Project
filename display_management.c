// display_management.c
#include "display_management.h"

#include "OrbitOLED/OrbitOLEDInterface.h"
#include "utils/ustdlib.h"
#include "yaw_management.h"
#include "pwm_management.h"

void
initDisplay (void)
{
    // intialise the Orbit OLED display
    OLEDInitialise ();
}

//*****************************************************************************
//
// Function to display the mean ADC value (10-bit value, note) and sample count.
//
//*****************************************************************************
void
displayMeanAndYaw(uint16_t meanVal, uint32_t helicopterLandedAltitude)
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
    int16_t yawd = calculateYawDegrees(getYaw());
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

void
displaySetupScreen (void) {
    //char string[17];  // 16 characters across the display
    OLEDStringDraw ("Switch 1: UP", 0, 0);
    OLEDStringDraw ("Expected: DOWN", 0, 1);
}

void
displayYaw_Altitude_PWMMain_PWMTail(int32_t alitude) {
    char string[17];  // 16 characters across the display

    int16_t yawd = calculateYawDegrees(getYaw());
    int16_t yawdwhole = yawd / 10;
    int16_t yawddec = abs(yawd % 10);
    int8_t dutyMain = getPWM_Main_DC();
    int8_t dutyTail = getPWM_Tail_DC();
    char deg=248;
    usnprintf(string, sizeof(string), "Yaw: %d.%d% deg   ", yawdwhole, yawddec);
    OLEDStringDraw(string, 0, 0);

    usnprintf(string, sizeof(string), "Altitude: %4d%%   ", alitude);
    OLEDStringDraw(string, 0, 1);

    usnprintf(string, sizeof(string), "DC Main: %4d%%   ", dutyMain);
    OLEDStringDraw(string, 0, 2);

    usnprintf(string, sizeof(string), "DC Tail: %4d%%   ", dutyTail);
    OLEDStringDraw(string, 0, 3);
}
