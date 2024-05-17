// display_management.h
#ifndef DISPLAY_MANAGEMENT_H_
#define DISPLAY_MANAGEMENT_H_

#include <stdint.h>


void initDisplay(void);
void displayMeanAndYaw(uint16_t meanVal, uint32_t helicopterLandedAltitude);
void displayPercentageVal(int32_t perVal);
void displayYaw_Altitude_PWMMain_PWMTail(int32_t percentagePower, int32_t set_altitude);
void displaySetupScreen (void);
void
displayCalibratingScreen (void);

#endif // DISPLAY_MANAGEMENT_H_
