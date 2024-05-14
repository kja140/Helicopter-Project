// display_management.h
#ifndef DISPLAY_MANAGEMENT_H_
#define DISPLAY_MANAGEMENT_H_

#include <stdint.h>


void initDisplay(void);
void displayMeanAndYaw(uint16_t meanVal, uint32_t helicopterLandedAltitude);
void displayPercentageVal(int32_t perVal);

#endif // DISPLAY_MANAGEMENT_H_
