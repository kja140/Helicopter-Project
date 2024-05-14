#ifndef ADC_MANAGEMENT_H_
#define ADC_MANAGEMENT_H_

#include <stdint.h>
#include <stdbool.h>

void initADC(void);
void ADCIntHandler(void);
uint16_t getADCAverage(void);

#endif // ADC_MANAGEMENT_H_
