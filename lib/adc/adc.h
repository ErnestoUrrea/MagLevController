#ifndef ADC_H
#define ADC_H

#include <avr/io.h>
#include <avr/interrupt.h>

void InitADC(uint8_t REFS, uint8_t LAR, uint8_t MUX, uint8_t ADPRE);

void StartADCAutoTrigger(uint8_t ATS);

void DisableADC(void);

void SetADCChannel(uint8_t MUX);

#endif

