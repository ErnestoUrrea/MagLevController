#ifndef TIMER0_H
#define TIMER0_H

#include <avr/io.h>
#include <avr/interrupt.h>

void InitTimer0(uint8_t WGM, uint8_t OCRAInit, uint8_t OCRBInit, uint8_t COMA, uint8_t COMB);

void OutputModeTimer0(uint8_t COMA, uint8_t COMB);

void StartTimer0(uint8_t CS);

void StopTimer0(void);

#endif