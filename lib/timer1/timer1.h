#ifndef TIMER1_H
#define TIMER1_H

#include <avr/io.h>
#include <avr/interrupt.h>

void InitTimer1(uint8_t WGM, uint16_t ICRInit, uint16_t OCRAInit, uint16_t OCRBInit, uint8_t COMA, uint8_t COMB);

void OutputModeTimer1(uint8_t COMA, uint8_t COMB);

void StartTimer1(uint8_t CS);

void StopTimer1(void);

#endif