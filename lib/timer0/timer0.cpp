#include "timer0.h"

void InitTimer0(uint8_t WGM = 0, uint8_t OCRAInit = 0xFF, uint8_t OCRBInit = 0xFF, uint8_t COMA = 0, uint8_t COMB = 0) {
    // Initialize Timer Control Registers
    TCCR0A = 0;
    TCCR0B = 0;
    
    // Set Waveform Generation Mode
    WGM &= 0b111;
    TCCR0A |= ((WGM & 0b011) << WGM00);
    TCCR0B |= (((WGM & 0b100) >> 2) << WGM02);

    // Set Compare Output Mode
    COMA &= 0b11;
    COMB &= 0b11;
    TCCR0A |= (COMA << COM0A0) | (COMB << COM0B0);

    // Set Initial Timer Counter Register value
    TCNT0 = 0;

    // Set Initial TOPs (Output Compare Registers)
    OCR0A = OCRAInit;
    OCR0B = OCRBInit;
}

void OutputModeTimer0(uint8_t COMA, uint8_t COMB) {
    // Set Compare Output Mode
    COMA &= 0b11;
    COMB &= 0b11;
    TCCR0A |= (COMA << COM0A0) | (COMB << COM0B0);
}

void StartTimer0(uint8_t CS = 0b1) {
    // Set prescaler and start timer
    TCCR0B &= ~((1<<CS02) & (1<<CS01) & (1<<CS00));
    TCCR0B |= (CS<<CS00);
}

void StopTimer0(void) {
    // Clear prescaler and stop timer
    TCCR0B &= ~((1<<CS02) & (1<<CS01) & (1<<CS00));
}