#include <avr/io.h>
#include <avr/interrupt.h>

void InitTimer1(uint8_t WGM = 0, uint16_t ICRInit = 0xFF, uint16_t OCRAInit = 0xFFFF, uint16_t OCRBInit = 0xFFFF, uint8_t COMA = 0, uint8_t COMB = 0) {
    // Initialize Timer Control Registers
    TCCR1A = 0;
    TCCR1B = 0;
    
    // Set Waveform Generation Mode
    WGM &= 0b1111;
    TCCR1A |= ((WGM & 0b011) << WGM10);
    TCCR1B |= (((WGM & 0b1100) >> 2) << WGM12);

    // Set Compare Output Mode
    COMA &= 0b11;
    COMB &= 0b11;
    TCCR1A |= (COMA << COM0A0) | (COMB << COM0B0);

    // Set Initial Timer Counter Register value
    TCNT1 = 0;

    // Set Initial TOP
    ICR1 = ICRInit;

    // Set Initial Output Compare Registers
    OCR1A = OCRAInit;
    OCR1B = OCRBInit;
}

void OutputModeTimer1(uint8_t COMA, uint8_t COMB) {
    // Set Compare Output Mode
    COMA &= 0b11;
    COMB &= 0b11;
    TCCR0A |= (COMA << COM1A0) | (COMB << COM1B0);
}

void StartTimer1(uint8_t CS = 0b1) {
    // Set prescaler and start timer
    TCCR1B &= ~((1<<CS12) & (1<<CS11) & (1<<CS10));
    TCCR1B |= (CS<<CS10);
}

void StopTimer1(void) {
    // Clear prescaler and stop timer
    TCCR1B &= ~((1<<CS12) & (1<<CS11) & (1<<CS10));
}