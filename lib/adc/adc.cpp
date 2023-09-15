#include "adc.h"

void InitADC(uint8_t REFS = 0b01, uint8_t LAR = 0b0, uint8_t MUX = 0b0, uint8_t ADPRE = 0b0) {
    // Initialize registers
    ADMUX = 0;
    ADCSRA &= ~((1<<ADEN) & (1<<ADATE) & (1<<ADIE) & (1<<ADPS2) & (1<<ADPS1) & (1<<ADPS0));

    // Set Voltage Reference
    REFS &= 0b11;
    ADMUX |= (REFS << REFS0);

    // Set Left Adjust Bit
    LAR &= 0b1; 
    ADMUX |= (LAR << ADLAR);

    // Set Initial Input Channel
    MUX &= 0b1111;
    ADMUX |= (MUX << MUX0);

    // Set ADC Clock Prescaler
    ADPRE &= 0b111;
    ADCSRA |= (ADPRE << ADPS0);
}

//void StartADCSingleConvertion(void) {
    
//}

void StartADCAutoTrigger(uint8_t ATS = 0b0) {
    // ADC Enable, Interrupt Enable and Auto Trigger Enable
    ADCSRA |= (1<<ADEN)|(1<<ADIE)|(1<<ADATE);
    
    // Set Auto Trigger Source
    ATS &= 0b111;
    ADCSRA &= ~((1<<ADTS2) & (1<<ADTS1) & (1<<ADTS0));
    ADCSRB |= (ATS<<ADTS0);

    // Start First Convertion
    ADCSRA |= (1<<ADSC);
}

void DisableADC(void) {
    // Disable ADC
    ADCSRA &= ~((1<<ADEN) | (1<<ADIE) | (1<<ADATE));
}

void SetADCChannel(uint8_t MUX = 0b0) {
    // Set ADC Input Channel
    MUX &= 0b1111;
    ADMUX &= ~((1<<MUX3) & (1<<MUX2) & (1<<MUX1) & (1<<MUX0));
    ADMUX |= (MUX<<MUX0);
}