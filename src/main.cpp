#include <avr/io.h>
#include <avr/interrupt.h>

#include "timer0.h"
#include "timer1.h"
#include "adc.h"

volatile uint8_t flag = 0;
uint16_t ADC_value;

// ADC Interrupt Service Routine
ISR(ADC_vect) {
  // Read ADC Value
  ADC_value = ADC;
  // Set Flag for ADC Complete
  flag = 1;
  // Clear Timer Compare Match Flag A
  TIFR0 = (1 << OCF0A);
}

int main(void){
  // Set Pins PB1, PB2, PD2, PD6 to Outputs
  DDRB |= (1 << PB1) | (1 << PB2);
  DDRD |= (1 << PD2) | (1 << PD6);

  // Initialize Timers 0 and 1
  InitTimer0(0b010,       49, 0, 0b01, 0b00);  // WGM,      OCRA, OCRB, COMA, COMB
  InitTimer1(0b1110, 799,  0, 0, 0b10, 0b10);  // WGM, ICR, OCRA, OCRB, COMA, COMB

  // Start Timers 0 and 1
  StartTimer0(0b010); // CS
  StartTimer1(0b001); // CS

  // Initialize ADC
  InitADC(0b01, 0b0, 0b0000, 0b001); // REFS, LAR, MUX, PRE

  // ! Pending: Modify SetADCChannel Function to Disable Digital Input Buffers
  DIDR0 = 0b00000001;

  // Start ADC Auto Trigger Mode
  StartADCAutoTrigger(0b011); // ATS

  // Enable global interrupts
  sei();

  while (1) {
    if (flag) {
      // Clear global interrupts
      cli();
      // Enable global interrupts
      sei();
      // Clear ADC Complete Flag
      flag = 0;
      // Set PWM Duty Cycle depending on ADC reading (5 to 95%)
      OCR1A = (ADC_value/1024.00)*720 + 40;
    }
  }
}