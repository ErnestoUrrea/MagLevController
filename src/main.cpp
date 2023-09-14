#include <avr/io.h>
#include <avr/interrupt.h>

#include "timer0.h"
#include "timer1.h"

volatile uint8_t flag = 0;
uint16_t ADC_value;

// ADC Interrupt Service Routine
ISR(ADC_vect) {
  // Read ADC Value
  ADC_value = ADC;
  // Set Flag for ADC Complete
  flag = 1;
  // Clear Timer Compare Match Flag
  TIFR0 = (1 << OCF0A);
}

int main(void){
  
  DDRB |= (1 << PB1) | (1 << PB2);
  DDRD |= (1<<PD2)|(1<<PD6);

  InitTimer0(0b010, 49, 0, 0b01, 0b00);
  InitTimer1(0b1110, 799, 0, 0, 0b10, 0b10);

  StartTimer0(0b010);
  StartTimer0(0b001);

  while (1) {
    if (flag) {
      // Clear global interrupts
      cli();
      // Clear global interrupts
      sei();
      // Clear ADC Complete Flag
      flag = 0;
      // Set PWM Duty Cycle depending on ADC reading
      OCR1A = (ADC_value/1024.00)*720 + 40;
    }
  }
}