#include <avr/io.h>
#include <avr/interrupt.h>

#include "timer0.h"
#include "timer1.h"
#include "adc.h"

volatile uint8_t flag = 0;

#define VR 5.0

volatile uint16_t sensorValue = 0;          // raw sensor reading
float sensorVoltage = 0;      // sensor voltage reading
 
// control parameters
float desiredVoltage = 2.5;   // desired position of the ball

// control variables
float errorK;                 // position error at the time instant k
float errorKm1 = 0;             // position error at the time instant k-1
float errorKm2 = 0;             // position error at the time instant k-2
float controlK = 0;             // control signal at the time instant k
float controlKm1 = 0;           // control signal at the time instant k-1
 
// control constants
float Kp = 0.2;               // proportional control 
float Ki = 10;                // integral control
float Kd = 0.4;               // derivative control
float h = 1/20000;            // discretization constant                        
 
float keK = Kp*(1+h/Ki+Kd/h);       // parameter that multiplies the error at the time instant k
float keKm1 = -Kp*(1+2*Kd/h);       // parameter that multiplies the error at the time instant k-1                    
float keKm2 = Kp*Kd/h;              // parameter that multiplies the error at the time instant k-2
 
// ADC Interrupt Service Routine
ISR(ADC_vect) {
  // Clear Timer Compare Match Flag A
  TIFR0 = (1 << OCF0A);
  // Read ADC Value
  sensorValue = ADC;
  // Set Flag for ADC Complete
  flag = 1;
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
  InitADC(0b01, 0b0, 0b0000, 0b101); // REFS, LAR, MUX, PRE

  // ! Pending: Modify SetADCChannel Function to Disable Digital Input Buffers
  DIDR0 = 0b00000001;

  // Start ADC Auto Trigger Mode
  StartADCAutoTrigger(0b011); // ATS

  // Enable global interrupts
  sei();

  while (1) {
    if (flag) {
      // clear the new data flag
      flag = 0;

      // sensor reading in volts
      sensorVoltage = sensorValue*VR/1024;
    
      // error at the time instant k;
      errorK = desiredVoltage - sensorVoltage;
    
      // compute the control signal
      controlK = controlKm1 + keK*errorK + keKm1*errorKm1 + keKm2*errorKm2;
    
      // update the values for the next iteration
      controlKm1 = controlK;
      errorKm2 = errorKm1;
      errorKm1 = errorK;
    
      // set pwm duty cycle
      OCR1A = (controlK/5.0)*720 + 360 + 40; // the number 94 is the control action necessary to keep the ball in the horizontal position
    }
  }
}