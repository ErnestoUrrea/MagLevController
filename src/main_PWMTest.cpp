// Codigo para probar el PWM usando el Encoder

#include <avr/io.h>
#include <avr/interrupt.h>

#include "timer0.h"
#include "timer1.h"
#include "adc.h"

volatile uint8_t flag = 0;
uint16_t ADC_value;

//EMPTY_INTERRUPT(TIMER1_COMPA_vect);

// Micro ports
#define SW_PORT    PORTD
#define SW_PIN     PIND
#define SW_DDR     DDRD
#define SW_BIT     PD4

#define CLK_PORT   PORTD
#define CLK_PIN    PIND
#define CLK_DDR    DDRD
#define CLK_BIT    PD2

#define DT_PORT    PORTD
#define DT_PIN     PIND
#define DT_DDR     DDRD
#define DT_BIT     PD3

// Current and previous value of the counter tuned by the rotary
volatile int curVal = 0;
volatile int prevVal = 0;

// Seven states of FSM (finite state machine)
#define IDLE_11 0
#define SCLK_01 1
#define SCLK_00 2
#define SCLK_10 3
#define SDT_10  4
#define SDT_00  5
#define SDT_01  6

volatile int state = IDLE_11;

ISR(INT0_vect) {
    //Void Rotary CLK
    if (!(CLK_PIN & (1<<CLK_BIT))) {
      if (state==IDLE_11) state = SCLK_01;
      else if (state==SCLK_10) state = SCLK_00;
      else if (state==SDT_10) state = SDT_00;
    } 
    else {
      if (state==SCLK_01) state = IDLE_11;
      else if (state==SCLK_00) state = SCLK_10;
      else if (state==SDT_00) state = SDT_10;
      else if (state==SDT_01) { state = IDLE_11; curVal--; }
    }
}

ISR(INT1_vect){
    // ISR for Rotary DT
    if (!(DT_PIN & (1<<DT_BIT))) {
      if (state==IDLE_11) state = SDT_10;
      else if (state==SDT_01) state = SDT_00;
      else if (state==SCLK_01) state = SCLK_00;
    } 
    else {
      if (state==SDT_10) state = IDLE_11;
      else if (state==SDT_00) state = SDT_01;
      else if (state==SCLK_00) state = SCLK_01;
      else if (state==SCLK_10) { state = IDLE_11; curVal++; }
    }
}

int main(void){
    // Set Pins PB1, PB2, PD2, PD6 to Outputs
    DDRB |= (1 << PB1) | (1 << PB2);
    DDRD |= (1 << PD6); //(1 << PD2) |

    // Initialize Timers 0 and 1
    //InitTimer0(0b010,       49, 0, 0b01, 0b00);  // WGM,      OCRA, OCRB, COMA, COMB
    InitTimer1(0b1110, 799,  0, 0, 0b10, 0b10);  // WGM, ICR, OCRA, OCRB, COMA, COMB

    // Start Timers 0 and 1
    //StartTimer0(0b010); // CS
    StartTimer1(0b001); // CS

    // Initialize ADC
    //InitADC(0b01, 0b0, 0b0000, 0b101); // REFS, LAR, MUX, PRE

    // ! Pending: Modify SetADCChannel Function to Disable Digital Input Buffers
    //DIDR0 = 0b00000001;

    // Start ADC Auto Trigger Mode
    //StartADCAutoTrigger(0b011); // ATS

    // Set the pins as inputs with pull-up resistors
    //SW_PORT |= (1 << SW_BIT);  // Set SW as input with pull-up
    SW_DDR &= ~(1 << SW_BIT);  // Set SW pin as input

    //CLK_PORT |= (1 << CLK_BIT);  // Set CLK as input with pull-up
    CLK_DDR &= ~(1 << CLK_BIT);  // Set CLK pin as input

    //DT_PORT |= (1 << DT_BIT);    // Set DT as input with pull-up
    DT_DDR &= ~(1 << DT_BIT);    // Set DT pin as input

    EIMSK |= (1 << INT0) | (1 << INT1);
    
    EICRA |= (1 << ISC10) | (1 << ISC00);

    // Enable global interrupts
    sei();

    while (1) {
        if (1) {
        // Clear global interrupts
        //cli();
        // Enable global interrupts
        //sei();
        // Clear ADC Complete Flag
        //PORTD = PORTD & ~(1<<PD2);
        //flag = 0;
        // TODO: PID Controller 
        // Set PWM Duty Cycle depending on ADC reading (5 to 95%)
        //OCR1A = (ADC_value/1024.00)*720 + 40;
            OCR1A = (curVal+20)/40.0*720 + 40;
        }

        // Any change in counter value is displayed in Serial Monitor
        if (curVal != prevVal) {
            // Serial.println(curVal);
            prevVal = curVal;
        }
    }
}