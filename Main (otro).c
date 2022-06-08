#include <stdint.h>

///////////////////////////////////////////////////////////////////////////
///////////////////////////////// INPUTS //////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#define   DUTY_POT1   A2
#define   DUTY_POT2   A3
#define   DUTY_POT3   A4
#define   FREQ_POT    A5

#define   PWM_OUT1    9
#define   PWM_OUT2    10
#define   PWM_OUT3    3

#define   LED         13

///////////////////////////////////////////////////////////////////////////
//////////////////////////////// VARIABLES ////////////////////////////////
///////////////////////////////////////////////////////////////////////////

char buffer[256];

uint32_t DUTY_CYCLE1;
uint32_t DUTY_CYCLE2;
uint32_t DUTY_CYCLE3;
uint32_t FREQUENCY;

float DUTY_CYCLE_FACTOR1;
float DUTY_CYCLE_FACTOR2;
float DUTY_CYCLE_FACTOR3;

///////////////////////////////////////////////////////////////////////////
////////////////////////////////// SETUP //////////////////////////////////
///////////////////////////////////////////////////////////////////////////

void setup() {
  
  Serial.begin(9600);

  pinMode(9, OUTPUT);  // OC1a
  pinMode(10, OUTPUT); // OC1b

  pinMode(11, OUTPUT); // OC2a
  pinMode(3, OUTPUT); // OC2b

  // INPUTS

  pinMode(DUTY_POT1, INPUT); 
  pinMode(DUTY_POT2, INPUT); 
  pinMode(DUTY_POT3, INPUT); 
  pinMode(FREQ_POT, INPUT); 

  // OUTPUTS  

//  pinMode(3, OUTPUT);
////  pinMode(5, OUTPUT);
////  pinMode(6, OUTPUT);
//  pinMode(9, OUTPUT);
//  pinMode(10, OUTPUT);
//  pinMode(11, OUTPUT);

  digitalWrite(3, HIGH);

}

///////////////////////////////////////////////////////////////////////////
////////////////////////////////// LOOP ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////

void loop() {

  DUTY_CYCLE1 = analogRead(DUTY_POT1);
  DUTY_CYCLE2 = analogRead(DUTY_POT2);
  DUTY_CYCLE3 = analogRead(DUTY_POT3);

  FREQUENCY = analogRead(FREQ_POT);

  
  
  //FREQUENCY = FREQUENCY * 213 / 1023;

//  DUTY_CYCLE_FACTOR1 = float(DUTY_CYCLE1 / 1023.0);
//  DUTY_CYCLE_FACTOR2 = float(DUTY_CYCLE1 / 1023.0);
//  DUTY_CYCLE_FACTOR3 = float(DUTY_CYCLE1 / 1023.0);

  DUTY_CYCLE_FACTOR1 = (float)DUTY_CYCLE1 / 1023.0;
  DUTY_CYCLE_FACTOR2 = (float)DUTY_CYCLE2 / 1023.0;
  DUTY_CYCLE_FACTOR3 = (float)DUTY_CYCLE3 / 1023.0;
  

  PWM_Freq_Set(FREQUENCY);
  
  

  DUTY_CYCLE1 = DUTY_CYCLE1 * 100 / 1023;
  DUTY_CYCLE2 = DUTY_CYCLE2 * 100 / 1023;
  DUTY_CYCLE3 = DUTY_CYCLE3 * 100 / 1023;

  

  
  //sprintf(buffer, "\nDC1 = %lu%% / DC2 = %lu%% / DC3= %lu%% / FR = %lu", DUTY_CYCLE1, DUTY_CYCLE2, DUTY_CYCLE3, FREQUENCY);

  //sprintf(buffer, "\nDC1 = %d / DC2 = %d / DC3= %d / FR = %lu", DUTY_CYCLE_FACTOR1, DUTY_CYCLE_FACTOR2, DUTY_CYCLE_FACTOR3, FREQUENCY);
  
  //Serial.print(buffer);

  delay(200);
  
}

///////////////////////////////////////////////////////////////////////////
///////////////////////////// PWM FUNCTIONS ///////////////////////////////
///////////////////////////////////////////////////////////////////////////

void PWM_Freq_Set(uint8_t PWM_Freq_Value){

  //uint32_t FREQ_VALUE1 = FREQUENCY * 213 / 1023;
  uint32_t FREQ_VALUE1;
  uint32_t FREQ_VALUE2;
  
  FREQ_VALUE1 = FREQUENCY;
  FREQ_VALUE1 = FREQ_VALUE1 * 213 / 1023;
  
  FREQ_VALUE2 = FREQUENCY;
  FREQ_VALUE2 = FREQ_VALUE2 * 27 / 1023;

///////////////////////////////////////////////////////////////////////////
  
  // RTM_TimerCalc 1.20,  RuntimeMicro.com
  // Timer-1 Mode_10_16Bit_Phase_TOP_is_ICR
  
  TCCR1B = 0x10; // 0001 0000, Disable Timer Clock 
  TCCR1A = 0xA2; // 1010 0010
  
  //ICR1 = 320 + FREQ_VALUE1;
  ICR1 = 533 - FREQ_VALUE1;
  OCR1A = (int) (ICR1 * DUTY_CYCLE_FACTOR1);
  OCR1B = (int) (ICR1 * DUTY_CYCLE_FACTOR2);
  
  TCNT1=0x0;
  
  TCCR1B |= 1; // Prescale=1, Enable Timer Clock

///////////////////////////////////////////////////////////////////////////

  // RTM_TimerCalc 1.20,  RuntimeMicro.com
  // Timer-2 Mode_5_8Bit_Phase_TOP_is_OCRa
  
  TCCR2B = 0x08; // 0000 1000, Disable Timer Clock 
  TCCR2A = 0x61; // 0110 0001
  
  //OCR2A = 40 + FREQ_VALUE2; // Low PWM Resolution > 1% step-size 
  OCR2A = 67 - FREQ_VALUE2; // Low PWM Resolution > 1% step-size 
  OCR2B = (byte) (OCR2A * 0.2);
  TCNT2=0x0;

 
  
  TCCR2B |= 2; // Prescale=8, Enable Timer Clock

}
