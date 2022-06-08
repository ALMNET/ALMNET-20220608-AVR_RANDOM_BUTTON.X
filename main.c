#define F_CPU  16000000UL     // CPU runs at 16 MHz

#include <avr/io.h>
#include <util/delay.h>			// AVR Delay Header (For delay_ms function)
#include <avr/interrupt.h>

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

//FUSES = {
//	.low = 0xDE, // LOW {SUT_CKSEL=EXTXOSC_8MHZ_XX_258CK_14CK_65MS, CKOUT=CLEAR, CKDIV8=CLEAR}
//	.high = 0xD9, // HIGH {BOOTRST=CLEAR, BOOTSZ=2048W_3800, EESAVE=CLEAR, WDTON=CLEAR, SPIEN=SET, DWEN=CLEAR, RSTDISBL=CLEAR}
//	.extended = 0xFF, // EXTENDED {BODLEVEL=DISABLED}
//};
//
//LOCKBITS = 0xFF; // {LB=NO_LOCK, BLB0=NO_LOCK, BLB1=NO_LOCK}

#include "Custom_GPIO.H"

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// DEFINES ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define ON					1
#define OFF					0

#define HIGH				1
#define LOW					0

#define TRUE				1
#define FALSE				0

#define I2C_READ			1
#define I2C_WRITE			0

#define TEST_BIT(var, bit)	((var) & (1<<bit))
#define SET_BIT(var, bit)	((var) |= (1<<bit))

#define HI_BYTE(w)			(((w) >> 8) & 0xFF)		// Extract high-order byte 
													// from unsigned word
#define LO_BYTE(w)			((w) & 0xFF)			// Extract low-order byte 
													// from unsigned word


#define DEBOUNCING_DELAY   10

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// INPUTS ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/*
#define		NAME_OF_DEFINE	MACRO						// ARDUINO PIN
 */

#define		DELAY_ADJ		ADCH0						// A0
#define		MOTION_SENS_1	(PINC & (1 << PINC1))		// A1
#define		MOTION_SENS_2	(PINC & (1 << PINC2))		// A2

#define		BUTTON_A		(PIND & (1 << PIND7))		// 7
#define		BUTTON_B		(PIND & (1 << PIND6))		// 6
#define		BUTTON_C		(PIND & (1 << PIND5))		// 5
#define		BUTTON_D		(PIND & (1 << PIND4))		// 4
#define		BUTTON_ENTER	(PIND & (1 << PIND3))		// 3
#define		BUTTON_CANCEL	(PIND & (1 << PIND2))		// 2

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// OUTPUTS ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/*
#define		NAME_OF_DEFINE			MACRO				// ARDUINO PIN
 */

#define		STROBE			OUTPUT_PIN(PORTB, 5)		// 13
#define		LED_ZONE_1		OUTPUT_PIN(PORTB, 4)		// 12
#define		LED_ZONE_2		OUTPUT_PIN(PORTB, 3)		// 11

#define		SIREN_OUT		OUTPUT_PIN(PORTB, 2)		// 10
#define		PWM_OUT_AUX		OUTPUT_PIN(PORTB, 1)		// 9
#define		LED_ARMED		OUTPUT_PIN(PORTB, 0)		// 8
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// CONSTANTS ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Values in milisecs
const uint32_t DEFAULT_PASSCODE_BTNS   = 2000;
const uint32_t DEFAULT_TIMEOUT         = 4000;
const uint32_t DEFAULT_LED_FLASH_DELAY = 1000;
const uint32_t DEFAULT_SIREN_TOGGLE    = 2000;




////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// VARIABLES ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

enum GEN_STATES {CLEAR, TRIGGERED, ARMED} SYSTEM_STATE;
enum ARM_STATES {SIREN_OFF, SIREN_INC, SIREN_DEC} SIREN_STATE;


enum BTN_STATES {NONE, BTN_A, BTN_B, BTN_C, BTN_D, BTN_ENT, BTN_CNL, BTN_PIN} BUTTON_STATE;


uint16_t timecount0, buzzer_count;
float adc_result[10];
float adc_average_result = 0;


unsigned int adc_sample = 0;

unsigned int adc_flag = 0;

unsigned int led_counter = 0;

unsigned char buffer[16];

uint32_t value_timer;
uint32_t  count_millisecs = 0;  // for function milliseconds()

 uint8_t addr = 0;
uint8_t Sta = 0;

// Timer variables
uint8_t SIREN_REFRESH_TMR = 5;
uint32_t PASSCODE     = 0;
uint32_t TIMEOUT      = 0;
uint32_t LED_FLASH    = 0;
uint32_t SIREN_SWEEP_TMR = 0;
uint32_t SIREN_TMR_TO_OFF = 0;

// Buton Variables
uint32_t BTN_PIN_TIMER = 0;

char PINCODE_READ[8];
char PIN_CODE[] = "ABCD    ";
unsigned int character_counter = 0;
uint32_t Timeout1 = 0;


char clear_msg[]    = "  CLEAR  STATE  ";
char armed_msg[]    = "  ARMED  STATE  ";
char ent_pass_msg[] = " ENTER PASSWORD ";
char set_pass_msg[] = "  PASSWORD SET! ";
char timeout_msg[]  = "     TIMEOUT    ";
char wrgpass_msg[]  = "   WRONG PASS!  ";
char pass_ok_msg[]  = "  PASSWORD OK!  ";
char rls_ent_msg[]  = " RELEASE  ENTER ";

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// PROTOTYPES //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// General Setup for Atmega328P and external peripherals
void Setup();

// Siren PWM Function
void Siren_PWM(uint32_t Freq_Value);

// EEPROM Functions
void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
unsigned char EEPROM_read(unsigned int uiAddress);

// I2C Functions
void setup_I2C();
void I2C_wait();
int I2C_Start();
int I2C_SLA(uint8_t addr, uint8_t rw);
int I2C_Send(uint8_t data);
void I2C_Stop();
int I2C_CheckAddress(uint8_t addr);

// PFC8574 Functions
int I2C_PCF8574_LCD_Nibble(uint8_t data);
int I2C_PCF8574_LCD_Byte(uint8_t data, uint8_t flags);
void I2C_SendData(uint8_t addr, uint8_t *array, uint8_t len, uint8_t flags);
int8_t LCD_PCF8574_Setup(uint8_t addr);

// LCD Functions
int8_t setup_LCD(uint8_t addr);
int8_t LCD_Clear(uint8_t addr);
int8_t LCD_Position(uint8_t addr, uint8_t posn);
int8_t LCD_Write(uint8_t addr, char *str, uint8_t len);
int8_t LCD_ConstWrite(uint8_t addr, const char *str, uint8_t len);
int8_t LCD_PosnWrite(uint8_t addr, uint8_t posn, const char *str, uint8_t len);

// ADC Functions
unsigned  ADC_ReadInput(unsigned char MUX_SEL);

// Timer1 Functions
void  Timer0_Init();
unsigned long milliseconds();
void delay_ms(uint16_t time);

// All button Check Function
//char Button_Check();
void Password_Set();
int Password_Enter();




#include "Aux_Functions.h"



////////////////////////////////////////////////////////////////////////////////
////////////////////////////// INTERRUPTS FUNCTION /////////////////////////////
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////// ADC ISR ////////////////////////////////////
//
//ISR(ADC_vect)
//{
//	
//	
//	
//	
//	
//}

/////////////////////////////////// TMR0 ISR ///////////////////////////////////

ISR ( TIMER1_COMPA_vect )
{
	
	
	if(!MOTION_SENS_1)	LED_ZONE_1 = ON; else LED_ZONE_1 = OFF;
	if(!MOTION_SENS_2)	LED_ZONE_2 = ON; else LED_ZONE_2 = OFF;
	
	//enum ARM_STATES {DEFAULT, LED, SIREN_INC, SIREN_DEC} SIREN_STATE;
	if(SYSTEM_STATE == ARMED){
		switch(SIREN_STATE){
			case SIREN_OFF:
				Siren_PWM_Off();
				STROBE = OFF;
				if(!MOTION_SENS_1 || !MOTION_SENS_2){
					STROBE = ON;
					SIREN_STATE = SIREN_INC;
					SIREN_SWEEP_TMR = 0;
					SIREN_TMR_TO_OFF = 5000;
				}
				break;

			case SIREN_INC:
				if(SIREN_TMR_TO_OFF == 0 || !BUTTON_D)
					
					SIREN_STATE = SIREN_OFF;
					
				else
					SIREN_TMR_TO_OFF--;
				if(SIREN_SWEEP_TMR > 1000)
					STROBE = OFF;
				if(SIREN_SWEEP_TMR < 2000){
					SIREN_SWEEP_TMR++;
				}
				else{
					SIREN_STATE = SIREN_DEC;
				}
				break;

			case SIREN_DEC:
				if(SIREN_TMR_TO_OFF == 0 || !BUTTON_D)
					SIREN_STATE = SIREN_OFF;
				else
					SIREN_TMR_TO_OFF--;
				if(SIREN_SWEEP_TMR > 0){
					SIREN_SWEEP_TMR--;
				}
				else{
					SIREN_STATE = SIREN_INC;
				}

				break;
		}

		if(SIREN_STATE != SIREN_OFF){
			if(SIREN_REFRESH_TMR > 0)
				SIREN_REFRESH_TMR--;
			else{
				SIREN_REFRESH_TMR = 5;
				Siren_PWM(SIREN_SWEEP_TMR);

			}
		}
	}
	
	if(BTN_PIN_TIMER > 0){
		BTN_PIN_TIMER--;
	}
	
	if(Timeout1 > 0){
		Timeout1--;
	}

	
	count_millisecs++;   // counter used for function milliseconds()

}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// MAIN FUNCTION ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


int main(void)
{
	Setup();
	Siren_PWM_Off();
	
	LCD_Clear(addr);
	LCD_Position(addr, 0xC0);
	LCD_Write(addr, clear_msg, 16);
	
	
	char PASS_STATUS;
	
	
	/////////////////////////////// MAIN PROCESS ///////////////////////////////
	
	do{
		if(!BUTTON_A){
			while(!BUTTON_A);
			delay_ms(25);
				
//			SYSTEM_STATE = TRIGGERED;
//			LCD_Clear(addr);
//			LCD_Position(addr, 0x00);
//			sprintf(buffer, "TRIGGERED  STATE");
//			LCD_Write(addr, buffer, strlen(buffer));
//			LED_ARMED = ON;	delay_ms(1000);
//			LED_ARMED = OFF;	delay_ms(1000);
//			LED_ARMED = ON;	delay_ms(1000);
//			LED_ARMED = OFF;	delay_ms(1000);
//			LED_ARMED = ON;	delay_ms(1000);
//			SYSTEM_STATE = ARMED;
//			LCD_Clear(addr);
//			LCD_Position(addr, 0x00);
//			sprintf(buffer, "  ARMED  STATE  ");
//			LCD_Write(addr, buffer, strlen(buffer));
			
			
			PASS_STATUS = Password_Enter();
			if(PASS_STATUS == 1){	
				LCD_Clear(addr);
				LCD_Position(addr, 0x00);
				LCD_Write(addr, pass_ok_msg, 16);

				delay_ms(1000);
				
				if(SYSTEM_STATE == CLEAR)
					SYSTEM_STATE = ARMED;
				else
					SYSTEM_STATE = CLEAR;
				
				LCD_Clear(addr);
				LCD_Position(addr, 0x00);

				if(SYSTEM_STATE == ARMED)
					LCD_Write(addr, armed_msg, 16);
				else if(SYSTEM_STATE == CLEAR)
					LCD_Write(addr, clear_msg, 16);
			
			
			}
			
			else{

				delay_ms(1000);
			
				LCD_Clear(addr);
				LCD_Position(addr, 0x00);

				if(SYSTEM_STATE == ARMED)
					LCD_Write(addr, armed_msg, 16);
				else if(SYSTEM_STATE == CLEAR)
					LCD_Write(addr, clear_msg, 16);
				
			}
			
			
			
		}
		
		else if(!BUTTON_ENTER && !BUTTON_CANCEL){
			if(BTN_PIN_TIMER == 0){
				Password_Set();
				
				LCD_Clear(addr);
				LCD_Position(addr, 0x00);
				
				if(SYSTEM_STATE == ARMED)
					LCD_Write(addr, armed_msg, 16);
				else if(SYSTEM_STATE == CLEAR)
					LCD_Write(addr, clear_msg, 16);
				
				
			}
					
		}
		
		else{
			BTN_PIN_TIMER = 4000;
		}
		
		
		
	}while(1);
	
    
}
