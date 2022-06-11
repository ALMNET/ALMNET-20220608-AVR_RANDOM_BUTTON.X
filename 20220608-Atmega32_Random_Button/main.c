#define F_CPU  16000000UL     // CPU runs at 16 MHz

#include <avr/io.h>
#include <avr/interrupt.h>


#include <util/delay.h>

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include "Custom_GPIO.H"
#include "LIB_TWI.h"
#include "LIB_PCF8574.h"
#include "LIB_LCD.h"

//#include "I2C_Master_H_file.h"


FUSES = {
	.low = 0xFF, // LOW {SUT_CKSEL=EXTHIFXTALRES_16KCK_64MS, BODEN=CLEAR, BODLEVEL=2V7}
	.high = 0x99, // HIGH {BOOTRST=CLEAR, BOOTSZ=2048W_3800, EESAVE=CLEAR, CKOPT=CLEAR, SPIEN=SET, JTAGEN=SET, OCDEN=CLEAR}
};

LOCKBITS = 0xFF; // {LB=NO_LOCK, BLB0=NO_LOCK, BLB1=NO_LOCK}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// DEFINES ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define ON					1
#define OFF					0

#define HIGH				1
#define LOW					0


#define PORT_AS_INPUT		0
#define PORT_AS_OUTPUT		0xFF
#define PORT_INPUT_PULLUPS	0xFF




#define DEBOUNCING_DELAY   10

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// INPUTS ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Modify next 3 lines if you want to change Buttons port
#define		BUTTON_PORT_DIR		DDRD	// Button Port
#define		BUTTON_P_IN			PIND	// Buttons Port define for Input asserts
#define		BUTTON_P_PORT		PORTD	// Buttons Port define for Input asserts

// Dont modify these lines if you want to keep the pin numeration of the 
// port previously selected for Buttons
#define		IN_BUTTON_1		(BUTTON_P_IN & (1 << 0))
#define		IN_BUTTON_2		(BUTTON_P_IN & (1 << 1))	
#define		IN_BUTTON_3		(BUTTON_P_IN & (1 << 2))	
#define		IN_BUTTON_4		(BUTTON_P_IN & (1 << 3))	
#define		IN_BUTTON_5		(BUTTON_P_IN & (1 << 4))
#define		IN_BUTTON_6		(BUTTON_P_IN & (1 << 5))
#define		IN_BUTTON_7		(BUTTON_P_IN & (1 << 6))
#define		IN_BUTTON_8		(BUTTON_P_IN & (1 << 7))

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// OUTPUTS ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Modify the next line if you want to change the LEDS port
#define		LEDS_PORT		PORTA

// Dont modify these lines if you want to keep the pin numeration of the
// port previously selected for Leds
#define		LED_1			OUTPUT_PIN(LEDS_PORT, 1)
#define		LED_2			OUTPUT_PIN(LEDS_PORT, 2)
#define		LED_3			OUTPUT_PIN(LEDS_PORT, 3)
#define		LED_4			OUTPUT_PIN(LEDS_PORT, 4)
#define		LED_5			OUTPUT_PIN(LEDS_PORT, 5)
#define		LED_6			OUTPUT_PIN(LEDS_PORT, 6)	
#define		LED_7			OUTPUT_PIN(LEDS_PORT, 7)	
#define		LED_8			OUTPUT_PIN(LEDS_PORT, 8)	

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// SCREENS  ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

char  msgWelcome[2][16] = {" WELCOME", "SCREEN"};
//const unsigned char  msgCrowded[2][16] = {"ROOM", "CROWDED"};
//const unsigned char  msgDenied [2][16] = {"ACCESS", "DENIED"};
//const unsigned char  msgGranted[2][16] = {"ACCESS", "GRANTED"};
	
	
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// VARIABLES ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

char buffer[16];

uint32_t timerCounter = 0;

uint8_t wrongPressingTimes = 0;

float sumaricReaction = 0;
float averageReaction = 0;



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// PROTOTYPES //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////////////////////////
////////////////////////////// INTERRUPTS FUNCTION /////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////// TMR0 ISR ///////////////////////////////////

ISR ( TIMER0_COMP_vect )
{
	
	timerCounter++;
	

}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// MAIN FUNCTION ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


int main(void)
{
   
	// Configures Button Port and internal pullup resistors
	BUTTON_PORT_DIR	= PORT_AS_INPUT;
	BUTTON_P_PORT = PORT_INPUT_PULLUPS;		// Port Pullups enabled
	
	// Configures Leds port 
	LEDS_PORT = PORT_AS_OUTPUT;
	
	LCD_Init();
	
	//LCD_sendData('A');
	
	LCD_Message(msgWelcome[0], msgWelcome[1]);
	
	
	
	/////////////////////////////// MAIN PROCESS ///////////////////////////////
	
	while(1){
		
		uint8_t button;
		uint32_t cnt = 0;
		
		if(button = (BUTTON_P_IN))
		{
			while(button == BUTTON_P_IN){
				cnt++;
			}
			
			LCD_Cmd(CLEAR_LCD);
			
			//srand((unsigned int) (0xFF && (cnt + BUTTON_P_IN + button)));
			
			srand(cnt);
			
			
			
			sprintf(buffer, "%d", rand());
			LCD_out(1, 8 - (strlen((const char *)buffer) / 2), buffer);
			
			_delay_ms(200);
			
			
			
			
			
			
			
			
			
		}
		
		
		
		
		
		
		
		
		
		//printf("\fTesting Results...\n");
		//
		//while(1)
		//{
			//if (!(PINA&0x01)) // here the button is pressed
			//{
				//int counter = 0;
				//while(!(PINA&0x01)){
					//counter++;
				//}
				//
				//srand(counter); //
				//if (!(PINA&0x00)) // and here we release the button and produce the output
				//{
					//
					//PORTB = 0b00000000; // reset port B for each iteration
					//int randomNumber= rand();
					//PORTB = randomNumber; // lights up the x'th bit in PORTB. Determined by the random number.
					//
					//printf("\nGenerated Number: %d", randomNumber);
				//}
			//}
		//}
        
    }
	
    
}
