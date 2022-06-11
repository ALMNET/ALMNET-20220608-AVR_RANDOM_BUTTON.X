

#include <stdint.h>
#include <string.h>

#include "LIB_TWI.h"
#include "LIB_PCF8574.h"
#include "LIB_LCD.h"

//#define RS_HIGH	1
//#define RW_HIGH	2
//#define EN_HIGH	4


/**
 * Clear the LCD display (and return the cursor to the home position
 * 
 * @param addr address of the LCD display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_Clear(uint8_t addr) {
    if (addr == 0) return -2;
    
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Byte(0x01, 0x08);   // clear screen command
    I2C_Stop();
    return 0;
}

/**
 * Setup the LCD display
 * 
 * @param addr address of the LCD display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_Init(uint8_t addr) {
    if (LCD_PCF8574_Setup(addr) != 0) {
        return -1;
    }
    LCD_Clear(addr);
    return 0;
}

/**
 * Set the cursor position on the LCD display
 * See the data sheet for mappings of position values to screen locations.
 * (0x00 top left, 0x40 second row left)
 * 
 * @param addr address of the LCD display
 * @param posn Location to where the cursor should be positioned
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_Position(uint8_t addr, uint8_t posn) {
    if (addr == 0) return -2;
    
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Byte(0x80 | posn, 7 & EN_HIGH);   // set DRAM address
    I2C_Stop();
    return 0;
}

/**
 * Write a string to the LCD display
 * 
 * @param addr address of the LCD display
 * @param str pointer to a character string to display
 * @param len length of the string to display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_Write(const uint8_t addr, const char *str, uint8_t len) {
    if (addr == 0) return -2;
    
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    while (len--) {
        I2C_PCF8574_LCD_Byte(*str++, 0x09);
    }
    I2C_Stop();
    return 0;
}

/**
 * Write a constant string to the LCD display
 * 
 * @param addr address of the LCD display
 * @param str pointer to a character string to display
 * @param len length of the string to display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_ConstWrite(uint8_t addr, const char *str, uint8_t len) {
    if (addr == 0) return -2;
    
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    while (len--) {
        I2C_PCF8574_LCD_Byte(*str++, 0x09);
    }
    I2C_Stop();
    return 0;
}

/**
 * Write a string at a position on the screen
 * 
 * @param addr address of the LCD display
 * @param posn Location to where the cursor should be positioned before write
 * @param str pointer to a constant character string to display
 * @param len length of the string to display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_PosnWrite(uint8_t addr, uint8_t posn, const char *str, uint8_t len) {
    return LCD_Position(addr, posn) || LCD_ConstWrite(addr, str, len);
}

/**
 * Write a centered message on the screen
 * 
 * @param addr address of the LCD display
 * @param str1 pointer to a constant character string to display at first line
 * @param str2 pointer to a constant character string to display at second line
 * @return -1 if the messages are beyond the character max range of the LCD
 * 				 (16 characters per line)
 */
int8_t LCD_Message(uint8_t addr, const char * str1, const char * str2){
	
	if (strlen(str1) > 0 && strlen(str1) <17 &&
		strlen(str2) > 0 && strlen(str2) <17)
	{
		int8_t pos_msg_1 = (int8_t) (0x00 + (8 - (strlen(str1) / 2)));		// Centers the position according first line message
		int8_t pos_msg_2 = (int8_t) (0x40 + (8 - (strlen(str2) / 2)));		// Centers the position according first line message
	
		// Clear the screen
		LCD_Clear(addr);
	
		// 1st Line Positioning and messaging
		LCD_Position(addr, pos_msg_1);
		LCD_Write(addr, str1, strlen(str1));
	
		// 2nd Line Positioning and messaging
		LCD_Position(addr, pos_msg_2);
		LCD_Write(addr, str2, strlen(str2));	
		
		return 0;
		
	}
	
	return -1;
	
	
	
}
