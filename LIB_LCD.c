

#include "LIB_TWI.h"
#include "LIB_PCF8574.h"
#include "LIB_LCD.h"


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
int8_t setup_LCD(uint8_t addr) {
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
    I2C_PCF8574_LCD_Byte(0x80 | posn, 0x08);   // set DRAM address
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
int8_t LCD_Write(uint8_t addr, char *str, uint8_t len) {
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
