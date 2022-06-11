

#include <stdint.h>
#include <string.h>

#include "LIB_TWI.h"
#include "LIB_PCF8574.h"
#include "LIB_LCD.h"

//#define RS_HIGH	1
//#define RW_HIGH	2
//#define EN_HIGH	4


/**
 * @brief       Sends a nibble (4 bit data) to the LCD data bus
 *              this function is not useful for user, its is
 *              just required for LCD_Cmd and LCD_sendData functions
 * 
 * @param       data:
 *              data to be sent to the LCD
 * 
 * @return      0 if the nibble is sucessfully sent
 *              1 if not
 * 
 */

lcdResult LCD_nibble(uint8_t data, uint8_t flags){
    
    lcdResult result = 1;
	
	uint8_t newData = (uint8_t) ((data & 0x0F) << 4);
	
	PFC8574_Write(newData | EN_HIGH | flags);
	_NOP();
	PFC8574_Write(newData | flags);
	_delay_us(1000);
	
	result = 0;
	
	return result;
    
}

/**
 * @brief       Send a command to the LCD bus
 *              
 *   
 * @param[in]   lcd_command: 
 *              The command which will be sent to the LCD
 *              The command is descripted in the LCD header file
 * 
 * @return      0 if the command is sucessfully sent
 *              1 if not
 */

lcdResult LCD_Cmd(unsigned char lcd_command){  
    
    lcdResult result = 1;
	
	//while(PFC8574_Read(0x80) == 0x80);
	       
    LCD_nibble((char) (lcd_command & 0xF0) >> 4, RW_LOW | RS_LOW);
    LCD_nibble(lcd_command, RW_LOW | RS_LOW);
    
    result = 0;
    
    return result;
   
} 

/**
 * @brief       Send a command to the LCD bus
 *              
 *   
 * @param[in]   lcd_command: 
 *              The command which will be sent to the LCD
 *              The command is descripted in the LCD header file
 * 
 * @return      0 if the command is sucessfully sent
 *              1 if not
 */

//#define RS_HIGH	1
//#define RW_HIGH	2
//#define EN_HIGH	4

lcdResult LCD_sendData(unsigned char lcd_data){    
    
    char result = 1;
    
    //while(PFC8574_Read(0x80) == 0x80);
          
    LCD_nibble((char) (((lcd_data & 0xF0) >> 4)), RW_LOW | RS_HIGH);
    LCD_nibble(lcd_data, RW_LOW | RS_HIGH);
    
    //while(PFC8574_Read(0x80) == 0x80);
    
    result = 0;
    
    return result;
    
}




/**
 * @brief       Send a text to the LCD using a specific cursor position
 *              
 *   
 * @param[in]   pos_y: 
 *              y position coord. must be 1 or 2, otherwise the 
 *              function will return an error (1)
 * @param[in]   pos_x: 
 *              x position coord. must be between 1 to 16, otherwise the 
 *              function will return an error (1)
 * @param[in]   lcd_msg: 
 *              Pointer / Array to the message to send. If the message
 *              is empty, the function will return 2
 * 
 * @return      0 if the command is sucessfully sent
 *              1 if the position is wrong
 *              2 if the message is empty
 */

lcdResult LCD_out(char pos_y, char pos_x, char * lcd_msg){
    
    unsigned char result;
    unsigned char cursor;
    
    if(*lcd_msg){
        
        if((pos_x > 0 && pos_x < 17) && (pos_y > 0 && pos_y < 3))
        {
            
            switch(pos_y){
                case 1:
                    LCD_Cmd(HOME_LINE1);
                    break;
                    
                case 2:
                    LCD_Cmd(HOME_LINE2);
                    break;
            }
            
            for(cursor = 1; cursor <= pos_x; cursor++)
                LCD_sendData(' ');
            
            LCD_out_CP(lcd_msg);
            
            // Use this lines in a stack overflow case
//            for (cursor = 0; lcd_msg[cursor] != 0; cursor++)     
//            LCD_sendData(lcd_msg[cursor]); 

			result = 0;
            
        }
        
        else
            result = 1;
    }
    
    else
        result = 2;
    
    return result;
}

/**
 * @brief       Send a text to the LCD using the current cursor position
 *              
 *   
 * @param[in]   lcd_msg: 
 *              Pointer / Array to the message to send
 * 
 * @return      none (void function)
 */

void LCD_out_CP(char * lcd_msg){
    for (int j=0; lcd_msg[j]!=0; j++)     
        LCD_sendData(lcd_msg[j]); 
}


void LCD_Message(char LINE1_MSG[], char LINE2_MSG[]){
       
    LCD_Cmd(CLEAR_LCD);              // Clear display  
    
    // Prints centered message on 1st line
    LCD_out(1, 8 - (strlen(LINE1_MSG) / 2), LINE1_MSG);
    
    // Prints centered message on 2nd line
    LCD_out(2, 8 - (strlen(LINE2_MSG) / 2), LINE2_MSG); 
    
}

void LCD_Init(){
	
	I2C_Init();	
	
	_delay_us(15 * 1000);

	
	LCD_Cmd(RETURN_HOME);            // Return cursor to home position
	LCD_Cmd(LCD_2LINES_5X8);         // Function Set - 8-bit, 2 lines, 5X7
	LCD_Cmd(CLEAR_LCD);              // Clear display
	LCD_Cmd(DISPLAY_ON_CURSOR_OFF);  // Display on, cursor off
	LCD_Cmd(INCREMENT_CURSOR);       // Entry mode - inc addr, no shift
}



















