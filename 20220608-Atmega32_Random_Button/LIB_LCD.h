/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef LIB_LCD_H
#define	LIB_LCD_H

#ifndef F_CPU
#define F_CPU  16000000UL     // CPU runs at 16 MHz
#endif

#include <avr/cpufunc.h>
#include <util/delay.h>

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// LCD COMMANDS  /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define CLEAR_LCD               0x01
#define RETURN_HOME             0x02
#define DECREMENT_CURSOR        0x04
#define INCREMENT_CURSOR        0x06
#define SHIFT_RIGHT             0x05
#define SHIFT_LEFT              0x07
#define DISPLAY_OFF_CURSOR_OFF  0x08
#define DISPLAY_OFF_CURSOR_ON   0x0A
#define DISPLAY_ON_CURSOR_OFF   0x0C
#define DISPLAY_ON_CURSOR_ON    0x0E
#define SHIFT_CURSOR_TO_LEFT    0x10
#define SHIFT_CURSOR_TO_RIGHT   0x14
#define SHIFT_DISPLAY_LEFT      0x18
#define SHIFT_DISPLAY_RIGHT     0x1C
#define HOME_LINE1              0x80
#define HOME_LINE2              0xC0
#define LCD_2LINES_5X7          0x38
#define LCD_2LINES_5X8          0x28

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// LCD CONTROL  /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define RS_HIGH	1
#define RW_HIGH	2
#define EN_HIGH	4

#define RS_LOW	0
#define RW_LOW	0
#define EN_LOW	0

typedef unsigned char lcdResult;
// Prototype functions

lcdResult LCD_nibble(uint8_t data, uint8_t flags);
lcdResult LCD_Cmd(char lcd_command);
lcdResult LCD_sendData(char lcd_data);
lcdResult LCD_out(char pos_y, char pos_x, char * lcd_msg);
void LCD_out_CP(char * lcd_msg);

void LCD_Message(char LINE1_MSG[], char LINE2_MSG[]);
void LCD_Init();


#endif	/* LIB_LCD_H */

