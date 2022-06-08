void Setup(){
	//////////////////////// CONFIGURING PORTB DIRECTION ///////////////////////
		
	DDRB |= (1 << DDB5);	// STROBE
	DDRB |= (1 << DDB4);	// LED_ZONE_1
	DDRB |= (1 << DDB3);	// LED_ZONE_2
	DDRB |= (1 << DDB2);	// SIREN
	DDRB |= (1 << DDB1);	// PWM_OUT_AUX
	DDRB |= (1 << DDB0);	// LED_ARMED
		
	////////////////// CONFIGURING PORTC DIRECTION AND PULLUPS /////////////////
	
	DDRC &= (0 << DDC0);	// AN0, DELAY ADJUST 
	DDRC &= (0 << DDC1);	// MOTION_SENS_1
	DDRC &= (0 << DDC2);	// MOTION_SENS_2
	
	PORTC |= (1 << PORTC1);	// MOTION_SENS_1 INTERNAL PULLUP RESISTOR ON
	PORTC |= (1 << PORTC2);	// MOTION_SENS_2 INTERNAL PULLUP RESISTOR ON
			
	////////////////// CONFIGURING PORTD DIRECTION AND PULLUPS /////////////////
	
	DDRD &= (0 << DDD7);	// BUTTON_A
	DDRD &= (0 << DDD6);	// BUTTON_B
	DDRD &= (0 << DDD5);	// BUTTON_C
	DDRD &= (0 << DDD4);	// BUTTON_D
	
	DDRD &= (0 << DDD3);	// BUTTON_ENTER
	DDRD &= (0 << DDD2);	// BUTTON_CANCEL
	
	PORTD |= (1 << PORTD7);	// BUTTON_A INTERNAL PULLUP RESISTOR ON
	PORTD |= (1 << PORTD6);	// BUTTON_B INTERNAL PULLUP RESISTOR ON
	PORTD |= (1 << PORTD5);	// BUTTON_C INTERNAL PULLUP RESISTOR ON
	PORTD |= (1 << PORTD4);	// BUTTON_D INTERNAL PULLUP RESISTOR ON
	
	PORTD |= (1 << PORTD3);	// BUTTON_ENTER INTERNAL PULLUP RESISTOR ON
	PORTD |= (1 << PORTD2);	// BUTTON_CANCEL INTERNAL PULLUP RESISTOR ON
	
	////////////////////////////// CONFIGURING ADC /////////////////////////////
	
	ADMUX  |= (1 << REFS1) | (1 << REFS0);
	ADCSRA |= (1 << ADEN);
	
	/////////////////////////// CONFIGURING I2C/TWI ////////////////////////////
	
	setup_I2C();
	
	///////////////////////// ENABLE GLOBAL INTERRUPTS /////////////////////////
		
	sei();					// Global interrupt enable
	
	///////////////////////////// CONFIGURING TMR0 /////////////////////////////
		
//	TCCR0A = 0; // set entire TCCR0A register to 0
//	TCCR0B = 0; // same for TCCR0B
//	TCNT0  = 0; // initialize counter value to 0
//	// set compare match register for 1000 Hz increments
//	OCR0A = 249; // = 16000000 / (64 * 1000) - 1 (must be <256)
//	// turn on CTC mode
//	TCCR0B |= (1 << WGM01);
//	// Set CS02, CS01 and CS00 bits for 64 prescaler
//	TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00);
//	// enable timer compare interrupt
//	TIMSK0 |= (1 << OCIE0A);
	
	///////////////////////////// CONFIGURING TMR1 /////////////////////////////
	
//	unsigned  top_count = (unsigned long) F_CPU / 8000;
//	
//	TCCR1A = 0x00;
//    TCCR1B = 0b00001010;            // CTC mode;  Prescaler = F_CPU / 8
//
//    OCR1AH = HI_BYTE(top_count);    // Load OCR1A register for 1ms Top count
//    OCR1AL = LO_BYTE(top_count);
//	
//	TIMSK1 |= (1 << OCIE1A);
    
    TCCR1A = 0x80;
    TCCR1B = 0x09;
    OCR1AH = 0x3E; 
    OCR1AL = 0x7F; 
    TIMSK1 |= (1 << OCIE1A);
    
	///////////////////////////// CONFIGURING PWM //////////////////////////////
		
	DDRB |= (1 << DDB2);	// SIREN
	
	TCCR1B = 0x10; // 0001 0000, Disable Timer Clock 
//	TCCR1A = 0xA2; // 1010 0010

	ICR1  = 4000;
	OCR1A = (int) (ICR1 * 0.50);
	OCR1B = (int) (ICR1 * 0.50);
	TCNT1 = 0x0;

	TCCR1B |= 1; // Prescale=1, Enable Timer Clock
	
	/////////////////////////////// EEPROM CHECK ///////////////////////////////
	
	Sta = EEPROM_read(0);
	if(Sta == 0xFF)			// If Eeprom is blank
		Sta = 0;			// Clear Status variable
	
	/////////////////////////// I2C ADDRESS CHECKING ///////////////////////////
		
	if (I2C_CheckAddress(0x27)) {
        addr = 0x27;
    } else if (I2C_CheckAddress(0x3F)) {
        addr = 0x3F;
    } else if (I2C_CheckAddress(0x20)) {
        addr = 0x20;
    }
	
	setup_LCD(addr);
	
	SYSTEM_STATE = CLEAR;
	SIREN_STATE = SIREN_OFF;
}




////////////////////////////////////////////////////////////////////////////////
////////////////////////////// SIREN PWM FUNCTION //////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void Siren_PWM(uint32_t Freq_Value){
	
	DDRB |= (1 << DDB2);	// SIREN
	
	TCCR1B = 0x10; // 0001 0000, Disable Timer Clock 
	TCCR1A = 0x22; // 0010 0010
	
	
	
	ICR1  = ((-8*Freq_Value) + 20000);
	OCR1A = (int) (ICR1 * 0.50);
	OCR1B = (int) (ICR1 * 0.50);
	TCNT1 = 0x0;
	
	TCCR1B |= 1; // Prescale=1, Enable Timer Clock

	
}


void Siren_PWM_Off(){
	TCCR1A = 0x00; // 1010 0010
//	DDRB &= (0 << DDB2);	// SIREN
//	PORTB &= (0 << PORTB2);	// SIREN

	
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// EEPROM FUNCTION ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{

	while(EECR & (1<<EEPE));	// Wait for completion of previous write

	EEAR = uiAddress;			// Set up address and Data Registers
	EEDR = ucData;		

	EECR |= (1 << EEMPE);			// Write logical one to EEMPE
	EECR |= (1 << EEPE);			// Start eeprom write by setting EEPE
}

unsigned char EEPROM_read(unsigned int uiAddress)
{
	while(EECR & (1 << EEPE));	// Wait for completion of previous write
	EEAR = uiAddress;			// Set up address register
	EECR |= (1<<EERE);			// Start eeprom read by writing EERE
	return EEDR;				// Return data from Data Register
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// I2C FUNCTIONS ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/**
 * Set up the I2C bus. We need to initialise the default pullups
 * on the two pins (Port C pins 5 and 6) and set the baud rate for the bus.
 * 
 * The baud rate is calculated by the formula:
 * SCL = Fosc / (16 + 2(TWBR).(TWPS[1:0]))
 *     = 16,000,000 / (16 + 2(TWBR).(TWPS[1:0]))
 *
 * We want an I2C clock of about 40kHz. We pick this, as it is slow
 * enough to allow the vagaries of jumper wires. We could run faster
 * If the wiring was printed circuitry and shielded.
 * 
 * 40,000 = 16,000,000 / (16 + 2(TWBR).(TWPS[1:0]))
 *
 * (16 + 2(TWBR).(TWPS[1:0])) = 16,000,000 / 40,000
 *
 * (16 + 2(TWBR).(TWPS[1:0])) = 400
 *
 * 2(TWBR).(TWPS[1:0]) = 400 - 16
 *
 * 2(TWBR).(TWPS[1:0]) = 386
 * 
 * (TWBR).(TWPS[1:0]) = 193
 * 
 */
void setup_I2C() {
    DDRC &= 0x0f;
    PORTC |= 0x30;  // Port C pins 4 and 5 set to input with pullups
    TWBR = 193;
    TWSR = 0;
}

/**
 * Wait for the current I2C operation to finish.
 * This possible allows the processor to wait forever, but if the I2C bus
 * is enabled it will eventually finish.
 */
void I2C_wait() {
    while ((TWCR & _BV(TWINT)) == 0) {
        ;
    }
}

/**
 * Send an I2C start bit.
 * 
 * @return true if the start bit was successfully transmitted
 */
int I2C_Start() {
    // Send I2C Start flag
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
    I2C_wait();
    return ((TWSR & 0xf8) == 0x08);
}

/**
 * Send an I2C address byte and R/W flag
 * 
 * @param addr I2C address of the slave
 * @param rw whether to read or write: 0 to write, 1 to read
 * @return true if the address byte was successfully transmitted
 */
int I2C_SLA(uint8_t addr, uint8_t rw) {
    // Send I2C slave address
    TWDR = (addr << 1) | (rw & 1);
    TWCR = _BV(TWINT) | _BV(TWEN);
    I2C_wait();
    return ((TWSR & 0xf8) == 0x18);
}

/**
 * Send a byte of data through the I2C bus
 * 
 * @param data data to transmit
 * @return true if the data was successfully transmitted
 */
int I2C_Send(uint8_t data) {
    // Send I2C data byte
    TWDR = data;
    TWCR = _BV(TWINT) | _BV(TWEN);
    I2C_wait();
    return ((TWSR & 0xf8) == 0x28);
}

/**
 * Send the stop flag on the I2C bus
 */
void I2C_Stop() {
    // Send I2C Stop flag
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
    for (volatile long x = 0; x < 100; x++) {
        ;
    }
}

/**
 * Check if there is a device on the I2C bus at address
 * 
 * @param addr address to check if a device exists there.
 * @return true if a device acknowledges the address probe
 */
int I2C_CheckAddress(uint8_t addr) {
    int ret;
    
    ret = I2C_Start() & I2C_SLA(addr, I2C_WRITE);
    if (ret) {
        I2C_Stop();
    }
    return ret;
}


////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// PFC8574 FUNCTIONS //////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/**
 * Send four bits of data to a PCF8574 controlled HD44780 LCD display
 * We need to toggle the E bit (bit 2) from high to low to transmit the data
 * 
 * The 8 bits transmitted are:
 * bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
 * DB7  DB6  DB5  DB4  BL   E    R/W  RS
 * BL is the back light (1 = on, 0 = off)
 * E is the enable bit (high to low transition latches the data
 * R/W is the read/write line (1 = read, 0 = write)
 * RS is Register Select (0 = control, 1 = data)
 * 
 * @param data the data to transmit
 * @return true if the data was transmitted
 */
int I2C_PCF8574_LCD_Nibble(uint8_t data) {
    TWDR = data | 0x04;
    TWCR = _BV(TWINT) | _BV(TWEN);
    I2C_wait();
    if ((TWSR & 0xf8) == 0x28) {
        TWDR = data & (~0x04);
        TWCR = _BV(TWINT) | _BV(TWEN);
        I2C_wait();
    }
    return ((TWSR & 0xf8) == 0x28);
}

/**
 * Transmit the 8 bits of data as two four bit nibbles to a HD44780 LCD
 * controller in 4 bit mode attached through a PCF8574 port expander.
 * 
 * The byte is transmitted as the top nibble then the bottom nibble with
 * the bottom four bits being the control flags.
 * 
 * @param data 8 bits of data to transmit
 * @param flags 4 bits if flags
 * @return true if the data was transmitted
 */
int I2C_PCF8574_LCD_Byte(uint8_t data, uint8_t flags) {
    return I2C_PCF8574_LCD_Nibble ((data & 0xf0) | (flags & 0x0f)) && 
    I2C_PCF8574_LCD_Nibble (((data << 4) & 0xf0) | (flags & 0x0f));
}

/**
 * Send multiple bytes of data to the LCD display
 * 
 * @param addr address of the display
 * @param array pointer to a char array of data to transmit
 * @param len number of bytes in the array
 * @param flags the flags to transmit as the lower 4 bits
 */
void I2C_SendData(uint8_t addr, uint8_t *array, uint8_t len, uint8_t flags) {
    if (addr == 0) return;
    
    if (I2C_Start() & I2C_SLA(addr, I2C_WRITE)) {
        while (len > 0) {
            len--;
            if (I2C_PCF8574_LCD_Byte(*array++, flags) == 0) {
                break;  // bad send
            }
        }
    }
    I2C_Stop(); 
}

/**
 * Send the initialisation string for a HD44780 LCD controller connected in
 * 4 bit mode. Taken from the data sheet. Transmit 0x30 three times to ensure
 * it is in 8 bit mode, then 0x20 to switch to 4 bit mode.
 * We then turn on the blinking cursor, backlight and clear the display.
 * 
 * @param addr address of the LCD display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_PCF8574_Setup(uint8_t addr) {
    if (addr == 0) return -2;
    
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_Send(0);    // ensure the PCF8574 enable line is low
    I2C_Stop();
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Nibble(0x30);
    I2C_Stop();
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Nibble(0x30);
    I2C_Stop();
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Nibble(0x30);
    I2C_Stop();
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Nibble(0x20);
    I2C_Stop();
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Byte(0x0c, 0x00);   // display on, cursor on and blinking
    I2C_Stop();
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Byte(0x01, 0x00);   // clear and move home
    I2C_Stop();
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// ADC FUNCTION /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

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


////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// ADC FUNCTION /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


unsigned  ADC_ReadInput(unsigned char MUX_SEL){
    unsigned char  low_byte;
    unsigned  result = 0;

    if (MUX_SEL > 15)  return 0;  // PC0/ADC0 is N/A (= LCD_E)

    ADMUX = 0x40 + MUX_SEL;  // Select Vref = AVCC (+5V);  select MUX channel

    ADCSRA = 0x06;            // Set prescaler to F_SYS/64 --> ADC clock = 125 kHz
    SET_BIT(ADCSRA, ADEN);    // Enable ADC
    SET_BIT(ADCSRA, ADSC);    // Start conversion

    while (TEST_BIT(ADCSRA, ADSC) != 0)
    {
        ;  // wait till conversion done (ADSC == 0)
    }

    low_byte = ADCL;
    result = ((unsigned) ADCH) << 8;  // High-order byte (2 LS bits)
    result += low_byte;  // Add low-order 8 bits

    return  result;
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// TIMER FUNCTION ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/*
 * Function:  TC1_initialize()
 *
 * This function initializes Timer-Counter TC1 to generate a periodic interrupt
 * request (IRQ) every millisecond precisely.
 *
 * TC1_initialize() must be called from main() before enabling global interrupts.
 * Global interrupts must be enabled for the timer functions to work.
 */
void  Timer0_Init(){
    
	// Timer 0 settings for 1ms overflow
	sei();
	OCR0A = 249; 
	TCCR0A = 0x28;
	TCCR0A |= 0x03;
	TIMSK0 |= (1 << OCIE1A);		// T0 Overflow Interrupt enable
		
}


/*
 * Function:  milliseconds()
 *
 * This function returns the value of a free-running 32-bit count variable,
 * incremented every millisecond by Timer TC1 interrupt handler (ISR, below).
 * It's purpose is to implement "non-blocking" time delays and event timers.
 */
unsigned long milliseconds(){
    unsigned long temp32bits;

    // Disable TC1 interrupt to prevent corruption of count_millisecs in case
    // interrupted here in the middle of copying the 4 bytes (32 bits)...
    //TC1_OCA_IRQ_DISABLE();
	TIMSK1 &= ~(1 << OCIE1A);

    temp32bits = count_millisecs;  // capture the count value (4 bytes)

    // Re-enable TC1 interrupt
    //TC1_OCA_IRQ_ENABLE();
	TIMSK1 |= (1 << OCIE1A);

    return  temp32bits;
}

//////////////////////////// GENERAL DELAY FUNCTION ////////////////////////////

void delay_ms(uint16_t time){
	for(uint16_t x = 0; x < time; x++)
	_delay_us(999);
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////// PASSWORD SET FUNCTION ////////////////////////////
////////////////////////////////////////////////////////////////////////////////


void Password_Set(){
    
    LCD_Clear(addr);
	LCD_Position(addr, 0x00);
	LCD_Write(addr, rls_ent_msg, 16);
	
    while(!BUTTON_ENTER);
    delay_ms(25);
    
    
	for(int i = 0; i <= 3; i++)	PINCODE_READ[i] = 0;
	
	character_counter = 0;	
	
	LCD_Clear(addr);
	LCD_Position(addr, 0x00);
	LCD_Write(addr, ent_pass_msg, 16);
	
	Timeout1 = 32000;
	
	do{
		if(Timeout1 == 0){
			
			LCD_Clear(addr);
			LCD_Position(addr, 0x00);
			LCD_Write(addr, timeout_msg, 16);
			
			delay_ms(1000);
			
			LCD_Clear(addr);
            LCD_Position(addr, 0x00);

            if(SYSTEM_STATE == ARMED)
                LCD_Write(addr, armed_msg, 16);
            else if(SYSTEM_STATE == CLEAR)
                LCD_Write(addr, clear_msg, 16);
			return;
		}

		else{
			if(!BUTTON_A){
				PINCODE_READ[character_counter] = 'A';
				character_counter++;
				while(!BUTTON_A);
				delay_ms(20);
			}

			if(!BUTTON_B){
				PINCODE_READ[character_counter] = 'B';
				character_counter++;
				while(!BUTTON_B);
				delay_ms(20);
			}

			if(!BUTTON_C){
				PINCODE_READ[character_counter] = 'C';
				character_counter++;
				while(!BUTTON_C);
				delay_ms(20);
			}

			if(!BUTTON_D){
				PINCODE_READ[character_counter] = 'D';
				character_counter++;
				while(!BUTTON_D);
				delay_ms(20);
			}
            
            if(!BUTTON_ENTER){
                for(int i = 0; i <= character_counter; i++)	
                    PIN_CODE[i] = PINCODE_READ[i];  
                break;
            }
			
		}
			
			
	}while(character_counter <= 7);
    
    while(character_counter <= 7){
        PIN_CODE[character_counter] = ' ';
        character_counter++;
    }
	
//    if(character_counter >= 8){
//        for(int i = 0; i <= 8; i++)	PIN_CODE[i] = PINCODE_READ[i];


        LCD_Clear(addr);
        LCD_Position(addr, 0x00);
        LCD_Write(addr, set_pass_msg, 16);

        delay_ms(1000);
//    }
	
				
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////// PASSWORD ENTER FUNCTION ////////////////////////////
////////////////////////////////////////////////////////////////////////////////


int Password_Enter(){
    
    unsigned int error_counter = 0;
	
	for(int i = 0; i <= 3; i++)	PINCODE_READ[i] = 0;
	
	LCD_Clear(addr);
	LCD_Position(addr, 0x00);
	LCD_Write(addr, ent_pass_msg, 16);
	
	Timeout1 = 32000;
	do{
        character_counter = 0;
        
        LCD_Clear(addr);
        LCD_Position(addr, 0x00);
        LCD_Write(addr, ent_pass_msg, 16);
        do{
            if(Timeout1 == 0){

                LCD_Clear(addr);
                LCD_Position(addr, 0x00);
                LCD_Write(addr, timeout_msg, 16);

                delay_ms(1000);

                LCD_Clear(addr);
                LCD_Position(addr, 0x00);

                if(SYSTEM_STATE == ARMED)
                    LCD_Write(addr, armed_msg, 16);
                else if(SYSTEM_STATE == CLEAR)
                    LCD_Write(addr, clear_msg, 16);
                return 0;
            }

            else{
                if(!BUTTON_A){
                    PINCODE_READ[character_counter] = 'A';
                    character_counter++;
                    while(!BUTTON_A);
                    delay_ms(20);
                }

                if(!BUTTON_B){
                    PINCODE_READ[character_counter] = 'B';
                    character_counter++;
                    while(!BUTTON_B);
                    delay_ms(20);
                }

                if(!BUTTON_C){
                    PINCODE_READ[character_counter] = 'C';
                    character_counter++;
                    while(!BUTTON_C);
                    delay_ms(20);
                }

                if(!BUTTON_D){
                    PINCODE_READ[character_counter] = 'D';
                    character_counter++;
                    while(!BUTTON_D);
                    delay_ms(20);
                }


                if(!BUTTON_ENTER){
                    break;
                }
                
                if(!BUTTON_CANCEL){
                    character_counter = 0;
                }

            }


        }while(character_counter <= 7);
        
        while(character_counter <= 7){
            PINCODE_READ[character_counter] = ' ';
            character_counter++;
        }

        if(!strcmp(PINCODE_READ, PIN_CODE)){

            return 1;
        }
        else{
            
            
            LCD_Clear(addr);
            LCD_Position(addr, 0x00);
            LCD_Write(addr, wrgpass_msg, 16);

            error_counter++;
            LED_ARMED = ON;
            delay_ms(100);
            LED_ARMED = OFF;
            delay_ms(100);
            LED_ARMED = ON;
            delay_ms(100);
            LED_ARMED = OFF;
            delay_ms(100);
            LED_ARMED = ON;
            delay_ms(100);
            LED_ARMED = OFF;
            delay_ms(100);
            
            delay_ms(400);
        }
    }while(error_counter < 3);
    
    return 0;
				
}


