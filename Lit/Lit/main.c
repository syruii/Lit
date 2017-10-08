/*
 * Lit.c
 *
 * Created: 7/10/2017 6:19:25 PM
 * Author : Syrup
 */ 
#ifndef F_CPU
// 1 MHz default internal system clock
#define F_CPU 1000000UL
#endif

#define F_SCL 100000UL // SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/twi.h>
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                    GLOBAL CONSTANTS RTCC - ADDRESSES
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define  ADDR_EEPROM_WRITE 0xae       //  DEVICE ADDR for EEPROM (writes)
#define  ADDR_EEPROM_READ  0xaf       //  DEVICE ADDR for EEPROM (reads)
#define  ADDR_RTCC_WRITE   0xde       //  DEVICE ADDR for RTCC MCHP  (writes)
#define  ADDR_RTCC_READ    0xdf       //  DEVICE ADDR for RTCC MCHP  (reads)
//.................................................................................
#define  SRAM_PTR          0x20       //  pointer of the SRAM area (RTCC)
#define  ADDR_EEPROM_SR    0xff       //  STATUS REGISTER in the  EEPROM
//.................................................................................
#define  ADDR_SEC          0x00       //  address of SECONDS      register
#define  ADDR_MIN          0x01       //  address of MINUTES      register
#define  ADDR_HOUR         0x02       //  address of HOURS        register
#define  ADDR_DAY          0x03       //  address of DAY OF WK    register
#define  ADDR_STAT         0x03       //  address of STATUS       register
#define  ADDR_DATE         0x04       //  address of DATE         register
#define  ADDR_MNTH         0x05       //  address of MONTH        register
#define  ADDR_YEAR         0x06       //  address of YEAR         register
#define  ADDR_CTRL         0x07       //  address of CONTROL      register
#define  ADDR_CAL          0x08       //  address of CALIB        register
#define  ADDR_ULID         0x09       //  address of UNLOCK ID    register
//.................................................................................
#define  ADDR_ALM0SEC      0x0a       //  address of ALARMO SEC   register
#define  ADDR_ALM0MIN      0x0b       //  address of ALARMO MIN   register
#define  ADDR_ALM0HR       0x0c       //  address of ALARMO HOUR  register
#define  ADDR_ALM0CTL      0x0d       //  address of ALARM0 CONTR register
#define  ADDR_ALM0DAT      0x0e       //  address of ALARMO DATE  register
#define  ADDR_ALM0MTH      0x0f       //  address of ALARMO MONTH register
//.................................................................................
#define  ADDR_ALM1SEC      0x11       //  address of ALARM1 SEC   register
#define  ADDR_ALM1MIN      0x12       //  address of ALARM1 MIN   register
#define  ADDR_ALM1HR       0x13       //  address of ALARM1 HOUR  register
#define  ADDR_ALM1CTL      0x14       //  address of ALARM1 CONTR register
#define  ADDR_ALM1DAT      0x15       //  address of ALARM1 DATE  register
#define  ADDR_ALM1MTH      0x16       //  address of ALARM1 MONTH register
//.................................................................................
#define  ADDR_SAVtoBAT_MIN 0x18       //  address of T_SAVER MIN(VDD->BAT)
#define  ADDR_SAVtoBAT_HR  0x19       //  address of T_SAVER HR (VDD->BAT)
#define  ADDR_SAVtoBAT_DAT 0x1a       //  address of T_SAVER DAT(VDD->BAT)
#define  ADDR_SAVtoBAT_MTH 0x1b       //  address of T_SAVER MTH(VDD->BAT)
//..................................................................................
#define  ADDR_SAVtoVDD_MIN 0x1c       //  address of T_SAVER MIN(BAT->VDD)
#define  ADDR_SAVtoVDD_HR  0x1d       //  address of T_SAVER HR (BAT->VDD)
#define  ADDR_SAVtoVDD_DAT 0x1e       //  address of T_SAVER DAT(BAT->VDD)
#define  ADDR_SAVtoVDD_MTH 0x1f       //  address of T_SAVER MTH(BAT->VDD)
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                  GLOBAL CONSTANTS RTCC - INITIALIZATION
//..................................................................................
#define  START_32KHZ  0x80            //  start crystal: ST = b7 (ADDR_SEC)
#define  LP           0x20            //  mask for the leap year bit(MONTH REG)
#define  HOUR_12      0x40            //  12 hours format   (ADDR_HOUR)
#define  PM           0x20            //  post-meridian bit (ADDR_HOUR)
#define  OUT_PIN      0x80            //  = b7 (ADDR_CTRL)
#define  SQWE         0x40            //  SQWE = b6 (ADDR_CTRL)
#define  ALM_NO       0x00            //  no alarm activated        (ADDR_CTRL)
#define  ALM_0        0x10            //  ALARM0 is       activated (ADDR_CTRL)
#define  ALM_1        0x20            //  ALARM1 is       activated (ADDR_CTRL)
#define  ALM_01       0x30            //  both alarms are activated (ADDR_CTRL)
#define  MFP_01H      0x00            //  MFP = SQVAW(01 HERZ)      (ADDR_CTRL)
#define  MFP_04K      0x01            //  MFP = SQVAW(04 KHZ)       (ADDR_CTRL)
#define  MFP_08K      0x02            //  MFP = SQVAW(08 KHZ)       (ADDR_CTRL)
#define  MFP_32K      0x03            //  MFP = SQVAW(32 KHZ)       (ADDR_CTRL)
#define  MFP_64H      0x04            //  MFP = SQVAW(64 HERZ)      (ADDR_CTRL)
#define  ALMx_POL     0x80            //  polarity of MFP on alarm  (ADDR_ALMxCTL)
#define  ALMxC_SEC    0x00            //  ALARM compare on SEC      (ADDR_ALMxCTL)
#define  ALMxC_MIN    0x10            //  ALARM compare on MIN      (ADDR_ALMxCTL)
#define  ALMxC_HR     0x20            //  ALARM compare on HOUR     (ADDR_ALMxCTL)
#define  ALMxC_DAY    0x30            //  ALARM compare on DAY      (ADDR_ALMxCTL)
#define  ALMxC_DAT    0x40            //  ALARM compare on DATE     (ADDR_ALMxCTL)
#define  ALMxC_ALL    0x70            //  ALARM compare on all param(ADDR_ALMxCTL)
#define  ALMx_IF      0x08            //  MASK of the ALARM_IF      (ADDR_ALMxCTL)
#define  OSCON        0x20            //  state of the oscillator(running or not)
#define  VBATEN       0x08            //  enable battery for back-up

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//				I2C DRIVERS
//	Based off https://github.com/g4lvanix/I2C-master-lib
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void i2c_init(void)
{
	TWBR = (uint8_t)TWBR_val;
}

//	Starts I2C communication and also sends the address of the register to connect
uint8_t i2c_start(uint8_t address)
{
	// reset TWI control register
	TWCR = 0;
	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	// wait for TWINT Flag set indicating START condition has been transmitted
	while( !(TWCR & (1<<TWINT)) );
	// check value of TWI Status Register against sent START
	if ((TWSR & 0xF8) != TW_START) return 1;
	
	// load slave address into data register
	TWDR = address;
	// clear TWINT bit in TWCR to start transmission
	TWCR = (1<<TWINT) | (1<<TWEN)
	// wait for transmission to finish
	while( !(TWCR & (1<<TWINT)) );
	// check value of status register to see if slave is acknowledged
	// by the master transmitter or receiver
	uint8_t twst = TWSR & 0xF8;
	if (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK)) return 1;
	
	return 0;
}

uint8_t i2c_write(uint8_t data)
{
	// load data in data register
	TWDR = data;
	// transmit data
	TWCR = (1<<TWINT) | (1<TWEN);
	//wait for transmission to finish
	while( !(TWCR & (1<<TWINT)) );
	
	if ((TWSR & 0xF8) != TW_MT_DATA_ACK) return 1;
	
	return 0;
}

uint8_t i2c_read_ack(void)
{
	
	// start TWI module and acknowledge data after reception
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR;
}

uint8_t i2c_read_nack(void)
{
	
	// start receiving without acknowledging reception
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR;
}

void ic_stop(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}

// writes a constant/variable in a RTCC register
void rtcc_wr(unsigned char time_var, unsigned char rtcc_reg)
{ 
	i2c_start(ADDR_RTCC_WRITE);  // write DEVICE ADDR for RTCC WRITES
	  i2c_wr(rtcc_reg)        ;  // write the register's ADDRESS
	  i2c_wr(time_var)        ;  // write byte variable in the register
	  i2c_stop()              ;  // stop I2C communication
}



// Should only be called once to program before. Date-time is set at an arbitrary date for demonstration
// purposes default is 9:30AM in 24HOUR
void rtcc_init(void)
{	
	rtcc_wr(ALM_0,ADDR_CTRL)   ;	 // enable alarm 0
	rtcc_wr(0x00,ADDR_ALM0SEC) ;     // set up alarm to trigger every minute
	rtcc_wr(ALMx_POL|ALMxC_SEC,ADDR_ALM0CTL) // interrupt polarity is logic high, triggers at 00 seconds
	// set up time
	rtcc_wr(0x10,ADDR_YEAR)    ;     // initialize YEAR  register : doesn't matter
	rtcc_wr(0x03,ADDR_MNTH)    ;     // initialize MONTH register : doesn't matter
	rtcc_wr(0x01,ADDR_DATE)    ;     // initialize DATE  register : doesn't matter
	rtcc_wr(0x09,ADDR_HOUR)    ;     // initialize HOUR  register
	rtcc_wr(0x00,ADDR_MIN)     ;     // initialize MIN   register
	rtcc_wr(0x30|START_32KHZ,ADDR_SEC) ;   //init SEC   register, set START bit 
}

void rtcc_int_reset(void)
{
	rtcc_wr(ALMx_POL|ALMxC_SEC,ADDR_ALM0CTL); //
}

ISR(INT0_vect)
{
	/* Interrupt code goes here */
}

void init(void)
{
	// http://ww1.microchip.com/downloads/en/DeviceDoc/20002266H.pdf
	//
	i2c_init();
	rtcc_init();
	rtcc_int_reset();
	// set up interrupts
	// http://www.atmel.com/Images/Atmel-42176-ATmega48PB-88PB-168PB_Datasheet.pdf
	EICRA |= ISC11	          ;     // trigger on rising edge of INT0
	EIMSK |= (1<<INT0)        ;     // turn on interrupt
}


int main(void)
{
	init();
	/*	Things to do:
		Which sets stack variables of RGBW by reading from the EEPROM which the PWMs use
		https://protostack.com.au/2011/01/reading-and-writing-atmega168-eeprom/
		Set up PWMS on timers, where duty cycle is color-value/255%	
		https://protostack.com.au/2011/06/atmega168a-pulse-width-modulation-pwm/
	*/
	
	
    /* Replace with your application code */
    while (1) 
    {
		
    }
}

