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
#include "rtc.h"
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                    STRUCT DEFINITIONS
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
typedef struct {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} SColorRGB;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                    INTERNAL EEPROM ADDRESSES
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define SCHEDULE_NUM      0x00        // ADDR for schedule number
#define SCHEDULE_0		  0x01        // ADDR for first schedule
#define SCHEDULE_1        0x48        // ADDR for second schedule
#define SCHEDULE_2        0x94        // ADDR for third schedule
#define RED_OFFSET        0x00        // offset for red byte
#define GREEN_OFFSET	  0x01		  // offset for green byte
#define BLUE_OFFSET       0x02        // offset for blue byte
#define HOUR_OFFSET       0x03		  // offset for another "hour"


void update_pwm(rtc_time *rtc)
{
	/* Interrupt code goes here */
	// read EEPROM for MIN + schedule
	// 24 different color settings in EEPROM, but for speed interrupt is based on minute change
	// minute is mod 24 to keep it working
	SColorRGB newRGB;
	// will always be zero
	unsigned int schedule_num = (unsigned int) (eeprom_read_byte((uint8_t*)SCHEDULE_NUM));
	uint8_t minute = rtc->minute;
	uint8_t hour_offset = (minute % 24) * HOUR_OFFSET;
	switch (schedule_num) {
		case 0:
			newRGB.red = (uint8_t) (eeprom_read_byte((uint8_t*) (SCHEDULE_0 + hour_offset + RED_OFFSET)));
			newRGB.green = (uint8_t) (eeprom_read_byte((uint8_t*) (SCHEDULE_0 + hour_offset + GREEN_OFFSET)));
			newRGB.blue = (uint8_t) (eeprom_read_byte((uint8_t*) (SCHEDULE_0 + hour_offset + BLUE_OFFSET)));
			break;
		case 1:
			newRGB.red = (uint8_t) (eeprom_read_byte((uint8_t*) (SCHEDULE_1 + hour_offset + RED_OFFSET)));
			newRGB.green = (uint8_t) (eeprom_read_byte((uint8_t*) (SCHEDULE_1 + hour_offset + GREEN_OFFSET)));
			newRGB.blue = (uint8_t) (eeprom_read_byte((uint8_t*) (SCHEDULE_1 + hour_offset + BLUE_OFFSET)));
			break;
		case 2:
			newRGB.red = (uint8_t) (eeprom_read_byte((uint8_t*) (SCHEDULE_2 + hour_offset + RED_OFFSET)));
			newRGB.green = (uint8_t) (eeprom_read_byte((uint8_t*) (SCHEDULE_2 + hour_offset + GREEN_OFFSET)));
			newRGB.blue = (uint8_t) (eeprom_read_byte((uint8_t*) (SCHEDULE_2 + hour_offset + BLUE_OFFSET)));
			break;
	}
	// OC0A = RED	OC1A = BLUE 	OC1B = GREEN	OC2A = WHITE	
	OCR0A = newRGB.red;
	OCR1A = newRGB.blue;
	OCR1B = newRGB.green;
}
// Should only be called once to program before. Date-time is set at an arbitrary date for demonstration
// purposes default is 9:30AM in 24HOUR
// PD4 - RESET, PD6 - CLK, PD7 - IO
void init(rtc_time* rtc)
{
	uint8_t color_array[72] = { 0x00, 0xFF, 0x00,
								0xEB, 0x78, 0xE6,
								0x25, 0x11, 0xAE,
								0x2D, 0x15, 0x18,
								0x6A, 0x4B, 0xC9,
								0x47, 0xBB, 0xCE,
								0x00, 0xFF, 0x00,
								0xEB, 0x78, 0xE6,
								0x25, 0x11, 0xAE,
								0x2D, 0x15, 0x18,
								0x6A, 0x4B, 0xC9,
								0x47, 0xBB, 0xCE,
								0x00, 0xFF, 0x00,
								0xEB, 0x78, 0xE6,
								0x25, 0x11, 0xAE,
								0x2D, 0x15, 0x18,
								0x6A, 0x4B, 0xC9,
								0x47, 0xBB, 0xCE,
								0x00, 0xFF, 0x00,
								0xEB, 0x78, 0xE6,
								0x25, 0x11, 0xAE,
								0x2D, 0x15, 0x18,
								0x6A, 0x4B, 0xC9,
								0x47, 0xBB, 0xCE };
		
	// actual code will run per hour, but for demonstration purposes, only 6
	// schedule number is set to 0
	eeprom_write_byte((uint8_t*)SCHEDULE_NUM, 0x00);
	for (int i = 0; i < 72; ++i) {
		uint8_t offset = i * RED_OFFSET;
		eeprom_write_byte((uint8_t*)(SCHEDULE_0 + offset), color_array[i]);
	}
	DDRB = 0b00001110;    // turn on PB1|OC1A and PB2|OC1B and PB3|OC2A  
	DDRD = 0b01000000;    // turn on PD6|OC0A
	TCCR0A = COM0A1 | WGM01 | WGM00;          // non-inverting mode on OC0A -> clear OC0A at MATCH, set at TOP (max value counter), fast PWM
	TCCR0B = CS00;                            // no pre-scaling
	TCCR1A = COM1A1 | COM1B1 | WGM01 | WGM00; // non-inverting mode on 0C1A, 0C1B, fast PWM
	TCCR1B = CS00;
	TCCR2A = COM2A1 | WGM01 | WGM00;          // non-inverting mode on OC2A
	TCCR2B = CS00;
	
	ds1302_init();
	ds1302_update(rtc);
	rtc->hour_format = AM;
	ds1302_set_time(rtc, YEAR, 1);
	ds1302_set_time(rtc, MONTH, 10);
	ds1302_set_time(rtc, DATE, 19);
	ds1302_set_time(rtc, DAY, thu);
	ds1302_set_time(rtc, HOUR, 9);
	ds1302_set_time(rtc, MIN, 0);
	ds1302_set_time(rtc, SEC, 0);		// setting seconds clears CLOCK HALT bit, which starts the timer
	
}


int main(void)
{
	struct rtc_time ds1302;
	struct rtc_time* rtc = &ds1302;
	init(rtc);
	/*	Things to do:
		Which sets stack variables of RGBW by reading from the EEPROM which the PWMs use
		https://protostack.com.au/2011/01/reading-and-writing-atmega168-eeprom/
		Set up PWMS on timers, where duty cycle is color-value/255%	
		https://protostack.com.au/2011/06/atmega168a-pulse-width-modulation-pwm/
	*/
	
    /* Arbitrary loop */
    while (1) {
		unsigned char temp = rtc->minute;
		ds1302_update_time(rtc, MIN);
		if (temp != rtc->minute) {
			update_pwm(rtc);
		}
		
	}
}

