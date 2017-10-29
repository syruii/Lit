/*
 * Lit.c
 *
 * Created: 7/10/2017 6:19:25 PM
 * Author : Syrup
 */ 

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/twi.h>
#include "rtc.h"

void update_pwm(struct rtc_time* rtc);
void init();
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
#define RED_OFFSET        0        // offset for red byte
#define GREEN_OFFSET	  1		  // offset for green byte
#define BLUE_OFFSET       2        // offset for blue byte
#define BYTE_OFFSET	      1	      // offset for one byte	
#define HOUR_OFFSET       3		  // offset for another "hour"


#define BUTTON_STATE ((PINA & (1 << PA5)) >> PA5)   

void update_pwm(struct rtc_time* rtc) {
	/* Interrupt code goes here */
	// read EEPROM for MIN + schedule
	// 24 different color settings in EEPROM, but for speed interrupt is based on minute change
	// minute is mod 24 to keep it working
	SColorRGB newRGB;
	newRGB.red = 0;
	newRGB.green = 0;
	newRGB.blue = 0;
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
	SColorRGB oldRGB;
	oldRGB.red = OCR1A;
	oldRGB.green = OCR1B;
	oldRGB.blue = OCR0;
	
	
	int8_t red_step = (newRGB.red - oldRGB.red) /30 ;	// truncating is fine
	int8_t green_step = (newRGB.green - oldRGB.green) /30;
	int8_t blue_step = (newRGB.blue - oldRGB.blue) /30;
	
	// steps through 9 different color steps until reach final RGB
	for (int i = 0; i < 30; ++i) {
		OCR1A += red_step;
		OCR0 += blue_step;
		OCR1B += green_step;
		_delay_ms(100);
	}
	OCR1A = newRGB.red;
	OCR0 = newRGB.blue;
	OCR1B = newRGB.green;
}
// Should only be called once to program before. Date-time is set at an arbitrary date for demonstration
// purposes default is 9:30AM in 24HOUR
// PD4 - RESET, PD6 - CLK, PD7 - IO
void init(struct rtc_time* rtc) {
		uint8_t color_array[72] = { 0x00, 0xFF, 0x00,
									0x00, 0x00, 0xFF,
									0xFF, 0x00, 0x00,
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
	 uint8_t color_array2[72] = { 0x00, 0xFF, 0xFF,
								 0xFF, 0x00, 0xFF,
								 0xFF, 0xFF, 0x00,
								 0xED, 0x4A, 0x4A,
								 0x6A, 0x6A, 0xEF,
								 0x00, 0xE3, 0x23,
								 0x00, 0xFF, 0xFF,
								 0xFF, 0x00, 0xFF,
								 0xFF, 0xFF, 0x00,
								 0xED, 0x4A, 0x4A,
								 0x6A, 0x6A, 0xEF,
								 0x00, 0xE3, 0x23,
								 0x00, 0xFF, 0xFF,
								 0xFF, 0x00, 0xFF,
								 0xFF, 0xFF, 0x00,
								 0xED, 0x4A, 0x4A,
								 0x6A, 0x6A, 0xEF,
								 0x00, 0xE3, 0x23,
								 0x00, 0xFF, 0xFF,
								 0xFF, 0x00, 0xFF,
								 0xFF, 0xFF, 0x00,
								 0xED, 0x4A, 0x4A,
								 0x6A, 0x6A, 0xEF,
								 0x00, 0xE3, 0x23 };
	uint8_t color_array3[72] = { 0xC2, 0x1A, 0x11,
								 0xE7, 0x88, 0x32,
								 0xF2, 0xDA, 0x4A,
								 0x90, 0xD0, 0x46,
								 0x53, 0xBE, 0xD4,
								 0xA2, 0x80, 0xF2,
								 0xC2, 0x1A, 0x11,
								 0xE7, 0x88, 0x32,
								 0xF2, 0xDA, 0x4A,
								 0x90, 0xD0, 0x46,
								 0x53, 0xBE, 0xD4,
								 0xA2, 0x80, 0xF2,
								 0xC2, 0x1A, 0x11,
								 0xE7, 0x88, 0x32,
								 0xF2, 0xDA, 0x4A,
								 0x90, 0xD0, 0x46,
								 0x53, 0xBE, 0xD4,
								 0xA2, 0x80, 0xF2,
								 0xC2, 0x1A, 0x11,
								 0xE7, 0x88, 0x32,
								 0xF2, 0xDA, 0x4A,
								 0x90, 0xD0, 0x46,
								 0x53, 0xBE, 0xD4,
								 0xA2, 0x80, 0xF2 };
	// SET EVERYTHING AS INPUT AND ENABLE PULL UP
	// DS1302 reconfigures later
	//DDRC = 0;
	//PORTC = 0xFF;
	//DDRB = 0;
	//PORTB = 0xFF;
	//DDRD = 0;
	//PORTD = 0XFF;
	DDRA |= (1 << PA2) | (1 << PA0) | (1 << PA4);
	DDRA &= ~(1 << PA5);
	PORTA &= ~(1 << PA2);
	PORTA &= ~(1 << PA0);
	PORTA &= ~(1 << PA4);
		
	// actual code will run per hour, but for demonstration purposes, only 6
	// schedule number is set to 0
	eeprom_write_byte((uint8_t*)SCHEDULE_NUM, 0x00);
	for (int i = 0; i < 72; ++i) {
		uint8_t offset = i * BYTE_OFFSET;
		eeprom_write_byte((uint8_t*)(SCHEDULE_0 + offset), color_array[i]);
	}

	for (int i = 0; i < 72; ++i) {
		uint8_t offset = i * BYTE_OFFSET;
		eeprom_write_byte((uint8_t*)(SCHEDULE_1 + offset), color_array2[i]);
	}
	
	for (int i = 0; i < 72; ++i) {
		uint8_t offset = i * BYTE_OFFSET;
		eeprom_write_byte((uint8_t*)(SCHEDULE_2 + offset), color_array3[i]);
	}
	TCCR0 |= (1 << WGM00) | (1 << WGM01) | (1 << COM01) | (1 << CS00) | (1 << COM00); // fast non-inverted pwm
	TCCR1A |= (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1) | (1 << COM1B0) | (1 << COM1A0); // fast non-inverted pwm
	TCCR1B |= (1 << WGM12) | ( 1 << CS10);
	DDRB |= (1 << PB3);    // turn on PB3 (OC0)
	DDRD |= (1 << PD5);    // turn on PD5 (OC1A) and PD4 (OC1B)
	DDRD |= (1 << PD4);
	// TCCR2 |= (1 << WGM21) | (1 << WGM20) | (1 << COM21) | (1 << CS20); // fast non-inverted pwm

	OCR1A = 0;
	OCR1B = 0;
	OCR0 = 0;
	
	
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

void swap_schedule(void) 
{
	uint8_t curr_schedule = (uint8_t) (eeprom_read_byte((uint8_t*) (SCHEDULE_NUM)));
	curr_schedule++;
	if (curr_schedule > 2) curr_schedule = 0;
	eeprom_write_byte((uint8_t*)SCHEDULE_NUM, curr_schedule);
}

 
int main(void)
{
	struct rtc_time ds1302;
	struct rtc_time* rtc = &ds1302;
	rtc->minute = 0;
	init(rtc);
	/*	Things to do:
		Which sets stack variables of RGBW by reading from the EEPROM which the PWMs use
		https://protostack.com.au/2011/01/reading-and-writing-atmega168-eeprom/
		Set up PWMS on timers, where duty cycle is color-value/255%	
		https://protostack.com.au/2011/06/atmega168a-pulse-width-modulation-pwm/
	*/
    /* Arbitrary loop */
	unsigned char temp = rtc->minute;
	uint8_t debounce_counter = 0;
    while (1) {
		ds1302_update_time(rtc, MIN);
		if (rtc->minute == 1) {
			PORTA |= (1 << PA2);
		} else if (rtc->minute == 0) {
			PORTA |= (1 << PA0);
		}
		
		if (temp != rtc->minute) {
			update_pwm(rtc);
			temp = rtc->minute;
			PORTA ^= (1 << PA4);
		}
		
		if (BUTTON_STATE != 0) {
			debounce_counter++;
			if (debounce_counter >= 4) {
				swap_schedule();
				update_pwm(rtc);
				debounce_counter = 0;
			}
		} else {
			debounce_counter = 0;
		}
		
	}
	return 0;
}

