

#include "delay.h"

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>

#define rst  5			//definitions
#define clk  6
#define io  7

#define mon 1
#define tue 2
#define wed 3
#define thu 4
#define fri 5
#define sat 6
#define sun 7

#define sec_w 0x80
#define sec_r 0x81
#define min_r 0x83
#define date_r 0x87
#define month_r 0x89
#define day_r 0x8b
#define year_r 0x8d
#define min_w 0x82
#define hour_w 0x84
#define date_w 0x86
#define month_w 0x88
#define day_w 0x8a
#define year_w 0x8c

#define w_protect 0x8e

void reset(void);		//define the functions
void write(unsigned char);
unsigned char read(void);
void write_byte(unsigned char,unsigned char);
unsigned char read_byte(unsigned char);
unsigned char get_hours(void);

unsigned char b10;
unsigned char bpm;

unsigned char get_hours(void)
{

	unsigned char i;
	unsigned char R_Byte;
	unsigned char TmpByte;

	reset();
	write(0x85);

	sbi(DDRB,io);

	R_Byte = 0x00;
	cbi(PORTB,io);

	cbi(DDRB,io);

	for(i = 0; i < 4; i++) //get the first 4 bits
	{
                TmpByte = 0;
                if(bit_is_set(PINB,io))
                        TmpByte = 1;
		TmpByte <<= 7;
		R_Byte >>= 1;
		R_Byte |= TmpByte;

		sbi(PORTB,clk);
		delay_us(2);
		cbi(PORTB,clk);
		delay_us(2);
	}

        b10 = 0;
	if(bit_is_set(PINB,io))
                b10 = 1;

	sbi(PORTB,clk);
	delay_us(2);
	cbi(PORTB,clk);
	delay_us(2);

        bpm = 0;
	if(bit_is_set(PINB,io))
                bpm = 1;

	sbi(PORTB,clk) ;
	delay_us(2);
	cbi(PORTB,clk);
	delay_us(2);

	cbi(PORTB,rst);
	cbi(PORTB,clk);

	R_Byte	>>= 4;
	return R_Byte;
}

unsigned char read_byte(unsigned char w_byte)	//read the byte with register w_byte
{
	unsigned char temp;
	reset();
	write(w_byte);
	temp = read();
	cbi(PORTB,rst);
	cbi(PORTB,clk);
	return temp;
}

void write_byte(unsigned char w_byte, unsigned char w_2_byte)	//read the byte with register w_byte
{
	reset();
	write(w_byte);
	write(w_2_byte);
	cbi(PORTB,rst);
	cbi(PORTB,clk);;
}

void reset(void)		//sets the pins to begin and end the ds1302 communication
{
	sbi(DDRB,rst);
	cbi(PORTB,clk);
	cbi(PORTB,rst);
	sbi(PORTB,rst);
}

void write(unsigned char W_Byte)	//writes the W_Byte to the DS1302
{
	unsigned char i;
	DDRB = 0xFF;

	for(i = 0; i < 8; i++)
	{
		cbi(PORTB,io);
		if(W_Byte &0x01)
		{
			sbi(PORTB,io);
		}
		sbi(PORTB,clk);
		delay_us(2);
		cbi(PORTB,clk);
		delay_us(2);
		W_Byte >>=1;
	}
}

unsigned char read(void)		//reads the ds1302 reply
{
	unsigned char i;
	unsigned char R_Byte, R_Byte2, TmpByte;

	sbi(DDRB,io);

	R_Byte = 0x00;
	R_Byte2 = 0x00;
	cbi(PORTB,io);

	cbi(DDRB,io);

	for(i = 0; i < 4; i++) //get the first 4 bits
	{
                TmpByte = 0;
		if(bit_is_set(PINB,io))
                        TmpByte = 1;
		TmpByte <<= 7;
		R_Byte >>= 1;
		R_Byte |= TmpByte;

		sbi(PORTB,clk);
		delay_us(2);
		cbi(PORTB,clk);
		delay_us(2);
	}
	for(i = 0; i < 4; i++) //get the next 3 bits
	{
                TmpByte = 0;
		if(bit_is_set(PINB,io))
                        TmpByte = 1;
		TmpByte <<= 7;
		R_Byte2 >>= 1;
		R_Byte2 |= TmpByte;

		sbi(PORTB,clk);
		delay_us(2);
		cbi(PORTB,clk);
		delay_us(2);
	}
	R_Byte >>= 4;
	R_Byte2 >>= 4;
	R_Byte = (R_Byte2 * 10) + R_Byte;
	return R_Byte;
}


