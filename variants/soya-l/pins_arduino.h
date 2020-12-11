/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id: wiring.h 249 2007-02-03 16:52:51Z mellis $
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

// Workaround for wrong definitions in "iom32u4.h".
// This should be fixed in the AVR toolchain.
#undef UHCON
#undef UHINT
#undef UHIEN
#undef UHADDR
#undef UHFNUM
#undef UHFNUML
#undef UHFNUMH
#undef UHFLEN
#undef UPINRQX
#undef UPINTX
#undef UPNUM
#undef UPRST
#undef UPCONX
#undef UPCFG0X
#undef UPCFG1X
#undef UPSTAX
#undef UPCFG2X
#undef UPIENX
#undef UPDATX
#undef TCCR2A
#undef WGM20
#undef WGM21
#undef COM2B0
#undef COM2B1
#undef COM2A0
#undef COM2A1
#undef TCCR2B
#undef CS20
#undef CS21
#undef CS22
#undef WGM22
#undef FOC2B
#undef FOC2A
#undef TCNT2
#undef TCNT2_0
#undef TCNT2_1
#undef TCNT2_2
#undef TCNT2_3
#undef TCNT2_4
#undef TCNT2_5
#undef TCNT2_6
#undef TCNT2_7
#undef OCR2A
#undef OCR2_0
#undef OCR2_1
#undef OCR2_2
#undef OCR2_3
#undef OCR2_4
#undef OCR2_5
#undef OCR2_6
#undef OCR2_7
#undef OCR2B
#undef OCR2_0
#undef OCR2_1
#undef OCR2_2
#undef OCR2_3
#undef OCR2_4
#undef OCR2_5
#undef OCR2_6
#undef OCR2_7

#define NUM_DIGITAL_PINS   34
#define NUM_ANALOG_INPUTS  12

#define TXLED0			4
#define TXLED1			4
#define RXLED0			4
#define RXLED1			4
#define TX_RX_LED_INIT 4

#define LED_BUILTIN 4

static const uint8_t LR = 4;
static const uint8_t LG = 5;
static const uint8_t LB = 6;

static const uint8_t DIO0 = 7;
static const uint8_t DIO1 = 8;
static const uint8_t DIO2 = 23;
static const uint8_t DIO3 = 24;
static const uint8_t DIO4 = 25;
static const uint8_t DIO5 = 9;

static const uint8_t RX1 = 11;
static const uint8_t TX1 = 12;
static const uint8_t RX2 = 13;
static const uint8_t TX2 = 14;

static const uint8_t SCL = 15;
static const uint8_t SDA = 16;

static const uint8_t SS   = 10;
static const uint8_t CS1  = 17;
static const uint8_t CS2  = 18;
static const uint8_t SCK  = 19;
static const uint8_t MOSI = 20;
static const uint8_t MISO = 21;

// Mapping of analog pins as digital I/O
// A4-A11 share with digital pins
static const uint8_t  A0 = 22;
static const uint8_t  A1 = 23;
static const uint8_t  A2 = 24;
static const uint8_t  A3 = 25;
static const uint8_t  A4 = 26; // D2
static const uint8_t  A5 = 27; // D3
static const uint8_t  A6 = 28; // D4
static const uint8_t  A7 = 29; // D5
static const uint8_t  A8 = 30; // D8
static const uint8_t  A9 = 31; // D13
static const uint8_t A10 = 32; // D17
static const uint8_t A11 = 33; // D18

#define digitalPinToPCICR(p)    ((((p) >= 3 && (p) <= 6) || ((p) == 10) || ((p) >= 19 && (p) <= 21) || ((p) >= A5 && (p) <= A7)) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) 0
#define digitalPinToPCMSK(p)    ((((p) >= 3 && (p) <= 6) || ((p) == 10) || ((p) >= 19 && (p) <= 21) || ((p) >= A5 && (p) <= A7)) ? (&PCMSK0) : ((uint8_t *)0))
#define digitalPinToPCMSKbit(p) (  (p) >= 3 && (p) <= 6 ? (p) + 1 : ((p) == 10 ? 0 : ((p) >= 19 && (p) <= 21 ? (p) - 18 : (p - A5 + 4))))

//	__AVR_ATmega32U4__ has an unusual mapping of pins to channels
extern const uint8_t PROGMEM analog_pin_to_channel_PGM[];
#define analogPinToChannel(P)  ( pgm_read_byte( analog_pin_to_channel_PGM + (P) ) )

#define digitalPinHasPWM(p) ((p) == 2 || ((p) >= 4 && (p) <= 6) || (p) == 9 || (p) == 14 || (p) == 15)

#define digitalPinToInterrupt(p) ((p) == 7 ? 4 : ((p) == 11 ? 2 : ((p) == 12 ? 3 : ((p) == 15 ? 0 : ((p) == 16 ? 1 : NOT_AN_INTERRUPT)))))

#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEL ATMEGA32U4 / ARDUINO LEONARDO / Flora
//
// D0				PD2					RXD1/INT2
// D1				PD3					TXD1/INT3
// D2				PD1		SDA			SDA/INT1
// D3#				PD0		PWM8/SCL	OC0B/SCL/INT0
// D4		A6		PD4					ADC8
// D5#				PC6		???			OC3A/#OC4A
// D6#		A7		PD7		FastPWM		#OC4D/ADC10
// D7				PE6					INT6/AIN0
//
// D8		A8		PB4					ADC11/PCINT4
// D9#		A9		PB5		PWM16		OC1A/#OC4B/ADC12/PCINT5
// D10#		A10		PB6		PWM16		OC1B/0c4B/ADC13/PCINT6
// D11#				PB7		PWM8/16		0C0A/OC1C/#RTS/PCINT7
// D12		A11		PD6					T1/#OC4D/ADC9
// D13#				PC7		PWM10		CLK0/OC4A
//
// A0		D18		PF7					ADC7
// A1		D19		PF6					ADC6
// A2		D20 	PF5					ADC5
// A3		D21 	PF4					ADC4
// A4		D22		PF1					ADC1
// A5		D23 	PF0					ADC0
//
// New pins D14..D17 to map SPI port to digital pins
//
// MISO		D14		PB3					MISO,PCINT3
// SCK		D15		PB1					SCK,PCINT1
// MOSI		D16		PB2					MOSI,PCINT2
// SS		D17		PB0					RXLED,SS/PCINT0
//
// TXLED	D30		PD5
// RXLED		        PB0
// HWB				PE2					HWB

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
	(uint16_t) &DDRF,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
	(uint16_t) &PORTF,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
	(uint16_t) &PINF,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	PE, // D0 - PE2
	PD,	// D1 - PD5
	PD, // D2 - PD7
	PB,	// D3 - PB4
	PB,	// D4 - LR - PB5
	PB, // D5 - LG - PB6
	PB, // D6 - LB - PB7
	PE, // D7 - DIO0 - PE6
	PD, // D8 - DIO1 - PD6
	PC,	// D9 - DI05 - PC7
	PB, // D10 - SS - PB0

	PD, // D11 - RX1 - PD2
	PD, // D12 - TX1 - PD3
	PD, // D13 - RX2 - PD4
	PC, // D14 - TX2 - PC6

	PD,	// D15 - SCL - PD0
	PD, // D16 - SDA - PD1

  PF,	// D17 - CS1 - PF1
	PF,	// D18 - CS2 - PF0
	PB,	// D19 - SCK - PB1
	PB,	// D20 - MOSI - PB2
  PB,	// D21 - MISO - PB3

	PF,	// D22 - A0 - PF7
	PF, // D23 - A1 - PF6
	PF, // D24 - A2 - PF5
	PF, // D25 - A3 - PF4

	PD, // D26 / D2 - A4 - PD7
	PB, // D27 / D3 - A5 - PB4
	PB, // D28 / D4 - A6 - PB5
	PB, // D29 / D5 - A7 - PB6
	PD, // D30 / D8 - A8 - PD6
	PD, // D31 / D13 - A9 - PD4
	PF, // D32 / D17 - A10 - PF1
	PF // D33 / D18 - A11 - PF0
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
  _BV(2), // D0 - PE2
	_BV(5),	// D1 - PD5
	_BV(7), // D2 - PD7
	_BV(4),	// D3 - PB4
	_BV(5),	// D4 - LR - PB5
	_BV(6), // D5 - LG - PB6
	_BV(7), // D6 - LB - PB7
	_BV(6), // D7 - DIO0 - PE6
	_BV(6), // D8 - DIO1 - PD6
	_BV(7),	// D9 - DI05 - PC7
	_BV(0), // D10 - SS - PB0

	_BV(2), // D11 - RX1 - PD2
	_BV(3), // D12 - TX1 - PD3
	_BV(4), // D13 - RX2 - PD4
	_BV(6), // D14 - TX2 - PC6

	_BV(0),	// D15 - SCL - PD0
	_BV(1), // D16 - SDA - PD1

  _BV(1),	// D17 - CS1 - PF1
	_BV(0),	// D18 - CS2 - PF0
	_BV(1),	// D19 - SCK - PB1
	_BV(2),	// D20 - MOSI - PB2
  _BV(3),	// D21 - MISO - PB3

	_BV(7),	// D22 - A0 - PF7
	_BV(6), // D23 - A1 - PF6
	_BV(5), // D24 - A2 - PF5
	_BV(4), // D25 - A3 - PF4

	_BV(7), // D26 / D2 - A4 - PD7
	_BV(4), // D27 / D3 - A5 - PB4
	_BV(5), // D28 / D4 - A6 - PB5
	_BV(6), // D29 / D5 - A7 - PB6
	_BV(6), // D30 / D8 - A8 - PD6
	_BV(4), // D31 / D13 - A9 - PD4
	_BV(1), // D32 / D17 - A10 - PF1
	_BV(0) // D33 / D18 - A11 - PF0
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	TIMER4D,       // D2 - PD7
	NOT_ON_TIMER,
	TIMER1A,		   // D4 - LR - PB5
	TIMER1B,		   // D5 - LG - PB6
	TIMER0A,       // D6 - LB - PB7
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	TIMER4A,       // D9 - DI05 - PC7
	NOT_ON_TIMER,

	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	TIMER3A,       // D14 - TX2 - PC6

	TIMER0B,       // D15 - SCL - PD0
	NOT_ON_TIMER,

	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,

	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,

	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER
};

const uint8_t PROGMEM analog_pin_to_channel_PGM[] = {
	7,	// A0				 PF7					ADC7
	6,	// A1				 PF6					ADC6
	5,	// A2				 PF5					ADC5
	4,	// A3				 PF4					ADC4
	10,	// A4		D2   PD7					ADC10
	11,	// A5		D3   PB4					ADC11
	12,	// A6		D4	 PB5					ADC12
	13,	// A7		D5	 PB6					ADC13
	9,	// A8		D8	 PD6					ADC9
	8,	// A9		D13	 PD4					ADC8
	1,	// A10	D17	 PF1					ADC1
	0,	// A11	D18	 PF0					ADC0
};

#endif /* ARDUINO_MAIN */

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR        Serial
#define SERIAL_PORT_USBVIRTUAL     Serial
#define SERIAL_PORT_HARDWARE       Serial1
#define SERIAL_PORT_HARDWARE_OPEN  Serial1

// Alias SerialUSB to Serial
#define SerialUSB SERIAL_PORT_USBVIRTUAL

// Bootloader related fields
// Old Caterina bootloader places the MAGIC key into unsafe RAM locations (it can be rewritten
// by the running sketch before to actual reboot).
// Newer bootloaders, recognizable by the LUFA "signature" at the end of the flash, can handle both
// the usafe and the safe location. Check once (in USBCore.cpp) if the bootloader in new, then set the global
// _updatedLUFAbootloader variable to true/false and place the magic key consequently
#ifndef MAGIC_KEY
#define MAGIC_KEY 0x7777
#endif

#ifndef MAGIC_KEY_POS
#define MAGIC_KEY_POS 0x0800
#endif

#ifndef NEW_LUFA_SIGNATURE
#define NEW_LUFA_SIGNATURE 0xDCFB
#endif

#endif /* Pins_Arduino_h */
