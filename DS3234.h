///////////////////////////////////////////////////////////////////////////////
//
//  Dallas / Maxim DS3234 RTC Driver Library for Arduino
//  Copyright (c) 2012, 2015 Roger A. Krupski <rakrupski@verizon.net>
//
//  Last update: 11 July 2015
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program. If not, see <http://www.gnu.org/licenses/>.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef DS_3234_H
#define DS_3234_H

#if ARDUINO < 100
#include "WProgram.h"
#else
#include "Arduino.h"
#endif

#define BSY     _BV(2) // device busy bit
#define CONV    _BV(5) // force temperature conversion bit
#define BBSQW   _BV(6) // enable battery backed swuare wave output
#define EOSC    _BV(7) // enable oscillator (active low)
#define OSF     _BV(7) // "has oscillator been stopped?" flag
#define CENTURY _BV(7) // century flag (0=1900, 1=2000)

#define SEC_REG 0 // some register addresses
#define MIN_REG 1 // (although not all of them)
#define HRS_REG 2
#define DOW_REG 3
#define DAY_REG 4
#define MON_REG 5
#define YRS_REG 6

#define SEC_MASK 0b01111111 // mask off other data in...
#define MIN_MASK 0b01111111 // ... the time registers
#define HRS_MASK 0b00111111
#define DOW_MASK 0b00000111
#define DAY_MASK 0b00111111
#define MON_MASK 0b00011111

class DS3234
{
	public:
		uint8_t init (uint8_t, uint8_t, uint8_t, uint8_t); // setup the IO pins & return RTC status
		uint8_t getTime (uint8_t &, uint8_t &, uint8_t &, uint8_t &, uint8_t &, uint16_t &); // get time
		uint8_t setTime (uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint16_t); // set time & clear OSF flag
		float getTempC (void); // read the temperature sensor in deg C.
		float getTempF (void); // read the temperature sensor & convert to deg F.
		uint8_t read (uint8_t); // 8 bits (same as readByte)
		uint8_t readByte (uint8_t); // 8 bits
		uint16_t readWord (uint8_t); // 16 bits
		uint32_t readDWord (uint8_t); // 32 bits
		uint64_t readQWord (uint8_t); // 64 bits
		float readFloat (uint8_t); // 32 bits
		double readDouble (uint8_t); // (now 32 bits, someday 64)
		long double readLongDouble (uint8_t); // (now 32 bits, someday 128)
		void write (uint8_t, uint8_t); // 8 bits (same as writeByte)
		void writeByte (uint8_t, uint8_t); // 8 bits
		void writeWord (uint8_t, uint16_t); // 16 bits
		void writeDWord (uint8_t, uint32_t); // 32 bits
		void writeQWord (uint8_t, uint64_t); // 64 bits
		void writeFloat (uint8_t, float); // 32 bits
		void writeDouble (uint8_t, double); // (now 32 bits, someday 64)
		void writeLongDouble (uint8_t, long double); // (now 32 bits, someday 128)
	private:
		char _buffer[7]; // clock I/O buffer
		uint8_t _SCK_BIT; // spi bitmasks
		uint8_t _MISO_BIT;
		uint8_t _MOSI_BIT;
		uint8_t _CS_BIT;
		volatile uint8_t *_SCK_PORT; // spi ports
		volatile uint8_t *_MISO_PORT;
		volatile uint8_t *_MISO_PIN;
		volatile uint8_t *_MOSI_PORT;
		volatile uint8_t *_CS_PORT;
		uint8_t _dw; // day of the week 0=Sun, 1=Mon...6=Sat
		template <class T> T _readAll (uint8_t, T &); // generic SRAM read template
		template <class T> void _writeAll (uint8_t, const T &); // generic SRAM write template
		uint8_t _get_dow (uint8_t, uint8_t, uint16_t); // get day of week using Zeller's congruence
		uint8_t _dec2bcd (uint8_t); // convert decimal -> BCD
		uint8_t _bcd2dec (uint8_t); // convert BCD -> decimal
		void _set_bit (uint8_t, uint8_t);
		void _clr_bit (uint8_t, uint8_t);
		void _read_time (void); // read 7 data bytes from clock
		void _write_time (void); // write 7 data bytes to clock
		void _busy_wait (uint32_t); // wait while chip is busy
		uint8_t _command (uint8_t, uint8_t); // send a command byte to the clock
		uint8_t _spi_transfer (uint8_t); // send or receive data SPI Mode 3
};

extern DS3234 RTC; // Preinstantiated real time clock object

#endif

