///////////////////////////////////////////////////////////////////////////////
//
//  Dallas / Maxim DS3234 RTC Driver Library for Arduino
//  Copyright (c) 2012, 2018 Roger A. Krupski <rakrupski@verizon.net>
//
//  Last update: 05 November 2018
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

#ifndef DS3234_H
#define DS3234_H

#if ARDUINO < 100
#include "WProgram.h"
#else
#include "Arduino.h"
#endif

class DS3234 {
	public:
		DS3234 (uint8_t, uint8_t, uint8_t, uint8_t, uint8_t=255); // setup the IO pins
		uint8_t begin (void); // setup the RTC registers and check if time is valid
		uint8_t getTime (int &, int &, int &, int &, int &, int &); // get time
		uint8_t getTime (uint8_t &, uint8_t &, uint8_t &, uint8_t &, uint8_t &, uint16_t &); // get time
		uint8_t setTime (int, int, int, int, int, int); // set time & clear OSF flag
		uint8_t setTime (uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint16_t); // set time & clear OSF flag
		double getTempC (void); // read the temperature sensor in deg C.
		double getTempF (void); // read the temperature sensor & convert to deg F
		uint8_t readByte (uint8_t); // 8 bits
		uint16_t readWord (uint8_t); // 16 bits
		uint32_t readDWord (uint8_t); // 32 bits
		uint64_t readQWord (uint8_t); // 64 bits
		float readFloat (uint8_t); // 32 bits
		double readDouble (uint8_t); // (now 32 bits, someday 64?)
		long double readLongDouble (uint8_t); // (now 32 bits, someday 128?)
		void writeByte (uint8_t, uint8_t); // 8 bits
		void writeWord (uint8_t, uint16_t); // 16 bits
		void writeDWord (uint8_t, uint32_t); // 32 bits
		void writeQWord (uint8_t, uint64_t); // 64 bits
		void writeFloat (uint8_t, float); // 32 bits
		void writeDouble (uint8_t, double); // (now 32 bits, someday 64?)
		void writeLongDouble (uint8_t, long double); // (now 32 bits, someday 128?)
		uint8_t status; // RTC init status
	private:
		// DS3234 register addresses
		// address + 0x00 == READ
		// address + 0x80 == WRITE
		#define RD    0x00
		#define WR    0x80
		#define SEC   0x00
		#define MIN   0x01
		#define HRS   0x02
		#define DOW   0x03
		#define DAY   0x04
		#define CENT  0x05
		#define MON   0x05
		#define YEAR  0x06
		#define A1M1  0x07
		#define A1M2  0x08
		#define A1M3  0x09
		#define A1M4  0x0A
		#define A2M2  0x0B
		#define A2M3  0x0C
		#define A2M4  0x0D
		#define CTRL  0x0E
		#define STAT  0x0F
		#define XAGE  0x10
		#define TMSB  0x11
		#define TLSB  0x12
		#define TDIS  0x13
		#define RADR  0x18
		#define RDAT  0x19
		// control register 0x0E/0x8E
		#define A1IE    (1<<0)      // alarm 1 interrupt enable
		#define A2IE    (1<<1)      // alarm 2 interrupt enable
		#define INTCN   (1<<2)      // interrupt control
		#define RS1     (1<<3)      // square wave rate select 1
		#define RS2     (1<<4)      // square wave rate select 2
		#define CONV    (1<<5)      // force temperature conversion bit
		#define BBSQW   (1<<6)      // enable battery backed swuare wave output
		#define EOSC    (1<<7)      // enable oscillator (active low)
		// control/status register 0x0F/0x8F
		#define A1F     (1<<0)      // alarm 1 flag
		#define A2F     (1<<1)      // alarm 2 flag
		#define BSY     (1<<2)      // device busy bit
		#define EN32KHZ (1<<3)      // enable 32 khz output
		#define CRATE0  (1<<4)      // temperature conversion rate 0
		#define CRATE1  (1<<5)      // temperature conversion rate 1
		#define BB32KHZ (1<<6)      // enable battery backed 32 khz output
		#define OSF     (1<<7)      // "oscillator has been stopped" flag
		// flags and masks for clock data
		#define CENTBIT (1<<7)      // century bit (0=1900, 1=2000)
		#define SEC_MASK 0b01111111 // mask off other data in...
		#define MIN_MASK 0b01111111 // ...
		#define HRS_MASK 0b00111111 // ...
		#define DOW_MASK 0b00000111 // ...
		#define DAY_MASK 0b00111111 // ...
		#define MON_MASK 0b00011111 // ... the time registers
		// misc defines
		#define MSEC 1000ULL
		#define USEC 1000000ULL
		#define NSEC 1000000000ULL
		// bitmasks
		uint8_t _SCK_BIT;
		uint8_t _MISO_BIT;
		uint8_t _MOSI_BIT;
		uint8_t _CS_BIT;
		uint8_t _SQW_BIT;
		// outputs
		volatile uint8_t *_SCK_OUT;
		volatile uint8_t *_MISO_OUT;
		volatile uint8_t *_MOSI_OUT;
		volatile uint8_t *_CS_OUT;
		volatile uint8_t *_SQW_OUT;
		// inputs
		volatile uint8_t *_MISO_INP;
		volatile uint8_t *_SQW_INP;
		// ddr's
		volatile uint8_t *_SCK_DDR;
		volatile uint8_t *_MISO_DDR;
		volatile uint8_t *_MOSI_DDR;
		volatile uint8_t *_CS_DDR;
		volatile uint8_t *_SQW_DDR;
		// RTC stuff
		uint8_t _clockBuf[7];                                      // clock register I/O buffer
		template <typename T> T _readAll (uint8_t, const T &);     // generic SRAM read template
		template <typename T> void _writeAll (uint8_t, const T &); // generic SRAM write template
		void _clearOSF (void);                                     // clear OSF flag
		uint8_t _get_dow (uint8_t, uint8_t, uint16_t);             // get day of week using Zeller's congruence
		uint8_t _dec2bcd (uint8_t);                                // convert decimal -> BCD
		uint8_t _bcd2dec (uint8_t);                                // convert BCD -> decimal
		void _set_rtc_bit (uint8_t, uint8_t);                      // set a bit in a clock register
		void _clr_rtc_bit (uint8_t, uint8_t);                      // clear a bit in a clock register
		void _read_time (void);                                    // read 7 data bytes from clock
		void _write_time (void);                                   // write 7 data bytes to clock
		uint8_t _command (uint8_t, uint8_t);                       // send a command byte to the clock
		uint8_t _spi_transfer (uint8_t);                           // send or receive data
};

#endif // #ifndef DS3234_H
