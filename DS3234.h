///////////////////////////////////////////////////////////////////////////////
//
//  Dallas / Maxim DS3234 RTC Driver Library for Arduino
//  Copyright (c) 2012, 2013 Roger A. Krupski <rakrupski@verizon.net>
//
//  Last update: 08 February 2014
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

#ifndef _DS3234_H
#define _DS3234_H

#if ARDUINO < 100
#include "WProgram.h"
#else
#include "Arduino.h"
#endif

class DS3234
{
public:
	uint8_t init (uint8_t, uint8_t, uint8_t, uint8_t);
	void clrOSF (void);
	uint8_t getTime (uint8_t *, uint8_t *, uint8_t *, uint8_t *, uint8_t *, uint16_t *);
	uint8_t readTime (uint8_t *, uint8_t *, uint8_t *, uint8_t *, uint8_t *, uint16_t *);
	uint8_t setTime (uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint16_t);
	uint8_t writeTime (uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint16_t);
	float readTempC (void);
	float readTempF (void);
	uint8_t readByte (uint8_t);
	uint16_t readWord (uint8_t);
	uint32_t readDWord (uint8_t);
	uint64_t readQWord (uint8_t);
	float readFloat (uint8_t);
	double readDouble (uint8_t);
	long double readLongDouble (uint8_t);
	void writeByte (uint8_t, uint8_t);
	void writeWord (uint8_t, uint16_t);
	void writeDWord (uint8_t, uint32_t);
	void writeQWord (uint8_t, uint64_t);
	void writeFloat (uint8_t, float);
	void writeDouble (uint8_t, double);
	void writeLongDouble (uint8_t, long double);
private:
	char _buffer[8];
	uint8_t _sck_pin;
	uint8_t _miso_pin;
	uint8_t _mosi_pin;
	uint8_t _ss_pin;
	uint8_t _dow;
	void _busy_wait (void);
	void _read_raw (void);
	void _write_raw (void);
	template <class T> T _readAll (uint8_t, T &);
	template <class T> void _writeAll (uint8_t, const T &);
	uint8_t _command (uint8_t, uint8_t);
	uint8_t _spi_transfer (uint8_t);
	uint8_t _get_dow (uint8_t, uint8_t, uint16_t);
	uint8_t _dec2bcd (uint8_t);
	uint8_t _bcd2dec (uint8_t);
};

#endif
