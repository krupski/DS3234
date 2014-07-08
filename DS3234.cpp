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

#include "DS3234.h"

// initialize the SPI interface & setup RTC. Return codes:
// 0 if time valid, 1 if osc stopped, 2 if DOW doesn't match, 3 if both
uint8_t DS3234::init (uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs)
{
	uint8_t errs;

	// get ports & pins
	_CS_PORT   = portOutputRegister (digitalPinToPort (cs));
	_SCK_PORT  = portOutputRegister (digitalPinToPort (sck));
	_MOSI_PORT = portOutputRegister (digitalPinToPort (mosi));
	_MISO_PORT = portOutputRegister (digitalPinToPort (miso));
	_MISO_PIN  = portInputRegister  (digitalPinToPort (miso)); // note this is an input!

	// get SPI bitmasks
	_CS_BIT   = digitalPinToBitMask (cs);
	_SCK_BIT  = digitalPinToBitMask (sck);
	_MOSI_BIT = digitalPinToBitMask (mosi);
	_MISO_BIT = digitalPinToBitMask (miso);

	// set initial SPI pin values
	*_CS_PORT   |= _CS_BIT; // set SS high makes us the SPI master
	*_SCK_PORT  |= _SCK_BIT; // SCK idles high in mode 3
	*_MOSI_PORT |= _MOSI_BIT; // set MOSI initially high
	*_MISO_PORT |= _MISO_BIT; // set MISO to "input pullup" mode

	// set SPI DDR's to output (except MISO which is input)
	*(_CS_PORT - 1)   |= _CS_BIT; // output
	*(_SCK_PORT - 1)  |= _SCK_BIT; // output
	*(_MOSI_PORT - 1) |= _MOSI_BIT; // output
	*(_MISO_PORT - 1) &= ~_MISO_BIT; // input

	_clr_bit (0x8E, EOSC); // enable the oscillator
	_set_bit (0x8E, BBSQW); // enable the square wave output

	// check "oscillator has been stopped" flag
	errs = ((_command (0x0F, 0x00) & OSF) ? 1 : 0);

	errs += ((getTime (
		(uint8_t &)  *(_buffer + SEC_REG), // borrow clock buffer...
		(uint8_t &)  *(_buffer + MIN_REG), // ...as dummy time vars
		(uint8_t &)  *(_buffer + HRS_REG), // clever, huh?  :)
		(uint8_t &)  *(_buffer + DOW_REG),
		(uint8_t &)  *(_buffer + DAY_REG),
		(uint16_t &) *(_buffer + MON_REG)
	) == 0xFF) ? 2 : 0);

	return errs; // return RTC status
}

// get the time, converted from BCD to decimal
// return day of week from chip (not from Zeller)
uint8_t DS3234::getTime (uint8_t &hrs, uint8_t &min, uint8_t &sec, uint8_t &mon, uint8_t &day, uint16_t &yrs)
{
	_read_time (); // read RTC into buffer
	sec = _bcd2dec (*(_buffer + SEC_REG) & SEC_MASK); // get seconds
	min = _bcd2dec (*(_buffer + MIN_REG) & MIN_MASK); // get minutes
	hrs = _bcd2dec (*(_buffer + HRS_REG) & HRS_MASK); // get hours
	_dw = _bcd2dec (*(_buffer + DOW_REG) % DOW_MASK); // get the stored day of week
	day = _bcd2dec (*(_buffer + DAY_REG) & DAY_MASK); // get the date
	mon = _bcd2dec (*(_buffer + MON_REG) & MON_MASK); // get the month
	yrs = _bcd2dec (*(_buffer + YRS_REG)) + ((*(_buffer + MON_REG) & CENTURY) ? 2000 : 1900); // year + century
	// Compare DOW stored in RTC to Zeller calculated DOW.
	// If mismatch, return 0xFF, if OK return DOW (as stored in the RTC)
	return (_dw == _get_dow (mon, day, yrs)) ? _dw : 0xFF;
}

// Set the time - decimal is converted to bcd for the clock chip.
// If year > 1999, set the century bit, else clear it.
// Day of week is automatically calculated with Zeller's congruence.
// Setting the time also clears the Oscillator Stop Flag (status
// register 0x0F, bit 7) to indicate that the time is valid.
// Time is invalid if the battery is removed
uint8_t DS3234::setTime (uint8_t hrs, uint8_t min, uint8_t sec, uint8_t mon, uint8_t day, uint16_t yrs)
{
	*(_buffer + SEC_REG) = (_dec2bcd (sec) & SEC_MASK); // seconds
	*(_buffer + MIN_REG) = (_dec2bcd (min) & MIN_MASK); // minutes
	*(_buffer + HRS_REG) = (_dec2bcd (hrs) & HRS_MASK); // hours
	*(_buffer + DOW_REG) = (_get_dow (mon, day, yrs)); // calculate day of week
	*(_buffer + DAY_REG) = (_dec2bcd (day) & DAY_MASK); // the date
	*(_buffer + MON_REG) = (_dec2bcd (mon) & MON_MASK) | (yrs < 2000 ? 0 : CENTURY); // month and century bit
	*(_buffer + YRS_REG) = (_dec2bcd (yrs % 100)); // year
	_clr_bit (0x8F, OSF); // clear "oscillator stopped" flag
	_write_time (); // send it to the RTC
	return (*(_buffer + DOW_REG)); // return day of week (as calculated by Zeller)
}

// read chip temperature in degrees C (0.25 degrees per bit resolution)
float DS3234::getTempC (void)
{
	int16_t temperature = 0;
	_busy_wait (20000); // wait for not busy
	_set_bit (0x8E, CONV); // force a temperature conversion
	temperature |= (_command (0x11, 0x00) << 8);
	temperature |= _command (0x12, 0x00);
	return (float) ((temperature >> 6) / 4.0);
}

// just convert C to F
float DS3234::getTempF (void)
{
	return ((getTempC () * 1.8) + 32.0);
}

// read one byte from battery backed SRAM
uint8_t DS3234::read (uint8_t address)
{
	return readByte (address);
}

// read one byte from battery backed SRAM
uint8_t DS3234::readByte (uint8_t address)
{
	uint8_t value;
	return _readAll (address, value);
}

uint16_t DS3234::readWord (uint8_t address)
{
	uint16_t value;
	return _readAll (address, value);
}

uint32_t DS3234::readDWord (uint8_t address)
{
	uint32_t value;
	return _readAll (address, value);
}

uint64_t DS3234::readQWord (uint8_t address)
{
	uint64_t value;
	return _readAll (address, value);
}

// float, double and long double are all the
// same in Arduino, but we look to the future...
float DS3234::readFloat (uint8_t address)
{
	float value;
	return _readAll (address, value);
}

double DS3234::readDouble (uint8_t address)
{
	double value;
	return _readAll (address, value);
}

long double DS3234::readLongDouble (uint8_t address)
{
	long double value;
	return _readAll (address, value);
}

// write one byte to battery backed SRAM
void DS3234::write (uint8_t address, uint8_t value)
{
	writeByte (address, value);
}

// write one byte to battery backed SRAM
void DS3234::writeByte (uint8_t address, uint8_t value)
{
	_writeAll (address, value);
}

void DS3234::writeWord (uint8_t address, uint16_t value)
{
	_writeAll (address, value);
}

void DS3234::writeDWord (uint8_t address, uint32_t value)
{
	_writeAll (address, value);
}

void DS3234::writeQWord (uint8_t address, uint64_t value)
{
	_writeAll (address, value);
}

void DS3234::writeFloat (uint8_t address, float value)
{
	_writeAll (address, value);
}

void DS3234::writeDouble (uint8_t address, double value)
{
	_writeAll (address, value);
}

void DS3234::writeLongDouble (uint8_t address, long double value)
{
	_writeAll (address, value);
}

///////////////////////////////////////////////////////////////////
/////////////////// private functions begin here //////////////////
///////////////////////////////////////////////////////////////////

// read from RTC BSRAM any data type
template <class T> T DS3234::_readAll (uint8_t address, T &value)
{
	uint8_t x = sizeof (value);
	uint8_t *ptr = (uint8_t *) (void *) &value;
	while (x--) {
		_command (0x98, (address + x));
		*(ptr + x) = _command (0x19, 0x00);
	}
	return value;
}

// write to RTC BSRAM any data type
template <class T> void DS3234::_writeAll (uint8_t address, const T &value)
{
	uint8_t x = sizeof (value);
	uint8_t *ptr = (uint8_t *) (void *) &value;
	while (x--) {
		_command (0x98, (address + x));
		_command (0x99, *(ptr + x));
	}
}

//////////////////////////////////////////////////////////////////////////
// zeller's congruence (calculates day of week) modified so that
// sunday == 0, monday == 1 .... friday == 5, saturday == 6
// reference: http://en.wikipedia.org/wiki/Zeller's_congruence
//////////////////////////////////////////////////////////////////////////
//
//      |     |(m + 1) * 26|       | y |       | y |   | y |     |
//  h = | q + |------------| + y + |---| + 6 * |---| + |---| - 1 | mod 7
//      |     |     10     |       | 4 |       |100|   |400|     |
//
//////////////////////////////////////////////////////////////////////////
uint8_t DS3234::_get_dow (uint8_t m, uint8_t d, uint16_t y)
{
	if (m < 3) { // convert months to 2...14
		m += 12;
		y -= 1;
	}
	return (((d + (((m + 1) * 26) / 10) + y + (y / 4) + (6 * (y / 100)) + (y / 400)) - 1) % 7);
}

// convert decimal to BCD
uint8_t DS3234::_dec2bcd (uint8_t dec)
{
	return ((dec / 10 * 16) + (dec % 10));
}

// convert BCD to decimal
uint8_t DS3234::_bcd2dec (uint8_t bcd)
{
	return ((bcd / 16 * 10) + (bcd % 16));
}

// set bit(s) in a register
void DS3234::_set_bit (uint8_t reg, uint8_t bits)
{
	// read from reg, OR on bit(s), write it back
	_command ((reg | 0x80), (_command ((reg & ~0x80), 0x00) | bits));
}

// clear bit(s) in a register
void DS3234::_clr_bit (uint8_t reg, uint8_t bits)
{
	// read from reg, AND off bit(s), write it back
	_command ((reg | 0x80), (_command ((reg & ~0x80), 0x00) & ~bits));
}

////////////////////////////////////////////////////////////////////////////////
// big thank-you to Arduino forum member "jboyton" for pointing out a very
// unlikely but very real bug in this code and providing help in fixing it!
// see: http://forum.arduino.cc/index.php?topic=123501.msg2305653#msg2305653
////////////////////////////////////////////////////////////////////////////////
// read 7 bytes of raw clock data to buffer
void DS3234::_read_time (void)
{
	uint8_t x;
	*_CS_PORT &= ~_CS_BIT; // assert CS
	_spi_transfer (0x00); // read from 0x00 command
	for (x = 0; x < 7; x++) {
		*(_buffer + x) = _spi_transfer (x); // read 7 bytes in sequence
	}
	*_CS_PORT |= _CS_BIT; // release CS
}

// write 7 bytes of raw clock data to buffer
void DS3234::_write_time (void)
{
	uint8_t x;
	*_CS_PORT &= ~_CS_BIT; // assert CS
	_spi_transfer (0x80); // write to 0x00 command
	for (x = 0; x < 7; x++) {
		_spi_transfer (*(_buffer + x)); // write 7 bytes in sequence
	}
	*_CS_PORT |= _CS_BIT; // release CS
}

// wait until status bit 2 (BSY) goes low or timeout
void DS3234::_busy_wait (uint32_t timeout)
{
	while (_command (0x0F, 0x00) & BSY & timeout--);
}

// send or receive data via SPI
uint8_t DS3234::_command (uint8_t cmd, uint8_t data)
{
	*_CS_PORT &= ~_CS_BIT;
	_spi_transfer (cmd);
	data = _spi_transfer (data);
	*_CS_PORT |= _CS_BIT;
	return data;
}

// SPI Mode 3 (CPOL 1, CPHA 1)
uint8_t DS3234::_spi_transfer (uint8_t data)
{
	uint8_t bits = 8;
	while (bits--) {
		*_SCK_PORT &= ~_SCK_BIT; // sck low
		(data & (1 << bits)) ? (*_MOSI_PORT |= _MOSI_BIT) : (*_MOSI_PORT &= ~_MOSI_BIT); // send bit
		*_SCK_PORT |= _SCK_BIT; // sck high
		(*_MISO_PIN & _MISO_BIT) ? (data |= (1ULL << bits)) : (data &= ~(1ULL << bits)); // receive bit
	}
	return data;
}

DS3234 RTC; // Preinstantiate real time clock object

// end of DS3234.cpp
