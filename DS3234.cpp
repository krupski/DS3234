///////////////////////////////////////////////////////////////////////////////
//
//  Dallas / Maxim DS3234 RTC Driver Library for Arduino
//  Copyright (c) 2012, 2016 Roger A. Krupski <rakrupski@verizon.net>
//
//  Last update: 13 August 2016
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

#include <DS3234.h>

DS3234::DS3234 (uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs, uint8_t vcc, uint8_t gnd)
{
	uint8_t x;

	if (gnd != 99) {
		x = digitalPinToPort (gnd); // pin -> port
		_GND_PORT = portOutputRegister (x); // set gnd pin as output
		_GND_DDR  = portModeRegister (x); // get gnd pin's DDR
		_GND_BIT = digitalPinToBitMask (gnd); // get gnd pin's bitmask
		*_GND_PORT &= ~_GND_BIT; // set gnd pin low
		*_GND_DDR |= _GND_BIT; // set gnd pin DDR to output
	}

	if (vcc != 99) {
		x = digitalPinToPort (vcc); // pin -> port
		_VCC_PORT  = portOutputRegister (x); // set vcc pin as output
		_VCC_DDR   = portModeRegister (x); // get vcc pin's DDR
		_VCC_BIT  = digitalPinToBitMask (vcc); // get vcc pin's bitmask
		*_VCC_PORT |= _VCC_BIT; // set vcc pin high
		*_VCC_DDR |= _VCC_BIT; // set vcc pin DDR to output
		__builtin_avr_delay_cycles ((F_CPU / 1e3) * 100); // let chip settle
	}

	status = _init (sck, miso, mosi, cs); // init rtc chip and set RTC status
}

// get the time, converted from BCD to decimal
// return day of week from chip (not from Zeller)
uint8_t DS3234::getTime (uint8_t &hrs, uint8_t &min, uint8_t &sec, uint8_t &mon, uint8_t &day, uint16_t &yrs)
{
	_read_time(); // read RTC into buffer
	sec = _bcd2dec (*(_buffer + SEC) & SEC_MASK); // seconds
	min = _bcd2dec (*(_buffer + MIN) & MIN_MASK); // minutes
	hrs = _bcd2dec (*(_buffer + HRS) & HRS_MASK); // hours
	dow = _bcd2dec (*(_buffer + DOW) % DOW_MASK); // STORED day of week
	day = _bcd2dec (*(_buffer + DAY) & DAY_MASK); // date
	mon = _bcd2dec (*(_buffer + MON) & MON_MASK); // month
	yrs = _bcd2dec (*(_buffer + YEAR)) + ((*(_buffer + MON) & CENTBIT) ? 2000 : 1900); // year+century
	// Compare DOW stored in RTC to Zeller calculated DOW.
	// If mismatch, return 0xFF, if OK return DOW (as stored in the RTC)
	return (dow == _get_dow (mon, day, yrs)) ? dow : 0xFF;
}

// Set the time - decimal is converted to bcd for the clock chip.
// If year > 1999, set the century bit, else clear it.
// Day of week is automatically calculated with Zeller's congruence.
// Setting the time also clears the Oscillator Stop Flag (status
// register 0x0F, bit 7) to indicate that the time is valid.
// Time is invalid if the battery is removed
// Note: Not all vars need to be specified. Hours, Mins and Secs ARE required.
uint8_t DS3234::setTime (uint8_t hrs, uint8_t min, uint8_t sec, uint8_t mon, uint8_t day, uint16_t yrs)
{
	_read_time(); // read RTC into buffer
	*(_buffer + HRS) = (_dec2bcd (hrs) & HRS_MASK); // set hours
	*(_buffer + MIN) = (_dec2bcd (min) & MIN_MASK); // set minutes
	*(_buffer + SEC) = (_dec2bcd (sec) & SEC_MASK); // set seconds
	if (mon && day && yrs) { *(_buffer + DOW) = (_get_dow (mon, day, yrs)); } // calculate day of week
	if (mon && yrs) { *(_buffer + MON) = (_dec2bcd (mon) & MON_MASK) | (yrs < 2000 ? 0 : CENTBIT); } // set the month and century bit
	if (day) { *(_buffer + DAY) = (_dec2bcd (day) & DAY_MASK); } // set the date
	if (yrs) { *(_buffer + YEAR) = (_dec2bcd (yrs % 100)); }  // set the year
	_write_time(); // update the RTC
	clearOSF(); // clear the "Oscillator has been stopped" flag bit
	return *(_buffer + DOW); // return day of week (as calculated by Zeller]
}

// clear the "Oscillator has been stopped" flag bit
uint8_t DS3234::clearOSF (void)
{
	_read_time(); // read RTC into buffer
	_clr_rtc_bit (STAT, OSF); // clear "oscillator stopped" flag
	_write_time(); // send it to the RTC
	_read_time(); // read RTC into buffer
	return ((_command ((STAT | RD), 0) & OSF) ? 1 : 0); // return actual OSF
}

// read chip temperature in degrees C (0.25 degrees per bit resolution)
double DS3234::getTempC (void)
{
	uint8_t timeout;
	timeout = 200; // prevent lockup if chip fails

	while ((_command ((STAT | RD), 0) & BSY) && timeout--); // wait for not busy

	_set_rtc_bit (CTRL, CONV); // force a temperature conversion
	timeout = 200; // prevent lockup if chip fails

	while ((_command ((CTRL | RD), 0) & (CONV | BSY)) && timeout--); // wait for conversion complete

	return (double) (((_command ((TMSB | RD), 0) << 8 | _command ((TLSB | RD), 0)) >> 6) / 4.0);
}

// just convert C to F
double DS3234::getTempF (void)
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

// initialize the SPI interface & setup RTC.
uint8_t DS3234::_init (uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs)
{
	uint8_t x;

	// get ports, pins & ddr's
	x = digitalPinToPort (sck);
	_SCK_PORT  = portOutputRegister (x);
	_SCK_DDR   = portModeRegister (x); // output pin

	x = digitalPinToPort (miso);
	_MISO_PIN  = portInputRegister (x);
	_MISO_DDR  = portModeRegister (x); // input pin

	x = digitalPinToPort (mosi);
	_MOSI_PORT = portOutputRegister (x); // output pin
	_MOSI_DDR  = portModeRegister (x);

	x = digitalPinToPort (cs);
	_CS_PORT   = portOutputRegister (x); // output pin
	_CS_DDR    = portModeRegister (x);

	// get SPI bitmasks
	_SCK_BIT  = digitalPinToBitMask (sck);
	_MISO_BIT = digitalPinToBitMask (miso);
	_MOSI_BIT = digitalPinToBitMask (mosi);
	_CS_BIT   = digitalPinToBitMask (cs);

	// set initial SPI pin values
	*_SCK_PORT  |= _SCK_BIT;  // SCK idles high in mode 3
	*_MISO_PORT |= _MISO_BIT; // set MISO to "input pullup" mode
	*_MOSI_PORT |= _MOSI_BIT; // set MOSI initially high
	*_CS_PORT   |= _CS_BIT;   // set SS high makes us the SPI master

	// set SPI DDR's to output (except MISO which is input)
	*_SCK_DDR  |= _SCK_BIT;   // output
	*_MISO_DDR &= ~_MISO_BIT; // input
	*_MOSI_DDR |= _MOSI_BIT;  // output
	*_CS_DDR   |= _CS_BIT;    // output

	// setup initial clock control register bits
	_clr_rtc_bit (STAT, CRATE0);  // set temperature conversion...
	_clr_rtc_bit (STAT, CRATE1);  // ...rate to 64 seconds
	_clr_rtc_bit (CTRL, INTCN); // direct square wave to pin
	_clr_rtc_bit (CTRL, RS1);   // set squarewave...
	_clr_rtc_bit (CTRL, RS2);   // ...rate to 1 Hz.
	_set_rtc_bit (CTRL, BBSQW); // enable the square wave output
	_clr_rtc_bit (CTRL, EOSC);  // enable the oscillator

	__builtin_avr_delay_cycles ((F_CPU / 1e3) * 100); // let chip settle

	// check "oscillator has been stopped" flag
	x = ((_command ((STAT | RD), 0) & OSF) ? 1 : 0);

	x += ((getTime (
		(uint8_t &) *(_buffer + SEC), // borrow clock buffer...
		(uint8_t &) *(_buffer + MIN), // ...as dummy time vars
		(uint8_t &) *(_buffer + HRS), // clever, huh?  ;)
		(uint8_t &) *(_buffer + DOW),
		(uint8_t &) *(_buffer + DAY),
		(uint16_t &) *(_buffer + MON)
	) == 0xFF) ? 2 : 0);

	return x; // return RTC status
}

// read from RTC BSRAM any data type
template <class T> T DS3234::_readAll (uint8_t address, T &value)
{
	uint8_t x = sizeof (value);
	uint8_t *ptr = (uint8_t *) (void *) &value;

	while (x--) {
		_command ((RADR | WR), address + x);
		*(ptr + x) = _command ((RDAT | RD), 0);
	}

	return value;
}

// write to RTC BSRAM any data type
template <class T> void DS3234::_writeAll (uint8_t address, const T &value)
{
	uint8_t x = sizeof (value);
	uint8_t *ptr = (uint8_t *) (void *) &value;

	while (x--) {
		_command ((RADR | WR), address + x);
		_command ((RDAT | WR), *(ptr + x));
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

// set bit(s) in a clock register
void DS3234::_set_rtc_bit (uint8_t reg, uint8_t value)
{
	// read from reg, OR on bit(s), write it back
	_command ((reg | WR), (_command ((reg | RD), 0) | value));
}

// clear bit(s) in a clock register
void DS3234::_clr_rtc_bit (uint8_t reg, uint8_t value)
{
	// read from reg, AND off bit(s), write it back
	_command ((reg | WR), (_command ((reg | RD), 0) & ~value));
}

////////////////////////////////////////////////////////////////////////////////
// big thank-you to Arduino forum member "jboyton" for pointing out a very
// unlikely but very real bug in this code and providing help in fixing it!
// see: http://forum.arduino.cc/index.php?topic=123501.msg2305653#msg2305653
// re: _read_time() and _write_time()
////////////////////////////////////////////////////////////////////////////////

// read 7 bytes of raw clock data to buffer
void DS3234::_read_time (void)
{
	uint8_t x;

	*_CS_PORT &= ~_CS_BIT; // assert cs
	_spi_transfer (RD); // send read command
	for (x = 0; x < 7; x++) {
		_buffer[x] = _spi_transfer (x); // read 7 bytes in sequence
	}
	*_CS_PORT |= _CS_BIT; // release cs
}

// write 7 bytes of raw clock data from buffer
void DS3234::_write_time (void)
{
	uint8_t x;

	*_CS_PORT &= ~_CS_BIT; // assert cs
	_spi_transfer (WR); // send write command
	for (x = 0; x < 7; x++) {
		_spi_transfer (_buffer[x]); // write 7 bytes in sequence
	}
	*_CS_PORT |= _CS_BIT; // release cs
}

// send or receive data via SPI
uint8_t DS3234::_command (uint8_t cmd, uint8_t data)
{
	*_CS_PORT &= ~_CS_BIT; // assert cs
	_spi_transfer (cmd);
	data = _spi_transfer (data);
	*_CS_PORT |= _CS_BIT; // release cs
	return data;
}

// SPI Mode 3 (CPOL 1, CPHA 1)
uint8_t DS3234::_spi_transfer (uint8_t data)
{
	uint8_t bits = 8;
	while (bits--) {
		*_SCK_PORT &= ~_SCK_BIT; // sck low
		data & _BV(bits) ? *_MOSI_PORT |= _MOSI_BIT : *_MOSI_PORT &= ~_MOSI_BIT;  // send bit
		*_SCK_PORT |= _SCK_BIT; // sck high
		*_MISO_PIN & _MISO_BIT ? data |= _BV(bits) : data &= ~_BV(bits);  // receive bit
	}

	return data & 0xFF;
}

// end of DS3234.cpp
