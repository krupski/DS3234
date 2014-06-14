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

#include "DS3234.h"

// initialize the SPI interface & setup RTC. Return codes:
// 0 if time valid, 1 if osc stopped, 2 if DOW doesn't match, 3 if both
uint8_t DS3234::init (uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t ss)
{
	uint8_t errs;

	errs = 0;

	// init SPI pin values
	_sck_pin = sck;
	_miso_pin = miso;
	_mosi_pin = mosi;
	_ss_pin = ss;

	// setup DDRS for SPI
	pinMode (_sck_pin, OUTPUT);
	pinMode (_miso_pin, INPUT);
	pinMode (_mosi_pin, OUTPUT);
	pinMode (_ss_pin, OUTPUT);

	// check "oscillator has been stopped" flag
	errs |= ((_command (0x0F, 0x00) & 0b10000000) ? 1 : 0);

	_read_raw (); // read RTC

	_dow = _bcd2dec (*(_buffer + 3) % 0b00000111); // get calculated DOW

	// check to see if stored DOW matches calculated DOW
	errs |= ((_dow == _get_dow (
			_bcd2dec (*(_buffer + 5) & 0b00011111), // month
			_bcd2dec (*(_buffer + 4) & 0b00111111), // date
			_bcd2dec (*(_buffer + 6)) + ((*(_buffer + 5) & 0x80) ? 2000 : 1900) // year
		)
	) ? 0 : 2);

	return errs; // return RTC status
}

void DS3234::clrOSF (void)
{
	// clear register 0x0F/0x8F, bit 7 (Oscillator Stopped Flag).
	_command (0x8F, (_command (0x0F, 0x00) & ~0b10000000));
}

// synonym for readTime ()
uint8_t DS3234::getTime (uint8_t *h, uint8_t *m, uint8_t *s, uint8_t *mn, uint8_t *dy, uint16_t *yr)
{
	return readTime (h, m, s, mn, dy, yr);
}

// get the time, converted from BCD to decimal
// return day of week from chip (not from Zeller)
uint8_t DS3234::readTime (uint8_t *h, uint8_t *m, uint8_t *s, uint8_t *mn, uint8_t *dy, uint16_t *yr)
{
	_read_raw (); // read RTC into buffer
	*s = _bcd2dec (*(_buffer + 0) & 0b01111111); // get seconds
	*m = _bcd2dec (*(_buffer + 1) & 0b01111111); // get minutes
	*h = _bcd2dec (*(_buffer + 2) & 0b00111111); // get hours
	_dow = _bcd2dec (*(_buffer + 3) % 0b00000111); // get the stored day of week
	*dy = _bcd2dec (*(_buffer + 4) & 0b00111111); // get the date
	*mn = _bcd2dec (*(_buffer + 5) & 0b00011111); // get the month
	*yr = _bcd2dec (*(_buffer + 6)) + ((*(_buffer + 5) & 0x80) ? 2000 : 1900); // year + century
	return _dow; // return day of week _from RTC_
}

// synonym for writeTime ()
uint8_t DS3234::setTime (uint8_t h, uint8_t m, uint8_t s, uint8_t mn, uint8_t dy, uint16_t yr)
{
	return writeTime (h, m, s, mn, dy, yr);
}

// Set the time - decimal is converted to bcd for the clock chip.
// If year > 1999, set the century bit, else clear it.
// Day of week is automatically calculated with Zeller's congruence.
// Setting the time also clears the Oscillator Stop Flag (status
// register 0x0F, bit 7) to indicate that the time is valid.
uint8_t DS3234::writeTime (uint8_t h, uint8_t m, uint8_t s, uint8_t mn, uint8_t dy, uint16_t yr)
{
	*(_buffer + 0) = _dec2bcd (s) & 0b01111111; // seconds
	*(_buffer + 1) = _dec2bcd (m) & 0b01111111; // minutes
	*(_buffer + 2) = _dec2bcd (h) & 0b00111111; // hours
	*(_buffer + 3) = _get_dow (mn, dy, yr); // calculate day of week
	*(_buffer + 4) = _dec2bcd (dy) & 0b00111111; // the date
	*(_buffer + 5) = ((_dec2bcd (mn) & 0b00011111) | (yr < 2000 ? 0x00 : 0x80)); // month and century bit
	*(_buffer + 6) = _dec2bcd (yr % 100); // year
	_write_raw (); // send it to the RTC

	// clear OSF when clock is set
	clrOSF ();

	return *(_buffer + 3); // return day of week(calculated by Zeller)
}

// read chip temperature in degrees C (0.25 degrees per bit resolution)
float DS3234::readTempC (void)
{
	int16_t temperature;
	_busy_wait (); // wait for not busy
	_command (0x8E, 0x20); // force a temperature conversion
	temperature = (_command (0x11, 0x00) << 8);
	temperature |= _command (0x12, 0x00);
	temperature >>= 6; // slide bits down
	return (float) (temperature / 4.0);
}

// just convert C to F
float DS3234::readTempF (void)
{
	return ((readTempC () * 1.8) + 32.0);
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

// read from RTC BSRAM any data type
template <class T> T DS3234::_readAll (uint8_t address, T &value)
{
	uint8_t x;
	uint8_t *ptr;

	x = sizeof (value);
	ptr = (uint8_t *) (void *) &value;

	while (x--) {
		_command (0x98, (address * sizeof (value)) + x);
		*(ptr + x) = _command (0x19, 0x00);
	}

	return value;
}

// write to RTC BSRAM any data type
template <class T> void DS3234::_writeAll (uint8_t address, const T &value)
{
	uint8_t x;
	uint8_t *ptr;

	x = sizeof (value);
	ptr = (uint8_t *) (void *) &value;

	while (x--) {
		_command (0x98, (address * sizeof (value)) + x);
		_command (0x99, *(ptr + x));
	}
}

// read 7 bytes of raw clock data to buffer
void DS3234::_read_raw (void)
{
	uint8_t x = 7;

	while (x--) {
		*(_buffer + x) = _command ((0x00 + x), 0x00);
	}
}

// write 7 bytes of raw clock data to buffer
void DS3234::_write_raw (void)
{
	uint8_t x = 7;

	while (x--) {
		_command ((0x80 + x), *(_buffer + x));
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
uint8_t DS3234::_get_dow (uint8_t mn, uint8_t dy, uint16_t yr)
{
	if (mn < 3) { // convert months to 2...14
		mn += 12;
		yr -= 1;
	}

	return (((dy + (((mn + 1) * 26) / 10) + yr + (yr / 4) + (6 * (yr / 100)) + (yr / 400)) - 1) % 7);
}

// convert decimal to BCD
uint8_t DS3234::_dec2bcd (uint8_t dec)
{
	return (dec / 10 * 16) + (dec % 10);
}

// convert BCD to decimal
uint8_t DS3234::_bcd2dec (uint8_t bcd)
{
	return (bcd / 16 * 10) + (bcd % 16);
}

// wait until status bit 2 (BSY) goes low
void DS3234::_busy_wait (void)
{
	while (_command (0x0F, 0x00) & 0b00000100);
}

// send command and send or receive data via SPI
uint8_t DS3234::_command (uint8_t cmd, uint8_t data)
{
	digitalWrite (_ss_pin, LOW);
	_spi_transfer (cmd);
	cmd = _spi_transfer (data);
	digitalWrite (_ss_pin, HIGH);
	return cmd;
}

// transfer one byte via SPI Mode 3
uint8_t DS3234::_spi_transfer (uint8_t data)
{
	uint8_t bits;

	bits = 8;

	while (bits--) {
		digitalWrite (_sck_pin, LOW);
		digitalWrite (_mosi_pin, data & _BV (bits) ? HIGH : LOW);
		digitalWrite (_sck_pin, HIGH);
		digitalRead (_miso_pin) ? data |= _BV (bits) : data &= ~_BV (bits);
	}

	return data;
}

// end of DS3234.cpp
