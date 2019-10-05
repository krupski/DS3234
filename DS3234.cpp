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

#include <DS3234.h>

DS3234::DS3234 (uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs, uint8_t sqw)
{
	uint8_t n;

	// get ports, pins & ddr's
	n = digitalPinToPort (sck);
	_SCK_OUT = portOutputRegister (n);
	_SCK_DDR = portModeRegister (n);

	n = digitalPinToPort (miso);
	_MISO_INP = portInputRegister (n);
	_MISO_OUT = portOutputRegister (n);
	_MISO_DDR = portModeRegister (n);

	n = digitalPinToPort (mosi);
	_MOSI_OUT = portOutputRegister (n);
	_MOSI_DDR = portModeRegister (n);

	n = digitalPinToPort (cs);
	_CS_OUT = portOutputRegister (n);
	_CS_DDR = portModeRegister (n);

	// bitmasks
	_SCK_BIT = digitalPinToBitMask (sck);
	_MISO_BIT = digitalPinToBitMask (miso);
	_MOSI_BIT = digitalPinToBitMask (mosi);
	_CS_BIT = digitalPinToBitMask (cs);

	// set initial pin values
	*_SCK_OUT |= _SCK_BIT; // SCK idles high in mode 3
	*_MISO_OUT |= _MISO_BIT; // set MISO to "input pullup" mode
	*_MOSI_OUT |= _MOSI_BIT; // set MOSI initially high
	*_CS_OUT |= _CS_BIT; // set SS high makes us the SPI master

	// set DDR's to output (except MISO and SQW which are input)
	*_SCK_DDR |= _SCK_BIT; // output
	*_MISO_DDR &= ~_MISO_BIT; // input
	*_MOSI_DDR |= _MOSI_BIT; // output
	*_CS_DDR |= _CS_BIT; // output

	if (sqw != 255) { // optional square wave / alarm interrupt pin
		n = digitalPinToPort (sqw);
		_SQW_INP = portInputRegister (n);
		_SQW_OUT = portOutputRegister (n);
		_SQW_DDR = portModeRegister (n);
		_SQW_BIT = digitalPinToBitMask (sqw);
		*_SQW_DDR &= ~_SQW_BIT; // input
		*_SQW_OUT |= _SQW_BIT; // enable pullup
	}
}

uint8_t DS3234::begin (void)
{
	uint8_t dow, mon, day;
	uint16_t year;

	// setup control register bits (0x0E/0x8E)
	_clr_rtc_bit (CTRL, EOSC); // enable the oscillator
	__builtin_avr_delay_cycles (F_CPU / (MSEC / 250)); // let oscillator stabilize
	_set_rtc_bit (CTRL, BBSQW); // enable the square wave output
	_clr_rtc_bit (CTRL, RS1); // set squarewave...
	_clr_rtc_bit (CTRL, RS2); // ...rate to 1 Hz.
	_clr_rtc_bit (CTRL, INTCN); // direct square wave to SQW pin

	// check "oscillator has been stopped" flag
	status = ((_command ((STAT | RD), 0) & OSF) ? 1 : 0);
	_read_time(); // read RTC into buffer as BCD

	// check if stored DOW matched calculated DOW
	dow = _bcd2dec (_clockBuf[DOW] % DOW_MASK); // dow = stored day of week
	day = _bcd2dec (_clockBuf[DAY] & DAY_MASK);
	mon = _bcd2dec (_clockBuf[MON] & MON_MASK);
	year = ((_clockBuf[MON] & CENTBIT) ? 2000 : 1900);
	year += _bcd2dec (_clockBuf[YEAR]);

	status += (dow == _get_dow (day, mon, year)) ? 0 : 2; // compare stored to calculated

	return status;
}

// get the time, converted from BCD to decimal
// return day of week from chip (not from Zeller)
uint8_t DS3234::getTime (int &sec, int &min, int &hrs, int &day, int &mon, int &yrs)
{
	return getTime (
		(uint8_t &)(sec),
		(uint8_t &)(min),
		(uint8_t &)(hrs),
		(uint8_t &)(day),
		(uint8_t &)(mon),
		(uint16_t &)(yrs)
	);
}

uint8_t DS3234::getTime (uint8_t &sec, uint8_t &min, uint8_t &hrs, uint8_t &day, uint8_t &mon, uint16_t &yrs)
{
	uint8_t dow;

	_read_time(); // read RTC into buffer as BCD

	sec = _bcd2dec (_clockBuf[SEC] & SEC_MASK);
	min = _bcd2dec (_clockBuf[MIN] & MIN_MASK);
	hrs = _bcd2dec (_clockBuf[HRS] & HRS_MASK);
	dow = _bcd2dec (_clockBuf[DOW] % DOW_MASK);
	day = _bcd2dec (_clockBuf[DAY] & DAY_MASK);
	mon = _bcd2dec (_clockBuf[MON] & MON_MASK);
	yrs = ((_clockBuf[MON] & CENTBIT) ? 2000 : 1900);
	yrs += _bcd2dec (_clockBuf[YEAR]);

	return dow;
}

// Set the time - decimal is converted to bcd for the clock chip.
// If year > 1999, set the century bit, else clear it.
// Day of week is automatically calculated with Zeller's congruence.
// Setting the time also clears the Oscillator Stop Flag (status
// register 0x0F, bit 7) to indicate that the time is valid.
// Time becomes invalid if the battery is removed or dies.
uint8_t DS3234::setTime (int sec, int min, int hrs, int day, int mon, int yrs)
{
	return setTime (
		(uint8_t)(sec),
		(uint8_t)(min),
		(uint8_t)(hrs),
		(uint8_t)(day),
		(uint8_t)(mon),
		(uint16_t)(yrs)
	);
}

uint8_t DS3234::setTime (uint8_t sec, uint8_t min, uint8_t hrs, uint8_t day, uint8_t mon, uint16_t yrs)
{
	uint8_t dow = _get_dow (day, mon, yrs); // calculate day of week with Zeller
	uint8_t cent = ((yrs < 2000) ? 0 : CENTBIT); // calculate century bit

	_clockBuf[SEC]  = _dec2bcd (sec); // decimal to BCD seconds
	_clockBuf[MIN]  = _dec2bcd (min); // decimal to BCD minutes
	_clockBuf[HRS]  = _dec2bcd (hrs); // decimal to BCD hours
	_clockBuf[DOW]  = _dec2bcd (dow); // decimal to BCD day of the week
	_clockBuf[DAY]  = _dec2bcd (day); // decimal to BCD day of the month
	_clockBuf[MON]  = _dec2bcd (mon) | cent; // decimal to BCD month and century bit
	_clockBuf[YEAR] = _dec2bcd (yrs - (cent ? 2000 : 1900)); // decimal to BCD year (2 digit)

	_write_time(); // update the RTC time

	_clearOSF(); // clear the "Oscillator has been stopped" flag bit

	return dow;
}

// read chip temperature in degrees C (0.25 degrees per bit resolution)
double DS3234::getTempC (void)
{
	uint8_t timeout = 200; // prevent lockup if chip fails
	while ((_command ((STAT | RD), 0) & BSY) && timeout--); // wait for not busy
	_set_rtc_bit (CTRL, CONV); // force a temperature conversion
	__builtin_avr_delay_cycles (F_CPU / (MSEC / 5)); // wait for BSY to become valid
	timeout = 200; // prevent lockup if chip fails
	while ((_command ((CTRL | RD), 0) & (CONV | BSY)) && timeout--); // wait for conversion complete
	return (double)(((_command ((TMSB | RD), 0) << 8 | _command ((TLSB | RD), 0)) >> 6) / 4.0);
}

// just convert C to F
double DS3234::getTempF (void)
{
	return ((getTempC() * 1.8) + 32.0);
}

uint8_t DS3234::readByte (uint8_t addr)
{
	uint8_t value;
	return _readAll (addr, value);
}

uint16_t DS3234::readWord (uint8_t addr)
{
	uint16_t value;
	return _readAll (addr, value);
}

uint32_t DS3234::readDWord (uint8_t addr)
{
	uint32_t value;
	return _readAll (addr, value);
}

uint64_t DS3234::readQWord (uint8_t addr)
{
	uint64_t value;
	return _readAll (addr, value);
}

float DS3234::readFloat (uint8_t addr)
{
	float value;
	return _readAll (addr, value);
}

double DS3234::readDouble (uint8_t addr)
{
	double value;
	return _readAll (addr, value);
}

long double DS3234::readLongDouble (uint8_t addr)
{
	long double value;
	return _readAll (addr, value);
}

void DS3234::writeByte (uint8_t addr, uint8_t value)
{
	_writeAll (addr, value);
}

void DS3234::writeWord (uint8_t addr, uint16_t value)
{
	_writeAll (addr, value);
}

void DS3234::writeDWord (uint8_t addr, uint32_t value)
{
	_writeAll (addr, value);
}

void DS3234::writeQWord (uint8_t addr, uint64_t value)
{
	_writeAll (addr, value);
}

void DS3234::writeFloat (uint8_t addr, float value)
{
	_writeAll (addr, value);
}

void DS3234::writeDouble (uint8_t addr, double value)
{
	_writeAll (addr, value);
}

void DS3234::writeLongDouble (uint8_t addr, long double value)
{
	_writeAll (addr, value);
}

///////////////////////////////////////////////////////////////////
/////////////////// private functions begin here //////////////////
///////////////////////////////////////////////////////////////////
// clear the "Oscillator has been stopped" flag bit
void DS3234::_clearOSF (void)
{
	_read_time(); // read RTC into buffer
	_clr_rtc_bit (STAT, OSF); // clear "oscillator stopped" flag
	_write_time(); // send it to the RTC
}

// read from RTC BSRAM any data type
template <class T> T DS3234::_readAll (uint8_t address, const T &value)
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
template <class T> void DS3234::_writeAll (uint8_t addr, const T &value)
{
	uint8_t x = sizeof (value);
	uint8_t *ptr = (uint8_t *) (void *) &value;

	while (x--) {
		_command ((RADR | WR), addr + x);
		_command ((RDAT | WR), *(ptr + x));
	}
}

//////////////////////////////////////////////////////////////////////////
// zeller's congruence (calculates day of week) modified
// so that sunday=0, monday=1 ... friday=5, saturday=6
// reference: http://en.wikipedia.org/wiki/Zeller's_congruence
//////////////////////////////////////////////////////////////////////////
//
// | |(m + 1) * 26| | y | | y | | y | |
// h = | q + |------------| + y + |---| + 6 * |---| + |---| - 1 | % 7
// | | 10 | | 4 | |100| |400| |
//
//////////////////////////////////////////////////////////////////////////
uint8_t DS3234::_get_dow (uint8_t d, uint8_t m, uint16_t y)
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
	uint8_t n;

	*_CS_OUT &= ~_CS_BIT; // assert cs
	_spi_transfer (RD); // send read command

	for (n = 0; n < 7; n++) {
		_clockBuf[n] = _spi_transfer (0); // read 7 bytes in sequence
	}

	*_CS_OUT |= _CS_BIT; // release cs
}

// write 7 bytes of raw clock data from buffer
void DS3234::_write_time (void)
{
	uint8_t n;

	*_CS_OUT &= ~_CS_BIT; // assert cs
	_spi_transfer (WR); // send write command

	for (n = 0; n < 7; n++) {
		_spi_transfer (_clockBuf[n]); // write 7 bytes in sequence
	}

	*_CS_OUT |= _CS_BIT; // release cs
}

// send or receive data via SPI
uint8_t DS3234::_command (uint8_t cmd, uint8_t data)
{
	*_CS_OUT &= ~_CS_BIT; // assert cs
	_spi_transfer (cmd);
	data = _spi_transfer (data);
	*_CS_OUT |= _CS_BIT; // release cs
	return data;
}

// SPI Mode 3 (CPOL 1, CPHA 1)
uint8_t DS3234::_spi_transfer (uint8_t data)
{
	uint8_t bits = 8;

	while (bits--) {
		*_SCK_OUT &= ~_SCK_BIT; // sck low
		data & (1 << bits) ? *_MOSI_OUT |= _MOSI_BIT : *_MOSI_OUT &= ~_MOSI_BIT; // send bit
		*_SCK_OUT |= _SCK_BIT; // sck high
		*_MISO_INP & _MISO_BIT ? data |= (1 << bits) : data &= ~ (1 << bits); // receive bit
	}

	return data;
}

// end of DS3234.cpp

