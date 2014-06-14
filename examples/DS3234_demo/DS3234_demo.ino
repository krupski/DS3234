////////////////////////////////////////////////////////////
//
//  Cheap & dirty test/demo program for the DS3234 library
//
//  Plug Sparkfun "Dead On" RTC board into Arduino as follows:
//
//	   _______________
//	  |   _________   |
//	  |  [         ]  |
//	  |  [ DS-3234 ]  |   GND  -> GND
//	  |  [_________]  |   VCC  -> VIN
//	  |               |   SQW  -> the gap
//	  | S M M C S V G |   CLK  -> A0
//	  | S O I L Q C N |   MISO -> A1
//	  |   S S K W C D |   MOSI -> A2
//	  |   I O         |   SS   -> A3
//	__________________________________________  DO NOT USE!
//	A A A A A A   V G G 5 3 R I        |     || EXTERNAL
//	5 4 3 2 1 0   I N N V . E O        |_____|| POWER JACK
//	              N D D   3 S R               |
//	                      V E E    ARDUINO    |
//	                        T F     BOARD     |
//
////////////////////////////////////////////////////////////

#include <DS3234.h>

static DS3234 RTC; // DS3234 object

#define GND_PIN  99 // bogus pin number because we have a real ground
#define VCC_PIN  13
#define SQW_PIN  12
#define CLK_PIN  11
#define MISO_PIN 10
#define MOSI_PIN  9
#define SS_PIN    8

#define BAUD_RATE 115200 // set as necessary

void setup (void)
{
	Serial.begin (BAUD_RATE);

	digitalWrite (GND_PIN, LOW); // bogus but needed if a different pin is used
	digitalWrite (VCC_PIN, HIGH); // steal VCC from Arduino port
	pinMode (GND_PIN, OUTPUT); // turn on GND and...
	pinMode (VCC_PIN, OUTPUT); // ... power pins.
	pinMode (SQW_PIN, INPUT); // don't load the SQW pin
	_delay_ms (100);

	// the driver returns error codes upon initialization:
	// 0 = Init OK
	// 1 = Oscillator has been stopped in the past or never was on
	// 2 = Stored Day of Week and calculated Day of Week do not match
	// 3 = Both errors
	// if the init returns non-zero, the time is probably wrong
	// and should be corrected or (re)set.
	const char *initmsg[] PROGMEM = {
		PSTR ("\r\nRTC Initialized OK\r\n"),
		PSTR ("\r\nRTC oscillator was stopped - time not accurate\r\n"),
		PSTR ("\r\nStored Day of Week mismatch - time not accurate\r\n"),
		PSTR ("\r\nOscillator and Day Of Week failure - time not accurate\r\n")
	};

	uint8_t init_status = RTC.init (CLK_PIN, MISO_PIN, MOSI_PIN, SS_PIN); // init the RTC chip

	Serial.print_P (initmsg[init_status]); // show startup status

	//	to set the RTC chip:
	//
	//	hour = 0; // setup proper time
	//	minute = 43;
	//	second = 0;
	//	month = 12;
	//	date = 14;
	//	year = 2013;
	//
	//	d_week = RTC.setTime (hour, minute, second, month, date, year);

	//  to READ the time:
	//	d_week = RTC.getTime (&hour, &minute, &second, &month, &day, &year);

}

// "walking bits" ram test
uint8_t test_sram (void)
{
	uint8_t data;
	uint8_t prev_data;
	uint16_t address;
	uint16_t errors;

	data = 0b00000000;
	errors = 0;

	for (address = 0; address < 256; address++) {
		RTC.writeByte (address, data); // fill ram with 0
	}

	do {

		Serial.print (".");

		prev_data = data; // copy old data
		data <<= 1; // shift up a bit
		data |= 1; // or in a bit

		for (address = 0; address < 256; address++) {

			if (prev_data != RTC.readByte (address)) {
				errors++;
			}

			RTC.writeByte (address, data);
		}

	} while (data != 0b11111111);

	do {

		Serial.print (".");

		prev_data = data; // copy old data
		data <<= 1; // shift up a bit
		data &= ~1; // and off bit 0

		for (address = 0; address < 256; address++) {

			if (prev_data != RTC.readByte (address)) {
				errors++;
			}

			RTC.writeByte (address, data);
		}

	} while (data != 0b00000000);

	return errors;
}


void loop (void)
{
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t old_second = 0xFF; // invalidate old_sec
	uint8_t day_of_week;
	uint8_t month;
	uint8_t day;
	uint16_t year; // 16 bits because year is full i.e. "2013".

	uint8_t test8;
	uint16_t test16;
	uint32_t test32;
	uint64_t test64;
	float testf;

	char buffer[64];

	const char *text[] PROGMEM = {
		PSTR ("\r\nTime: %02d:%02d:%02d\r\n"),
		PSTR ("Date: %s, %02d %s %04d\r\n"),
		PSTR ("Testing SRAM, please wait"),
		PSTR ("\r\nSRAM test: %d error%s\r\n"),
		PSTR ("Chip temperature is: "),
		PSTR ("Writing uint8_t value to chip %u\r\n"),
		PSTR ("Read back uint8_t value %u from chip\r\n"),
		PSTR ("Writing uint16_t value to chip %u\r\n"),
		PSTR ("Read back uint16_t value %u from chip\r\n"),
		PSTR ("Writing uint32_t value to chip %lu\r\n"),
		PSTR ("Read back uint32_t value %lu from chip\r\n"),
		PSTR ("Writing float value: "),
		PSTR ("Read back float value: ")
	};

	const char *days[] = { // these can't be in PROGMEM
		"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"
	};

	const char *months[] = { // these can't be in PROGMEM
		"NUL",
		"JAN", "FEB", "MAR", "APR", "MAY", "JUN",
		"JUL", "AUG", "SEP", "OCT", "NOV", "DEC"
	};

	do { // get time, wait until time changes to print it
		day_of_week = RTC.getTime (&hour, &minute, &second, &month, &day, &year);

	} while (old_second == second);

	old_second = second; // prepare for next second

	// print first message (time)
	sprintf_P (buffer, text[0], hour, minute, second);
	Serial.print (buffer); // display time

	// print second message (date)
	sprintf_P (buffer, text[1], days[day_of_week], day, months[month], year);
	Serial.print (buffer); // display date

	// "testing sram" message
	sprintf_P (buffer, text[2], days[day_of_week], day, months[month], year);
	Serial.print (buffer); // display date

	// test the SRAM
	uint16_t errors = test_sram ();

	// show if there were errors
	sprintf_P (buffer, text[3], errors, (errors == 1) ? "" : "s");
	Serial.print (buffer);

	// show chip temp degrees C
	sprintf_P (buffer, text[4]);
	Serial.print (buffer);
	dtostrf (RTC.readTempC(), 3, 2, buffer);
	Serial.print (buffer);
	Serial.print_P (PSTR (" degrees C\r\n"));

	// show chip temp degrees F
	sprintf_P (buffer, text[4]);
	Serial.print (buffer);
	dtostrf (RTC.readTempF(), 3, 2, buffer);
	Serial.print (buffer);
	Serial.print_P (PSTR (" degrees F\r\n"));

	test8 = 123;
	RTC.writeByte (0, test8);
	sprintf_P (buffer, text[5], test8);
	Serial.print (buffer);
	test8 = 0; // erase it
	test8 = RTC.readByte (0);
	sprintf_P (buffer, text[6], test8);
	Serial.print (buffer);

	test16 = 45678;
	RTC.writeWord (0, test16);
	sprintf_P (buffer, text[7], test16);
	Serial.print (buffer);
	test16 = 0; // erase it
	test16 = RTC.readWord (0);
	sprintf_P (buffer, text[8], test16);
	Serial.print (buffer);

	test32 = 1234567890;
	RTC.writeDWord (0, test32);
	sprintf_P (buffer, text[9], test32);
	Serial.print (buffer);
	test32 = 0; // erase it
	test32 = RTC.readDWord (0);
	sprintf_P (buffer, text[10], test32);
	Serial.print (buffer);

	testf = 1234.567890;
	RTC.writeFloat (0, testf);
	sprintf_P (buffer, text[11]);
	Serial.print (buffer);
	dtostrf (testf, 4, 3, buffer);
	Serial.println (buffer);

	testf = 0; // erase it
	testf = RTC.readFloat (0);
	sprintf_P (buffer, text[12]);
	Serial.print (buffer);
	dtostrf (testf, 4, 3, buffer);
	Serial.println (buffer);
}

