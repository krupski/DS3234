/////////////////////////////////////////////////////////////
//
//  Cheap & dirty test/demo program for the DS3234 library
//
//  Plug Sparkfun "Dead On" RTC board into an Arduino UNO,
//  DUEMILANOVE or MEGA board as follows:
//
//    RTC PIN  ARDUINO PIN
//    -------  -----------
//      GND -----> GND
//      VCC -----> 13
//      SQW -----> 12
//      CLK -----> 11
//      MISO ----> 10
//      MOSI ---->  9
//      SS ------>  8
//                                           |
//               +---\_____/---+             |
//               |  +-------+  |             |
//               |  |DS-3234|  |             |
//               |  +-------+  |             |
//               |  M M        |             |
//               |  O I C S V G|             |
//               |S S S L Q C N|             |
//               |S I O K W C D|         +--------+
//               +-------------+         |  USB   |
//                | | | | | | |          |  PORT  |
//                _ _ _ _ _ _ _ _ _ _    +--------+
//               |_|_|_|_|_|_|_|_|_|_|       |
//    ---------------------------------------+
//    arduino         1 1 1 1 G A
//    pins------> 8 9 0 1 2 3 N R
//                            D E
//                              F
//
/////////////////////////////////////////////////////////////

#include <DS3234.h>

uint8_t init_status;
char buffer[64];

// the driver returns error codes upon initialization:
// 0 = Init OK
// 1 = Oscillator has been stopped in the past or never was on
// 2 = Stored Day of Week and calculated Day of Week do not match
// 3 = Both errors
// if the init returns non-zero, the time is probably wrong
// and should be corrected or (re)set.
const char *initmsg[] = {
	"\nRTC Initialized OK\n",
	"\nRTC oscillator was stopped - time not accurate\n",
	"\nStored Day of Week mismatch - time not accurate\n",
	"\nOscillator and Day Of Week failure - time not accurate\n",
};

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

	delay (100); // let clock settle (why not?)

	init_status = RTC.init (CLK_PIN, MISO_PIN, MOSI_PIN, SS_PIN); // init the RTC chip

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
	//	d_week = RTC.getTime (hour, minute, second, month, day, year);
	//  note: all time vars are uint8_t except year which is uint16_t

}

// "walking bits" ram test
uint8_t test_sram (void)
{
	uint8_t data;
	uint8_t prev_data;
	uint8_t pct;
	uint16_t address;
	uint16_t errors;
	const uint8_t percent[] = { 6, 12, 18, 25, 31, 37, 43, 50, 56, 62, 68, 75, 81, 87, 93, 100 };

	data = 0b00000000;
	errors = 0;
	pct = 0;

	for (address = 0; address < 256; address++) {
		RTC.writeByte (address, data); // fill ram with 0
	}

//	Serial.print ("\nSetting SRAM bits..."); // show progress

	do {

		sprintf (buffer, " %u%%", percent[pct++]);
		Serial.print (buffer);
	//	Serial.print ("."); // show progress

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

//	Serial.print ("\nClearing SRAM bits..."); // show progress

	do {

		sprintf (buffer, " %u%%", percent[pct++]);
		Serial.print (buffer);
	//	Serial.print (".");

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

	const char *text[] = {
		"\nTime: %02d:%02d:%02d\n",
		"Date: %s, %02d %s %04d\n",
		"\nTesting SRAM, please wait...\n",
		"\nSRAM test: %d error%s\n",
		"Chip temperature is: ",
		"Writing uint8_t value to chip %u\n",
		"Read back uint8_t value %u from chip",
		"Writing uint16_t value to chip %u\n",
		"Read back uint16_t value %u from chip",
		"Writing uint32_t value to chip %lu\n",
		"Read back uint32_t value %lu from chip",
		"Writing float value to chip: ",
		"Read back float value from chip: ",
	};

	const char *days[] = {
		"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT",
	};

	const char *months[] = {
		"NUL", // no month zero!
		"JAN", "FEB", "MAR", "APR", "MAY", "JUN",
		"JUL", "AUG", "SEP", "OCT", "NOV", "DEC",
	};

	do { // get time, wait until time changes to print it
		day_of_week = RTC.getTime (hour, minute, second, month, day, year);

	} while (old_second == second);

	old_second = second; // prepare for next second

	Serial.print (initmsg[init_status]); // show startup status

	// print first message (time)
	sprintf (buffer, text[0], hour, minute, second);
	Serial.print (buffer); // display time

	// print second message (date)
	sprintf (buffer, text[1], days[day_of_week], day, months[month], year);
	Serial.print (buffer); // display date

	// "testing sram" message
	sprintf (buffer, text[2], days[day_of_week], day, months[month], year);
	Serial.print (buffer); // display date

	// test the SRAM
	uint16_t errors = test_sram ();

	// show if there were errors
	sprintf (buffer, text[3], errors, (errors == 1) ? "" : "s");
	Serial.print (buffer);

	// show chip temp degrees C
	sprintf (buffer, text[4]);
	Serial.print (buffer);
	dtostrf (RTC.getTempC(), 3, 2, buffer);
	Serial.print (buffer);
	Serial.print (" degrees C\n");

	// show chip temp degrees F
	sprintf (buffer, text[4]);
	Serial.print (buffer);
	dtostrf (RTC.getTempF(), 3, 2, buffer);
	Serial.print (buffer);
	Serial.print (" degrees F\n");

	test8 = 123;
	RTC.writeByte (0, test8);
	sprintf (buffer, text[5], test8);
	Serial.print (buffer);
	if (test8 == RTC.readByte (0)) {
		sprintf (buffer, text[6], test8);
		Serial.print (buffer);
		Serial.print (" PASS!\n");
	} else {
		sprintf (buffer, text[6], RTC.readByte (0));
		Serial.print (buffer);
		Serial.print (" FAIL!\n");
	}

	test16 = 45678;
	RTC.writeWord (0, test16);
	sprintf (buffer, text[7], test16);
	Serial.print (buffer);
	if (test16 == RTC.readWord (0)) {
		sprintf (buffer, text[8], test16);
		Serial.print (buffer);
		Serial.print (" PASS!\n");
	} else {
		sprintf (buffer, text[8], RTC.readWord (0));
		Serial.print (buffer);
		Serial.print (" FAIL!\n");
	}

	test32 = 1234567890;
	RTC.writeDWord (0, test32);
	sprintf (buffer, text[9], test32);
	Serial.print (buffer);
	if (test32 == RTC.readDWord (0)) {
		sprintf (buffer, text[10], test32);
		Serial.print (buffer);
		Serial.print (" PASS!\n");
	} else {
		sprintf (buffer, text[10], RTC.readDWord (0));
		Serial.print (buffer);
		Serial.print (" FAIL!\n");
	}

	testf = 1234.567890;
	RTC.writeFloat (0, testf);
	sprintf (buffer, text[11]);
	Serial.print (buffer);
	dtostrf (testf, 4, 3, buffer);
	Serial.println (buffer);

	sprintf (buffer, text[12]);
	Serial.print (buffer);
	dtostrf (testf, 4, 3, buffer);
	Serial.print (buffer);
	if (testf = RTC.readFloat (0)) {
		Serial.print (" PASS!\n");
	} else {
		Serial.print (" FAIL!\n");
	}

	Serial.print ("\nWaiting 3 seconds.");
	delay (500);
	Serial.print (".");
	delay (500);
	Serial.print (".");
	delay (500);
	Serial.print (".");
	delay (500);
	Serial.print (".");
	delay (500);
	Serial.print (".");
	delay (500);
	Serial.print ("\n");
}

