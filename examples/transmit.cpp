#include "Arduino.h"
#include "ArxTypeTraits.h"

// Arduino Libraries
#include "TimerOne.h"

// Custom Libraries
#include "bitwise.h"
#include "arduino_serial.h"

// Preprocessor Directives
#define DEBUG 0
#if DEBUG == 1
#define toConsole(x) Serial.print(x);
#define toConsoleLn(x) Serial.println(x)
#else
#define toConsole(x)
#define toConsoleLn(x)
#endif

// Function Prototypes
void timer1ISR(void);

void setup() {
	uint32_t baud_rate = 115200;
	Serial.begin(baud_rate);						// open serial port
	delay(100);										// wait for serial port to open
	//flushSerialInputBuffer();						// clears input buffer
	Serial.flush();									// clears output buffer

	uint32_t timer1_period = 1000;					// timer period [ms]
	Timer1.attachInterrupt(timer1ISR);				// start program/timer ISR
	Timer1.initialize(timer1_period*1000);
}

void loop() {}

void timer1ISR(void) {
	//Serial.println("timer1 loop");

	const char* terminator = "\r\n";				// write extra newline if transmitting to a C++ script

	// Transmit a single int or float
	//static uint8_t tx = 254;
	//static uint16_t tx = 12345;
	//static uint32_t tx = 111222333;
	//static int8_t tx = -100;
	//static int16_t tx = -12345;
	//static int32_t tx = 111222333;
	//static int tx = -12345;
	static float tx = 1.1;

	//serialTransmitBinary(tx);
	serialTransmitAscii(tx);
	//serialTransmitAscii(tx, terminator);
	//serialTransmitAscii("yo");					// can send strings too

	toConsoleLn(tx);								// print to console for debugging if DEBUG = 1

	tx--;
}
