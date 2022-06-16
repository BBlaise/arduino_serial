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
	Serial.begin(baud_rate);					// open serial port
	delay(100);								// wait for serial port to open

	//flushSerialInputBuffer();		// clears any bytes waiting in the UART's receive buffer

	uint32_t timer1_period = 1000;		// timer period [ms]
	Timer1.attachInterrupt(timer1ISR);		// start program/timer ISR
	Timer1.initialize(timer1_period*1000);
}

void loop(){}

void timer1ISR(void) {
	//toConsoleLn("timer1 loop");

	const char* deliminator = "\t";
	const char* terminator = "\r\n";		// write extra newline if transmitting to a C++ script
	// const char* terminator = "\r\n\n";		// write extra newline if transmitting to a C++ script

	// Transmit an array of floats or ints
	//static uint8_t tx_data[2] = {1, 254};		// Array of uint8_t values to be transmitted
	//static uint16_t tx_data[2] = {1, 65534};		// Array to uint16_t values to be transmitted
	//static uint32_t tx_data[2] = {1, 111222333};	
	//static int8_t tx_data[2] = {1, 127};
	//static int16_t tx_data[2] = {1, 32765};
	//static int32_t tx_data[2] = {-1, 111222333};		// Array of int values (int16_t) be transmitted
	static int tx_data[2] = {-1, 10000};		// Array of int values (int16_t) be transmitted
	//static float tx_data[2] = {-1.2, 3.4};		// Array of float values to be transmitted
	size_t n_tx = sizeof(tx_data) / sizeof(tx_data[0]); // number of elements to be transmitted
	
	//serialTransmitBinary(tx_data, n_tx);
	//serialTransmitAscii(tx_data, n_tx);
	serialTransmitAscii(tx_data, n_tx, deliminator, terminator);

	toConsoleLn(tx_data);

	tx_data[0]--; tx_data[1]++;
}
