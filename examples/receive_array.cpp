#include "Arduino.h"

// Arduino Libraries
#include <TimerOne.h>

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
void blinkLED(int n_blinks);

// Pin Assignments
const int led_pin = 13;

// Global Variables
char rx_buffers[2][20] = {};
unsigned int buffer_indxs[2] = {0, 0};
bool rx_done = false;

void setup(){
	pinMode(led_pin, OUTPUT);				// configure LED pin for debugging

	uint32_t baud_rate = 115200;
	Serial.begin(baud_rate);				// open serial port
	//Serial.setTimeout(1500);				
	delay(100);								// wait for serial port to open
	Serial.flush();
	flushSerialInputBuffer();					// clear any bytes that are in the UART buffer

	uint32_t timer1_period = 1000;		// timer period [in milliseconds]
	Timer1.initialize(timer1_period*1000);
	Timer1.attachInterrupt(timer1ISR);		// start program/timer ISR
}

// Do all your calculations/sensor readings in this folder
void timer1ISR(void) {}

void loop() {
	// uint8_t rx_array[2] = {};				uint8_t rx_expected[2] = {1, 254};
	// uint16_t rx_array[2] = {};			uint16_t rx_expected[2] = {1, 65534};
	// uint32_t rx_array[2] = {};			uint32_t rx_expected[2] = {1, 111222333};
	// int8_t rx_array[2] = {};				int8_t rx_expected[2] = {-1, 126};
	// int16_t rx_array[2] = {};				int16_t rx_expected[2] = {-1, -32766};
	// int32_t rx_array[2] = {};				int32_t rx_expected[2] = {1, -111222333};
	int rx_array[2] = {};					int rx_expected[2] = {-1, 10000};
	// float rx_array[2] = {};				float rx_expected[2] = {-1.2, 3.4};
	unsigned int n_rx = sizeof(rx_array) / sizeof(rx_array[0]);

	// -------- Receive An Array of Values As Binary --------- //
	//unsigned int n_rx_bytes = n_rx * sizeof(rx_array[0]);				// receiving binary
	//if (Serial.available() >= n_rx_bytes) serialReceiveBinary(rx_array, n_rx);		// if total amount of bytes expected -> read buffer data and convert from bytes to ints or floats

	// -------- Receive An Array of Values As Ascii -------- //
	if (rx_done == true) {
		noInterrupts();

		//toConsole(rx_buffers[0]); toConsole('\t'); toConsoleLn(rx_buffers[1]);
		serialReceiveAscii(rx_array, n_rx, rx_buffers);
		//toConsole(rx_array[0]); toConsole(','); toConsoleLn(rx_array[1]);

		if(rx_array[0] == rx_expected[0] && rx_array[1] == rx_expected[1]) blinkLED(4);	// check to make sure values are as expected
		else if(rx_array[0] == rx_expected[0] && rx_array[1] != rx_expected[1]) blinkLED(1);
		else if(rx_array[0] != rx_expected[0] && rx_array[1] == rx_expected[1]) blinkLED(2);

		rx_done = false;
		interrupts();
	}
}

void serialEvent() {
	noInterrupts();
	rx_done = serialReadChar(rx_buffers, buffer_indxs);
	interrupts();
}

// Blink the LED pin to debug
void blinkLED(int n_blinks) {
	for (int ii = 0; ii < n_blinks; ii++) {
		digitalWrite(led_pin, HIGH);
		delay(100);
		digitalWrite(led_pin, LOW);
		delay(50);			
	}
}
