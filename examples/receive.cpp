#include "Arduino.h"

// Arduino Libraries
#include <TimerOne.h>

// C/C++ Libraries

// Custom Libraries
#include "bitwise.h"

#define SERIAL_TEMPLATES 0
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
void timer1ISR(void);			// timer loop
void blinkLED(int n_blinks);

// Pin Assignments
const int led_pin = 13;

// Global Variables
char rx_buffer[20] = {};
unsigned int buffer_indx = 0;
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
	// uint8_t rx = 0;		 uint8_t rx_expected = 254U;
	// uint16_t rx = 0;			uint16_t rx_expected = 65534U;
	// uint32_t rx = 0;		uint32_trx_expected = 111222333UL;
	// int8_t rx = 0;				int8_t rx_expected = -126;
	// int16_t rx = 0;				int16_t rx_expected = -32766;
	// int32_t rx = 0;			int32_t rx_expected = -111222333;
	// int rx = 0;		int rx_expected = -10;
	float rx = 0.0;		float rx_expected = -1.2;
		
	// ---------- Receive Individual Values As Binary -------- //
	// int n_rx_bytes = sizeof(rx);
	// if (Serial.available() >= n_rx_bytes) {
	// 	rx = serialReceiveBinary(rx);
	// 	if (rx == rx_expected) blinkLED(n_rx_bytes);
	// }

	// -------- Receive Individual Values As Ascii -------- //
	if (rx_done == true) {
		noInterrupts();

		// toConsoleln(rx_buffer);
		rx = serialReceiveAscii(rx, rx_buffer);	
		if (rx == rx_expected) blinkLED(3);		// doesn't work if float b/c precision but number is correct
		// toConsoleLn(rx);					// shows that floats are coming in correctly

		rx_done = false;
		interrupts();
	}
}

void serialEvent() {
	rx_done = serialReadChar(rx_buffer, buffer_indx);
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
