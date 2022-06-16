#include "Arduino.h"
#include "ArxTypeTraits.h"

// Arduino Libraries
#include "TimerOne.h"

// Custom Libraries
#include "bitwise.h"
#include "arduino_serial.h"

// Preprocessor Directives
#define DEBUG 1
#if DEBUG == 1
#define toConsole(x) Serial.print(x);
#define toConsoleLn(x) Serial.println(x)
#else
#define toConsole(x)
#define toConsoleLn(x)
#endif

// Function Prototypes
void timer1ISR(void);
void buttonISR(void);
void blinkLED(void);

// Pin Assignments
//const int led_pin = 13;
const int led_pin = 7;
int button_pin = 2;
volatile bool tx_now = false;
volatile int last_press = 0;
const int debounce_delay = 200;

void setup() {
	pinMode(led_pin, OUTPUT);
	pinMode(button_pin, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(button_pin), buttonISR, FALLING);

	uint32_t baud_rate = 115200;
	Serial.begin(baud_rate);					// open serial port
	delay(100);								// wait for serial port to open

	//flushSerialInputBuffer();		// clears any bytes waiting in the UART's receive buffer

	uint32_t timer1_period = 1000;		// timer period [ms]
	//Timer1.attachInterrupt(timer1ISR);		// start program/timer ISR
	//Timer1.initialize(timer1_period*1000);
}

void loop() {
	//const char* terminator = "\r\n";		// write extra newline if transmitting to a C++ script
	if (tx_now == true) {
		noInterrupts();
		//static int tx_msg = 1;
		// serialTransmitAscii(tx_msg);
		// tx_msg--;
		char tx_msg[5] = {'a', 'b', 'c', 'd', 'e'};
		static int ii = 0; static int s = 1;

		serialTransmitAscii(tx_msg[ii]);
		if (ii == 4) s = -1;
		else if (ii == 0) s = 1;
		ii = ii + s;
		blinkLED();
		
		tx_now = false;

		interrupts();
	}
}

void buttonISR(void) {
	int current_press = millis();
	if( (current_press - last_press) > debounce_delay ) {
		last_press = current_press;
		tx_now = true;
	}
}

void timer1ISR(void) {
	//Serial.println("timer1 loop");
	// blinkLED();
}

void blinkLED(void) {
	digitalWrite(led_pin, HIGH);
	delay(100);
	digitalWrite(led_pin, LOW);
}
