// Do not remove the include below
#include "arduino_uart_devel.h"

// Arduino Libraries
#include "TimerOne.h"

// C/C++ Libraries
#include <stdint.h>

// Custom Libraries
#include "bitwise.h"
#include "arduino_uart.h"

// Function Prototypes
void timer1ISR(void);

// Objects
uint32_t timer1_period = 1000;		// timer period [ms]

float tx_floats[2] = {1.2, -3.4};		// Array to be transmitted
int n_tx_floats = sizeof(tx_floats) / sizeof(tx_floats[0]);

void setup(){
	Timer1.initialize(timer1_period*1000);
	Timer1.attachInterrupt(timer1ISR);		// start program/timer ISR

	Serial.begin(115200);
	delay(100);
}

void loop(){}

void timer1ISR(void){
	//Serial.println("timer1 loop");

	// Transmit a single integer
	//uartTransmit(tx_int);

	// Transmit an array of floats
	uartTransmit(tx_floats, n_tx_floats);
}
