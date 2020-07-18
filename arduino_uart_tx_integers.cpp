
/*
This script shows how to communicate via UART to a PC or another serial device.
Every timer period, the Arduino will convert an array (vector) of integers to
their corresponding bytes and send those bytes over the TX line
*/

// Do not remove the include below
#include "arduino_uart_devel.h"

// Arduino Libraries
#include <TimerOne.h>

// C/C++ Libraries
#include <stdint.h>

// Custom Libraries
#include "bitwise.h"
#include "arduino_uart.h"

// Function Prototypes
void timer1ISR(void);			// timer loop

// Objects
uint32_t timer1_period = 1000;		// timer period [ms]

//int tx_int = 5;			// if sending single integer
int tx_ints[2] = {3, 4};		// Array to be transmitted
//int tx_ints[2] = {24930, 25444};	// {ab, cd} in ASCII (for debugging)
int n_tx_ints = sizeof(tx_ints) / sizeof(tx_ints[0]);	// number of ints sending

void setup(){
	Serial.begin(115200);	// start/open serial port
	delay(100);				// wait for serial port to be opened

	Timer1.initialize(timer1_period*1000);
	Timer1.attachInterrupt(timer1ISR);		// start program/timer ISR
}

void loop(){}

void timer1ISR(void){
	//Serial.println("timer1 loop");

	// Transmit a single integer
	//uartTransmit(tx_int);

	// Transmit an array of integers
	uartTransmit(tx_ints, n_tx_ints);
}
