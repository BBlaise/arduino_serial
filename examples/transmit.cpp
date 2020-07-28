#include "Arduino.h"

// Arduino Libraries
#include "TimerOne.h"

// C/C++ Libraries
#include <stdint.h>

// Custom Libraries
#include "bitwise.h"
#include "arduino_uart.h"

// Function Prototypes
void timer1ISR(void);

// Pin Assignments

// Objects

// Global Variables
uint32_t timer1_period = 1000;		// timer period [ms]
uint32_t baud_rate = 115200;
//uint32_t baud_rate = 9600;
float tx_data[2] = {-1.2, 3.4};		// Array of floats to be transmitted
//int tx_data[2] = {-1, 10000};		// Array to ints (int16_t) be transmitted
int n_tx = sizeof(tx_data) / sizeof(tx_data[0]); // number of bytes to be transmitted

void setup(){
	Serial.begin(baud_rate);					// open serial port
	delay(100);								// wait for serial port to open

	Timer1.initialize(timer1_period*1000);
	Timer1.attachInterrupt(timer1ISR);		// start program/timer ISR
}

void loop(){}

void timer1ISR(void){
	//Serial.println("timer1 loop");

	// Transmit an array of floats
	uartTransmit(tx_data, n_tx);
	Serial.write('\n');			// write newline if transmitting to a C++ script
}
