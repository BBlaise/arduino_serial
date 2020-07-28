#include "Arduino.h"

// Arduino Libraries
#include <TimerOne.h>

// C/C++ Libraries
#include <stdint.h>

// Custom Libraries
#include "bitwise.h"
#include "arduino_uart.h"

// Function Prototypes
void timer1ISR(void);			// timer loop
//void flushSerialBuffer(void);
void blinkLED(void);

// Pin Assignments
const int led_pin = 13;

// Objects

// Global Variables
uint32_t timer1_period = 1000;			// timer period [in milliseconds]
uint32_t baud_rate = 115200;
//uint32_t baud_rate = 9600;
bool driving_uart = false;			// false if the other device controls the UART communication timing
//bool driving_uart = true;				// false the Arduino controls the UART communication timing

// Data to Transmit via UART
float tx_data[2] = {-1.2, 3.4};		// Array of floats to be transmitted
//int tx_data[2] = {-1, 10000};		// Array to ints (int16_t) be transmitted
int n_tx = sizeof(tx_data) / sizeof(tx_data[0]); // number of bytes to be transmitted

// Data to Receive via UART
float rx_data[2] = {};	float rx_expected[2] = {-1.2, 3.4};			// values you expect to be receiving via UART
//int rx_data[2] = {};	float rx_expected[2] = {-1, 10000};			// values you expect to be receiving via UART
int n_rx = sizeof(rx_data) / sizeof(rx_data[0]);
int n_rx_bytes = n_rx * sizeof(rx_data[0]);

void setup(){
	pinMode(led_pin, OUTPUT);				// configure LED pin for debugging

	Serial.begin(baud_rate);				// open serial port
	delay(100);								// wait for serial port to open

	flushSerialBuffer();					// clear any bytes that are in the UART buffer

	Timer1.initialize(timer1_period*1000);
	Timer1.attachInterrupt(timer1ISR);		// start program/timer ISR
}

void loop(){
	// Read then transmit
	if(Serial.available() >= n_rx_bytes+1){		// if total amount of bytes expected + 1 (newline) have arrived
		uartReceive(rx_data, n_rx);			// read buffer data and convert from bytes to ints or floats
		if(rx_data[0] == rx_expected[0] && rx_data[1] == rx_expected[1]) blinkLED();	// check to make sure values are as expected
		Serial.read();			// read/clear newline (when reading from C++)

		if(driving_uart == false){
			uartTransmit(tx_data, n_tx);	// transmit array of numbers as binary via UART
			Serial.write('\n');			// write newline if transmitting to a C++ script
		}
	}
}

// Do all your calculations/sensor readings in this folder
void timer1ISR(void){
	//Serial.println("timer1 loop");
	if(driving_uart == true){
		uartTransmit(tx_data, n_tx);	// transmit array of numbers as binary via UART
		Serial.write('\n');			// write newline if transmitting to a C++ script
	}
}

// Blink the LED pin to debug
void blinkLED(void){
	digitalWrite(led_pin, HIGH);
	delay(250);
	digitalWrite(led_pin, LOW);
	delay(100);
}
