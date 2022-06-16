/*
 * arduino_serial.cpp
 *
 *  Created on: May 31, 2020
 *      Author: Bryan
 */

#include "arduino_serial.h"
#include "Arduino.h"
#include "bitwise.h"
#include "ArxTypeTraits.h"

// This function clears any bytes waiting in the UART's receive buffer
void flushSerialInputBuffer() {
	  while (Serial.available()) Serial.read();
}

// This function transmits a single signed integer of any size via UART
void serialTransmitBinaryInt(int tx_int) {
	int int_size = sizeof(tx_int);
	uint8_t tx_bytes[int_size] = {};

	intToBytes(tx_int, tx_bytes);
	for (int jj = 0; jj < int_size; jj++) {
		Serial.write(tx_bytes[jj]);
	}
}

// This function transmits a single 32-bit float via UART
void serialTransmitBinaryFloat(float tx_float) {
	uint8_t tx_bytes[4] = {};
    floatToBytes(tx_float, tx_bytes);
	for (int jj = 0; jj < 4; jj++) Serial.write(tx_bytes[jj]);
}

// This function transmits an array of integers via UART
void serialTransmitBinary(int* tx_ints, unsigned int n_tx) {
	int int_size = sizeof(tx_ints[0]);
	uint8_t tx_bytes[n_tx][int_size] = {};

    for (int ii = 0; ii < n_tx; ii++) {
    	intToBytes(tx_ints[ii], tx_bytes[ii]);
        for(int jj = 0; jj < int_size; jj++) Serial.write(tx_bytes[ii][jj]);
    }
}

// This function transmits an array of 32-bit floats via UART
void serialTransmitBinaryFloat(float* tx_floats, unsigned int n_tx) {
	uint8_t tx_bytes[n_tx][4] = {};
    for(int ii = 0; ii < n_tx; ii++){
        floatToBytes(tx_floats[ii], tx_bytes[ii]);
        for(int jj = 0; jj < 4; jj++) Serial.write(tx_bytes[ii][jj]);
    }
}

// This function transmits an array of integers via UART
void serialTransmitBinary(int* tx_ints, unsigned int n_tx, char* starter, char* terminator) {
	int int_size = sizeof(tx_ints[0]);
	uint8_t tx_bytes[n_tx][int_size] = {};

    for(int ii = 0; ii < n_tx; ii++) {
    	intToBytes(tx_ints[ii], tx_bytes[ii]);
        for(int jj = 0; jj < int_size; jj++) Serial.write(tx_bytes[ii][jj]);
    }
}

// transmit a delimeted array of bytes as ASCII
void serialTransmitAscii(uint8_t* tx_bytes, unsigned int n_tx, char delimeter = ',') {
	for(int ii = 0; ii < n_tx; ii++){
		Serial.print(tx_bytes[ii]);
		if(ii != n_tx-1) Serial.write(delimeter);
	}
	Serial.write('\n');
}

//Transmit a delimited array of bytes (uint8_t) as ASCII (or to the serial monitor)
void serialTransmitAscii(uint8_t* tx_bytes, unsigned int n_tx, const char* delimeter = ",", const char* terminator = "\r\n") {
	for (int ii = 0; ii < n_tx; ii++) {
		Serial.print(tx_bytes[ii]);
		if (ii != n_tx-1) Serial.write(delimeter);
	}

	for (int ii = 0; ii < strlen(terminator); ii++) Serial.write(terminator[ii]);
}

//Transmit a delimited array of unsigned 16-bit integers as ASCII (or to the serial monitor)
void serialTransmitAscii(uint16_t* tx_ints, unsigned int n_tx, const char* delimeter = ",", const char* terminator = "\r\n") {
	for (int ii = 0; ii < n_tx; ii++) {
		Serial.print(tx_ints[ii]);
		if (ii != n_tx-1) Serial.write(delimeter);
	}

	for (int ii = 0; ii < strlen(terminator); ii++) Serial.write(terminator[ii]);
}

//Transmit a delimited array of integers as ASCII (or to the serial monitor)
void serialTransmitAscii(int* tx_ints, unsigned int n_tx, const char* delimeter = ",", const char* terminator = "\r\n") {
	for (int ii = 0; ii < n_tx; ii++) {
		Serial.print(tx_ints[ii]);
		if (ii != n_tx-1) Serial.write(delimeter);
	}

	for (int ii = 0; ii < strlen(terminator); ii++) Serial.write(terminator[ii]);
}

// Transmit a tab-delimited array of floats as ASCII (or to the serial monitor)
void serialTransmitAscii(float* tx_floats, unsigned int n_tx, const char* delimeter = ",", const char* terminator = "\r\n") {
	for (int ii = 0; ii < n_tx; ii++) {
		Serial.print(tx_floats[ii]);
		if (ii != n_tx-1) Serial.write(delimeter);
	}

	for (int ii = 0; ii < strlen(terminator); ii++) Serial.write(terminator[ii]);
}

// ------------ Receive Funcitons ------------------- //
uint8_t serialReceiveBinaryUint8() {
	uint8_t rx = Serial.read();
	return rx;
}

uint16_t serialReceiveBinaryUint16() {
	int n_bytes = sizeof(uint16_t);
	uint8_t rx_bytes[n_bytes] = {};
	for (int ii = 0; ii < n_bytes; ii++) rx_bytes[ii] = Serial.read();
	uint16_t rx = bytesToUint16(rx_bytes);
	return rx;
}

uint32_t serialReceiveBinaryUint32() {
	int n_bytes = sizeof(uint32_t);
	uint8_t rx_bytes[n_bytes] = {};
	for (int ii = 0; ii < n_bytes; ii++) rx_bytes[ii] = Serial.read();
	uint32_t rx = bytesToUint32(rx_bytes);
	return rx;
}

int8_t serialReceiveBinaryInt8() {
	uint8_t rx_byte = Serial.read();
	int8_t rx_int = byteToInt8(rx_byte);
	return rx_int;
}

int16_t serialReceiveBinaryInt16() {
	int n_bytes = sizeof(int16_t);
	uint8_t rx_bytes[n_bytes] = {};
	for (int ii = 0; ii < n_bytes; ii++) rx_bytes[ii] = Serial.read();

	int16_t rx_int = bytesToInt16(rx_bytes);
	return rx_int;
}

int32_t serialReceiveBinaryInt32() {
	int n_bytes = sizeof(int32_t);
	uint8_t rx_bytes[n_bytes] = {};
	for (int ii = 0; ii < n_bytes; ii++) rx_bytes[ii] = Serial.read();

	int32_t rx_int = bytesToInt32(rx_bytes);
	return rx_int;
}

int serialReceiveBinaryInt() {
	int n_bytes = sizeof(int);
	uint8_t rx_bytes[n_bytes] = {};
	for (int ii = 0; ii < n_bytes; ii++) rx_bytes[ii] = Serial.read();
	int rx_int;
	bytesToInt(rx_bytes, rx_int);
	return rx_int;
}

// This function receives 4 bytes and returns a 32-bit float
float serialReceiveBinaryFloat() {
	uint8_t rx_bytes[4] = {};
	for (int ii = 0; ii < 4; ii++) rx_bytes[ii] = Serial.read();

	float rx_float = bytesToFloat(rx_bytes);
	return rx_float;
}


// This function receives an array of 32-bit integers via UART
void serialReceiveBinaryInt(int* rx_ints, unsigned int n_tx){
	int int_size = sizeof(rx_ints[0]);
	uint8_t rx_bytes[n_tx][int_size] = {};

    for (int ii = 0; ii < n_tx; ii++) {
    	for(int jj = 0; jj < int_size; jj++) rx_bytes[ii][jj] = Serial.read();

    	rx_ints[ii] = bytesToInt(rx_bytes[ii]);
    }
}

// This function receives an array of 32-bit floats via UART (implemented as template function in h file)
// void serialReceiveBinary(float* rx_floats, unsigned int n_tx) {
// 	uint8_t rx_bytes[n_tx][4] = {};

//     for(int ii = 0; ii < n_tx; ii++) {
//     	for(int jj = 0; jj < 4; jj++) rx_bytes[ii][jj] = Serial.read();
//     	rx_floats[ii] = bytesToFloat(rx_bytes[ii]);
//     }
// }

// ---------- Ascii Receive Functions --------- //
bool serialReadChar(char* rx_buffer, unsigned int& buffer_indx) {
	//TODO use strstr(bigstring, substring) to check if terminator is in buffer
	char newline = '\n'; 				// line feed. send with (CTRL + J) on Putty
	char carriage_return = '\r';		// carriage return. send with ENTER on Putty
	//char terminator[1] = {newline}; unsigned int terminator_len = 1;  // for MATLAB
	//char terminator[1] = {carriage_return}; unsigned int terminator_len = 1;  // for Putty
	char terminator[2] = {carriage_return, newline}; unsigned int terminator_len = 2;
	//char terminator[2] = {newline, carriage_return}; unsigned int terminator_len = 2;
	//char temrinator[2] = {'A', '\n'}; unsigned int terminator_len = 2;
	bool terminator_received = false;

	char next_char = Serial.read();
	if ( buffer_indx == 0 && (next_char == newline || next_char == carriage_return) ) { 
		// Case A: if newline or carriage return arrives at beginning of transmission
		terminator_received = false;
		Serial.println("A");
	} else if (terminator_len == 1 && next_char == terminator[0]) {
		// Case B: terminator is single char and matches incomming char
		buffer_indx = 0; 
		terminator_received = true;
		Serial.println("B");
	} else if ( terminator_len == 2 && next_char == terminator[1] && rx_buffer[buffer_indx-1] == terminator[0] ) {
		// Case C: If incomming char and previous char match terminator sequence
		rx_buffer[buffer_indx-1] = 0;			// clear terminator from buffer
		buffer_indx = 0;
		terminator_received = true;
		Serial.println("C");
	} else {
		// If not receiving last char in terminator
		rx_buffer[buffer_indx] = next_char;
		buffer_indx++;
		terminator_received = false;
		Serial.println("D");
	}

	return terminator_received;
}

bool serialReadChar(char (*rx_buffer)[20], unsigned int* buffer_indxs) {
	char newline = '\n'; char carriage_return = '\r';
	char delimeter = ','; //char delimeter = '\t'; // char delimeter = ' ';
	//char terminator[1] = {newline}; unsigned int terminator_len = 1;  // for MATLAB
	//char terminator[1] = {carriage_return}; unsigned int terminator_len = 1;  // for Putty
	char terminator[2] = {carriage_return, newline}; unsigned int terminator_len = 2;
	//char terminator[2] = {newline, carriage_return}; unsigned int terminator_len = 2;
	//char temrinator[2] = {'A', '\n'}; unsigned int terminator_len = 2;
	bool terminator_received = false;

	int ii = buffer_indxs[0]; int jj = buffer_indxs[1];

	char next_char = Serial.read();
	if ( (ii == 0 && jj == 0) && (next_char == newline || next_char == carriage_return || next_char == delimeter) ) { 
		terminator_received = false;
	} else if ( (terminator_len == 1 && next_char == terminator[0]) ) {
		buffer_indxs[0] = 0; buffer_indxs[1] = 0;
		terminator_received = true;
	} else if ( terminator_len == 2 && (next_char == terminator[1]) && (rx_buffer[ii][jj-1] == terminator[0])) {
	//( (next_char == newline) && rx_buffer[ii][jj-1] == terminator[0]) {
		rx_buffer[ii][jj-1] = 0;			// clear carriage_return or other elements of terminator in buffer
		buffer_indxs[0] = 0; buffer_indxs[1] = 0;
		terminator_received = true;
	} else if (next_char == delimeter) {
        buffer_indxs[0]++;
        buffer_indxs[1] = 0;
		terminator_received = false;
    } else {
		rx_buffer[ii][jj] = next_char;
		buffer_indxs[1]++;
		terminator_received = false;
	}
	
	return terminator_received;
}

uint32_t serialReceiveAsciiUint32(uint32_t& rx, char (&rx_buffer)[20]) {
	rx = atol(rx_buffer);
	memset(rx_buffer, 0, sizeof(rx_buffer));      // Clear the current buffer
	return rx;
}

int serialReceiveAsciiInt(char (&rx_buffer)[20]) {
	int rx = atoi(rx_buffer);
	memset(rx_buffer, 0, sizeof(rx_buffer));      // Clear the current buffer
	return rx;
}

int serialReceiveAsciiInt(int& rx, char (&rx_buffer)[20]) {
	rx = atoi(rx_buffer);
	memset(rx_buffer, 0, sizeof(rx_buffer));      // Clear the current buffer
	return rx;
}

int serialReceiveAsciiInt(int& rx, char* rx_buffer, int buffer_size) {
	rx = atoi(rx_buffer);
	memset(rx_buffer, 0, buffer_size);      // Clear the current buffer
	return rx;
}

float serialReceiveAsciiFloat(float& rx, char (&rx_buffer)[20]) {
	rx = atof(rx_buffer);
	memset(rx_buffer, 0, sizeof(rx_buffer));      // Clear the current buffer
	return rx;
}

int32_t serialReceiveAsciiInt32(int32_t& rx, char (&rx_buffer)[20]) {
	rx = atol(rx_buffer);
	memset(rx_buffer, 0, sizeof(rx_buffer));      // Clear the current buffer
	return rx;
}

// bool serialReceiveAscii(float* rx_floats, unsigned int n_tx) {}

