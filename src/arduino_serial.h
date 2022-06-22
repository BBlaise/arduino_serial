/*
 * arduino_serial.h
 *
 *  Created on: May 31, 2020
 *      Author: Bryan
 */

/* TODO
* - use printf for serial monitor
*/

#ifndef ARDUINO_SERIAL_H_
#define ARDUINO_SERIAL_H_

#include "Arduino.h"
#include "bitwise.h"

void flushSerialInputBuffer(void);		// clears any bytes waiting in the serial's receive buffer

// --- Transmit Functions --- //
void serialTransmitAscii(uint8_t tx_msg, const char* terminator = "\r\n");
void serialTransmitAscii(int8_t tx_msg, const char* terminator = "\r\n");
void serialTransmitAscii(uint16_t tx_float, const char* terminator = "\r\n");
void serialTransmitAscii(int16_t tx_int, const char* terminator = "\r\n");
void serialTransmitAscii(uint32_t tx_int, const char* terminator = "\r\n");
void serialTransmitAscii(int32_t tx_int, const char* terminator = "\r\n");

void serialTransmitAscii(uint8_t* tx_bytes, unsigned int n_tx, const char* delimeter = ",", const char* terminator = "\r\n");
void serialTransmitAscii(int8_t* tx_bytes, unsigned int n_tx, const char* delimeter = ",", const char* terminator = "\r\n");
void serialTransmitAscii(uint16_t* tx_ints, unsigned int n_tx, const char* delimeter = ",", const char* terminator = "\r\n");
void serialTransmitAscii(int16_t* tx_ints, unsigned int n_tx, const char* delimeter = ",", const char* terminator = "\r\n");
void serialTransmitAscii(uint32_t* tx_ints, unsigned int n_tx, const char* delimeter = ",", const char* terminator = "\r\n");
void serialTransmitAscii(int32_t* tx_ints, unsigned int n_tx, const char* delimeter = ",", const char* terminator = "\r\n");
void serialTransmitAscii(float* tx_floats, unsigned int n_tx, const char* delimeter = ",", const char* terminator = "\r\n");

void serialTransmitBinaryInt(int tx_int);                          // transists a single int as binary
void serialTransmitBinaryFloat(float tx_float);                      // transmits a single float as binary
void serialTransmitBinaryInt(int* tx_ints, unsigned int n_tx);              // transmits an array of ints as binary
void serialTransmitBinaryFloat(float* tx_floats, unsigned int n_tx);          // transmists an array of floats as binary

// --- Receive Functions --- //
//TODO (when transmission is definitely done/periodic);
bool serialReadChar(char* rx_buffer, unsigned int& buffer_indx);
bool serialReadChar(char (*rx_buffer)[20], unsigned int* buffer_indx);
int8_t serialReceiveAscii(int8_t& rx, char (&rx_buffer)[20]);
uint8_t serialReceiveAscii(uint8_t& rx, char (&rx_buffer)[20]);
int16_t serialReceiveAscii(int16_t& rx, char (&rx_buffer)[20]);
uint16_t serialReceiveAscii(uint16_t& rx, char (&rx_buffer)[20]);
int32_t serialReceiveAscii(int32_t& rx, char (&rx_buffer)[20]);
uint32_t serialReceiveAscii(uint32_t& rx, char (&rx_buffer)[20]);
float serialReceiveAscii(float& rx, char (&rx_buffer)[20]);

// void serialReceiveBinary(int* rx_ints, unsigned int n_rx);
// void serialReceiveBinary(float* rx_ints, unsigned int n_rx);
uint8_t serialReceiveBinaryUint8(void);
uint16_t serialReceiveBinaryUint16(void);
uint32_t serialReceiveBinaryUint32(void);
int8_t serialReceiveBinaryInt8(void);
int16_t serialReceiveBinaryInt16(void);
int32_t serialReceiveBinaryInt32(void);
int serialReceiveBinaryInt(void);
float serialReceiveBinaryFloat(void);

// ------------------------- Templated Type Independent Functions ---------------------------//
#if SERIAL_TEMPLATES == 1
	#include "ArxTypeTraits.h"
	// Transmits an individaul ascii-encoded numerical value with an option to specify the terminator
	template <typename T> void serialTransmitAscii(T& tx_val, const char* terminator = "\r\n") {
		Serial.print(tx_val);
		for (int ii = 0; ii < strlen(terminator); ii++) Serial.write(terminator[ii]);
	}

	// Transmits an array of numerical values as ascii with options to specify the delimeter and terminator
	template <typename T> void serialTransmitAscii(T* tx_array, unsigned int n_tx, const char* delimeter = ",", const char* terminator = "\r\n") {
		for (int ii = 0; ii < n_tx; ii++) {
			Serial.print(tx_array[ii]);
			if(ii != n_tx-1) Serial.write(delimeter);
		}

		for (int ii = 0; ii < strlen(terminator); ii++) Serial.write(terminator[ii]);
	}

	// Transmits a single binary-encoded numerical value via serial
	template <typename T> void serialTransmitBinary(T& tx) {
		int n_bytes = sizeof(T);
		uint8_t tx_bytes[n_bytes] = {};
		if (std::is_same<T, float>::value) floatToBytes((float)tx, tx_bytes);
		else intToBytes(tx, tx_bytes);
		for (int jj = 0; jj < n_bytes; jj++) Serial.write(tx_bytes[jj]);
	}

	// Transmits an array of binary-ecoded numerical values via serial
	// template <typename T> void serialTransmitBinary(T* tx_array, int n_tx) {
	// 	int n_bytes = sizeof(T);
	// 	uint8_t tx_bytes[n_tx][n_bytes] = {};

	// 	for (int ii = 0; ii < n_tx; ii++) {
	// 		if (std::is_same<T, float>::value) {
	// 			float temp = tx_array[ii];
	// 			floatToBytes(temp, tx_bytes[ii]);
	// 		} else { 
	// 			T temp = tx_array[ii];
	// 			intToBytes(temp, tx_bytes[ii]);
	// 		}
	// 		for(int jj = 0; jj < n_bytes; jj++) Serial.write(tx_bytes[ii][jj]);
	// 	}
	// }

	template <typename T> void serialTransmitBinary(T* tx_array, int n_tx) {
		int n_bytes = sizeof(T);
		uint8_t tx_bytes[n_tx][n_bytes] = {};

		for (int ii = 0; ii < n_tx; ii++) {
			if (std::is_same<T, float>::value) floatToBytes(tx_array[ii], tx_bytes[ii]);
			else intToBytes(tx_array[ii], tx_bytes[ii]);
			for(int jj = 0; jj < n_bytes; jj++) Serial.write(tx_bytes[ii][jj]);
		}
	}

	template <typename T> T serialReceiveAscii(T& rx, char (&rx_buffer)[20]) {
		if (std::is_same<T, float>::value) rx = (float)atof(rx_buffer);
		else if (sizeof(rx) == 4) rx = atol(rx_buffer);
		else rx = atoi(rx_buffer);

		memset(rx_buffer, 0, sizeof(rx_buffer));      // Clear the current buffer
		return rx;
	}

	template <typename T> void serialReceiveAscii(T* rx, unsigned int n_rx, char (*rx_buffer)[20]) {
		for(int ii = 0; ii < n_rx; ii++) {
			if (std::is_same<T, float>::value) rx[ii] = atof(rx_buffer[ii]);
			else if (sizeof(rx[ii]) == 4) rx[ii] = atol(rx_buffer[ii]);
			else rx[ii] = atoi(rx_buffer[ii]);

			memset(rx_buffer, 0, sizeof(rx_buffer[ii]));      // Clear the current buffer
		}
	}

	// --- Receive Functions --- // 
	// Receives a single binary-encoded numerical value via serial
	template <typename T> T serialReceiveBinary(T& rx) {
		int n_bytes = sizeof(T);
		uint8_t rx_bytes[n_bytes] = {};
		for (int ii = 0; ii < n_bytes; ii++) rx_bytes[ii] = Serial.read();

		if (std::is_same<T, float>::value) {
			rx = bytesToFloat(rx_bytes);
		} else {
			rx = bytesToInt(rx_bytes, rx);
			//rx = bytesToInt(rx_bytes);			// doesn't work since it expects 4 bytes even if 2 byte int is needed
		}

		return rx;
	}

	// Receives an array of binary-encoded numerical values via serial
	template <typename T> void serialReceiveBinary(T* rx_array, unsigned int n_tx) {
		int n_bytes = sizeof(T);
		uint8_t rx_bytes[n_bytes] = {};

		for (int ii = 0; ii < n_tx; ii++) {
			for (int jj = 0; jj < n_bytes; jj++) rx_bytes[jj] = Serial.read();

			if (std::is_same<T, float>::value) {
				rx_array[ii] = bytesToFloat(rx_bytes);
				// T rx = bytesToFloat(rx_bytes);
				// rx_array[ii] = rx;
			} else {
				rx_array[ii] = bytesToInt(rx_bytes, rx_array[ii]);
				// T rx = bytesToInt(rx_bytes, rx);
				// rx_array[ii] = rx;
			}
		}
	}
#endif /* SERIAL_TEMPLATES */

#endif /* ARDUINO_SERIAL_H_ */
