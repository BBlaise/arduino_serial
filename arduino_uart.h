/*
 * arduino_uart.h
 *
 *  Created on: May 31, 2020
 *      Author: Bryan
 */

#ifndef ARDUINO_UART_H_
#define ARDUINO_UART_H_

#include "Arduino.h"
#include "stdio.h"
#include "bitwise.h"

void uartTransmit(int tx_int);
void uartTransmit(float tx_float);
void uartTransmit(int* tx_ints, int n_tx);
void uartTransmit(float* tx_floats, int n_tx);

void uartTransmitMultiAscii(uint8_t* tx_msg, int n_bytes);

// Receive Funcions
void uartReceive(int* rx_ints, uint8_t rx_bytes[][4], int n_rx);
void uartReceive(int* rx_ints, int n_rx);
void uartReceive(float* rx_ints, int n_rx);

int uartReceiveInt(void);
int32_t uartReceiveInt32(void);
int16_t uartReceiveInt16(void);
float uartReceiveFloat(void);

void uartReceiveAscii(uint8_t* rx_msg, int n_bytes);

// Print To Serial Monitor
void serialPrintArray(int* array, int n_ints);
void serialPrintArray(float* float_array, int n_floats);

#endif /* ARDUINO_UART_H_ */