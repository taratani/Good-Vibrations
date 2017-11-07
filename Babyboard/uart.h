/* uart.c
 * Authors: Andrew Dudney and Matthew Haney
 * Copyright 2016 Andrew Dudney
 * Provides a UART interface for the MSP430G2553.
 *
*/

#ifndef UART_H
#define UART_H

#define UART_BAUD_9600 0
#define UART_BAUD_19200 4
#define UART_BAUD_38400 2
#define UART_BAUD_56000 3
#define UART_BAUD_115200 1

// Parity disabled, LSB first, 8 bit data, 1 stop bit

// set_clock: Set the internal clock to the provided speed, in MHz.
void set_clock(int speed);

// init_uart: Initialize everything necessary for the UART functionality you are implementing.  Be sure not to affect pins other than the TX and RX pins (output values, directions, or select registers).  You must support a baud rate of 9600 (UART_BAUD_9600) and 115200 (UART_BAUD_115200).  The other baud rates are optional.
void init_uart(char baud);

// uninit_uart: Uninitialize the uart driver.
void uninit_uart(void);

// putch: Send an unsigned char via UART.  This function should transmit characters correctly regardless of how many times this function is called in rapid succession.
void putch(unsigned char c);

// put_str: Send each element of a null-terminated array of unsigned chars via UART.  Do not send the final null-termination character.
void put_str(unsigned char* c);

// uart_rx: Return the most recent character received via UART.
// The block parameter determines the behavior of uart_rx if no character has been received.  The functionality is defined as follows:
//   If a character has been received, return the most recently received character.
//   If no character has been received and block is set to zero, return -1.
//   If no character has been received and block is set to one, wait for a character to be received and then return that character.
// Thus, if the microcontroller receives 'a' one time, and this function is called twice with block = 0, the first call should return 'a' and the second should return -1.  If the microcontroller receives 'a' one time, and this function is called twice with block = 1, the first call should return 'a' and the second should wait indefinitely until a character is received, and then return that character.
int uart_rx(char block);

#endif
