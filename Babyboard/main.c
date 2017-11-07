/* main.c
 * Authors: Andrew Dudney and Matthew Haney
 * Copyright 2016 Andrew Dudney
 * Provides the main functionality for the BabyBoard demo
 */

#include <msp430.h>
#include "uart.h"

#include <string.h>
#include <stdlib.h>

// Input functions (using uart_rx)
int is_whitespace(char input, int newline);
void read_line(char buffer[], unsigned int word_count, unsigned int word_length);

// User input buffer: no more than 2 words, no more than 8 characters each
#define WC 2
#define WL 8
char buffer[WC][WL];

// These commands are valid
// Will compare input to these constants later
const char* clock_cmd = "clock";
const char* baud_cmd = "baud";
const char* quote_cmd = "quote";
const char* current_baud = "current";
const char* quit_cmd = "quit";

// Queen quote to print
unsigned char* quote = "In the year of '39, assembled here the volunteers, in the days when the lands were few.  Here the ship sailed out into the ...\n";

int main(void) {
  WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer

  // Begin with clock at 8MHz
  set_clock(1);

  // Initialize port 1
  P1DIR |= BIT4;
  P1SEL |= BIT4;
  P1SEL2 &= ~BIT4;

  P2DIR = BIT1; // Initialize port 2
  P2OUT = 0x00;

  P3DIR = 0x00; // Initialize port 3
  P3OUT = 0x00;

  // Initialize UART with 9600 baud
  char baud = UART_BAUD_115200;
  init_uart(baud);

  // Main loop
  while (1) {
    // Prompt
    put_str("\n");
    // Read input into buffer
    read_line((char*)buffer, WC, WL);
    // Perform actions based on input
    // CLOCK
    if (!strcmp(buffer[0], clock_cmd)) {
      int clock_speed = atoi(buffer[1]);
      switch (clock_speed) {
      case 1:
	uninit_uart();
	// Set clock speed to 1MHz
	set_clock(1);
	// Reinitialize UART with new clock speed, keeping baud rate
	init_uart(baud);
	put_str("Set clock speed to 1MHz\n");
	break;
      case 8:
	uninit_uart();
	// Set clock speed to 8MHz
	set_clock(8);
	// Reinitialize UART with new clock speed, keeping baud rate
	init_uart(baud);
	put_str("Set clock speed to 8MHz\n");
	break;
      case 16:
	uninit_uart();
	// Set clock speed to 16MHz
	set_clock(16);
	// Reinitialize UART with new clock speed, keeping baud rate
	init_uart(baud);
	put_str("Set clock speed to 16MHz\n");
	break;
      default:
	put_str("Invalid clock speed\n");
	break;
      }
    }
    // BAUD
    else if (!strcmp(buffer[0], baud_cmd)) {
      int baud_rate = atoi(buffer[1]);
      if (!baud_rate) {
	// If just "baud" is entered
	put_str("Available baud rates:\n");
	put_str("\t[1] 9600\n");
	put_str("\t[2] 115200\n");
      } else if (baud_rate <= 2) {
	baud = baud_rate - 1; // because indices start from 0
	put_str("Setting baud rate\n");
	// Reinitialize UART with new baud rate, keeping clock speed
	uninit_uart();
	init_uart(baud);
      } else {
	put_str("Invalid baud rate\n");
      }
    }
    // QUOTE
    else if (!strcmp(buffer[0], quote_cmd)) {
      put_str(quote);
    }
    else if (!strcmp(buffer[0], current_baud)){
            if(baud == 0){
                put_str("9600\n");
            }
            else{
                put_str("115200\n");
            }
    }
    // QUIT
    else if (!strcmp(buffer[0], quit_cmd)) {
      put_str("Exiting\n");
      break;
    }
  }

  uninit_uart();

  int flag = 1;
  // Disable LED if anything isn't set properly
  if (P1SEL != 0xA8) flag = 0;
  if (P1SEL2 != 0xA8) flag = 0;
  if (IE2 & UCA0TXIE) flag = 0;
  if (IE2 & UCA0RXIE) flag = 0;
  if ((IFG2 & UCA0TXIFG)==0) flag = 0;
  if (IFG2 & UCA0RXIFG) flag = 0;

  while(flag) {
    // After breaking: MSP LED follows button input
    P2OUT = (P2IN & BIT0) << 1;
  }

  LPM4;
  while (1) {
    // Nop
  }
}

void read_line(char buffer[], unsigned int word_count, unsigned int word_length) {
  unsigned int i, j;
  // Zero out all unwanted information from buffer
  for (i = 0; i < word_count; i++) {
    for (j = 0; j < word_length; j++) {
      buffer[i*word_length + j] = 0x00;
    }
  }

  unsigned int count = 0, length = 0;

  int word_too_long = 0, too_many_words = 0;

  char input = 0;
  while (1) {
    do {
      // Read input from UART (blocking = true)
      input = uart_rx(1);
    } while (is_whitespace(input, 0)); // Skip spaces, tabs, and carriage returns

    while (!is_whitespace(input, 1)) {
      if (length < word_length - 1) {
          buffer[count*word_length + length] = input;
          length += 1;
      } else {
          word_too_long = 1;
          break;
      }
      input = uart_rx(1);
    }

    // Exit if word is too long
    if (word_too_long) {
      break;
    }

    if (input == '\n') {
      break;
    } else {
      count += 1;
      length = 0;

      if (count >= word_count) {
	do {
	  input = uart_rx(1);
	} while (is_whitespace(input, 0));
	if (input == '\n') {
	  break;
	}
	too_many_words = 1;
	break;
      }
    }
  }
  if (word_too_long || too_many_words) {
    do {
      input = uart_rx(1);
    } while (input != '\n');
    if (word_too_long) {
      put_str("A word was too long\n");
    }
    if (too_many_words) {
      put_str("There were too many words\n");
    }
    for (i = 0; i < word_count; i++) {
      for (j = 0; j < word_length; j++) {
	buffer[i*word_length + j] = 0x00;
      }
    }
  }
}

int is_whitespace(char input, int newline) {
  return input == ' ' || input == '\t' || input == '\r' || (input == '\n' && newline);
}
