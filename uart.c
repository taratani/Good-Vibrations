/* uart.c
 * Authors: Andrew Dudney and Matthew Haney
 * Copyright 2016 Andrew Dudney
 * Provides an implementation to the UART interface defined in uart.h
 */

#include <msp430.h>
#include "uart.h"

void set_clock(int speed) {
    DCOCTL = 0;

    // change frequency to 1Mhz
    if (speed == 1){
        BCSCTL1 = CALBC1_1MHZ;
        DCOCTL = CALDCO_1MHZ;
    }
    // change frequency to 8Mhz
    else if (speed == 8){
        BCSCTL1 = CALBC1_8MHZ;
        DCOCTL = CALDCO_8MHZ;
    }
    // change frequency to 16Mhz
    else if (speed == 16){
        BCSCTL1 = CALBC1_16MHZ;
        DCOCTL = CALDCO_16MHZ;
    }
}

void init_uart(char baud) {
    P1SEL |= BIT1 + BIT2 ; // P1.1 = RXD, P1.2=TXD
    P1SEL2 |= BIT1 + BIT2 ; // P1.1 = RXD, P1.2=TXD

    UCA0CTL1 |= UCSSEL_2; // Use SMCLK

    // if baud is 9600
    if(baud == 0){
        // frequency = 1Mhz
        if(DCOCTL == CALDCO_1MHZ){
            UCA0BR0 = 104;
            UCA0BR1 = 0;
            UCA0MCTL = UCBRS1;
        }
        // frequency = 8Mhz
        else if(DCOCTL == CALDCO_8MHZ){
            UCA0BR0 = 0x41;
            UCA0BR1 = 0x03;
            UCA0MCTL = UCBRS2;
        }
        // frequency = 16Mhz
        else if(DCOCTL == CALDCO_16MHZ){
            UCA0BR0 = 0x82;
            UCA0BR1 = 0x06;
            UCA0MCTL = 0b0110;
        }
    }

    // if baud is 115200
    else if(baud == 1){
        // frequency = 1Mhz
        if(DCOCTL == CALDCO_1MHZ){
            UCA0BR0 = 8;
            UCA0BR1 = 0;
            UCA0MCTL = 12;
        }
        // frequency = 8Mhz
        else if(DCOCTL == CALDCO_8MHZ){
            UCA0BR0 = 69;
            UCA0BR1 = 0;
            UCA0MCTL = 0b1000;
        }
        // frequency = 16Mhz
        else if(DCOCTL == CALDCO_16MHZ){
            UCA0BR0 = 138;
            UCA0BR1 = 0;
            UCA0MCTL = 0b1110;
        }
    }

    UCA0CTL1 &= ~UCSWRST; // Initialize USCI state machine
}

void uninit_uart() {
    UCA0CTL1 &= UCSWRST;
    IFG2 &= UCA0TXIFG;
}

void putch(unsigned char c) {
    while (!(IFG2 & UCA0TXIFG));
    UCA0TXBUF = c;
}

void put_str(unsigned char* c) {
    while(*c != '\n'){
        while (!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = *c;
        c++;
    }
    putch(*c);
    putch('\r');
}

int uart_rx(char block) {
    int rx_char;
    // if new character
    if(IFG2 & UCA0RXIFG){
        rx_char = UCA0RXBUF;
        //putch((char) rx_char);
    }
    // if no new character and block is set
    else if(block){
        while(!(IFG2 & UCA0RXIFG));
        rx_char = UCA0RXBUF;
        //putch((char) rx_char);
    }
    // if no new character and block is not set
    else{
        rx_char = -1;
    }

   return rx_char;
}
