/*
 * File:   main.c
 * Author: guy
 *
 * Created on September 7, 2018, 10:56 AM
 */


#include <xc.h>

#include <pic18f4550.h>

#pragma config FOSC = HS    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config FCMEN = ON
#pragma config PBADEN = OFF				// RB0, RB1, RB2, RB3, & RB4 are configured as digital I/O on reset
#pragma config WDT = OFF
#pragma config IESO = OFF
#pragma config LVP = OFF
#pragma config MCLRE = OFF


#define _XTAL_FREQ 8000000
void main() {
    IRCF2 = 1;
    IRCF1 = 1;
    IRCF0 = 1;
    TRISB0 =0;
    while(1){ 
        PORTB = 0x01;
        __delay_ms(1000);
        PORTB = 0x00;
        __delay_ms(1000);
    }
}
    

