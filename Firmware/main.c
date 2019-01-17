/*
 * This file is part of the MSP430 hardware UART example.
 *
 * Copyright (C) 2012 Stefan Wendler <sw@kaltpost.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/******************************************************************************
 * Hardware UART example for MSP430.
 *
 * Stefan Wendler
 * sw@kaltpost.de
 * http://gpio.kaltpost.de
 *
 * Echos back each character received. Blinks green LED in main loop. Toggles
 * red LED on RX.
 *
 * Use /dev/ttyACM0 at 9600 Bauds (and 8,N,1).
 *
 * Note: RX-TX must be swaped when using the MSPg2553 with the Launchpad! 
 *       You could easliy do so by crossing RX/TX on the jumpers of the 
 *       Launchpad.
 ******************************************************************************/

#include <msp430.h>

#include "uart.h"

// Quadrature encoders
#define c_EncoderPinA 4 // Pin 2.4
#define c_EncoderPinB 3 // Pin 2.3
#define c_EncoderPinZ 5 // Pin 2.5

#define BitA (char) 0x10
#define BitB (char) 0x08
#define BitZ (char) 0x20

#define EncPIN P2IN
#define EncPIE P2IE
#define EncPIFG P2IFG

#define LED_POUT P2OUT
#define LED_PDIR P2DIR
#define PWR_LED 0x01 // Pin 2.0
#define STATUS_LED 0x02 // Pin 2.1

volatile int IndexTicks = 0;
volatile long int EncoderTicks = 0;
volatile int LastFullCycle = -1;
volatile int FullCycleTicks = 0;
volatile int DoubleInterrupt = 0;


void print_num(long int Temp) {
     char *iptr;
     char anum[8];
  
     if (Temp < 0) {
      uart_putc('-');
      Temp = -Temp;
     }
     
     iptr = (char*) &Temp;
     
     anum[7] = (*iptr & 0xF);
     anum[6] = ((*iptr++ & 0xF0) >> 4);
     anum[5] = (*iptr & 0xF);
     anum[4] = ((*iptr++ & 0xF0) >> 4);
     anum[3] = (*iptr & 0xF);
     anum[2] = ((*iptr++ & 0xF0) >> 4);
     anum[1] = (*iptr & 0xF);
     anum[0] = ((*iptr++ & 0xF0) >> 4);
     
     uart_putsn(anum,8);
}

/**
 * Main routine
 */
int main(void)
{
 
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

    // Set up system clocks
    BCSCTL1 = CALBC1_8MHZ;   // Set range
    DCOCTL = CALDCO_8MHZ;    // Set DCO step + modulation

    TA1CTL = TASSEL_2 + MC_1;  // SMCLK (8 MHz), up mode, enable interrupt
    TA1CCR0 = 8000;            // 1 ms period
    TA1CCTL0 = CCIE;           // CCR0 interrupt enabled
 
    LED_PDIR  = STATUS_LED + PWR_LED;     // P1.0 and P1.6 are the red+green LEDs 
    LED_POUT  = STATUS_LED + PWR_LED;     // All LEDs off

    EncPIE = BitA;
    EncPIFG = 0;
  
    uart_init();

    __bis_SR_register(GIE);

    uart_puts((char *)"MSP430 Quadrature!\n\r");

    while(1) {
     LED_POUT ^= STATUS_LED;       // Toggle P1.6 output (green LED) using exclusive-OR
     __bis_SR_register(CPUOFF + GIE); // Enter LPM0 w/ interrupts
     uart_putc('E');
     print_num(EncoderTicks);
     //uart_putc(' ');
     //print_num(FullCycleTicks);
     uart_putc('\n');
    } 
}



#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR(void) 
{
  static int ZInterlock = 4;
    if ((EncPIE & BitA) > 0) {
    if (!(EncPIN && BitB)) {
      EncoderTicks += 1; 
    }
    else {
      EncoderTicks -= 1;
    }
    EncPIFG &= ~(BitA + BitB); // Clear A Flag (why do I have to clear B here???)
    EncPIE &= ~BitA;  // Turn off A
    EncPIE |= BitB;  // Turn on B
    if ((EncPIN & BitZ) == 0) {
      ZInterlock--;
      if (ZInterlock == 0){
        EncPIE |= BitZ;
        ZInterlock = 4;
      }
    }
  }
  else if ((EncPIE & BitB) > 0)  {
    if (EncPIN && BitA) {
      EncoderTicks += 1; 
    }
    else {
      EncoderTicks -= 1;
    }
    EncPIFG &= ~(BitB + BitA); // Clear B Flag (why do I have to clear A here???)
    EncPIE &= ~BitB; // Turn off B
    EncPIE = BitA; // Turn on A
    if ((EncPIN & BitZ) == 0) {
      ZInterlock--;
      if (ZInterlock == 0){
        EncPIE |= BitZ;
        ZInterlock = 4;
      }
    }
  }
  
  if ((EncPIE & BitZ) > 0) { // Index!
    if ((EncPIFG & BitZ) > 0) {
      IndexTicks++;
      FullCycleTicks = EncoderTicks;
      EncoderTicks = 0;
      EncPIFG &= ~(BitZ);
      EncPIE &= ~(BitZ); // Disable for a while
    }
  }
}  

// Timer A1 interrupt service routine => Assume CCR0 set for 1 ms ticks
#pragma vector=TIMER1_A0_VECTOR
__interrupt void MasterClockISR (void)
{
    static unsigned int SecondCounter = 1000; // should be 1000!

    if (--SecondCounter == 0) {
        SecondCounter = 1000;
        __bic_SR_register_on_exit(CPUOFF);
    }
}
