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

#include "Quadrature.h"
#include "uart.h"

#include <msp430.h>
#include <stdint.h>

#define LED_POUT P1OUT
#define LED_PDIR P1DIR
#define PWR_LED 0x20 // Pin 2.0
#define STATUS_LED 0x10 // Pin 2.1

volatile uint16_t MasterClockHigh = 0;


char *cEncoderTicksPtr;
char *cMasterClockHighPtr;
char *cMasterClockLowPtr;

void SendData()
{
    register uint16_t MasterClockHighCopy;
    register uint16_t MasterClockLowCopy;
    register int16_t  EncoderTicksCopy;
    register unsigned char GPIO;

    // NOTE: We're using POLLING for serial comms to make
    //   sure that if there's a conflict between timing
    //   or quadrature measurements and data x-mission,
    //   timing/quadrature wins.

    // Copy current clock and encoder data atomically.
    __disable_interrupt();
    MasterClockHighCopy = MasterClockHigh;
    MasterClockLowCopy = TA0R;
    EncoderTicksCopy = EncoderTicks;
    GPIO = P3IN;
    __enable_interrupt();
    
    uart_putc('E');
 
    // Send timestamp (remember that we're little endian)
    uart_putw(MasterClockHighCopy); 
    uart_putw(MasterClockLowCopy); 

    // Send wheel data
    uart_putw(EncoderTicksCopy);

   // Send GPIO data
    uart_putc(GPIO);

    uart_putc('\n');
    return;
}


/**
 * Main routine
 */
int main(void)
{
 
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

    // Set up system clocks
    // Drive master clock at 8MHz
    BCSCTL1 = CALBC1_8MHZ;   // Set range
    DCOCTL = CALDCO_8MHZ;    // Set DCO step + modulation
    
    P1DIR = 0x00;
    P1SEL = 0x01;
    P1SEL2 = 0x00;
    TA0CTL = TASSEL_0 + MC_2 + ID_1 + TAIE + TACLR;  // TA0CLK Input (2KHz), 
      // continuous mode, divide by 2 (1 kHz), enable timer overflow interrupt, reset counter
    TA0CCR0 = 100;            // interrupt on overflow
    TA0CCTL0 = CCIE;           // CCR0 interrupt enabled
 
    LED_PDIR |= STATUS_LED + PWR_LED;     
    LED_POUT |= STATUS_LED + PWR_LED;     // All LEDs on

    quadrature_init();
    uart_init();

    P3DIR = 0x00;
    //P3REN = 0xF0;
    //P3OUT = 0x55; // set pull ups
  
    __bis_SR_register(GIE);

    while(1) {
     LED_POUT ^= STATUS_LED;       // Toggle LED using exclusive-OR
     if (NewGPIOFlag) {
       P3OUT = NewGPIO;
       NewGPIOFlag = 0;
     }
     SendData();
     __bis_SR_register(CPUOFF + GIE); // Enter LPM0 w/ interrupts
    } 
}


// Timer A0 CCR0 interrupt service routine => Wake up every 100 ms
//   Assumes TimerA0 is set up for 1 ms, continuous mode.
#pragma vector=TIMER0_A0_VECTOR
__interrupt void WakeupClockISR (void)
{
  //TA0CCR0 += 100;
  //__bic_SR_register_on_exit(CPUOFF);
  
  // Using assembly saves a push and pop
  __asm__("add #2, %[TIMERREG]\n" // SERIAL TRANSMIT PERIOD in milliseconds
          "bic #16,  0(R1)\n" // bic_SR_on_exit(CPUOFF) //"reti\n"
  :[TIMERREG] "=m" (TA0CCR0) );
}


// Timer A0 overflow interrupt service routine => increment higher 16bit counter
//   Assumes TimerA0 is set up for 1 ms, continuous mode.
#pragma vector=TIMER0_A1_VECTOR
__interrupt void MasterClockISR (void)
{
  //if (TA0IV & TAIFG)
  //  MasterClockHigh++;

  // Use ISR table construction from Users Guide
  __asm__("add %[Offset], r0\n"
          "reti\n" // Vector 0: No Interrupt
          "reti\n" // Vector 2: TACCR1; would be a JMP if used
          "reti\n" // Vector 4: TACCR1; would be a JMP if used
          "reti\n" // Vector 4: Reserved
          "reti\n" // Vector 8: Reserved
          "inc %[MasterClockHigh]\n"
  : [MasterClockHigh] "=m" (MasterClockHigh) : [Offset] "m" (TA0IV) );
  // RETI is automatically added at end 
}
