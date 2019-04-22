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
#include <stdint.h>

#include "uart.h"

// Quadrature encoders
#define c_EncoderPinA 4 // Pin 2.4
#define c_EncoderPinB 3 // Pin 2.3
#define c_EncoderPinZ 6 // Pin 2.5

#define BitA 0x02
#define BitB 0x01
#define BitZ 0x04

#define EncPIN P2IN
#define EncPIE P2IE
#define EncPIFG P2IFG

#define LED_POUT P1OUT
#define LED_PDIR P1DIR
#define PWR_LED 0x20 // Pin 2.0
#define STATUS_LED 0x10 // Pin 2.1

volatile uint16_t IndexTicks = 0;
//volatile long int EncoderTicks = 0;
volatile int16_t EncoderTicks = 0;
volatile uint16_t MasterClockHigh = 0;
volatile int LastFullCycle = -1;
volatile int16_t FullCycleTicks = 0;
volatile int DoubleInterrupt = 0;


volatile unsigned int SecondCounter = 1000; // should be 1000!

char *cEncoderTicksPtr;
char *cMasterClockPtr;

void SendData()
{
    uart_putc('E');

    // Send timestamp (remember that we're little endian)
    uart_putc(*(cMasterClockPtr+1));
    uart_putc(*cMasterClockPtr);
    uart_putc(*((char *)TA0R + 1));
    uart_putc(*((char *)TA0R));

    // Send wheel data
    uart_putc(*cEncoderTicksPtr);
    uart_putc(*(cEncoderTicksPtr+1));
    //uart_putc(*(cEncoderTicksPtr+2));
    //uart_putc(*(cEncoderTicksPtr+3));

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

    EncPIE |= BitA;
    EncPIFG = 0;

    cEncoderTicksPtr = (char *)&EncoderTicks;
    cMasterClockPtr = (char *)&MasterClockHigh;

    __delay_cycles(1000000);
  
    uart_init();

    __bis_SR_register(GIE);

    while(1) {
     LED_POUT ^= STATUS_LED;       // Toggle LED using exclusive-OR
     __bis_SR_register(CPUOFF + GIE); // Enter LPM0 w/ interrupts
     SendData();
    } 
}

#define STRINGIFY2(x) #x
#define STRINGIFY(x) STRINGIFY2(x)

#pragma vector=PORT2_VECTOR
__interrupt void QuadratureISR(void) 
{
  static int ZInterlock = 4;

  __asm__(
    "mov.b %[PxIE], r15\n"
    "mov.b %[PxIN], r14\n"
    "bit.b #" STRINGIFY(BitA) ", r15\n"
    "jz CHECK_B\n"
    "A_TRIGGERED:\n"
    "bit.b #" STRINGIFY(BitB) ", r14\n"
    "jnz DEC_COUNTER\n"
    "jmp INC_COUNTER\n"
    "CHECK_B:\n"
    "bit.b #" STRINGIFY(BitB) ", r15\n"
    "jz CHECK_Z:\n"
    "B_TRIGGERED:\n"
    "bit.b #" STRINGIFY(BitA) ", r14\n"
    "jnz INC_COUNTER\n"
    "DEC_COUNTER:\n"
    "dec %[EncoderTicks]\n"
    "jmp TOGGLE_CHANNEL\n"
    "INC_COUNTER:\n"
    "inc %[EncoderTicks]\n"
    "TOGGLE_CHANNEL:\n"
    "xor.b #" STRINGIFY(BitA+BitB) ", %[PxIE]\n"
    "bic.b #" STRINGIFY(BitA+BitB) ", %[PxIFG]\n"
    "bit.b #" STRINGIFY(BitZ) ", r14\n"
    "jnz CHECK_Z\n"
    "DECREMENT_Z_INTERLOCK:\n"
    "dec %[ZInterlock]\n"
    "jnz CHECK_Z\n"
    "bis.b #" STRINGIFY(BitZ) ", %[PxIE]\n"
    "mov #4, %[ZInterlock]\n"
    "CHECK_Z:\n"  
    "bit.b #" STRINGIFY(BitZ) ", %[PxIE]\n"
    "jz CONCLUDE:\n"
    "bit.b #" STRINGIFY(BitZ) ", %[PxIFG]\n"
    "jz CONCLUDE:\n"
    "inc %[IndexTicks]\n"
    "mov %[EncoderTicks], %[FullCycleTicks]\n"
    "mov #0, %[EncoderTicks]\n"
    "bic.b #" STRINGIFY(BitZ) ", %[PxIE]\n"
    "bic.b #" STRINGIFY(BitZ) ", %[PxIFG]\n"
    "CONCLUDE:\n"
  : [PxIE] "=m" (EncPIE), [PxIFG] "=m" (EncPIFG), [EncoderTicks] "=m" (EncoderTicks), 
    [ZInterlock] "=m" (ZInterlock), [IndexTicks] "=m" (IndexTicks), [FullCycleTicks] "=m" (FullCycleTicks)
  : [PxIN] "m" (EncPIN) : "r14", "r15" );

/*
  if ((EncPIE & BitA) > 0) {
    if (!(EncPIN && BitB)) {
      EncoderTicks += 1; 
    }
    else {
      EncoderTicks -= 1;
    }
    EncPIE &= ~BitA;  // Turn off A
    EncPIE |= BitB;  // Turn on B
    //EncPIE ^= BitA + BitB;
    EncPIFG &= ~(BitA + BitB); // Clear A Flag (why do I have to clear B here???)
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
    EncPIE &= ~BitB; // Turn off B
    EncPIE = BitA; // Turn on A
    //EncPIE ^= BitA + BitB;
    EncPIFG &= ~(BitA + BitB); // Clear A Flag (why do I have to clear B here???)
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
*/
}  

// Timer A0 CCR0 interrupt service routine => Wake up every 100 ms
//   Assumes TimerA0 is set up for 1 ms, continuous mode.
//#pragma vector=TIMER0_A0_VECTOR
//__interrupt void WakeupClockISR (void)
__attribute__((interrupt(TIMER0_A0_VECTOR)))
void WakeupClockISR()
{
  //TA0CCR0 += 100;
  //__bic_SR_register_on_exit(CPUOFF);
  
  // Using assembly saves a push and pop
  __asm__("add #100, %[TIMERREG]\n"
          "bic #16,  0(R1)\n" // bic_SR_on_exit(CPUOFF) //"reti\n"
  :[TIMERREG] "=m" (TA0CCR0) );
}


// Timer A0 overflow interrupt service routine => increment higher 16bit counter
//   Assumes TimerA0 is set up for 1 ms, continuous mode.
//#pragma vector=TIMER0_A1_VECTOR
//__interrupt void MasterClockISR (void)
__attribute__((interrupt(TIMER0_A1_VECTOR)))
void MasterClockISR()
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
