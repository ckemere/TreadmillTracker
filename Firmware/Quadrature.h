#ifndef __UART_H
#define __UART_H

#include <msp430.h>
#include <stdint.h>

// Macros to shortcut which ports / pins are used for Quadrature encoder
//    interface. 

#define EncPIN P2IN
#define EncPIE P2IE
#define EncPIFG P2IFG

#define BitA 0x02 // Pin 2.1
#define BitB 0x01 // Pin 2.0
#define BitZ 0x04 // Pin 2.2

#define c_EncoderPinA 4 // Pin 2.4
#define c_EncoderPinB 3 // Pin 2.3
#define c_EncoderPinZ 6 // Pin 2.5

extern volatile uint16_t IndexTicks;
extern volatile int16_t EncoderTicks;
extern volatile int LastFullCycle;
extern volatile int16_t FullCycleTicks;

void quadrature_init(void);


#endif
