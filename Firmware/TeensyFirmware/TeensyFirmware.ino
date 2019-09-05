


#define CLOCK_PIN 0 // B16

#define QUADZ 30 // C11
#define QUADA 31 // E0
#define QUADB 32 // B18

#define BDIO0 16 // B0
#define BDIO1 17 // B1
#define BDIO2 19 // B2
#define BDIO3 18 // B3

#define D0 2 // D0
#define D1 14 // ...
#define D2 7
#define D3 8
#define D4 6
#define D5 20
#define D6 21
#define D7 5 // D7

#define LED0 10 //PIN_C4;
#define LED1 13 //PIN_C5;
#define LED2 11 //PIN_C6;
#define LED3 12 //PIN_C7;

#define ENCODER_OPTIMIZE_INTERRUPTS // this messes up attachInterrupts
#include <Encoder.h>


IntervalTimer sendDataTimer;

Encoder wheelPosition(QUADA, QUADB);

uint32_t ledState = 0x11111111;

struct _TreadmillDataStruct {
  char StartChar;
  char DataLength;
//  uint16_t MasterClockLow;
//  uint16_t MasterClockHigh;
  uint32_t MasterClock;
  int16_t  EncoderTicks;
  int32_t  UnwrappedEncoderTicks;
  uint8_t  GPIO;
  char EndChar;
} __attribute__((packed));
typedef struct _TreadmillDataStruct TreadmillDataStruct;

TreadmillDataStruct TreadmillData;

volatile uint32_t MasterClock = 0;

boolean SerialTransmitFlag = false;

// ISR routine for FlexTimer1 Module
extern "C" void ftm1_isr(void) {
  if ((FTM1_SC & FTM_SC_TOF) != 0) {  //read the timer overflow flag (TOF in FTM1_SC)
    MasterClock++;
    FTM1_SC &= ~FTM_SC_TOF;           //if set, clear overflow flag

    if ((MasterClock & 0x01) == 0)
      SerialTransmitFlag  = true;
  }
}

void sendData() {

  if (Serial.dtr()) {
    TreadmillData.MasterClock = MasterClock;
    TreadmillData.GPIO = GPIOD_PDIR;
    TreadmillData.UnwrappedEncoderTicks = wheelPosition.read();
    Serial.write((char *) &TreadmillData, sizeof(TreadmillDataStruct));
    Serial.flush();
  }

/*
  if ((MasterClock & 0x3F) == 0) {
    GPIOC_PDOR = ledState & 0xF0;
    ledState = (ledState << 1) | (ledState >> 31);
  }*/
}


void setup() {
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  digitalWrite(LED0,0);
  digitalWrite(LED1,0);
  digitalWrite(LED2,0);
  digitalWrite(LED3,0);

  //GPIOD_PDDR = GPIOC_PDDR & ~0xFF00; // set pins 0-7 to output
  pinMode(D0, INPUT);
  pinMode(D1, INPUT);
  pinMode(D2, INPUT);
  pinMode(D3, INPUT);
  pinMode(D4, OUTPUT);
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  digitalWrite(D4, 0);
  digitalWrite(D5, 0);
  digitalWrite(D6, 0);
  digitalWrite(D7, 0);
  
  TreadmillData.StartChar = 'E';
  TreadmillData.EndChar = '\n';
  TreadmillData.DataLength = sizeof(TreadmillDataStruct);

  /* 
   *  Set up FTM timer module which connects to external 2kHz
   *   clock for master timing. The clock is set up both to increment
   *   the master clock counter. When it overflows, this also triggers
   *   the high 16 bits of the master clock to increment.
   *   
   *   Note that unlike with the MSP430, we can't independently set
   *   a second interrupt to trigger every millisecond to drive serial
   *   data transmission, so we'll just use the Teensy for that.
   *   
   */
  CORE_PIN0_CONFIG = PORT_PCR_MUX(4);
  NVIC_DISABLE_IRQ(IRQ_FTM1);
  FTM1_SC = 0;
  FTM1_CNT = 0;
  FTM1_MOD = 0x1; // set to overflow every two ticks (1 kHz)
  FTM1_SC = FTM_SC_CLKS(3) + FTM_SC_PS(0) + FTM_SC_TOIE; // External clock input
  NVIC_ENABLE_IRQ(IRQ_FTM1);
  
  Serial.begin(256000);
  //sendDataTimer.begin(sendData, 2000); // 2 ms
}


void loop() {
  // put your main code here, to run repeatedly:
  interrupts();
  if (SerialTransmitFlag) {
    sendData();
    SerialTransmitFlag = false; // I'm worried about the case where I set this right when the timer is trying to wake us up.
                                // But I guess in that case, it would indicate we're on the hairy edge of stability, which is bad anyway.
  }
  if (Serial.available()) {
    uint8_t incomingByte = Serial.read();  // will not be -1
    GPIOD_PDOR = incomingByte & 0xF0; // Recalling that higher order bits are output
    GPIOC_PDOR = incomingByte & 0xF0; // Recalling that higher order bits are output
  }
}
