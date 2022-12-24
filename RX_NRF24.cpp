/*
Using nRF24L01 module for radio control.
Tested with 16MHz pro-mini, QuadX and https://github.com/gcopeland/RF24
nRF24 connections (left is nRF24, right is arduino)
    CE        7
    CSN       8
    MOSI     11
    MISO     12
    SCK      13
You can change CE and CSN in RX_NRF24.cpp
Motor pins:
    Right front    3
    Right rear     4
    Left rear      5
    Left front     6
Warning LED:
    LED            2
*/

#include "config.h"

#ifdef RX_NRF24

//#include <arduino.h>
#include <RF24.h>
#include <nRF24L01.h>

//#define USE_ACK_PAYLOAD

//Set to true when signal is received
bool haveRxSignal = false;                                     

// The sizeof this struct should not exceed 32 bytes
struct RF24Data {
  uint8_t throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte dial1;
  byte dial2;
  byte switches;
};

struct RF24AckPayload {
  float lat;
  float lon;
  int16_t heading;
  int16_t pitch;
  int16_t roll;  
  int32_t alt;
  byte flags;
};

extern int receiver_input_roll, receiver_input_pitch, receiver_input_throttle, receiver_input_yaw;

// Single radio pipe address for the 2 nodes to communicate.
static const uint64_t pipe = 0xE8E8F0F0E1LL;

RF24 radio(7, 8); // CE, CSN

RF24Data nrf24Data;
#ifdef USE_ACK_PAYLOAD
  RF24AckPayload nrf24AckPayload;
  extern RF24AckPayload nrf24AckPayload;
#endif

void resetRF24Data() 
{
  nrf24Data.throttle = 0;
  nrf24Data.yaw = 128;
  nrf24Data.pitch = 128;
  nrf24Data.roll = 128;
  nrf24Data.dial1 = 0;
  nrf24Data.dial2 = 0;
  nrf24Data.switches = 0;
}

#ifdef USE_ACK_PAYLOAD
void resetRF24AckPayload() 
{
  nrf24AckPayload.lat = 0;
  nrf24AckPayload.lon = 0;
  nrf24AckPayload.heading = 0;
  nrf24AckPayload.pitch = 0;
  nrf24AckPayload.roll = 0;
  nrf24AckPayload.alt = 0;
  nrf24AckPayload.flags = 0;
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void receiver_setup() {

  resetRF24Data();
#ifdef USE_ACK_PAYLOAD
  resetRF24AckPayload();
#endif

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
#ifdef USE_ACK_PAYLOAD
  radio.setAutoAck(true);
  radio.enableAckPayload();
#else
  radio.setAutoAck(false);
#endif

  radio.openReadingPipe(1,pipe);
  radio.startListening();  
}

void receiver_loop() {
  
  static unsigned long loopsSinceLastRecv = 0;

#ifdef USE_ACK_PAYLOAD
  nrf24AckPayload.lat = ...;
  nrf24AckPayload.lon = ...;
  nrf24AckPayload.heading = ...;
  nrf24AckPayload.pitch = ...;
  nrf24AckPayload.roll = ...;
  nrf24AckPayload.alt = ...;
  nrf24AckPayload.flags = ...;
#endif

  loopsSinceLastRecv++;

  //unsigned long now = millis();
  while ( radio.available() ) {
#ifdef USE_ACK_PAYLOAD
    radio.writeAckPayload(1, &nrf24AckPayload, sizeof(RF24AckPayload));
#endif
    radio.read(&nrf24Data, sizeof(RF24Data));
    loopsSinceLastRecv = 0;
    haveRxSignal = true;
  }

  // Usually at most a single packet is received each time, in which case this function takes 220us.
  // If no packet is received this function takes around 31us, so we'll wait 220 - 31 us to keep the time constant.
  if ( loopsSinceLastRecv != 0 ) {
    delayMicroseconds(189); // 220 - 31
  }

  // With regular PWM the loop time is 250Hz so 250 loops is 1 second.
  // With oneshot125 the loop time is around 350Hz so 350 loop is 1 second.
  // Use a value somewhere in the middle.
  if ( loopsSinceLastRecv > 300 ) {
    // signal lost?
    resetRF24Data();
    haveRxSignal = false;
  }
  
  receiver_input_throttle = nrf24Data.throttle;  //map(nrf24Data.throttle, 0, 255, 1000, 2000);
  receiver_input_yaw =      nrf24Data.yaw;       //map(nrf24Data.yaw,      0, 255, 1000, 2000);
  receiver_input_pitch =    nrf24Data.pitch;     //map(nrf24Data.pitch,    0, 255, 2000, 1000);
  receiver_input_roll =     nrf24Data.roll;      //map(nrf24Data.roll,     0, 255, 1000, 2000);

  /*
  nrf24_rcData[AUX1] =     nrf24Data.switches & 0x1 ? 2000 : 1000;
  nrf24_rcData[AUX2] =     nrf24Data.switches & 0x2 ? 2000 : 1000;
  nrf24_rcData[AUX3] =     map(nrf24Data.dial1,    0, 255, 1000, 2000);
  nrf24_rcData[AUX4] =     map(nrf24Data.dial2,    0, 255, 1000, 2000);
  */
  
}

#endif // RX_NRF24

