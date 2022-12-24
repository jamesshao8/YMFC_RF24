/*  
A basic 4 channel transmitter using the nRF24L01 module.
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint64_t pipeOut = 0xE8E8F0F0E1LL;

RF24 radio(9, 10);

// The sizeof this struct should not exceed 32 bytes
struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
};

MyData data;

void resetData() 
{
  data.throttle = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
}

void setup()
{
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);

  radio.openWritingPipe(pipeOut);

  resetData();
}

/**************************************************/

// Returns a corrected value for a joystick position that takes into account
// the values of the outer extents and the middle of the joystick range.
int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
}

void loop()
{
  // The calibration numbers used here should be measured 
  // for your joysticks using the TestJoysticks sketch.
  data.throttle = mapJoystickValues( analogRead(0), 13, 524, 1015, true );
  data.yaw      = mapJoystickValues( analogRead(1),  1, 505, 1020, true );
  data.pitch    = mapJoystickValues( analogRead(2), 12, 544, 1021, true );
  data.roll     = mapJoystickValues( analogRead(3), 34, 522, 1020, true );

  radio.write(&data, sizeof(MyData));
}



