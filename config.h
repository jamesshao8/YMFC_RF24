
// Pick one of these
//#define GYRO_L3G4
#define GYRO_MPU6050

// Pick one of these
//#define RX_PWM // probably not working, sorry!
#define RX_NRF24

// Uncomment only if your ESCs support oneshot125
//#define ONESHOT125

// Set the order in which the gyro values are read
#define GYRO_AXIS_1    gyro_pitch
#define GYRO_AXIS_2    gyro_roll
#define GYRO_AXIS_3    gyro_yaw

// Set a value of -1 for gyro axes that should be reversed
#define GYRO_INVERT_PITCH  -1
#define GYRO_INVERT_ROLL    1
#define GYRO_INVERT_YAW     1

// Set a value of -1 for RC input axes that should be reversed
#define RC_INVERT_PITCH     -1
#define RC_INVERT_ROLL       1
#define RC_INVERT_YAW        1

#define STICK_DEADZONE      3 // steps either side of 128

#define PITCH_RATE     2.0
#define ROLL_RATE      2.0
#define YAW_RATE       2.0

// all pins used below must be on port D (digital pins 0-7)
#define BUZZER_PORTD        B00000100 // digital pin 2

#define MOTOR1_PORTD        B00010000 // digital pin 4
#define MOTOR2_PORTD        B00100000 // digital pin 5
#define MOTOR3_PORTD        B01000000 // digital pin 6
#define MOTOR4_PORTD        B00001000 // digital pin 3

// Don't touch /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define BUZZER_ON           PORTD |=  BUZZER_PORTD
#define BUZZER_OFF          PORTD &= ~BUZZER_PORTD

#define MOTOR1_ON           PORTD |=  MOTOR1_PORTD
#define MOTOR1_OFF          PORTD &= ~MOTOR1_PORTD

#define MOTOR2_ON           PORTD |=  MOTOR2_PORTD
#define MOTOR2_OFF          PORTD &= ~MOTOR2_PORTD

#define MOTOR3_ON           PORTD |=  MOTOR3_PORTD
#define MOTOR3_OFF          PORTD &= ~MOTOR3_PORTD

#define MOTOR4_ON           PORTD |=  MOTOR4_PORTD
#define MOTOR4_OFF          PORTD &= ~MOTOR4_PORTD

#define ALL_MOTORS_ON       PORTD |=  (MOTOR1_PORTD | MOTOR2_PORTD | MOTOR3_PORTD | MOTOR4_PORTD)
#define ALL_MOTORS_OFF      PORTD &= ~(MOTOR1_PORTD | MOTOR2_PORTD | MOTOR3_PORTD | MOTOR4_PORTD) 

#define CALIBRATION_COUNT        1000

#ifdef GYRO_MPU6050
  #define GYRO_ADDRESS           0x68
  #define GYRO_VALUES_REGISTER   0x43
  #define GYRO_HIGH_BYTE_FIRST
#else // L3G4
  #define GYRO_ADDRESS            105
  #define GYRO_VALUES_REGISTER    168
#endif


