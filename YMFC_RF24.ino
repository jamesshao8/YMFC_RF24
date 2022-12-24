///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro.
#include <SPI.h>
#include <RF24.h>

#include "config.h"

#ifdef ONESHOT125
  // Oneshot 125 - UNTESTED!!!
  #define MIN_ESC_PULSE 125
  #define MAX_ESC_PULSE 250
  #define ESC_PULSE_MARGIN 15
  // we will also disable all interrupts - cannot use delay(), millis() or micros() !!!
#else
  // Regular PWM
  #define MIN_ESC_PULSE      1000
  #define MAX_ESC_PULSE      2000
  #define ESC_PULSE_MARGIN    200          // to leave some free space for PID to adjust motor speeds
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.5;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.05;              //Gain setting for the roll I-controller (0.05)
float pid_d_gain_roll = 20;                //Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.01;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)
int pid_max_yaw_i = 100;                   //Maximum value of I term of the PID-controller (+/-)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

enum _armedState {
  DISARMED,
  PENDING,   // yaw stick has been moved to the side but not yet returned to center
  ARMED
};

float gyro_roll_input, gyro_pitch_input, gyro_yaw_input;
int receiver_input_roll, receiver_input_pitch, receiver_input_throttle, receiver_input_yaw;

float pid_i_mem_roll, pid_roll_setpoint, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, pid_output_yaw, pid_last_yaw_d_error;

float mm1, mm2, mm3, mm4; // the non-throttle part of the esc values
int esc_1, esc_2, esc_3, esc_4;
_armedState armed_state;
unsigned long loop_start_time;
float gyro_pitch, gyro_roll, gyro_yaw;
float gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

unsigned long beepCounter = 0;
extern bool haveRxSignal;
bool hadRxSignalLastTime;

void receiver_setup();
void receiver_loop();

void i2c_write_reg(int address, byte reg, byte val) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void resetPIDs() {
  pid_i_mem_roll = 0;
  pid_last_roll_d_error = 0;
  pid_i_mem_pitch = 0;
  pid_last_pitch_d_error = 0;
  pid_i_mem_yaw = 0;
  pid_last_yaw_d_error = 0;
}

// Give ESCs a pulse to stop them beeping from lack of input.
void silenceESCs() {
  ALL_MOTORS_ON;
  delayMicroseconds(MIN_ESC_PULSE);                          
  ALL_MOTORS_OFF;
}

// Calibrates the ESCs and then goes into an endless loop. To use this,
// 1. Remove props
// 2. Uncomment the call to calibrateESCs in the setup() function
// 3. Disconnect power to ESCs
// 4. Upload the sketch and within 5 seconds of it starting, connect power to the ESCs
// 5. After calibration beeps have finished, disconnect power to ESCs
// 6. Comment out the call to calibrateESCs in the setup() function
// 7. Upload the sketch again
void calibrateESCs() {
  delay(5000); // 5 seconds, make it longer if you need to
  // Give ESCs a high pulse for approximately 5 seconds
  for (int i = 0; i < 1000; i++) {
    ALL_MOTORS_ON;
    delayMicroseconds(MAX_ESC_PULSE);                          
    ALL_MOTORS_OFF;
    delay(3);
  }
  // Give ESCs a low pulse for approximately 5 seconds
  for (int i = 0; i < 1000; i++) {
    ALL_MOTORS_ON;
    delayMicroseconds(MIN_ESC_PULSE);                          
    ALL_MOTORS_OFF;
    delay(3);
  }
  while(true);
}

void beep(int num) {
  for (int i = 0; i < num; i++) {
    BUZZER_ON;
    delayMicroseconds(16000); // 50ms
    delayMicroseconds(16000);
    delayMicroseconds(16000);
    BUZZER_OFF;
    delayMicroseconds(16000); // 50ms
    delayMicroseconds(16000);
    delayMicroseconds(16000);
  }
  beepCounter = 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){

  //Serial.begin(9600);
  Wire.begin();                                                //Start the I2C as master.

  //pinMode(1, OUTPUT);
  
  DDRD |= (BUZZER_PORTD                                        //Configure motor and buzzer pins as output.
           | MOTOR1_PORTD
           | MOTOR2_PORTD
           | MOTOR3_PORTD
           | MOTOR4_PORTD);
  
  BUZZER_OFF;

  //calibrateESCs();

  for (int i = 0; i < 60; i++)                                 //Wait 1 second before continuing.
    delayMicroseconds(16000);

#ifdef GYRO_MPU6050
  i2c_write_reg(GYRO_ADDRESS, 0x6B, 0x80);                     //PWR_MGMT_1    -- DEVICE_RESET 1
  delayMicroseconds(50000);
  i2c_write_reg(GYRO_ADDRESS, 0x6B, 0x03);                     //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_write_reg(GYRO_ADDRESS, 0x1A, 0);                        //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2c_write_reg(GYRO_ADDRESS, 0x1B, 0x08);                     //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 500 deg/sec
#else
  i2c_write_reg(GYRO_ADDRESS, 0x20, 0x0F);                     //Set the register 0x20 bits as 00001111 (Turn on the gyro and enable all axis)
  i2c_write_reg(GYRO_ADDRESS, 0x23, 0x90);                     //Set the register 0x23 bits as 10010000 (Block Data Update active & 500dps full scale)
#endif

  receiver_setup();

  for (int i = 0; i < 30; i++)                                 //Give the gyro and receiver time to start (500ms)
    delayMicroseconds(16000);

  beep(1);

  // Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  // Note most ESCs will chirp or beep when they first receive a valid signal, which causes some movement
  // of the motors but it is only brief and probably does not cause any significant rotation of the frame, 
  // so we just do the calibration while that chirp is happening and hope the gyro readings are unaffected.
  for (int i = 0; i < CALIBRATION_COUNT ; i++) {
    gyro_read_raw();                                           //Read the gyro output.
    gyro_pitch_cal += gyro_pitch;                              //Add pitch value to gyro_pitch_cal.
    gyro_roll_cal  += gyro_roll;                               //Add roll value to gyro_roll_cal.
    gyro_yaw_cal   += gyro_yaw;                                //Add yaw value to gyro_yaw_cal.
    silenceESCs();                                             //Prevent ESCs beeping annoyingly.
    delayMicroseconds(3000);                                   //Wait 3 milliseconds before the next loop.
  }
  
  //Now that we have samples, we need to divide by the sample count to get the average gyro offset.
  gyro_pitch_cal /= CALIBRATION_COUNT;
  gyro_roll_cal  /= CALIBRATION_COUNT;
  gyro_yaw_cal   /= CALIBRATION_COUNT;
  
  receiver_loop(); // update haveRxSignal to prevent 5 beeps on startup (3 from gaining rx signal)

  beep(2);
  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  
  hadRxSignalLastTime = haveRxSignal;
  receiver_loop();
  
  //Let's get the current gyro data and scale it to degrees per second for the pid calculations.
  gyro_read_raw();  
  gyro_apply_calibration();
  gyro_apply_inversion_and_scale();
  
  //PORTD |=  B00000010;
  
  gyro_pitch_input = (gyro_pitch_input * 0.8) + (gyro_pitch * 0.2);
  gyro_roll_input  = (gyro_roll_input * 0.8)  + (gyro_roll * 0.2);
  gyro_yaw_input   = (gyro_yaw_input * 0.8)   + (gyro_yaw * 0.2);

  //PORTD &= ~B00000010;
  
/*
Serial.print(gyro_pitch_cal);
Serial.print("       ");
Serial.print(gyro_roll_cal);
Serial.print("       ");
Serial.print(gyro_yaw_cal);
Serial.println("       ");
*/
/*
Serial.print(gyro_roll_input);
Serial.print("       ");
Serial.print(gyro_pitch_input);
Serial.print("       ");
Serial.print(gyro_yaw_input);
Serial.println("       ");
*/
/*
Serial.print(receiver_input_pitch);
Serial.print("       ");
Serial.print(receiver_input_roll);
Serial.print("       ");
Serial.print(receiver_input_yaw);
Serial.println("       ");
*/
/*
Serial.print(pid_pitch_setpoint);
Serial.print("       ");
Serial.print(pid_roll_setpoint);
Serial.print("       ");
Serial.print(pid_yaw_setpoint);
Serial.println("       ");
*/
/*
Serial.print(esc_1);
Serial.print("       ");
Serial.print(esc_2);
Serial.print("       ");
Serial.print(esc_3);
Serial.print("       ");
Serial.print(esc_4);
Serial.println();
*/
/*
Serial.print(armed_state);
Serial.println();
*/

  beepCounter++;  

  //For starting the motors: throttle low and yaw left (step 1).
  if(receiver_input_throttle < 13 && receiver_input_yaw > 242)
    armed_state = PENDING;
  
  //When yaw stick is back in the center position start the motors (step 2).
  if(armed_state == PENDING && receiver_input_throttle < 13 && receiver_input_yaw < 141){
    armed_state = ARMED;
    beep(3);
    //Reset the pid controllers for a bumpless start.
    resetPIDs();
  }
  
  //Stopping the motors: throttle low and yaw right.
  if(receiver_input_throttle < 13 && receiver_input_yaw < 13) {
    if ( armed_state == ARMED ) {
      beep(2);
    }
    else {
      if ( beepCounter % 100 == 0 ) {
        beep(1);
      }
    }
    armed_state = DISARMED;
  }

  if ( !hadRxSignalLastTime && haveRxSignal ) {
    // gained rx signal
    beep(3);
  }
  else if ( hadRxSignalLastTime && !haveRxSignal ) {
    // lost rx signal
    beep(2);
  }
  else if ( !haveRxSignal || (armed_state == ARMED && receiver_input_throttle < 13) ) {
    // slow beep when armed, or when no rx signal
    if ( beepCounter % 400 == 0 ) {
      beep(1);
    }
  }
  
  pid_pitch_setpoint = RC_INVERT_PITCH * (receiver_input_pitch - 128);
  pid_roll_setpoint  = RC_INVERT_ROLL  * (receiver_input_roll - 128);
  pid_yaw_setpoint   = RC_INVERT_YAW   * (receiver_input_yaw - 128);
  
  if ( abs(pid_pitch_setpoint) < STICK_DEADZONE )
    pid_pitch_setpoint = 0;
  if ( abs(pid_roll_setpoint) < STICK_DEADZONE )
    pid_roll_setpoint = 0;
  if ( abs(pid_yaw_setpoint) < STICK_DEADZONE )
    pid_yaw_setpoint = 0;

  // Don't try to yaw when landing
  if ( receiver_input_throttle < 13 )
    pid_yaw_setpoint = 0;

  //The PID set point in degrees per second is determined by the receiver input.
  pid_pitch_setpoint *= PITCH_RATE;
  pid_roll_setpoint  *= ROLL_RATE;
  pid_yaw_setpoint   *= YAW_RATE;
  
  //PID inputs are known. So we can calculate the pid output.
  calculate_pid();
  

  int throttle = map(receiver_input_throttle, 0, 255, MIN_ESC_PULSE, MAX_ESC_PULSE - ESC_PULSE_MARGIN);
  
  if (armed_state == ARMED && receiver_input_throttle > 13 ){

    //#define MIXBAL_PITCH 0.772 // ZMR250
    #define MIXBAL_PITCH  0.85 // Woody3

/*
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
*/

    {
      // Constrain PID output values so that the final value given to ESC does not exceed what is achievable in reality (2000us).
      mm1 = - MIXBAL_PITCH * pid_output_pitch + pid_output_roll - pid_output_yaw;
      mm2 = + MIXBAL_PITCH * pid_output_pitch + pid_output_roll + pid_output_yaw;
      mm3 = + MIXBAL_PITCH * pid_output_pitch - pid_output_roll - pid_output_yaw;
      mm4 = - MIXBAL_PITCH * pid_output_pitch - pid_output_roll + pid_output_yaw;
    
      float mm = max(abs(mm1), max(abs(mm2), max(abs(mm3), abs(mm4))));
    
      if ( mm > ESC_PULSE_MARGIN ) {
        float mod = ESC_PULSE_MARGIN / mm;
        mm1 *= mod;
        mm2 *= mod;
        mm3 *= mod;
        mm4 *= mod;
        pid_output_pitch *= mod;
        pid_output_roll *= mod;
        pid_output_yaw *= mod;
      }
    }

    esc_1 = throttle + mm1;
    esc_2 = throttle + mm2;
    esc_3 = throttle + mm3;
    esc_4 = throttle + mm4;

    //Limit pulses to 2000us.
    // should not be necessary since the values are already arranged to be achievable values
    
    esc_1 = constrain(esc_1, 1000, throttle + 200);
    esc_2 = constrain(esc_2, 1000, throttle + 200);
    esc_3 = constrain(esc_3, 1000, throttle + 200);
    esc_4 = constrain(esc_4, 1000, throttle + 200);
    
    /*esc_1 = constrain(esc_1, 1100, 2000);
    esc_2 = constrain(esc_2, 1100, 2000);
    esc_3 = constrain(esc_3, 1100, 2000);
    esc_4 = constrain(esc_4, 1100, 2000);*/
  }
  else { // disarmed
    esc_1 = MIN_ESC_PULSE;                                                           //If start is not 2 keep a 1000us pulse for esc-1.
    esc_2 = MIN_ESC_PULSE;                                                           //If start is not 2 keep a 1000us pulse for esc-2.
    esc_3 = MIN_ESC_PULSE;                                                           //If start is not 2 keep a 1000us pulse for esc-3.
    esc_4 = MIN_ESC_PULSE;                                                           //If start is not 2 keep a 1000us pulse for esc-4.
  }

  // Use this to check which motor is which
  //esc_1 = esc_2 = esc_3 = esc_4 = MIN_ESC_PULSE;
  //esc_4 = throttle;

#ifdef ONESHOT125
  writeOneshot125ToESCs();
#else
  writeRegularPWMToESCs();
#endif

}

void writeRegularPWMToESCs()
{
  //esc_1 = map(esc_1, 125, 250, 1000, 2000);
  //esc_2 = map(esc_2, 125, 250, 1000, 2000);
  //esc_3 = map(esc_3, 125, 250, 1000, 2000);
  //esc_4 = map(esc_4, 125, 250, 1000, 2000);
  
  // Regular PWM output to ESCs
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loop_start_time < 4000);                                      //We wait until 4000us are passed.
  loop_start_time = micros();                                                    //Set the timer for the next loop.

  ALL_MOTORS_ON;                                                                 //Set motor outputs high.
  unsigned long timer_channel_1 = esc_1 + loop_start_time;                       //Calculate the time of the faling edge of the esc-1 pulse.
  unsigned long timer_channel_2 = esc_2 + loop_start_time;                       //Calculate the time of the faling edge of the esc-2 pulse.
  unsigned long timer_channel_3 = esc_3 + loop_start_time;                       //Calculate the time of the faling edge of the esc-3 pulse.
  unsigned long timer_channel_4 = esc_4 + loop_start_time;                       //Calculate the time of the faling edge of the esc-4 pulse.

  byte cnt = 0;
  while(cnt < 4){                                                                //Stay in this loop all motor outputs are low.
    cnt = 0;
    unsigned long esc_loop_start_time = micros();                                //Read the current time.
    if(timer_channel_1 <= esc_loop_start_time) {MOTOR1_OFF; cnt++;}              //Set motor 1 low if the time is expired.
    if(timer_channel_2 <= esc_loop_start_time) {MOTOR2_OFF; cnt++;}              //Set motor 2 low if the time is expired.
    if(timer_channel_3 <= esc_loop_start_time) {MOTOR3_OFF; cnt++;}              //Set motor 3 low if the time is expired.
    if(timer_channel_4 <= esc_loop_start_time) {MOTOR4_OFF; cnt++;}              //Set motor 4 low if the time is expired.
  }
}

void writeOneshot125ToESCs()
{
  int remainder = 1000 - esc_1 - esc_2 - esc_3 - esc_4;
  MOTOR1_ON; delayMicroseconds(esc_1); MOTOR1_OFF;
  MOTOR2_ON; delayMicroseconds(esc_2); MOTOR2_OFF;
  MOTOR3_ON; delayMicroseconds(esc_3); MOTOR3_OFF;
  MOTOR4_ON; delayMicroseconds(esc_4); MOTOR4_OFF;
  delayMicroseconds(remainder);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Requires open I2C transmission with two bytes available
int gyro_read_int() {

#ifdef GYRO_HIGH_BYTE_FIRST
  byte highByte = Wire.read();
  byte lowByte = Wire.read();
#else
  byte lowByte = Wire.read();                                       
  byte highByte = Wire.read();
#endif

  return (highByte << 8) | lowByte;
}

void gyro_read_raw(){

#ifdef ONESHOT125
  sei();
#endif

  Wire.beginTransmission(GYRO_ADDRESS);                        //Start communication with the gyro
  Wire.write(GYRO_VALUES_REGISTER);                            //Start reading and auto increment with every read
  Wire.endTransmission();                                      //End the transmission
  Wire.requestFrom(GYRO_ADDRESS, 6);                           //Request 6 bytes from the gyro

  while (Wire.available() < 6);                                //Wait until the 6 bytes are received
  
  GYRO_AXIS_1 = gyro_read_int();
  GYRO_AXIS_2 = gyro_read_int();
  GYRO_AXIS_3 = gyro_read_int();

#ifdef ONESHOT125
  cli();
#endif
}

inline void gyro_apply_calibration() {
  gyro_pitch -= gyro_pitch_cal;
  gyro_roll  -= gyro_roll_cal;
  gyro_yaw   -= gyro_yaw_cal;
}

inline void gyro_apply_inversion_and_scale() {
  gyro_pitch *= GYRO_INVERT_PITCH / 57.14286;                   //Gyro pid input is deg/sec.
  gyro_roll  *= GYRO_INVERT_ROLL / 57.14286;                    //Gyro pid input is deg/sec.
  gyro_yaw   *= GYRO_INVERT_YAW / 57.14286;                     //Gyro pid input is deg/sec.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video series:
//www.youtube.com/watch?v=JBvnB0279-Q

void calculate_pid(){

  float pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  pid_i_mem_pitch = constrain(pid_i_mem_pitch, -pid_max_pitch, pid_max_pitch);
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  pid_output_pitch = constrain(pid_output_pitch, -pid_max_pitch, pid_max_pitch);
    
  pid_last_pitch_d_error = pid_error_temp;

  

  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  pid_i_mem_roll = constrain(pid_i_mem_roll, -pid_max_roll, pid_max_roll);
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  pid_output_roll = constrain(pid_output_roll, -pid_max_roll, pid_max_roll);
  
  pid_last_roll_d_error = pid_error_temp;


  
  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  pid_i_mem_yaw = constrain(pid_i_mem_yaw, -pid_max_yaw_i, pid_max_yaw_i);
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  pid_output_yaw = constrain(pid_output_yaw, -pid_max_yaw, pid_max_yaw);
    
  pid_last_yaw_d_error = pid_error_temp;

  
}





