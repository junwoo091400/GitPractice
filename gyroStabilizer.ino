#include <Wire.h>
#include <Servo.h>

//#define DOPRINTS

#define CALIBRATION_COUNT      1000
#define MPU6050_I2C_ADDRESS    0x68
#define MPU6050_PWR_MGMT_1     0x6B   // R/W
#define GYRO_ADDRESS           0x68
#define GYRO_VALUES_REGISTER   0x43
#define GYRO_HIGH_BYTE_FIRST

// Set the order in which the gyro values are read
#define GYRO_AXIS_1    gyro_pitch
#define GYRO_AXIS_2    gyro_roll
#define GYRO_AXIS_3    gyro_yaw

// Set a value of -1 for gyro axes that should be reversed
#define GYRO_INVERT_PITCH   1
#define GYRO_INVERT_ROLL    1
#define GYRO_INVERT_YAW     1

// Set a value of -1 for RC input axes that should be reversed
#define RC_INVERT_PITCH      1
#define RC_INVERT_ROLL       1
#define RC_INVERT_YAW        1

#define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING); //PIN 0

#define PITCH_RATE     1.8 // adjust to make servos move max range with tx control
#define ROLL_RATE      1.8 // adjust to make servos move max range with tx control
#define YAW_RATE       2.0

#define MIN_SERVO_PULSE      1100
#define MAX_SERVO_PULSE      1900

#define STICK_DEADZONE      6

#define RC_CHANS 8

#define ROLL_CHANNEL        0
#define PITCH_CHANNEL       1
#define THROTTLE_CHANNEL    2
#define YAW_CHANNEL         3
#define AUX1_CHANNEL        4
#define AUX2_CHANNEL        5

#define SERVO1_PORTD        B00001000 // digital pin 3
#define SERVO2_PORTD        B00010000 // digital pin 4
#define SERVO3_PORTD        B00100000 // digital pin 5
#define SERVO4_PORTD        B01000000 // digital pin 6

#define SERVO1_ON           PORTD |=  SERVO1_PORTD
#define SERVO1_OFF          PORTD &= ~SERVO1_PORTD

#define SERVO2_ON           PORTD |=  SERVO2_PORTD
#define SERVO2_OFF          PORTD &= ~SERVO2_PORTD

#define SERVO3_ON           PORTD |=  SERVO3_PORTD
#define SERVO3_OFF          PORTD &= ~SERVO3_PORTD

#define SERVO4_ON           PORTD |=  SERVO4_PORTD
#define SERVO4_OFF          PORTD &= ~SERVO4_PORTD

#define ALL_SERVOS_ON       PORTD |=  (SERVO1_PORTD | SERVO2_PORTD | SERVO3_PORTD | SERVO4_PORTD)
#define ALL_SERVOS_OFF      PORTD &= ~(SERVO1_PORTD | SERVO2_PORTD | SERVO3_PORTD | SERVO4_PORTD) 

volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]

//Servo servo1;

unsigned long loop_start_time = 0;

float pid_p_gain_roll = 0.5;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.0;               //Gain setting for the roll I-controller (0.05)
float pid_d_gain_roll = 0.0;               //Gain setting for the roll D-controller (15)
int   pid_max_roll = 400;                  //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = 0.5;              //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.0;              //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 0.0;              //Gain setting for the pitch D-controller.
int pid_max_pitch = 400;                   //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.01;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)
int pid_max_yaw_i = 100;                   //Maximum value of I term of the PID-controller (+/-)

float gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
float gyro_pitch, gyro_roll, gyro_yaw;
float gyro_roll_input, gyro_pitch_input, gyro_yaw_input;

int receiver_input_roll, receiver_input_pitch, receiver_input_throttle, receiver_input_yaw;
float pid_i_mem_roll, pid_roll_setpoint, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, pid_output_yaw, pid_last_yaw_d_error;

void i2c_write_reg(int address, byte reg, byte val) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

// Requires open I2C transmission with two bytes available
int gyro_read_int() {

  byte highByte = Wire.read();
  byte lowByte = Wire.read();

  return (highByte << 8) | lowByte;
}

void gyro_read_raw(){

  Wire.beginTransmission(GYRO_ADDRESS);                        //Start communication with the gyro
  Wire.write(GYRO_VALUES_REGISTER);                            //Start reading and auto increment with every read
  Wire.endTransmission();                                      //End the transmission
  Wire.requestFrom(GYRO_ADDRESS, 6);                           //Request 6 bytes from the gyro

  while (Wire.available() < 6);                                //Wait until the 6 bytes are received
  
  GYRO_AXIS_1 = gyro_read_int();
  GYRO_AXIS_2 = gyro_read_int();
  GYRO_AXIS_3 = gyro_read_int();

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

void setup() {
#ifdef DOPRINTS
  Serial.begin(9600);
#endif
  
  //Configure servo pins as output.
  DDRD |= (  SERVO1_PORTD
           | SERVO2_PORTD
           | SERVO3_PORTD
           | SERVO4_PORTD);
    
  PPM_PIN_INTERRUPT;
  
  i2c_write_reg(GYRO_ADDRESS, 0x6B, 0x80);                     //PWR_MGMT_1    -- DEVICE_RESET 1
  delayMicroseconds(50000);
  i2c_write_reg(GYRO_ADDRESS, 0x6B, 0x03);                     //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_write_reg(GYRO_ADDRESS, 0x1A, 0);                        //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2c_write_reg(GYRO_ADDRESS, 0x1B, 0x08);                     //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 500 deg/sec
  
  // Clear the 'sleep' bit to start the sensor.
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
  
  gyro_pitch_cal = 0;
  gyro_roll_cal = 0;
  gyro_yaw_cal = 0;
  
    for (int i = 0; i < CALIBRATION_COUNT ; i++) {
    gyro_read_raw();                                           //Read the gyro output.
    gyro_pitch_cal += gyro_pitch;                              //Add pitch value to gyro_pitch_cal.
    gyro_roll_cal  += gyro_roll;                               //Add roll value to gyro_roll_cal.
    gyro_yaw_cal   += gyro_yaw;                                //Add yaw value to gyro_yaw_cal.
    delayMicroseconds(3000);                                   //Wait 3 milliseconds before the next loop.
  }
  
  //Now that we have samples, we need to divide by the sample count to get the average gyro offset.
  gyro_pitch_cal /= CALIBRATION_COUNT;
  gyro_roll_cal  /= CALIBRATION_COUNT;
  gyro_yaw_cal   /= CALIBRATION_COUNT;
}

int loopCounter = 0;

void loop() {
  
  static int reads = 0;

  //Let's get the current gyro data and scale it to degrees per second for the pid calculations.
  gyro_read_raw();  
  gyro_apply_calibration();
  gyro_apply_inversion_and_scale();
  
  float uptake = 0.2;
  float oneMinusUptake = 1 - uptake;
  
  gyro_pitch_input = (gyro_pitch_input * oneMinusUptake) + (gyro_pitch * uptake);
  gyro_roll_input  = (gyro_roll_input * oneMinusUptake)  + (gyro_roll * uptake);
  gyro_yaw_input   = (gyro_yaw_input * oneMinusUptake)   + (gyro_yaw * uptake);
  
  float rollStabMagnitude  = (constrain((float)rcValue[AUX1_CHANNEL] - 1000, 0, 1000)) / 1000.0f;
  float pitchStabMagnitude = (constrain((float)rcValue[AUX2_CHANNEL] - 1000, 0, 1000)) / 1000.0f;
  gyro_roll_input *= rollStabMagnitude;
  gyro_pitch_input *= pitchStabMagnitude;
  
  /* do this to disable gyro stabilization
  gyro_roll_input = 0;
  gyro_pitch_input = 0;
  gyro_yaw_input = 0;
  */
  
  reads++;
    
  pid_pitch_setpoint = RC_INVERT_PITCH * ((float)rcValue[PITCH_CHANNEL] - 1500.0f);
  pid_roll_setpoint  = RC_INVERT_ROLL  * ((float)rcValue[ROLL_CHANNEL] - 1500.0f);
  pid_yaw_setpoint   = RC_INVERT_YAW   * ((float)rcValue[YAW_CHANNEL] - 1500.0f);
  
  if ( abs(pid_pitch_setpoint) < STICK_DEADZONE )
    pid_pitch_setpoint = 0;
  if ( abs(pid_roll_setpoint) < STICK_DEADZONE )
    pid_roll_setpoint = 0;
  if ( abs(pid_yaw_setpoint) < STICK_DEADZONE )
    pid_yaw_setpoint = 0;

  //The PID set point in degrees per second is determined by the receiver input.
  pid_pitch_setpoint *= PITCH_RATE;
  pid_roll_setpoint  *= ROLL_RATE;
  pid_yaw_setpoint   *= YAW_RATE;
  
  
#ifdef DOPRINTS
  Serial.print(gyro_roll_input);
  Serial.print("       ");
  Serial.print(rcValue[0]);
  Serial.print("       ");
  Serial.print(pid_roll_setpoint);
  Serial.print("       ");
#endif
  
  //PID inputs are known. So we can calculate the pid output.
  calculate_pid();
  
  int servoVal1 = pid_output_roll;
  int servoVal2 = pid_output_pitch;
  
#ifdef DOPRINTS
  Serial.print(pid_output_roll);
  Serial.print("       ");
  Serial.print(servoVal1);
  Serial.print("       ");
  
  Serial.println();
#endif
  
  
  /**/
  // elevon mixing
  int leftServo =  0.5 * servoVal2 + 0.5 * servoVal1;
  int rightServo = 0.5 * servoVal2 - 0.5 * servoVal1;
  servoVal1 = leftServo;
  servoVal2 = rightServo;
  /**/
  
  
  
  servoVal1 += 1500;
  servoVal2 += 1500;
  servoVal1 = constrain(servoVal1, MIN_SERVO_PULSE, MAX_SERVO_PULSE);
  servoVal2 = constrain(servoVal2, MIN_SERVO_PULSE, MAX_SERVO_PULSE);
  
  int servoVal3 = rcValue[THROTTLE_CHANNEL];
  
  while(micros() - loop_start_time < 4000);                                      //We wait until 4000us are passed.
  loop_start_time = micros();                                                    //Set the timer for the next loop.
  
  loopCounter++;
  
  if ( loopCounter >= 5 ) {
  
    loopCounter = 0;
    
    ALL_SERVOS_ON;                                                                 //Set servo outputs high.
    unsigned long timer_channel_1 = servoVal1 + loop_start_time;                   //Calculate the time of the faling edge of the esc-1 pulse.
    unsigned long timer_channel_2 = servoVal2 + loop_start_time;                   //Calculate the time of the faling edge of the esc-2 pulse.
    unsigned long timer_channel_3 = servoVal3 + loop_start_time;                   //Calculate the time of the faling edge of the esc-3 pulse.
    //unsigned long timer_channel_4 = esc_4 + loop_start_time;                       //Calculate the time of the faling edge of the esc-4 pulse.
  
    byte cnt = 0;
    while(cnt < 3){                                                                //Stay in this loop all servo outputs are low.
      cnt = 0;
      unsigned long esc_loop_start_time = micros();                                //Read the current time.
      if(timer_channel_1 <= esc_loop_start_time) {SERVO1_OFF; cnt++;}              //Set servo 1 low if the time is expired.
      if(timer_channel_2 <= esc_loop_start_time) {SERVO2_OFF; cnt++;}              //Set servo 2 low if the time is expired.
      if(timer_channel_3 <= esc_loop_start_time) {SERVO3_OFF; cnt++;}              //Set servo 3 low if the time is expired.
      //if(timer_channel_4 <= esc_loop_start_time) {SERVO4_OFF; cnt++;}              //Set servo 4 low if the time is expired.
    }
  }
}





// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
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
#ifdef DOPRINTS
  //Serial.print("err: ");
  //Serial.println(pid_error_temp);
#endif
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



/**************************************************************************************/
/***************                PPM SUM RX Pin reading             ********************/
/**************************************************************************************/

// attachInterrupt fix for promicro
ISR(INT6_vect){rxInt();}

// Read PPM SUM RX Data
  void rxInt(void) {
    uint16_t now,diff;
    static uint16_t last = 0;
    static uint8_t chan = 0;
  
    now = micros();
    sei();
    diff = now - last;
    last = now;
    if(diff>3000) chan = 0;
    else {
      if(900<diff && diff<2200 && chan<RC_CHANS ) {   //Only if the signal is between these values it is valid, otherwise the failsafe counter should move up
        rcValue[chan] = diff;
      }
    chan++;
  }
}
