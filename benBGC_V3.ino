/* Controls a 1D self-balancing robot.
 * Adapted from "Bal_Code_v8_with_bluetooth_working_SVPWM" By Matt_M
 * online at http://www.instructables.com/id/Brushless-Gimbal-Balancing-Robot/
 * Note: All comments involving "Ben" in code refers to "Ben Ma" unless otherwise
 * noted.
 */

 /* Good starting point for variables:
  *  Freq Counter: 0x05
  *  P: 0.2
  *  I: 0
  *  D: 0
  *  Gyro_amount 0.97
  * 
  * If you want to edit the actual PID control code, a good place to start would be the function PID_and_motor_comd()
  */

const int FREQ_COUNTER = 0x05;
const double P = 0.2;
const double I = 0.0;
const double D = 0.0;
const int GYRO_AMOUNT = 0.97;

const int CALIBRATION_STEPS = 300; //adjust for shorter or longer calibration period, default=1024

#pragma GCC optimize ("3")


//main loop--------------------------------------------------------
#define LEDPIN   A1

bool power = false;            //run when power is true

uint16_t freqCounter = 0;
uint16_t oldfreqCounter = 0;
uint16_t loop_time = 0;         //how fast is the main loop running

//Battery stuff---------------------------------------------------
uint16_t Bat_Voltage = 0;
bool Bat_Discharged = false;      //true when (Bat_Voltage < dischargeVoltage)

#define dischargeVoltage  3*3.0*50.76    //number of cells * lipo cell discharged voltage / ((5/1023)*(13.3/3.3))
#define resetVoltage  3*3.7*50.76        //voltage to reset Bat_Discharged to false 


//angle calculations from MPU-6050--------------------------------------------------
#include <Wire.h>

#define MPU6050 0x68              //Device address
#define ACCEL_CONFIG 0x1C         //Accelerometer configuration address
#define GYRO_CONFIG 0x1B          //Gyro configuration address

//Registers: Accelerometer, Temp, Gyroscope

//These aren't necessarily correct for the way we have the IMU mounted on the robot.
//See the angle_calc function for more info -Ben
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C


//Sensor output scaling
#define accSens 0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s


int16_t  AcX, AcY, GyZ;

//IMU offset values
int16_t  AcX_offset = 0;
int16_t  AcY_offset = 0;
int16_t  GyZ_offset = 0;
int32_t  GyZ_offset_sum = 0;

int32_t GyZ_filter[32];
uint8_t filter_count = 0;
int32_t  GyZ_F;
float robot_angle;
float Acc_angle;            //angle calculated from acc. measurments

bool vertical = false;      //is the robot vertical enough to run (disabled right now -Ben)

bool GyZ_filter_on = true;  //apply simple average filter to Z gyro reading

#define Gyro_amount GYRO_AMOUNT //closer to 0 makes for quicker angle adjustment
//#define Gyro_amount 0.996   //percent of gyro in complementary filter T/(T+del_t) del_t: sampling rate, T acc. timeconstant ~1s


//motor control-------------------------------------------------------
#define PWM_A_MOTOR1 OCR2A
#define PWM_B_MOTOR1 OCR1B
#define PWM_C_MOTOR1 OCR1A

#define PWM_A_MOTOR0 OCR0A
#define PWM_B_MOTOR0 OCR0B
#define PWM_C_MOTOR0 OCR2B

// Space Vector PWM lookup table
// using uint8_t overflow for stepping
int8_t pwmSinMotor[] = {0,  5,  10, 16, 21, 27, 32, 37, 43, 48, 53, 59, 64, 69, 74, 79, 84, 89, 94, 99, 104,  109, 
111,  112,  114,  115,  116,  118,  119,  120,  121,  122,  123,  123,  124,  125,  125,  126,  126,  126,  127,  
127,  127,  127,  127,  127,  126,  126,  126,  125,  125,  124,  123,  123,  122,  121,  120,  119,  118,  116,  
115,  114,  112,  111,  110,  112,  113,  114,  116,  117,  118,  119,  120,  121,  122,  123,  124,  124,  125,  
125,  126,  126,  126,  127,  127,  127,  127,  127,  126,  126,  126,  125,  125,  124,  124,  123,  122,  121,  
120,  119,  118,  117,  116,  114,  113,  112,  110,  106,  101,  97, 92, 87, 82, 77, 72, 66, 61, 56, 51, 45, 40, 
35, 29, 24, 18, 13, 8,  2,  -2, -8, -13,  -18,  -24,  -29,  -35,  -40,  -45,  -51,  -56,  -61,  -66,  -72,  -77,  
-82,  -87,  -92,  -97,  -101, -106, -110, -112, -113, -114, -116, -117, -118, -119, -120, -121, -122, -123, -124, 
-124, -125, -125, -126, -126, -126, -127, -127, -127, -127, -127, -126, -126, -126, -125, -125, -124, -124, -123, 
-122, -121, -120, -119, -118, -117, -116, -114, -113, -112, -110, -111, -112, -114, -115, -116, -118, -119, -120, 
-121, -122, -123, -123, -124, -125, -125, -126, -126, -126, -127, -127, -127, -127, -127, -127, -126, -126, -126, 
-125, -125, -124, -123, -123, -122, -121, -120, -119, -118, -116, -115, -114, -112, -111, -109, -104, -99,  -94,  
-89,  -84,  -79,  -74,  -69,  -64,  -59,  -53,  -48,  -43,  -37,  -32,  -27,  -21,  -16,  -10,  -5, 0};

uint16_t MotorPower = 120; // 0 to 255, 255==100% power

uint8_t idl_motor_power = 80;

//motor numbers
#define L_Motor 0
#define R_Motor 1

//motor pole angle, 0->255 overflow to loop after >>8 shift
uint16_t R_MotorStep = 0;
uint16_t L_MotorStep = 0;


float robot_speed;


// speed of motors, -127 to 127
int16_t R_Speed = 0;
int16_t L_Speed = 0;

bool started = false;

//PID stuff---------------------------------------------------------------
int32_t robot_position = 0; //offset from corect position


//loop tuning values
/*
 * Original
#define kc 0.0002           //position feedback gain, (reposition robot after disturbance)0.0002
#define kv 0.02             //velocity feedback gain 0.02
#define kp 0.2              //angle gain 0.2
#define kd 0.025/(2*0.004) //angle velocity gain 0.025
*/

const int DT = 5; //delay between each loop step
double error;
double sum = 0;
double lastError = 0;
double deriv;

#define PID_out_max 100
#define PID_out_min -100

float PID_in, last_PID_in, Last_last_PID_in, PID_out;


//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
//------------------------------------------------------------------------------------
//------------ (o.o) (Ishan in the county jail for public drunkenness)----------------
//------------------------------------------------------------------------------------
//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA

void setup()
{
  pinMode (LEDPIN, OUTPUT);

  // Start Serial Port
  Serial.begin(230400);
  Serial.println("place robot on front or back side within 5sec. of power for gyroscope calibration");

  for (int i = 0; i < 50; i++)
  {
    digitalWrite(LEDPIN, !digitalRead(LEDPIN));
    delay (100);
  }

  //setup robot angle calculation
  Serial.println("calibrating gyroscope...........");
  angle_setup();

  //test battery voltage
  battery_test();
  Serial.print("Battery voltage = " );
  Serial.println(Bat_Voltage * 0.019698);


  //Setup brushless motor Controller
  Bl_Setup();
  Serial.println("place robot vertical to run");

  while (Serial.available())  Serial.read();        // empty RX buffer
}


void loop()
{
  //Serial.println("top of loop");
  //run main loop every ~4ms
  if ((freqCounter & 0x07f) == 0)
  {
    // record when loop starts
    oldfreqCounter = freqCounter;

     //test battery voltage every ~1s
    if ((freqCounter & 0x7FFF) == 0)
    {
      battery_test();
    }

    //calculate angle of robot
    angle_calc();
    PID_and_motor_comd();

    //calculate loop time
    if (freqCounter > oldfreqCounter)
    {
      if (loop_time < (freqCounter - oldfreqCounter)) loop_time = freqCounter - oldfreqCounter;
    }

  }
}



//--------------------------------------------------------------------------------------
//angle PID calculation and motor command signal generation-----------------------------
//--------------------------------------------------------------------------------------
void PID_and_motor_comd()
{
    if(vertical)
    {
      started = true;
      //calculate loop output, try to set robot_angle (error) to zero
      float adjusted_angle = (robot_angle-90);
      if(adjusted_angle<-180) adjusted_angle+=360;
      int error = adjusted_angle;
      Serial.print("Angle: ");
      Serial.print(adjusted_angle);
      Serial.print("\t\t");
  
      sum+=error; //I
      deriv = (error - lastError)/DT; //D
      PID_out = P*error + I*sum - D*deriv;
      lastError = error;
      
      if (PID_out > PID_out_max) PID_out = PID_out_max;
      else if (PID_out < PID_out_min) PID_out = PID_out_min;
  
      Serial.print("PID Out: ");
      Serial.println(PID_out);
      //robot_speed += PID_out; //integrate acceleration for velocity
      robot_speed = PID_out*100;
  
      //robot_position += robot_speed  - 1.5 * resp_rate * joyY; //integrate velocity for displacement and add offset to move robot
      robot_position += robot_speed; //this version doesn't need bluetooth stuff
  
      R_Speed = robot_speed;
      L_Speed = robot_speed;
      Serial.print("R_Speed:\t");
      Serial.print(R_Speed);
      Serial.print("\tL_Speed:\t");
      Serial.println(L_Speed);
  
      //adjust motors to turn robot
      //NO.  BAD 2D MOTION ATTEMPT. BAD BOY. -Ben
      /*
      rot_Speed = - 0.3 * joyX;
      R_Speed += rot_Speed;
      L_Speed -= rot_Speed;
      */
  
      //adjust motor power with respect to needed acc. and velocity
      if (PID_out >= 0 ) MotorPower = (idl_motor_power + PID_out * 100) * (0.000035 * robot_speed * robot_speed + 1);
      if (PID_out < 0 ) MotorPower = (idl_motor_power - PID_out * 100) * (0.000035 * robot_speed * robot_speed + 1);
  
      if (MotorPower > 255) MotorPower = 255;
  
      //run motors
      MoveMotors(R_Motor, (uint8_t)(R_MotorStep >> 8), MotorPower);
      MoveMotors(L_Motor, (uint8_t)(L_MotorStep >> 8), MotorPower);
      /*
      Original
      MoveMotors(R_Motor, (uint8_t)(R_MotorStep >> 8), MotorPower);
      MoveMotors(L_Motor, (uint8_t)(L_MotorStep >> 8), MotorPower);
      */
    }
    //if not vertical turn everything off
    else
    {
      motorPowerOff();
      Last_last_PID_in = 0;
      last_PID_in = 0;
      MotorPower = 0;
      robot_speed = 0;
      robot_position = 0;
    }
}


//--------------------------------------------------------------------
//Motor control stuff-------------------------------------------------
//--------------------------------------------------------------------
void Bl_Setup()
{
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  digitalWrite(LEDPIN, HIGH);

  cli();//stop interrupts

  //timer setup for 31.250KHZ phase correct PWM
  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
  TCCR0B = _BV(CS00);
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS10);
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);

  // enable Timer 1 interrupt
  TIMSK1 = 0;
  TIMSK1 |= _BV(TOIE1);
  // disable arduino standard timer interrupt
  TIMSK0 &= ~_BV(TOIE1);

  sei(); // Start Interrupt

  //turn off all PWM signals
  OCR2A = 0;  //11  APIN
  OCR2B = 0;  //D3
  OCR1A = 0;  //D9  CPIN
  OCR1B = 0;  //D10 BPIN
  OCR0A = 0;  //D6
  OCR0B = 0;  //D5

  // switch off PWM Power
  motorPowerOff();
}



// switch off motor power
void motorPowerOff() {
  MoveMotors(L_Motor, 0, 0);
  MoveMotors(R_Motor, 0, 0);
}


//TODO: FIGURE OUT HOW TO MOVE MOTORS (LOOK UP CODE?)

void MoveMotors(uint8_t motorNumber, uint8_t posStep, uint16_t power)
{
  uint16_t pwm_a;
  uint16_t pwm_b;
  uint16_t pwm_c;

  // lookup sine values from table with 120deg. offsets
  pwm_a = pwmSinMotor[(uint8_t)posStep];
  pwm_b = pwmSinMotor[(uint8_t)(posStep + 85)];
  pwm_c = pwmSinMotor[(uint8_t)(posStep + 170)];

  // scale motor power
  pwm_a = power * pwm_a;
  pwm_a = pwm_a >> 8;
  pwm_a += 128;

  pwm_b = power * pwm_b;
  pwm_b = pwm_b >> 8;
  pwm_b += 128;

  pwm_c = power * pwm_c;
  pwm_c = pwm_c >> 8;
  pwm_c += 128;

  // set motor pwm variables
  if (motorNumber == 0)
  {
    PWM_A_MOTOR0 = (uint8_t)pwm_a;
    PWM_B_MOTOR0 = (uint8_t)pwm_b;
    PWM_C_MOTOR0 = (uint8_t)pwm_c;
  }

  if (motorNumber == 1)
  {
    PWM_A_MOTOR1 = (uint8_t)pwm_a;
    PWM_B_MOTOR1 = (uint8_t)pwm_b;
    PWM_C_MOTOR1 = (uint8_t)pwm_c;
  }
}





//------------------------------------------------------------------
//Robot angle calculations------------------------------------------
//------------------------------------------------------------------

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}



//setup MPU6050
void angle_setup()
{
  Serial.println("In angle setup");
  
  Wire.begin();
  Serial.println("Wire begun");
  delay (100);
  Serial.println("About to access MPU");
  writeTo(MPU6050, PWR_MGMT_1, 0);
  Serial.println("1");
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  Serial.println("2");
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  Serial.println("3");
  delay (100);

  

  // calc Z gyro offset by averaging CALIBRATION_STEPS values
  // Note: if you want to reduce calibration time, reduce CALIBRATION_STEPS at top
  if (GyZ_filter_on == true)
  {
    GyZ_filter_on = false;
    for (int i = 0; i < CALIBRATION_STEPS; i++)
    {
      Serial.println("Angle setup inner loop");
      angle_calc();
      Serial.println(GyZ);
      GyZ_offset_sum += GyZ;
      digitalWrite(LEDPIN, !digitalRead(LEDPIN));
      delay (10);
    }
    GyZ_filter_on = true;
  }
  else
  {
    for (int i = 0; i < CALIBRATION_STEPS; i++)
    {
      Serial.println("Angle setup inner loop");
      angle_calc();
      Serial.println(GyZ);
      GyZ_offset_sum += GyZ;
      digitalWrite(LEDPIN, !digitalRead(LEDPIN));
      delay (10);
    }
  }
  GyZ_offset = GyZ_offset_sum >> 10;
  Serial.print("GyZ offset value = ");
  Serial.println(GyZ_offset);
}



//calculate robot tilt angle
void angle_calc()
{
  // read raw accel/gyro measurements from device

  /*
  //Original
  Wire.requestFrom(MPU6050, 4, true);  // request a total of 4 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)

  Wire.beginTransmission(MPU6050);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  // request a total of 2 registers
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  */
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  // request a total of 2 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)

  Wire.beginTransmission(MPU6050);
  Wire.write(0x3F); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  // request a total of 2 registers
  AcY = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  Wire.beginTransmission(MPU6050);
  Wire.write(0x45);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  // request a total of 2 registers
  GyZ = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)

  /*
  Serial.print("AcX: ");
  Serial.println(AcX);
  Serial.print("AcY: ");
  Serial.println(AcY);
  Serial.print("GyZ: ");
  Serial.println(GyZ);
  */

  if (GyZ_filter_on == true)
  {
    // simple low pass filter on gyro
    GyZ_filter[filter_count] = GyZ;

    filter_count++;

    if (filter_count > 15) filter_count = 0;

    GyZ_F = 0;
    for (int i = 0; i < 16; i++)
    {
      GyZ_F += GyZ_filter[i];
      GyZ = GyZ_F >> 4;
    }
  }

  // add mpu6050 offset values
  AcX += AcX_offset;
  AcY += AcY_offset;
  GyZ += GyZ_offset;

  //use complementary filter to calculate robot angle
  robot_angle += GyZ * 6.07968E-5;                      //integrate gyroscope to get angle       * 0.003984 (sec) / 65.536 (bits / (deg/sec))
  Acc_angle =  atan2(AcY, -AcX) * 57.2958;              //angle from acc. values       * 57.2958 (deg/rad)
  robot_angle = robot_angle * Gyro_amount + Acc_angle * (1.0 - Gyro_amount);
  
  //check if robot is vertical
  float adjusted_angle = (robot_angle-90);
  if(adjusted_angle<-180) adjusted_angle+=360;
  if (adjusted_angle > 50 || adjusted_angle < -50) vertical = false;
  if (adjusted_angle < 1 && adjusted_angle > -1) vertical = true;
}


void battery_test()
{
  Bat_Voltage = analogRead(A2);
  
  if (Bat_Voltage < dischargeVoltage)
  {
    Bat_Discharged = true;
    digitalWrite(LEDPIN, HIGH);    //LED on when battery discharged
  }
  else
  {
    if (Bat_Voltage > resetVoltage) Bat_Discharged = false;

    if (Bat_Discharged == false) digitalWrite(LEDPIN, !digitalRead(LEDPIN));  //flash LED on/off every 2sec when battery charged
  }
  Serial.print("In battery test. Current voltage is : ");
  Serial.print(Bat_Voltage * 0.019698);
  Serial.print(", necessary voltage is: ");
  Serial.println(dischargeVoltage * 0.019698);
}


//--------------------------------------------------------------
// code loop timing---------------------------------------------
//--------------------------------------------------------------
// minimize interrupt code length
// is called every 31.875us (510 clock cycles)  ???????
ISR( TIMER1_OVF_vect )
{
  //every 32 count of freqCounter ~1ms
  freqCounter++;

  if ((freqCounter & FREQ_COUNTER) == 0)
  {
    R_MotorStep += R_Speed;
    L_MotorStep -= R_Speed;
    
    /*
    Original
    R_MotorStep += R_Speed;
    L_MotorStep += L_Speed;
    */
  }
}












