/*
        Programm: PID
        val_poti - value potentiometer
        acc_rawX, acc_rawY, acc_rawZ - accelerometer raw
        gyr_rawX, gyr_rawY, gyr_rawZ - gyroskop raw
*/
#include <Wire.h>
#include <Servo.h>

Servo motor_right;
Servo motor_left;

int16_t acc_rawX, acc_rawY, acc_rawZ;
int16_t gyr_rawX, gyr_rawY, gyr_rawZ;
 
float acceleration_angle[2], gyro_angle[2], total_angle[2];

float time_elapsed, time, time_previous;
float rad_to_deg = 180 / PI;

float  pwm_left, pwm_right;
float PID, PID_P = 0, PID_I = 0, PID_D = 0;
float error, error_previous;
// PID CONSTANTS
double Kp = 5;
double Ki = 0; 
double Kd = 6;
int val_poti;

double throttle = val_poti;
float desired_angle = 0; 


void setup() {
  Wire.begin(); 
  Wire.beginTransmission(0x68); // I2C address of the MPU-6050
  Wire.write(0x6B); // 0x6B | PWR_MGMT_1 --> MPU-6050 Register Map
  Wire.write(0);  // set to zero(wakes up the MPU 6050)
  Wire.endTransmission(true);
  
  Serial.begin(250000);
  motor_right.attach(3); // attatch the right motor to pin 3
  motor_left.attach(5);  // attatch the left motor to pin 5

  time = millis();
  
  motor_right.writeMicroseconds(2300);
  motor_left.writeMicroseconds(2300);
  delay(2000);
  motor_right.writeMicroseconds(800);
  motor_left.writeMicroseconds(800);
  delay(6000);
}

void loop() {
  // IMU

  int val_poti = map(analogRead(0), 0, 1023, 800, 2300);  // Potentiometer ist an analog Pin 0
  motor_right.writeMicroseconds(val_poti);
  motor_left.writeMicroseconds(val_poti);
  
  time_previous = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  time_elapsed = (time - time_previous) / 1000; 

  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // starting with register 0x3B | ACCEL_XOUT_H --> MPU-6050 Register Map
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);  // request a total of 6 registers
    
  acc_rawX = Wire.read()<<8|Wire.read();  // 0x3B | 59 | ACCEL_XOUT_H and 0x3C | 60 | ACCEL_XOUT_L
  acc_rawY = Wire.read()<<8|Wire.read();  // 0x3D | 61 | ACCEL_YOUT_H and 0x3E | 62 | ACCEL_YOUT_L
  acc_rawZ = Wire.read()<<8|Wire.read();  // 0x3F | 63 | ACCEL_ZOUT_H and 0x40 | 64 | ACCEL_ZOUT_L

  acceleration_angle[0] = atan((acc_rawY / 16384.0) / sqrt(pow((acc_rawX / 16384.0),2) + pow((acc_rawZ / 16384.0),2))) * rad_to_deg;
  acceleration_angle[1] = atan(-1 * (acc_rawX / 16384.0) / sqrt(pow((acc_rawY / 16384.0),2) + pow((acc_rawZ / 16384.0),2))) * rad_to_deg;
 
  Wire.beginTransmission(0x68);
  Wire.write(0x43); // starting with register 0x43 | GYRO_XOUT_H --> MPU-6050 Register Map
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);
   
  gyr_rawX = Wire.read()<<8|Wire.read();  // 0x43 | 67 | GYRO_XOUT_H and 0x44 | 68 | GYRO_XOUT_L
  gyr_rawY = Wire.read()<<8|Wire.read();  // 0x45 | 69 | GYRO_YOUT_H and 0x46 | 70 | GYRO_YOUT_L
  //gyr_rawZ = Wire.read()<<8|Wire.read();  // 0x47 | 71 | GYRO_ZOUT_H and 0x48 | 72 | GYRO_ZOUT_L
  
  gyro_angle[0] = gyr_rawX / 131.0; 
  gyro_angle[1] = gyr_rawY / 131.0;

  total_angle[0] = 0.98 * (total_angle[0] + gyro_angle[0] * time_elapsed) + 0.02 * acceleration_angle[0];
  total_angle[1] = 0.98 * (total_angle[1] + gyro_angle[1] * time_elapsed) + 0.02 * acceleration_angle[1];

  //Serial.print(acceleration_angle[0]);
  //Serial.print("\t");
  //Serial.print(acceleration_angle[1]);
  //Serial.print("\t");
  //Serial.print(gyro_angle[0]);
  //Serial.print("\t");
  //Serial.print(gyro_angle[1]);
  //Serial.print("\t");
  Serial.print(total_angle[0]);
  Serial.print("\t");
  Serial.print(total_angle[1]);
  Serial.print("\t");
  //Serial.print("\n");
  
  error = total_angle[1] - desired_angle;
  PID_P = Kp * error;
  if(-3 < error < 3){
    PID_I = PID_I + (Ki * error);  
  }
  
  PID_D = Kd * ((error - error_previous) / time_elapsed);
  PID = PID_P + PID_I + PID_D;
  
  if(PID < -1000){
    PID = -1000;
  }
  
  if(PID > 1000){
    PID = 1000;
  }

  pwm_left = throttle + PID;
  pwm_right = throttle - PID;

  //Right
  if(pwm_right < 1000){
    pwm_right = 1000;
  }
  
  if(pwm_right > 2000){
    pwm_right = 2000;
  }
  //Left
  if(pwm_left < 1000){
    pwm_left = 1000;
  }
  
  if(pwm_left > 2000){
    pwm_left = 2000;
  }
  motor_right.writeMicroseconds(pwm_right);
  motor_left.writeMicroseconds(pwm_left);
  error_previous = error; //Remember to store the previous error.
  //Serial.print(error);
  //Serial.print("\t");
  //Serial.print(PID_P);
  //Serial.print("\t");
  //Serial.print(PID_I);
  //Serial.print("\t");
  //Serial.print(PID_D);
  //Serial.print("\t");
  //Serial.print(PID);
  Serial.print("\t");
  Serial.print(pwm_left);
  Serial.print("\t");
  Serial.print(pwm_right);
  Serial.print("\n");
}
