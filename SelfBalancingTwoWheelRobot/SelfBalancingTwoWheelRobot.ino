/*
* Author: Mohamed Yassine Ben Salah
* Date: 24/03/2024
* Description: Arduino code for controlling a two-wheeled robot using a PID controller 
*              with feedback from an MPU6050 accelerometer/gyroscope sensor.
*/

#include <Wire.h>
#include "mpu6050.h"

MPU6050 mpu6050;

// Kp, and Kd parameters
float Kp = 14.0f;  //P term
float Kd = 8.0f;   //D term

float P = 0.0f, D = 0.0f;
float PID = 0.0f;  // PID output
float error = 0.0f, prevError = 0.0f;

// L298N motor driver  // ALL PINS MUST BE PWM
int Motor_L = 5;      //--> 11->IN4
int Motor_L_PWM = 6;  //--> 10->IN3
int Motor_R_PWM = 9;  //--> 9->IN2
int Motor_R = 10;     //--> 6->IN1

int PWM_Speed = 50;  //Base speed : 50
int MAX_SP = 255;    //Max speed : 255
int MIN_SP = 0;      //Min speed : 0

int PWM_BACKWARD = 0, PWM_FORWARD = 0;

float target_dir = 0.0f;
float current_dir = 0.0f;
float steerErr = 0;

float alpha = 0.5f;          // Adjust this value based on the filtering effect you desire
float filteredValue = 0.0f;  // Initialize filtered value

void setup() {
  Serial.begin(115200);
  delay(10);

  // Motor PIN as output
  pinMode(Motor_L, OUTPUT);
  pinMode(Motor_R, OUTPUT);
  pinMode(Motor_L_PWM, OUTPUT);
  pinMode(Motor_R_PWM, OUTPUT);

  digitalWrite(Motor_L, LOW);
  digitalWrite(Motor_R, LOW);
  digitalWrite(Motor_L_PWM, LOW);
  digitalWrite(Motor_R_PWM, LOW);

  delay(500);
  mpu6050_begin();
}

void loop() {
  current_dir = mpu6050_data();  // Update current direction

  steerErr = target_dir - current_dir;
  error = -steerErr;

  pidController();
  driveMotors();

  print4Debug();
}


void mpu6050_begin() {
  Wire.begin();
  delay(10);
  mpu6050.begin();
  delay(10);
  Serial.print("MPU6050: Starting calibration; leave device flat and still ... ");
  int error = mpu6050.begin();
  Serial.println(mpu6050.error_str(error));
}

float mpu6050_data() {
  MPU6050_t data = mpu6050.get();

  // Check if data is NaN
  if (isnan(data.dir.yaw)) {  //Error Handler
    Serial.println("Error: Invalid value detected! : NaN");
    Wire.begin();
    delay(5);
    mpu6050.begin();
    delay(5);
    data = mpu6050.get();
    return 0.0f;  // Default value when error occurs
  } else {
    filteredValue = alpha * data.dir.yaw + (1 - alpha) * filteredValue;
    return filteredValue;
    //return data.dir.roll;
    //return data.dir.yaw;
    //return data.dir.pitch;
    //angle may not = roll. It can roll, pitch, yaw or minus version of the three}
  }
}

void print4Debug(void) {
  Serial.print(" dir=");
  Serial.print(current_dir, 2);
  Serial.print(" tgt=");
  Serial.print(target_dir, 2);
  Serial.print(" steer=");
  Serial.print(steerErr);
  //
  Serial.print(" || PWM_B=");
  Serial.print(PWM_BACKWARD);
  Serial.print(" PWM_F=");
  Serial.print(PWM_FORWARD);
  Serial.print(" || PID =");
  Serial.print(PID);
  Serial.print(" P term=");
  Serial.print(P);
  Serial.print(" D term=");
  Serial.print(D);
  Serial.println();
}

void pidController(void) {
  // PID calculations
  P = error;
  D = error - prevError;
  PID = (Kp * P) + (Kd * D);

  // Error used in the next iteration
  prevError = error;
}

void driveMotors() {
  if (error > 0) {
    PWM_FORWARD = PWM_Speed + PID;
    PWM_FORWARD = constrain(PWM_FORWARD, MIN_SP, MAX_SP);
    analogWrite(Motor_L, 0);
    analogWrite(Motor_L_PWM, PWM_FORWARD);
    analogWrite(Motor_R_PWM, 0);
    analogWrite(Motor_R, PWM_FORWARD);
  } else if (error < 0) {
    PWM_BACKWARD = PWM_Speed - PID;
    PWM_BACKWARD = constrain(PWM_BACKWARD, MIN_SP, MAX_SP);
    analogWrite(Motor_L, PWM_BACKWARD);
    analogWrite(Motor_L_PWM, 0);
    analogWrite(Motor_R_PWM, PWM_BACKWARD);
    analogWrite(Motor_R, 0);
  } else {  // error == 0
    analogWrite(Motor_L, 0);
    analogWrite(Motor_L_PWM, 0);
    analogWrite(Motor_R_PWM, 0);
    analogWrite(Motor_R, 0);
  }
}