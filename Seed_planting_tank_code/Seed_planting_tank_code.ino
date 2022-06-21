//https://www.baldengineer.com/arduino-how-do-you-reset-millis.html

unsigned long previous_time = 0; //for timing 

int yaw; //yaw angle that mpu6050 detects

unsigned long timer = 0; //for mpu6050 code(that I got from a library)

//variables for l298n motor driver below

// motor one
int enA = 10; //enable A
int in1 = 9; 
int in2 = 8;
// motor two
int enB = 5; //enable B
int in3 = 7;
int in4 = 6;


//PID variables below

//errors 
int error; 
int previous_error; 

//gains
int kP = 1; 
int kI = 0;
int kD = 5;

//outputs
int p_output; 
int d_output; 
int pd_output; 


//specific angle tank needs to turn to and maintain while moving 
int target_angle;

//variables used to switch target angle from 0 to 180 and viceversa each iteration of void loop()
bool change_angle = true; 
int counter = 0; 

//libraries
#include "Wire.h"
#include <MPU6050_light.h>

//MPU6050 stuff
MPU6050 mpu(Wire);


void setup()
{
  Serial.begin(9600);

  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  

  Wire.begin();
  
  //mpu6050 stuff below, got it from a library
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  delay(1000); 

}

//this function turns the tank to the target angle and makes it move forward while maintaining that angle
void turn_and_move(int yaw, int target_angle){ 
    error = target_angle-yaw; 

    //calculate P outputs for left and right motors
    p_output = kP*error;   
    d_output = (error-previous_error)*kD; 
    pd_output = p_output-d_output; 

    //motor power clamps to keep motors running w/in operating range(230-255 for my battery's current voltage, 
    //will increase operating change w/ DC boost converter later)  
    if (pd_output > 25){
      pd_output = 25; 
      }
      
    if (pd_output <0 and error > 0){
      pd_output = 0; 
      }

    if(pd_output <-25){
      pd_output = -25; 
      }
      
    if(pd_output>0 and error <0){
      pd_output = 0; 
      } 
        
    if (error>0){ //if tank is turning right      
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW); 
      analogWrite(enA, 0); //right motor
      analogWrite(enB, pd_output+230); //left motor
      }
    
    if(error<0){ //if tank is turning left 
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);  
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW); 
      analogWrite(enA, (pd_output-230)*-1); //right motor
      analogWrite(enB, 0); //left motor
      }
   
    previous_error = error; 
  
    if(error >-5 and error <5){
      // turn on motor A
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enA, 255);
      analogWrite(enB, 255);  
      }    
} 


void loop() {
    if (millis()-previous_time >= 10000){ //switch target angle from 0 to 180 and vice versa every 10 seconds. Switching the angle allows the robot to turn   
      if (change_angle == true){
        target_angle = 180;   
        counter += 1; 
      }
      
      if (counter == 2){
        counter =  0; 
        target_angle = 0; 
      }
      
      previous_time = millis(); 
      
    }
    
     mpu.update();
     yaw = mpu.getAngleZ();
     turn_and_move(yaw, target_angle);  
}
