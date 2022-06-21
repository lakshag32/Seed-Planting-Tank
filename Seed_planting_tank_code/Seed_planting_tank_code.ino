//https://www.youtube.com/watch?v=UZKxUFkwCc8&t=92s
//https://arduinogetstarted.com/tutorials/arduino-serial-plotter
//https://www.baldengineer.com/arduino-how-do-you-reset-millis.html
#define throttle_pin 4
#define turn_pin 2

unsigned long previous_time = 0;
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


//PID constants below

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
int target_angle_left;
int set_angle_right = 0; 
int current_angle = 180; 

bool turn_motors_off = false; 
bool change_angle = true; 

int counter = 0; 

//libraries
#include "Wire.h"
#include <MPU6050_light.h>

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
  pinMode(throttle_pin, INPUT);
  pinMode(turn_pin, INPUT); 
  

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
void turn_full_left(int yaw, int target_angle_left){
     error = target_angle_left-yaw; 
  
      //calculate P outputs for left and right motors
      p_output = kP*error;   
      d_output = (error-previous_error)*kD; 
      pd_output = p_output-d_output; 
  
      
      
      //motor power clamps to keep motors running w/in operating range(230-255 for my battery's current voltage, 
      //will step up w/ DC boost converter later)
        
  
      
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
       
       
     
      if (error>0){ //use one of the errors to determine whether the tank is turning left or right
        
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW); 
        analogWrite(enA, 0); //right motor
        analogWrite(enB, pd_output+230); // give robot//left motor
       
        }
      
      if(error<0){
      
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);  
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW); 
        analogWrite(enA, (pd_output-230)*-1); //right
        analogWrite(enB, 0); //left
  
         
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


void turn_full_right(int yaw){
   if (turn_motors_off == false){
     error = set_angle_right-yaw; 
  
      //calculate P outputs for left and right motors
      p_output = kP*error;   
      d_output = (error-previous_error)*kD; 
      pd_output = p_output-d_output; 
  
      
      
      //motor power clamps to keep motors running w/in operating range(230-255 for my battery's current voltage, 
      //will step up w/ DC boost converter later)
        
  
      
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
       
       
     
      if (error>0){ //use one of the errors to determine whether the tank is turning left or right
        
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW); 
        analogWrite(enA, 0); //right motor
        analogWrite(enB, pd_output+230); // give robot//left motor
       
        }
      
      if(error<0){
      
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);  
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW); 
        analogWrite(enA, (pd_output-230)*-1); //right
        analogWrite(enB, 0); //left
  
         
        }
      
     
      previous_error = error; 
    
    if(error >-5 and error <5){
        // turn on motor A
        turn_motors_off = true; 
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(enA, 0);
        analogWrite(enB, 0);
        
  
  
    }
    
   }
} 

void move_forward(int yaw){
    error = current_angle-yaw; 
    //calculate P outputs for left and right motors
    p_output = kP*error;   
    d_output = (error-previous_error)*kD; 
    pd_output = p_output-d_output; 

    
    
    //motor power clamps to keep motors running w/in operating range(230-255 for my battery's current voltage, 
    //will step up w/ DC boost converter later)
      

    
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
     
     
   
    if (error>0){ //use one of the errors to determine whether the tank is turning left or right
      
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW); 
      analogWrite(enA, 0); //right motor
      analogWrite(enB, pd_output+230); // give robot//left motor
     
      }
    
    if(error<0){
    
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);  
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW); 
      analogWrite(enA, (pd_output-230)*-1); //right
      analogWrite(enB, 0); //left

       
      }
    
   
  previous_error = error; 
  
  if(error >-5 and error <5){
      // turn on motor A
      turn_motors_off = true; 
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enA, 0);
      analogWrite(enB, 0);
      


  }
  
 
} 


void loop() {
  
  
    if (millis()-previous_time >= 10000){
        
        if (change_angle == true){
          target_angle_left = 180;   
          counter += 1; 
        }
        
        if (counter == 2){
          counter =  0; 
          target_angle_left = 0; 
          }
        
        previous_time = millis(); 
     }
    
    mpu.update();
    yaw = mpu.getAngleZ();
    Serial.println(target_angle_left); 
    turn_full_left(yaw, target_angle_left);  

    //if((millis()-timer)>10){ // print data every 10ms
      //yaw = mpu.getAngleZ(); //backside left = positive, backside right = negative
      //Serial.print("yaw: ");
      //Serial.print(yaw);
      //Serial.print("\n"); 
      //timer = millis();  
    //}
     
    //}
    
  
      
    
}
   /*
    //get yaw value
    while(true){
      mpu.update();
      if((millis()-timer)>10){ // print data every 10ms
        yaw = mpu.getAngleZ(); //backside left = positive, backside right = negative
        //Serial.print("yaw: ");
        //Serial.print(yaw);
        //Serial.print("\n"); 
        timer = millis();  
      }
  
      //get left error(positive_error, I think) and right error(negative_error, I think) 
      
      turn_full_left(yaw); 
      if (turn_motors_off == true){
        break;
        }
    }
    
    current_angle = 180; 
    move_forward(yaw);
    /*
    for (int i = 0; i < 100; i++) {
      mpu.update();
      if((millis()-timer)>10){ // print data every 10ms
        yaw = mpu.getAngleZ(); //backside left = positive, backside right = negative
        //Serial.print("yaw: ");
        //Serial.print(yaw);
        //Serial.print("\n"); 
        timer = millis();  
      }
       
      move_forward(yaw);
    }
      
    turn_motors_off = false;
     
    
    while(true){
      mpu.update();
      if((millis()-timer)>10){ // print data every 10ms
        yaw = mpu.getAngleZ(); //backside left = positive, backside right = negative
        //Serial.print("yaw: ");
        //Serial.print(yaw);
        //Serial.print("\n"); 
        timer = millis();  
      }
  
      //get left error(positive_error, I think) and right error(negative_error, I think) 
      
      turn_full_right(yaw); 
      if (turn_motors_off == true){
        break;
        }
    }

    current_angle = 0; 

    for (int i = 0; i < 5; i++) {
        mpu.update();
        if((millis()-timer)>10){ // print data every 10ms
          yaw = mpu.getAngleZ(); //backside left = positive, backside right = negative
          //Serial.print("yaw: ");
          //Serial.print(yaw);
          //Serial.print("\n"); 
          timer = millis();  
        }
         
        move_forward(yaw);
      }
        
    turn_motors_off = false;

    */
//}
