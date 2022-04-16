//Tuttorial:https://youtu.be/FVCmw2u9j3k
//Rover controled by an android app
#include <Wire.h>
#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <Adafruit_PWMServoDriver.h>

SoftwareSerial mySerial(10, 9);// RX, TX 

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0;

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

char bt='S';
//int Speed = 50;
void setup() 
{
  mySerial.begin(9600);
  motor1.setSpeed(50);
  motor2.setSpeed(50);
  motor3.setSpeed(50);
  motor4.setSpeed(50);
  
  Serial.begin(9600);
  Serial.println("Rover Program version 1");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
   
  //yield();
  LegsStraight();
}

void loop() 
{
bt=mySerial.read();
//LegsStraight();
//sideways();



if(bt=='f'|| bt=='F')
{
 forward(); 
}
if(bt=='b'|| bt=='B')
{
 backward(); 
}
if(bt=='L'|| bt=='l')
{
 Legsleft(); 
}
if(bt=='R'|| bt=='r')
{
 Legsright(); 
}
if(bt=='W'|| bt=='w')
{
 LegsStraight(); 
}
if(bt=='S')
{
 Stop(); 
}


}

/*
 * angleToPulse(int ang)
 * gets angle in degree and returns the pulse width
 * also prints the value on seial monitor
 * written by Ahmad Shamshiri for Robojax, Robojax.com
 */
int angleToPulse(int ang)
{
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
  // Serial.print("Angle: ");Serial.print(ang);
  // Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;
}
void LegsStraight()
{
        pwm.setPWM(4, 0, angleToPulse(120) );//1
        pwm.setPWM(9, 0, angleToPulse(90) ); //2
        pwm.setPWM(11, 0, angleToPulse(120) );//3
        pwm.setPWM(12, 0, angleToPulse(120) );//4   

}
void sideways()
{
        pwm.setPWM(4, 0, angleToPulse(10) );//1
        pwm.setPWM(9, 0, angleToPulse(180) ); //2
        pwm.setPWM(11, 0, angleToPulse(15) );//3
        pwm.setPWM(12, 0, angleToPulse(210) );//4   

}
void Legsright()
{
        pwm.setPWM(4, 0, angleToPulse(120) );//1
        pwm.setPWM(9, 0, angleToPulse(120) ); //2
        pwm.setPWM(11, 0, angleToPulse(160) );//3
        pwm.setPWM(12, 0, angleToPulse(120) );//4   
}
void Legsleft()
{
        pwm.setPWM(4, 0, angleToPulse(120) );//1
        pwm.setPWM(9, 0, angleToPulse(60) ); //2
        pwm.setPWM(11, 0, angleToPulse(80) );//3
        pwm.setPWM(12, 0, angleToPulse(120) );//4   
}

void saas()
{
for(int i=0; i<16; i++)
  {
    for( int angle =0; angle<181; angle +=10){
      delay(100);
        pwm.setPWM(4, 0, angleToPulse(angle) );//1
        pwm.setPWM(9, 0, angleToPulse(angle) ); //2
        pwm.setPWM(11, 0, angleToPulse(angle) );
       pwm.setPWM(12, 0, angleToPulse(angle) );         
    }
 
  }

  delay(1000);

}

void forward()
{
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void backward()
{ 
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}
void left()
{
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD); 
}
void right()
{
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD); 
}
void Stop()
{
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
