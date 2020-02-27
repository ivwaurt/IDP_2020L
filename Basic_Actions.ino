/*------ Arduino Line Follower Code----- */
/*-------definning Inputs------*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motorL = AFMS.getMotor(1);
Adafruit_DCMotor *motorR = AFMS.getMotor(2);

//change motor voltage 
/*-------definning Outputs------*/
#define LM1 3       // left motor //(only 2 motors)
#define RM1 4       // right motor


//Variables
int speed_L;
int speed_R;

//Parameters
double ang2t = 20;   //time taken to rotate one degree(20.37)
double dis2t = 100;     //time taken to move one cm

uint8_t v = 128;      //Motor speed during movement


//Functions

//Update motor speed
void motor_L(int speed){
  if (speed == speed_L){
    return;  
  }
  else{
    if (speed>=0){
      motorL->run(FORWARD);
    } else {
      motorL->run(BACKWARD);
    }
    motorL->setSpeed(speed);
    speed_L = speed;
  }
}    

void motor_R(int speed){
  if (speed == speed_R){
    return; 
  }else{
    if (speed>=0){
      motorR->run(FORWARD);
    } else {
      motorR->run(BACKWARD);
    }
    motorR->setSpeed(speed);
    speed_R = speed;
  }
}    

//Rotate robot x degrees clockwise
void rotate(double angle){
  motor_L(v);
  motor_R(-v);
  delay(angle*ang2t);
  motor_L(0);
  motor_R(0);
}

void forward(double dist){
  motor_L((dist>=0) ? v : -v);
  motor_R((dist>=0) ? v : -v);
  delay(dist*dis2t);  
  motor_L(0);
  motor_R(0);
}

//Loop
void setup()
{
  Serial.begin(9600);
  AFMS.begin();
  delay(2000);
  
  //Rotate 720 degrees
  rotate(720.);
  
  //Wait
  delay(2000);
}

void loop()
{
  //Run in a square
  forward(30);
  rotate(90);
}
