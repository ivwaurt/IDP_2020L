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
bool sensor_l;      // 1 = white, 0 = black
bool sensor_r;
bool sensor_s;      // side sensors (3rd one, at the right of AGV)
bool speed_l;       // 1 = faster, 0 = slower
bool speed_r;


//Parameters
int tol = 700;          //Boundary between black/white
int v = 100;        //Motor speed during line following
int v_mov = 128;    //Motor speed during rotation
double ang2t = 20;      //time taken to rotate one degree(20.37)
double dis2t = 100;     //time taken to move one cm

//-------------Functions------------//

//Update motor speed
void motor_L(int speed){
  if (speed != speed_L){
    motorL->run( (speed>=0) ? FORWARD : BACKWARD );
    //if (speed>=0){
    //  motorL->run(FORWARD);
    //} else {
    //  motorL->run(BACKWARD);
    //}
    motorL->setSpeed(speed);
    speed_L = speed;
  }
}    

void motor_R(int speed){
  if (speed != speed_R){
    motorR->run( (speed>=0) ? FORWARD : BACKWARD );
    //if (speed>=0){
    //  motorR->run(FORWARD);
    //} else {
    //  motorR->run(BACKWARD);
    //}
    motorR->setSpeed(speed);
    speed_R = speed;
  }
}    

//Rotate robot x degrees clockwise
void rotate(double angle){
  motor_L(v_mov);
  motor_R(-v_mov);
  delay(angle*ang2t);
  motor_L(0);
  motor_R(0);
}

//Move robot x cm forward
void forward(double dist){
  motor_L((dist>=0) ? v_mov : -v_mov);
  motor_R((dist>=0) ? v_mov : -v_mov);
  delay(dist*dis2t);  
  motor_L(0);
  motor_R(0);
}


//Setup and loop 
void setup()
{
  Serial.begin(9600);
  pinMode(A0, INPUT); //left sensor as input
  pinMode(A1, INPUT); //right sensor as input
  pinMode(A2, INPUT); //Side sensor as input
  AFMS.begin();
}

void loop()
{
  //Print readings from sensors
  Serial.print("Left sensor: ");
  Serial.println(analogRead(A0));
  Serial.print("Right sensor: ");
  Serial.println(analogRead(A1));
  Serial.print("Side sensor: ");
  Serial.println(analogRead(A2));
  
  
  //Sensor reading to bool
  sensor_l = (analogRead(A0)>tol) ? 1 : 0;
  sensor_r = (analogRead(A1)>tol) ? 1 : 0;
  sensor_s = (analogRead(A2)>tol) ? 1 : 0;
  
  
  //Print sensor readings
  Serial.print(sensor_l);
  Serial.print(sensor_r);
  Serial.println(sensor_s);
  Serial.println("----");
  
  //Determine motor speeds using boolean logic
  speed_l = sensor_l || sensor_r;
  speed_r = ! sensor_r;
  
  //Set left motor speed
  if(speed_l){
      motor_L(v);
  } else {
      motor_L(0);
  }
  
  //Set right motor speed
  if(speed_r){
      motor_R(v);
  } else {
      motor_R(0);
  }
  
  //If at end point
  if(sensor_s){
    motor_L(0);
    motor_R(0);
    
    //Turn around
    forward(5);
    rotate(180);
    forward(7);
    
  }
  //Delay till next loop
  delay(100);
}
