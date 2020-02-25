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
bool sensor_L;      // 1 = white, 0 = black
bool sensor_R;
bool sensor_s;      // side sensors (3rd one)
bool fast_L;       // 1 = faster, 0 = slower
bool fast_R;
int speed_L;
int speed_R;

//Parameters
double tol = 700;  //Boundary between black/white 
uint8_t v = 150;      //Motor speed
int i=0;

//Functions
void motor_L(int speed){
  if (speed == speed_L){
    return  
  }else{
    motorL->setSpeed(speed);
    speed_L = speed;
  }
}    

void motor_R(int speed){
  if (speed == speed_R){
    return  
  }else{
    motorR->setSpeed(speed);
    speed_R = speed;
  }
} 

//Loop
void setup()
{
  Serial.begin(9600);
  pinMode(A0, INPUT); //left sensor as input
  pinMode(A1, INPUT); //right sensor as input
  //pinMode(A2, INPUT); //3rd sensor as input
  AFMS.begin();
}

void loop()
{
  //Print readings from sensors
  Serial.print("Left sensor: ");
  Serial.println(analogRead(A0));
  Serial.print("Right sensor: ");
  Serial.println(analogRead(A1));
  
  //?
  motorL->run(FORWARD);
  motorR->run(FORWARD);
  
  //Sensor reading to bool
  sensor_L = (analogRead(A0)>tol) ? 1 : 0;
  sensor_R = (analogRead(A1)>tol) ? 1 : 0;
  //sensor_s = (analogRead(A2)>tol) ? 1 : 0;
  
  Serial.print(sensor_L);
  Serial.println(sensor_R);
  Serial.println("----");

  
  //Determine motor speeds using boolean logic
  fast_L = sensor_L || sensor_R;
  fast_R = !sensor_R;
  
  //Set left motor speed
  if(fast_L){
      motor_L(v);
  } else {
      motor_L(v/4);
  }
  
  //Set right motor speed
  if(fast_R){
      motor_R(v);
  } else {
      motor_R(v/4);
  }
  
  //Delay till next loop
  delay(250);
  
  //If 3rd sensor give reading, Stop
  if(sensor_s){
    motor_L(0);
    motor_R(0);
    //Set state to 1

}
