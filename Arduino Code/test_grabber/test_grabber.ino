/*------ Arduino Line Follower Code----- */
/*-------definning Inputs------*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motorL = AFMS.getMotor(1);
Adafruit_DCMotor *motorR = AFMS.getMotor(2);
Adafruit_DCMotor *motorGrL = AFMS.getMotor(3);
Adafruit_DCMotor *motorGrR = AFMS.getMotor(4);

//change motor voltage 

/*-------definning Outputs------*/
#define LM1 3       // left motor //(only 2 motors)
#define RM1 4       // right motor


//Variables
bool sensor_l=0;      // 1 = white, 0 = black
bool sensor_r=0;
bool sensor_s=0;      // side sensors (3rd one, at the right of AGV)
bool sensor_s_prev; 
bool speed_L;       // 1 = faster, 0 = slower
bool speed_R;


//Parameters
int tol = 700;          //Boundary between black/white
uint8_t v = 128;        //Motor speed during line following
uint8_t v_m = 128;  
double ang2t = 20;      //time taken to rotate one degree(20.37)
double ang2t_ml = 10.5;
double ang2t_mr = 35;
double dis2t = 100;     //time taken to move one cm

//-------------Functions------------//

//Update motor speed
void motor_L(int speed){
  if (speed != speed_L){
    //motorL->run( (speed>=0) ? FORWARD : BACKWARD );
    if (speed>=0){
      motorL->run(FORWARD);
    } else {
      motorL->run(BACKWARD);
    }
    motorL->setSpeed(abs(speed));
    speed_L = speed;
  }
}    

void motor_R(int speed){
  if (speed != speed_R){
    //motorR->run( (speed>=0) ? FORWARD : BACKWARD );
    if (speed>=0){
      motorR->run(FORWARD);
    } else {
      motorR->run(BACKWARD);
    }
    motorR->setSpeed(abs(speed));
    speed_R = speed;
  }
}    

//Rotate robot x degrees clockwise
void rotate(double angle){
  motor_L(v);
  motor_R(-v);
  delay(abs(angle)*ang2t);
  motor_L(0);
  motor_R(0);
}

//Move robot x cm forward
void forward(double dist){
  motor_L((dist>=0) ? v : -v);
  motor_R((dist>=0) ? v : -v);
  delay(abs(dist)*dis2t);  
  motor_L(0);
  motor_R(0);
}

void grabber_R(int angle){
  /*
  if (angle>0){
    motorGrR->run(BACKWARD);
  } else {
    motorGrR->run(FORWARD);
  }*/
  motorGrR->run( (angle>=0) ? FORWARD : BACKWARD );
  motorGrR->setSpeed(v_m);
  delay(abs(angle)*ang2t_mr);
  motorGrR->setSpeed(0);
}

void grabber_L(int angle){
  motorGrL->run( (angle>=0) ? BACKWARD : FORWARD );
  motorGrL->setSpeed(v_m);
  delay(abs(angle)*ang2t_ml);
  motorGrL->setSpeed(0);
}

void grab(){
  forward(-15);
  grabber_R(90);
  grabber_L(120);
  forward(20);
  grabber_R(-70);
  grabber_R(6);
  grabber_L(-130);
  delay(1000);
  grabber_R(-35);
}

//Release target
void dump(){
  grabber_R(90);
  forward(-25);
  grabber_R(-90);
}

//Follow line
/*void follow_line(bool keepRight, int count){
  while (1){
    //Sensor readings
    sensor_s_prev = sensor_s;
    sensor_l = (analogRead(A0)>tol) ? 1 : 0;
    sensor_r = (analogRead(A1)>tol) ? 1 : 0;
    sensor_s = (analogRead(A2)>tol) ? 1 : 0;
    //Determine motor speeds via boolean logic
    speed_L = keepRight ? (sensor_l || sensor_r) : (!sensor_l);
    speed_R = keepRight ? (!sensor_r) : (sensor_l || sensor_r);  
    //Sensor_S
    if (sensor_s && !sensor_s_prev){
      count--;
      if (count <= 0){
        motor_L(0);
        motor_R(0);
        return;
      }
    }
    //Update speeds
    motor_L(speed_L ? v : 0);
    motor_R(speed_R ? v : 0);
    delay(50);
  }
}
*/


//Setup and loop 
void setup()
{
  Serial.begin(9600);
  pinMode(A0, INPUT); //left sensor as input
  pinMode(A1, INPUT); //right sensor as input
  pinMode(A2, INPUT); //Side sensor as input
  AFMS.begin();
  grab();
  //forward(-20);
  //forward(20);
  dump();
  //forward(5);
}

void loop()
{
  Serial.println("123");
}
