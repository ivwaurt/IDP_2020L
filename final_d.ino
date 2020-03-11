#include <SPI.h>
#include <WiFiNINA.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

//Motor init
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motorL = AFMS.getMotor(1);     //Left wheel motor
Adafruit_DCMotor *motorR = AFMS.getMotor(2);     //Right wheel motor
Adafruit_DCMotor *motorGrL = AFMS.getMotor(3);   //Left grabber motor
Adafruit_DCMotor *motorGrR = AFMS.getMotor(4);   //Right wheel motor
#define LM1 3
#define RM1 4


//Variables
//----------------------------

//State of the robot i.e. current objective
int state = 0;             // 0 = line following, 1 = pathfinding ,2 = ....
int targetsCollected = 0;  //Number of targets currently collected

//Line following
bool sensor_l;          // Line following sensor readings
bool sensor_r;          // 1 = white, 0 = black
bool sensor_s;
int sensor_s_timer=0;   //Ignore side sensor for first x cycles
bool L_faster_LF;       // 1 = faster, 0 = slower
bool R_faster_LF;
double tol=700;         //Tolerance for left and right sensor
double tol_s=850;       //Tolerance for side sensor

//Wifi
char ssid[] = "OnePlus 7 Pro";                          //SSID
char pass[] = "www.youtube.com/watch?v=dQw4w9WgXcQ";    //password
uint8_t msg=0;                  //Variable to store message
uint8_t buf;                    //Byte in buffer
size_t siz = 1;                 //Size of buffer
int status = WL_IDLE_STATUS;    //Wifi status
bool alreadyConnected;

WiFiServer server(23);  //Start server

//Motor variables
int speed_L_current=0;   //Current speed of the motor (L)
int speed_R_current=0;   //Current speed of the motor (R)
int speed_L;       //Intended speed of the motor (L)
int speed_R;       //Intended speed of the motor (R)

uint8_t v=128;     //Default speed
uint8_t v_LF=200;  //Line following speed

//Motor parameters
double ang2t = 24;      //time (ms) taken to rotate one degree for AGV
double dis2t = 100;     
uint8_t v_m = 128;      //Speed for grabber motor

//Gripper calibration
double ang2t_ml = 10.5; //time (ms) taken to rotate one degree for gripper motor (L)
double ang2t_mr = 35;   //time (ms) taken to rotate one degree for gripper motor (R)



//Functions
//----------------------------

//Update motor speed
void motor_L(int speed){
  if (speed != speed_L_current){
    motorL->run( (speed>=0) ? FORWARD : BACKWARD );
    motorL->setSpeed(abs(speed));
    speed_L_current = speed;
  }
}    

void motor_R(int speed){
  if (speed != speed_R_current){
    motorR->run( (speed>=0) ? FORWARD : BACKWARD );
    motorR->setSpeed(abs(speed));
    speed_R_current = speed;
  }
} 

//Rotation and move forward
void rotate(double angle){
  //Angle +ve clockwise
  motor_L((angle>=0) ? v : -v);
  motor_R((angle>=0) ? -v : v);
  delay(abs(angle)*ang2t);
  motor_L(0);
  motor_R(0);
}

void forward(double dist){
  //dist +ve forward
  motor_L((dist>=0) ? v : -v);
  motor_R((dist>=0) ? v : -v);
  delay(abs(dist)*dis2t);  
  motor_L(0);
  motor_R(0);
}


//Line Follower
void follow_line(bool keepRight, int sensor_s_timer){
  // keepRight:  1 -> AGV follows right side of line , 0-> AGV follows left side of line
  // sensor_s_timer = min number of cycles for line following
  
  while (1){
    //Sensor readings
    sensor_l = (analogRead(A0)>tol) ? 1 : 0;
    sensor_r = (analogRead(A1)>tol) ? 1 : 0;
    sensor_s = (analogRead(A2)>tol_s) ? 1 : 0;
    
    //Determine motor speeds via boolean logic
    L_faster_LF = keepRight ? (sensor_l || sensor_r) : (!sensor_l);
    R_faster_LF = keepRight ? (!sensor_r) : (sensor_l || sensor_r);  
    
    //Exit condition
    if (sensor_s && sensor_s_timer<=0){
      motor_L(0);
      motor_R(0);
      return;
    }
    
    //Update speeds
    motor_L(L_faster_LF ? v_LF : 0);
    motor_R(R_faster_LF ? v_LF : 0);
    
    //Delay and move to next loop
    sensor_s_timer --;
    delay(50);
  }
}

//Update motor speeds from bytes sent from python
void msgMotor(uint8_t msg){
  //Left motor: 0th bit = move/not move, 1st bit = reverse
  motor_L( (bitRead(msg,3) ? v : 0) * (bitRead(msg,2) ? -1 : 1) );
  
  //Right motor: 2nd bit = move/not move, 3rd bit = reverse
  motor_R( (bitRead(msg,1) ? v : 0) * (bitRead(msg,0) ? -1 : 1) );
}

//Move grabber arm
void grabber_R(int angle){
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

//Grab target sequence
void grab(){
  forward(-15);
  grabber_R(110);
  grabber_L(150);
  forward(22);
  grabber_R(-95);
  grabber_R(7);
  grabber_L(-150);
  delay(1000);
  grabber_R(-35);
}

//Release target sequence
void dump(){
  grabber_R(90);
  forward(-20);
  grabber_R(-110);
}

//Actual code in robot
//----------------------------
void setup(){
  //Turn on motorshield and serial
  AFMS.begin();
  Serial.begin(9600);
  server.begin();
  
  //Sensor inputs
  pinMode(A0, INPUT); //left sensor as input
  pinMode(A1, INPUT); //right sensor as input
  pinMode(A2, INPUT); //Side sensor as input
}

void loop(){
  
  //Wifi stuff
  if (status != WiFi.status()) {
    status = WiFi.status();
  }
  
  //Keep tring to connect if not connected
  if (status != WL_CONNECTED){
    //Stop robot if disconnected
    motor_L(0);
    motor_R(0);
    while (status != WL_CONNECTED) {
      status = WiFi.begin(ssid, pass);
      // wait 5 seconds for each attempt
      delay(5000);
    }
    alreadyConnected = false; 
  }
  
  //Read connection if there is client
  WiFiClient client = server.available();
  if (client) {
    if (!alreadyConnected) {
      // clead out the input buffer:
      client.flush();
      alreadyConnected = true;
    }

    if (client.available() > 0) {
      // read the bytes incoming from the client:
      client.read(&buf , siz);
      msg = buf+1-1;
    }
  }
  
  
  //Switch statement based on current state of the robot
  
  switch(state){
    //State 0: First Line following
    case 0:
      forward(20);
      follow_line(1,150);
      forward(15);
      state = 1;
    break;
    
    
    //State 1: Pathfinding to target
    case 1:
      //Update motor speeds based on inputs from python
      msgMotor(msg);
      
      //Exit condition (4th bit is signal from python that it is in front of the robot)
      if (bitRead(msg,4)){
        motor_L(0);
        motor_R(0);
        state = 2;
      }
    break;
    
    
    //State 2: Picking up target
    case 2:
      grab();
      state=3;
      targetsCollected++;
      break;
      
    //State 3: return to T junction
    case 3:
      //Sensor readings
      sensor_l = (analogRead(A0)>tol) ? 1 : 0;
      sensor_r = (analogRead(A1)>tol) ? 1 : 0;
      
      //Update motor speeds based on inputs from python
      msgMotor(msg);
      
      //Exit condition (if both sensors read white it thinks it reached the T junction)
      if (sensor_r && sensor_l){
        state = 4;
        motor_L(0);
        motor_R(0);
      }
      break;
      
    //State 4: Line follow to charging/service area
    case 4:
      forward(10);
      rotate(10);
      follow_line(1,0); 
      //Dump after line follow
      dump();
      forward(10);
      
      //If all target collected, go to state 6, else go to state 5      
      if (targetsCollected >= 3){
        state = 6;
      } else {
        state = 5;
      }
      break;
      
    //State 5: Turn around and line follow back to T junction
    case 5:
      rotate(-200);
      follow_line(0,150);
      forward(15);
      state = 1;
      msg=0; 
      break;
    
    //State 6: End - Return to starting white box
    case 6:
      rotate(-150);
      forward(45);
      follow_line(1,0);
      forward(30);
      state = 7;
      break;
    
    //State 7: end
    case 7:
        motor_L(0);
        motor_R(0);
        //do nothing
        break;
    
  }
  //Send current state to python
  server.write(state);
}
