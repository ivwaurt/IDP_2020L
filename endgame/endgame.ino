#include <SPI.h>
#include <WiFiNINA.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

//Motor init
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motorL = AFMS.getMotor(1);
Adafruit_DCMotor *motorR = AFMS.getMotor(2);
Adafruit_DCMotor *motorGrL = AFMS.getMotor(3);
Adafruit_DCMotor *motorGrR = AFMS.getMotor(4);
#define LM1 3       // left motor
#define RM1 4       // right motor


//Skeleton/Pseudo code for final robot




//Variables
//----------------------------

//State of the robot i.e. current objective
int state = 0; // 0 = line following, 1 = pathfinding ,2 = ....
int targetsCollected = 0;
bool target3Pulse;

//Line following
bool sensor_l;      // 1 = white, 0 = black
bool sensor_r;
bool sensor_s;
int sensor_s_timer=0;
bool L_faster_LF;       // 1 = faster, 0 = slower
bool R_faster_LF;
double tol=700;
double tol_s=850;

//Wifi
char ssid[] = "OnePlus 7 Pro";  //SSID
char pass[] = "11123456";       //password
uint8_t msg=0;
uint8_t buf;       //Byte
size_t siz = 1;
int status = WL_IDLE_STATUS;
bool alreadyConnected;

WiFiServer server(23);

//Motor variables
int speed_L_current=0;
int speed_R_current=0;
int speed_L;
int speed_R;
uint8_t v=128;//Default speed
uint8_t v_LF=200;//Line following speed

//Motor parameters
double ang2t = 24;      //time taken to rotate one degree(20.37)
double dis2t = 100;     //time taken to move one cm
uint8_t v_m = 128;  
//Gripper calibration
double ang2t_ml = 10.5;
double ang2t_mr = 35;



//Functions
//----------------------------

//Update motor speed
void motor_L(int speed){
  if (speed != speed_L_current){
    motorL->run( (speed>=0) ? FORWARD : BACKWARD );
    motorL->setSpeed(speed);
    speed_L_current = speed;
  }
}    

void motor_R(int speed){
  if (speed != speed_R_current){
    motorR->run( (speed>=0) ? FORWARD : BACKWARD );
    motorR->setSpeed(speed);
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
  while (1){
    //Sensor readings
    sensor_l = (analogRead(A0)>tol) ? 1 : 0;
    sensor_r = (analogRead(A1)>tol) ? 1 : 0;
    sensor_s = (analogRead(A2)>tol_s) ? 1 : 0;
    
    //Determine motor speeds via boolean logic
    L_faster_LF = keepRight ? (sensor_l || sensor_r) : (!sensor_l);
    R_faster_LF = keepRight ? (!sensor_r) : (sensor_l || sensor_r);  
    
    //Sensor_S
    if (sensor_s && sensor_s_timer<=0){
      motor_L(0);
      motor_R(0);
      return;
    }
    sensor_s_timer --;
    //Update speeds
    motor_L(L_faster_LF ? v_LF : 0);
    motor_R(R_faster_LF ? v_LF : 0);
    delay(50);
  }
}

//Bytes to motor movement
void msgMotor(uint8_t msg){
  //0th bit = move/not move, 1st bit = reverse
  motor_L( (bitRead(msg,3) ? v : 0) * (bitRead(msg,2) ? -1 : 1) );
  //2nd bit = move/not move, 3rd bit = reverse
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

//Grab target
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

//Release target
void dump(){
  grabber_R(90);
  forward(-20);
  grabber_R(-110);
}

//Pulse counter
bool findpulse(){
  //Return true if 3 pulse
  return 1; //Return 1 for now
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
  
  if (status != WL_CONNECTED){
    motor_L(0);
    motor_R(0);
      while (status != WL_CONNECTED) {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      status = WiFi.begin(ssid, pass);
      // wait 5 seconds for connection:
      delay(5000);
    }
    alreadyConnected = false; 
  }
  
  //Read connection
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
    //State 0: Line following
    case 0:
      Serial.println(state);
      forward(20);
      follow_line(1,150);
      forward(15);
      delay(1000);
      state = 1;
    break;
    
    
    //State 1: Pathfinding to target
    case 1:
      //Set motor speeds based on inputs
      msgMotor(msg);
      
      //Exit condition
      if (bitRead(msg,4)){
        motor_L(0);
        motor_R(0);
        state = 2;
        delay(1000);
      }
      
    break;
    
    
    //State 2: Picking up target
    case 2:
      grab();
      state=3;
      targetsCollected++;
      break;
      
      
    //State 3: return to grey dot
    case 3:
      //Sensor readings
      sensor_l = (analogRead(A0)>tol) ? 1 : 0;
      sensor_r = (analogRead(A1)>tol) ? 1 : 0;
      
      msgMotor(msg);
      
      //End condition
      if (sensor_r && sensor_l){
        state = 4;
        motor_L(0);
        motor_R(0);
      }
      break;
      
    //State 4: Line follow to charging/service area
    case 4:
      delay(1000);
      forward(10);
      delay(1000);
      rotate(10);
      follow_line(1,0);  //keep right and stop on 1st instance sensor_s = 1
      forward(10);
      delay(1000);
      //Dump robot mechanism
      dump();
      
      
      //If all target collected, go to state 6      
      if (targetsCollected >= 3){
        state = 6;
      } else {
        state = 5;
      }
      break;
      
    //State 5: Line follow to grey dot
    case 5:
      rotate(-200);
      follow_line(0,150);
      forward(15);
      delay(1000);
      state = 1;
      client.flush();
      msg=0;
      break;
    
    //State 6: End- Return to starting area
    case 6:
      rotate(-150);
      forward(45);
      delay(1000);
      follow_line(1,0);
      forward(30);
      state = 8;
      break;
    
    //State 8: end
    case 8:
        motor_L(0);
        motor_R(0);
        //do nothing
        break;
    
  }
  server.write(state);
}
