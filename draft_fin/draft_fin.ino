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
int state = 3; // 0 = line following, 1 = pathfinding ,2 = ....
int counter=0;
int pulse;
int targetsCollected = 0;
bool target3Pulse;

//Line following
bool sensor_l;      // 1 = white, 0 = black
bool sensor_r;
bool sensor_s=0;
int sensor_s_timer=0;
bool sensor_s_prev;
bool L_faster_LF;       // 1 = faster, 0 = slower
bool R_faster_LF;
double tol=700;

//Wifi
char ssid[] = "OnePlus 7 Pro";  //SSID
char pass[] = "11123456";       //password
uint8_t msg=0;
uint8_t buf;       //Byte
size_t siz = 1;
int status = WL_IDLE_STATUS;
WiFiServer server(23);
bool alreadyConnected;

//Motor functions
int speed_L_current=0;
int speed_R_current=0;
int speed_L;
int speed_R;
uint8_t v=128;     //Default speed
double ang2t = 24;      //time taken to rotate one degree(20.37)
double dis2t = 100;     //time taken to move one cm
uint8_t v_m = 128;  
double ang2t_ml = 10.5;
double ang2t_mr = 35;


//Parameters
//----------------------------
//Wifi


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


//Rotate robot x degrees clockwise
void rotate(double angle){
  motor_L((angle>=0) ? v : -v);
  motor_R((angle>=0) ? -v : v);
  delay(abs(angle)*ang2t);
  motor_L(0);
  motor_R(0);
}

void forward(double dist){
  motor_L((dist>=0) ? v : -v);
  motor_R((dist>=0) ? v : -v);
  delay(abs(dist)*dis2t);  
  motor_L(0);
  motor_R(0);
}

void grabber_R(int angle){
  Serial.println("RIGHT");
  if (angle>0){
    motorGrR->run(BACKWARD);
  } else {
    motorGrR->run(FORWARD);
  }
  motorGrR->setSpeed(v_m);
  delay(abs(angle)*ang2t_mr);
  motorGrR->setSpeed(0);
}

void grabber_L(int angle){
  Serial.println("LEFT");
  if (angle>0){
    motorGrL->run(BACKWARD);
  } else {
    motorGrL->run(FORWARD);
  }
  motorGrL->setSpeed(v_m);
  delay(abs(angle)*ang2t_ml);
  motorGrL->setSpeed(0);
}


//Line Follower
void follow_line(bool keepRight, int count){
  sensor_s_timer = 0;
  while (1){
    //Sensor readings
    sensor_l = (analogRead(A0)>tol) ? 1 : 0;
    sensor_r = (analogRead(A1)>tol) ? 1 : 0;
    sensor_s = (analogRead(A2)>tol) ? 1 : 0;
    //Determine motor speeds via boolean logic
    L_faster_LF = keepRight ? (sensor_l || sensor_r) : (!sensor_l);
    R_faster_LF = keepRight ? (!sensor_r) : (sensor_l || sensor_r);  
    //Sensor_S
    if (sensor_s && sensor_s_timer<=0){
      count--;
      if (count <= 0){
        motor_L(0);
        motor_R(0);
        return;
      }
      sensor_s_timer = 50;
    }
    sensor_s_timer --;
    //Update speeds
    motor_L(L_faster_LF ? v : 0);
    motor_R(R_faster_LF ? v : 0);
    delay(50);
  }
}

//Grabber
void grab(){
  forward(-15);
  grabber_R(90);
  grabber_L(120);
  forward(20);
  grabber_R(-70);
  grabber_R(6);
  grabber_L(-120);
  delay(1000);
  grabber_R(-35);
}

void dump(){
  grabber_R(90);
  forward(-20);
  grabber_R(-90);
}

//Pulse counter
void findpulse(){
  return 1;
}


//Actual code in robot
//----------------------------
void setup(){
  //Turn on motorshield and serial
  AFMS.begin();
  Serial.begin(9600);
  
  //Sensor inputs
  pinMode(A0, INPUT); //left sensor as input
  pinMode(A1, INPUT); //right sensor as input
  pinMode(A2, INPUT); //Side sensor as input
  
  //Connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 5 seconds for connection:
    delay(5000);
  }
  
  //Start server
  server.begin();
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
    server.begin();
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
      motor_R(100);
      follow_line(1,1);  //keep right and stop on 1st instance sensor_s = 1
      forward(15);
      delay(1000);
      state = 1;
    break;
    
    
    //State 1: Pathfinding to target
    case 1:
      //Set motor speeds based on inputs
      //0th bit = move/not move, 1st bit = reverse
      speed_L = (bitRead(msg,3) ? v : 0) * (bitRead(msg,2) ? -1 : 1);
      //2nd bit = move/not move, 3rd bit = reverse
      speed_R = (bitRead(msg,1) ? v : 0) * (bitRead(msg,0)? -1 : 1);
      motor_L(speed_L);
      motor_R(speed_R);
      
      //Termination condition
      if (bitRead(msg,4)){
        motor_L(0);
        motor_R(0);
        state = 2;
        delay(1000);
      }
      
    break;
    
    
    //State 2: Picking up target
    case 2:
      //Check Robot pulse
      target3Pulse = findpulse();
      if ( (target3Pulse && targetsCollected <2) || (targetsCollected>=2)) {
        //Grab
        grab()
        targetsCollected++;
        state = 3;
      } else {
        state = 7;
      }
      break;
      
      
    //State 3: return to grey dot
    case 3:
      //Send signal to rotate agv and go back to grey dot
      //Sensor readings
      sensor_l = (analogRead(A0)>tol) ? 1 : 0;
      sensor_r = (analogRead(A1)>tol) ? 1 : 0;
      sensor_s = (analogRead(A2)>tol) ? 1 : 0;
      
      //0th bit = move/not move, 1st bit = reverse
      speed_L = (bitRead(msg,3) ? v : 0) * (bitRead(msg,2) ? -1 : 1);
      //2nd bit = move/not move, 3rd bit = reverse
      speed_R = (bitRead(msg,1) ? v : 0) * (bitRead(msg,0)? -1 : 1);
      
      motor_L(speed_L);
      motor_R(speed_R);
      
      //End condition
      if (sensor_r || sensor_l){
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
      follow_line(1,1);  //keep right and stop on 1st instance sensor_s = 1
      forward(10);
      delay(1000);
      //Dump robot mechanism
      dump();
      rotate(-190);
      
      //If all target collected, go to state 6
      targetsCollected = 4;  // Set all targets colleted for now
      
      if (targetsCollected >= 4){
        state = 6;
      } else {
        state = 5;
      }
      break;
      
    //State 5: Line follow to grey dot
    case 5:
      follow_line(0,2);
      forward(15);
      delay(1000);
      state = 1;
      msg=0;
    
    //State 6: End- Return to starting area
    case 6:
      follow_line(0,1);
      forward(15);
      follow_line(1,1);
      forward(10);
      state = 8;
    
    //State 7 Return to grey dot and try again
    case 7:
      //0th bit = move/not move, 1st bit = reverse
      speed_L = (bitRead(msg,3) ? v : 0) * (bitRead(msg,2) ? -1 : 1);
      //2nd bit = move/not move, 3rd bit = reverse
      speed_R = (bitRead(msg,1) ? v : 0) * (bitRead(msg,0)? -1 : 1);
      motor_L(speed_L);
      motor_R(speed_R);
      
      //Termination condition
      if (bitRead(msg,4)){
        motor_L(0);
        motor_R(0);
        state = 1;
        delay(1000);
      }
    
    //State 8 or more: end
    
  }
  server.write(state);
}
