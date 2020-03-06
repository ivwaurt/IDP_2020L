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
int counter=0;
int pulse;
String stateString;

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
bool alreadyConnected = false; 

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


//Follow Line
void follow_line(bool keepRight, int count){
  while (1){
    //Sensor readings
    sensor_s_prev = sensor_s;
    sensor_l = (analogRead(A0)>tol) ? 1 : 0;
    sensor_r = (analogRead(A1)>tol) ? 1 : 0;
    sensor_s = (analogRead(A2)>tol) ? 1 : 0;
    Serial.print(sensor_l);
    Serial.print(sensor_r);
    Serial.println(sensor_s);
    //Determine motor speeds via boolean logic
    L_faster_LF = keepRight ? (sensor_l || sensor_r) : (!sensor_l);
    R_faster_LF = keepRight ? (!sensor_r) : (sensor_l || sensor_r);  
    //Sensor_S
    if (sensor_s && !sensor_s_prev){
      count--;
      if (count <= 0){
        motor_L(0);
        motor_R(0);
        return;
      }
    }
    Serial.println(R_faster_LF ? v : 0);
    Serial.println("---");
    //Update speeds
    motor_L(L_faster_LF ? v : 0);
    motor_R(R_faster_LF ? v : 0);
    delay(50);
  }
}

void grab(){
  Serial.println("TESTGRAB");
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
  
  Serial.println(state);
 
  
  
  //Switch State
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
      //To motor
      
      //0th bit = move/not move, 1st bit = reverse
      speed_L = (bitRead(msg,3) ? v : 0) * (bitRead(msg,2) ? -1 : 1);
      //2nd bit = move/not move, 3rd bit = reverse
      speed_R = (bitRead(msg,1) ? v : 0) * (bitRead(msg,0)? -1 : 1);
      motor_L(speed_L);
      motor_R(speed_R);
      
      if (bitRead(msg,4)){
        motor_L(0);
        motor_R(0);
        state = 2;
        delay(1000);
      }
      
    break;
    
    
    //State 2: Picking up target
    case 2:
      
      /*//Check robot pulse
      //Switch blinking LED to countinouly lit for 1 second
      //If service (red), charging (green)
      //if((pulse == 3) && (counter < 2)){//3 pulse robot
        //grab robot mechanism
          grab();
        //counter++;
        state=3;
        break;
      }
      //if ((pulse == 5) && (counter > 1)){
        //grab robot mechanism
        //grab();
        //state=3;
        //break;
      }
      //reverse robot back to grey dot (initial position)
      //python must know to delete current tested robot and proceed to the other ones
      //server.write("Reverse");
      //state=1;
      */
      grab();
      state = 3;
      break;
      
      
    //State 3: return to grey dot
    case 3:
      //Send signal to rotate agv and go back to grey dot
      //To motor
      sensor_l = (analogRead(A0)>tol) ? 1 : 0;
      sensor_r = (analogRead(A1)>tol) ? 1 : 0;
      sensor_s = (analogRead(A2)>tol) ? 1 : 0;
      //0th bit = move/not move, 1st bit = reverse
      speed_L = (bitRead(msg,3) ? v : 0) * (bitRead(msg,2) ? -1 : 1);
      //2nd bit = move/not move, 3rd bit = reverse
      speed_R = (bitRead(msg,1) ? v : 0) * (bitRead(msg,0)? -1 : 1);
      motor_L(speed_L);
      motor_R(speed_R);
      if (sensor_r || sensor_l){
        state = 4;
        motor_L(0);
        motor_R(0);
      }
      break;
      
    //State 4: Line follow to charging/service area
    case 4:
      Serial.println(state);
      delay(10000);
      forward(10);
      delay(5000);
      follow_line(1,1);  //keep right and stop on 1st instance sensor_s = 1
      forward(10);
      delay(1000);
      state = 5;
      break;
      
    //State 5: Dump and reverse line follow to grey dot
    case 5:
      //Dump robot mechanism
      dump();
      rotate(-190);
      follow_line(0,2);
      state = 1;
  }
  stateString = String(state);
  server.write(state);
}
