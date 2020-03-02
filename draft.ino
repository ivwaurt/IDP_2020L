

//Skeleton/Pseudo code for final robot




//Variables
//----------------------------

//State of the robot i.e. current objective
int state = 0; // 0 = line following, 1 = pathfinding ,2 = ....

//Line following
bool sensor_l;      // 1 = white, 0 = black
bool sensor_r;
bool sensor_s;
bool L_faster_LF;       // 1 = faster, 0 = slower
bool R_faster_LF;

//Wifi
char ssid[] = "OnePlus 7 Pro";  //SSID
char pass[] = "11123456";       //password
uint8_t msg;
uint8_t buf;       //Byte
size_t siz = 1;
int status = WL_IDLE_STATUS;
WiFiServer server(23);
bool alreadyConnected = false; 

//Motor functions
int speed_L_current=0;
int speed_R_current=0;
int speed_L=0;
int speed_R=0;
uint8_t v=128;     //Default speed


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


//Follow Line
void follow_line(bool keepRight, int count){
  while 1{
    //Sensor readings
    sensor_s_prev = sensor_s;
    sensor_l = (analogRead(A0)>tol) ? 1 : 0;
    sensor_r = (analogRead(A1)>tol) ? 1 : 0;
    sensor_s = (analogRead(A2)>tol) ? 1 : 0;
    //Determine motor speeds via boolean logic
    L_faster_LF = keepRight ? (sensor_l || sensor_r) : (!sensor_l);
    R_faster_LF = keepRight ? (!sensor_r) : (sensor_l || sensor_r);  
    //Sensor_S
    if (sensor_s && !sensor_s_prev){
      count--;
      if (count <= 0){
        motor_L(0);
        motor_R(0);
        return
      }
    }
    //Update speeds
    motor_L(L_faster_LF ? v : 0);
    motor_R(R_faster_LF ? v : 0);
    delay(50);
  }
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
    // wait 10 seconds for connection:
    delay(10000);
  }
  
  //Start server
  server.begin();
}

void loop(){
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
  
  
  //Switch State
  switch(state){
    //State 0: Line following
    case 0:
      follow_line(1,1);  //keep right and stop on 1st instance sensor_s = 1
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
      
      if (bitRead(msg,5)){
        state = 2
      }
      
    break;
    
    
    //State 2: Picking up target
    case 2:
      //Pick up robot
      state = 3
    break;
}










