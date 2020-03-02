#include <SPI.h>
#include <WiFiNINA.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

//Motor init
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motorL = AFMS.getMotor(1);
Adafruit_DCMotor *motorR = AFMS.getMotor(2);
#define LM1 3       // left motor
#define RM1 4       // right motor

char ssid[] = "OnePlus 7 Pro";        // your network SSID (name)
char pass[] = "11123456";    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;
WiFiServer server(23);
boolean alreadyConnected = false; // whether or not the client was connected previously

int speed_L_current=0;
int speed_R_current=0;
int speed_L=0;
int speed_R=0;
uint8_t v=128;
int keyIndex = 0;            // your network key Index number (needed only for WEP)
uint8_t msg;
uint8_t buf; //Byte
size_t siz = 1;

//Variables
bool sensor_l=0;      // 1 = white, 0 = black
bool sensor_r=0;
bool sensor_s=0;      // side sensors (3rd one, at the right of AGV)
bool sensor_s_prev; 
bool speed_L_lf;       // 1 = faster, 0 = slower
bool speed_R_lf;

//Parameters
int tol = 700;          //Boundary between black/white
uint8_t v = 128;        //Motor speed during line following  
double ang2t = 20;      //time taken to rotate one degree(20.37)
double dis2t = 100;     //time taken to move one cm

//Update motor speed functions
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

void follow_line(bool keepRight, int count){
  while 1{
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
        speed_L = 0;
        speed_R = 0;
        return;
      }
    }
    //Update speeds
    motor_L(speed_L ? v : 0);
    motor_R(speed_R ? v : 0);
    delay(50);
  }
}
void reach_robot(){
  while 1{
    if (client.available() > 0) {
      // read the bytes incoming from the client:
      client.read(&buf , siz);
      // echo the bytes back to the client:
      // echo the bytes to the server as well:
      msg = buf+1-1;
      //0th bit = move/not move, 1st bit = reverse
      speed_L = (bitRead(msg,3) ? v : 0) * (bitRead(msg,2) ? -1 : 1);
      
      //2nd bit = move/not move, 3rd bit = reverse
      speed_R = (bitRead(msg,1) ? v : 0) * (bitRead(msg,0)? -1 : 1);

      Serial.println(speed_L);
      Serial.println(speed_R);
      server.write("1234HELLO");
      
      /*
      motorL->run( (speed_L>=0) ? FORWARD : BACKWARD );
      motorR->run( (speed_R>=0) ? FORWARD : BACKWARD );

      motorL->setSpeed(40);
      motorR->setSpeed(40);
      */
      motor_L(speed_L);
      motor_R(speed_R);
      if (buf="000000000"){
        return;
      }
    }
    
  }
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void setup()
{
  Serial.begin(9600);
  pinMode(A0, INPUT); //left sensor as input
  pinMode(A1, INPUT); //right sensor as input
  pinMode(A2, INPUT); //Side sensor as input
  AFMS.begin();
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // start the server:
  server.begin();
  // you're connected now, so print out the status:
  printWifiStatus();
  follow_line(1,1);
  forward(5);
  delay(1000);
  reach_robot();
}
