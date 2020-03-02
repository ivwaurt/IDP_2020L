/*
 Chat  Server

 A simple server that distributes any incoming messages to all
 connected clients.  To use telnet to  your device's IP address and type.
 You can see the client's input in the serial monitor as well.

 This example is written for a network using WPA encryption. For
 WEP or WPA, change the Wifi.begin() call accordingly.


 Circuit:
 * Board with NINA module (Arduino MKR WiFi 1010, MKR VIDOR 4000 and UNO WiFi Rev.2)

 created 18 Dec 2009
 by David A. Mellis
 modified 31 May 2012
 by Tom Igoe

 */

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

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = "OnePlus 7 Pro";        // your network SSID (name)
char pass[] = "11123456";    // your network password (use for WPA, or use as key for WEP)
int speed_L_current=0;
int speed_R_current=0;
int speed_L=0;
int speed_R=0;
uint8_t v=128;
int keyIndex = 0;            // your network key Index number (needed only for WEP)
uint8_t msg;

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

int status = WL_IDLE_STATUS;

WiFiServer server(23);

boolean alreadyConnected = false; // whether or not the client was connected previously

void setup() {
  AFMS.begin();
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
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
}


void loop() {
  // wait for a new client:
  WiFiClient client = server.available();


  // when the client sends the first byte, say hello:
  if (client) {
    if (!alreadyConnected) {
      // clead out the input buffer:
      client.flush();
      Serial.println("We have a new client");
      client.println("Hello, client!");
      Serial.println(bitRead(15,0));
      alreadyConnected = true;
    }

    if (client.available() > 0) {
      // read the bytes incoming from the client:
      uint8_t buf; //Byte
      size_t siz = 1;
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
      Serial.println("----");
      server.write("1234HELLO");
      
      /*
      motorL->run( (speed_L>=0) ? FORWARD : BACKWARD );
      motorR->run( (speed_R>=0) ? FORWARD : BACKWARD );

      motorL->setSpeed(40);
      motorR->setSpeed(40);
      */
      motor_L(speed_L);
      motor_R(speed_R);
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
