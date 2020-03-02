#include <SPI.h>
#include <WiFiNINA.h>
#include <Adafruit_MotorShield.h>

//Motor init
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motorL = AFMS.getMotor(1);
Adafruit_DCMotor *motorR = AFMS.getMotor(2);
#define LM1 3       // left motor
#define RM1 4       // right motor




//NetworkSSID and password
char ssid[] = "OnePlus 7 Pro";        
char pass[] = "11123456";    
int keyIndex = 0;            
int status = WL_IDLE_STATUS;

int speed_L;
int speed_R;
uint8_t v = 128;      //Motor speed during movement

//Update motor speed functions
void motor_L(int speed){
  if (speed != speed_L){
    motorL->run( (speed>=0) ? FORWARD : BACKWARD );
    motorL->setSpeed(speed);
    speed_L = speed;
  }
}    

void motor_R(int speed){
  if (speed != speed_R){
    motorR->run( (speed>=0) ? FORWARD : BACKWARD );
    motorR->setSpeed(speed);
    speed_R = speed;
  }
}     


//Wifi server
WiFiServer server(23);

boolean alreadyConnected = false; //whether or not the client was connected previously

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

void setup() {
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

      // start the server:
  server.begin();
  // you're connected now, so print out the status:
  printWifiStatus();
  }
}

void loop() {
  // compare the previous status to the current status
    WiFiClient client = server.available();
  if (status != WiFi.status()) {
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }
  


  if (client) {
    if (!alreadyConnected) {
      //New Client
      Serial.print("Connected");
      client.flush();
      client.println("123 testing 345hello");
      alreadyConnected = true;
    }

    if (client.available() > 0) {
      uint8_t buf; //Byte
      size_t siz=1;
      client.read(&buf , siz);
      Serial.print(buf);
      
      //0th bit = move/not move, 1st bit = reverse
      speed_L = (((buf) & (1<<0)) ? v : 0) * (((buf) & (1<<1)) ? -1 : 1);
      
      //2nd bit = move/not move, 3rd bit = reverse
      speed_R = (((buf) & (1<<2)) ? v : 0) * (((buf) & (1<<3)) ? -1 : 1);

      Serial.print(speed_L);
      Serial.println(speed_R);
      
      server.write("1234HELLO");
      
      
    }

    
  }
}
