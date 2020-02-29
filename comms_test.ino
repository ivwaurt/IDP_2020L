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

void setup() {
  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);

  // wait 10 seconds for connection:
  delay(10000);

  // start the web server on port 80
  server.begin();
}


void loop() {
  // compare the previous status to the current status
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
      client.flush();
      client.println("123 testing 345hello");
      alreadyConnected = true;
    }

    if (client.available() > 0) {
      uint8_t buf; //Byte
      client.read(&buf , size_t 1);
      
      //0th bit = move/not move, 1st bit = reverse
      speed_L = (((buf) & (1<<0)) ? v : 0) * (((buf) & (1<<1)) ? -1 : 1)
      
      //2nd bit = move/not move, 3rd bit = reverse
      speed_R = (((buf) & (1<<2)) ? v : 0) * (((buf) & (1<<3)) ? -1 : 1)
      
      server.write("1234HELLO");
      
      
    }

    
  }
}
