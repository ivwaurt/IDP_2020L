#include <SPI.h>
#include <WiFiNINA.h>
#include <Wire.h>

//Wifi
char ssid[] = "OnePlus 7 Pro";  //SSID
char pass[] = "11123456";       //password
uint8_t msg=0;
uint8_t buf;       //Byte
size_t siz = 1;
int status = WL_IDLE_STATUS;
bool alreadyConnected;
int count=0;

WiFiServer server(23);

void setup(){
  Serial.begin(9600);
  server.begin();
}

void loop(){
  msg=0;
    //Wifi stuff
    if (status != WiFi.status()) {
    status = WiFi.status();
    }
      if (status != WL_CONNECTED){
      while (status != WL_CONNECTED) {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      status = WiFi.begin(ssid, pass);
      // wait 5 seconds for connection:
      delay(5000);
    }
    //server.begin();
    alreadyConnected = false; 
  }
  WiFiClient client = server.available();
  
  //Read message
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
  count++;
  server.write(1);
  Serial.println(msg);
  Serial.println(status);
  Serial.println(client);
  delay(2000);
}
