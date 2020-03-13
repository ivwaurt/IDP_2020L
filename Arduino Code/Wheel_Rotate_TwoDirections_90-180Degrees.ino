#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);
  double R = 0.05; // Radius of wheel
  int m_rpm = 36; // Speed of motor (in rpm)
  double wid = 0.22; // Width of AGV (between middle of wheels)
  double t = 2*15*wid*1000/(2*R*m_rpm);
  int i=128;
  //int t=1000
void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  Serial.print(t);// time required to turn 90 degree

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(i);
  myOtherMotor->setSpeed(i);
  myMotor->run(FORWARD);
  myOtherMotor->run(FORWARD);
   // turn on motor
  myMotor->run(RELEASE);
  myOtherMotor->run(RELEASE);
}

void loop() {
  Serial.print("tick");
  
  //rotate 90 degrees
  myMotor->run(FORWARD);
  myOtherMotor->run(BACKWARD);
  myMotor->setSpeed(i);
  myOtherMotor->setSpeed(i);
  delay(t);
  Serial.print("tech");
  myMotor->run(RELEASE);
  myOtherMotor->run(RELEASE);
  delay(1000);
  Serial.print("tock");
  myMotor->run(BACKWARD);
  myOtherMotor->run(FORWARD);
  myMotor->setSpeed(i);
  myOtherMotor->setSpeed(i);
  delay(t);
  Serial.print("tech");
  myMotor->run(RELEASE);
  myOtherMotor->run(RELEASE);
  delay(1000);

  //rotate 180 degrees
  myMotor->run(FORWARD);
  myOtherMotor->run(BACKWARD);
  myMotor->setSpeed(i);
  myOtherMotor->setSpeed(i);
  delay(2*t);
  Serial.print("tech");
  myMotor->run(RELEASE);
  myOtherMotor->run(RELEASE);
  delay(1000);
  Serial.print("tock");
  myMotor->run(BACKWARD);
  myOtherMotor->run(FORWARD);
  myMotor->setSpeed(i);
  myOtherMotor->setSpeed(i);
  delay(2*t);
  Serial.print("tech");
  myMotor->run(RELEASE);
  myOtherMotor->run(RELEASE);
  delay(1000);

  
  //change direction of rotation
  myMotor->run(BACKWARD);
  myOtherMotor->run(FORWARD);
  myMotor->setSpeed(i);
  myOtherMotor->setSpeed(i);
  delay(t);
  Serial.print("tech");
  myMotor->run(RELEASE);
  myOtherMotor->run(RELEASE);
  delay(1000);
  Serial.print("tock");
  myMotor->run(FORWARD);
  myOtherMotor->run(BACKWARD);
  myMotor->setSpeed(i);
  myOtherMotor->setSpeed(i);
  delay(t);
  Serial.print("tech");
  myMotor->run(RELEASE);
  myOtherMotor->run(RELEASE);
  delay(1000);
  myMotor->run(BACKWARD);
  myOtherMotor->run(FORWARD);
  myMotor->setSpeed(i);
  myOtherMotor->setSpeed(i);
  delay(2*t);
  Serial.print("tech");
  myMotor->run(RELEASE);
  myOtherMotor->run(RELEASE);
  delay(1000);
  Serial.print("tock");
  myMotor->run(FORWARD);
  myOtherMotor->run(BACKWARD);
  myMotor->setSpeed(i);
  myOtherMotor->setSpeed(i);
  delay(2*t);
  Serial.print("tech");
  myMotor->run(RELEASE);
  myOtherMotor->run(RELEASE);
  delay(1000);
}
