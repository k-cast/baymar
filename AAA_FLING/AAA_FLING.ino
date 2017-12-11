
/*
 AAA MACHINE FOR WHITE CLOUD MANUFACTORING 
 Contraced through Baymar Solutions
 Written by Kyle Casteline 

*/

#include <AccelStepper.h>
AccelStepper Xstepper(1, 27, 26); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper Ystepper(1, 25, 24); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper Zstepper(1, 43, 42);
AccelStepper Rstepper(1, 11, 12);
const long interval = 1000; //int for how long to run after tripping sensor
const int XlimitSensorPin = 49; 
const int YlimitSensorPin = 50;
const int ZlimitSensorPin = 51;
const int CartSensorPin = 40;
const int SyrSensorPin = 36;
const int Valvepin = 44;
const int screwpitch = 2; //the pitch of the screw in mm of travel per revolution
const int Steps_per_Rev = 400; // The Steps in one revolution; 200= full step; 400= half step; 800= 1/4step

//variables will change:

int Xhomed = 0;
int Yhomed = 0;
int Zhomed = 0;
int Rhomed = 0;
int XLimitSensorState = 0;
int YLimitSensorState = 0;
int ZLimitSensorState = 0;
int CartsensorState = 0;
unsigned long previousMillis = 0; // will store last time sensor was tripped
unsigned long currentMillis = 0; //will store last time senor was tripped
int Stepsmm = 0;
int stepstotarget = 0; //will be used to store the math of the distance to get to the target
int filling = 1;
char inByte = '0';


void setup() {

  //-----INPUTS-----

  pinMode(XlimitSensorPin, INPUT);
  pinMode(YlimitSensorPin, INPUT);
  pinMode(ZlimitSensorPin, INPUT);
  pinMode(CartSensorPin, INPUT);
  pinMode(SyrSensorPin, INPUT);

  //-----OUTPUTS-----

  pinMode(Valvepin, OUTPUT);
  Serial.begin(9600);
  Serial1.begin(9600);
  Xstepper.setMaxSpeed(5000);
  Xstepper.setAcceleration(5000);
  Xstepper.setSpeed(50);
  Ystepper.setMaxSpeed(20000);
  Ystepper.setAcceleration(10000);
  Ystepper.setSpeed(1000);
  Zstepper.setMaxSpeed(10000);
  Zstepper.setAcceleration(5000);
  Zstepper.setSpeed(1000);
  Rstepper.setMaxSpeed(5000);
  Rstepper.setAcceleration(2000);
  Rstepper.setSpeed(50);

  Stepsmm = (Steps_per_Rev / 2);
}

void homingprocess() {
  //Set the Home Position
  //
  //  Step 1) Run the stepper backwards until it trips the sensor.
  //  Step 2) Run the stepper forwards about 1cm
  //  Step 3) Very slowly bring the stepper back until it trips the sensor
  //  Step 4) Set this position as the Home position

  ZLimitSensorState = digitalRead(ZlimitSensorPin);
  currentMillis = millis();
  Serial.println("wiggling the applicator");

  //wiggle applicator

  //Rstepper.setCurrentPosition(0);
  //Rstepper.moveTo(1600);
  //while (Rstepper.distanceToGo() > 0)   {
  //  Rstepper.run();
  //}
  //delay(10);
 // Rstepper.moveTo(0);
  //while (Rstepper.distanceToGo() < 0) {Rstepper.run();}
  //Rstepper.setCurrentPosition(0);

while (digitalRead(CartSensorPin) == 0) {
    Rstepper.setSpeed(-1000);
    Rstepper.runSpeed();

  }

  Serial.println("Moving Z down 1cm");

  //Step 2) run the stepper forwards about 1cm

  for (previousMillis = currentMillis = millis(); currentMillis - previousMillis <= 500;) {
    Rstepper.setSpeed(200);
    Rstepper.runSpeed();
    currentMillis = millis();
    Serial.println(digitalRead(CartSensorPin));
  }

  Serial.println("Running Z up until it trips sensor");
  while ( digitalRead(CartSensorPin) == 0 ) {
    Rstepper.setSpeed(-100);
    Rstepper.runSpeed();
    Serial.println(digitalRead(CartSensorPin));
  }
  Rstepper.setCurrentPosition(0);
  
  Rstepper.setMaxSpeed(500); //unwind head
  Rstepper.moveTo(350);
  while (Rstepper.distanceToGo() > 0) {Rstepper.run();}
  delay(500);

  //Step 4) Set the home position for steppers

  Rhomed = 1;
  Rstepper.setCurrentPosition(0);
  //----------------------------------- FOR Z AXIS --------------------------------------------------------

  //  Step 1) Run the stepper backwards until it trips the sensor.

  while (digitalRead(ZlimitSensorPin) == 0) {
    Zstepper.setSpeed(-1000);
    Zstepper.runSpeed();

  }

  Serial.println("Moving Z down 1cm");

  //Step 2) run the stepper forwards about 1cm

  for (previousMillis = currentMillis = millis(); currentMillis - previousMillis <= 500;) {
    Zstepper.setSpeed(200);
    Zstepper.runSpeed();
    currentMillis = millis();
    Serial.println(digitalRead(ZlimitSensorPin));
  }

  Serial.println("Running Z up until it trips sensor");
  while ( digitalRead(ZlimitSensorPin) == 0) {
    Zstepper.setSpeed(-100);
    Zstepper.runSpeed();
    Serial.println(digitalRead(ZlimitSensorPin));
  }

  //Step 4) Set the home position for steppers

  Zhomed = 1;
  Zstepper.setCurrentPosition(0);

  //-------------------------Y AXIS HOMING --------------------------------------------------------------------

  Serial.println("Begin Y Homing");

  //  Step 1) Run the stepper backwards until it trips the sensor.

  while (digitalRead(YlimitSensorPin) == 0) {
    Ystepper.setSpeed(-5000);
    Ystepper.runSpeed();
  }

  //Step 2) run the stepper forwards about 1cm

  for (previousMillis = currentMillis = millis(); currentMillis - previousMillis <= 500;) {
    Ystepper.setSpeed(200);
    Ystepper.runSpeed();
    currentMillis = millis();
    Serial.println(digitalRead(YlimitSensorPin));
  }

  while ( digitalRead(YlimitSensorPin) == 0) {
    Ystepper.setSpeed(-100);
    Ystepper.runSpeed();
    Serial.println(digitalRead(YlimitSensorPin));
  }

  //Step 4) Set the home position for steppers

  Yhomed = 1;
  Ystepper.setCurrentPosition(0);
  Serial.println("Y Homed");


  //-------------------------X AXIS HOMING --------------------------------------------------------------------

  Serial.println("Begin X Homing");

  //  Step 1) Run the stepper backwards until it trips the sensor.

  while (digitalRead(XlimitSensorPin) == 0) {
    Xstepper.setSpeed(-5000);
    Xstepper.runSpeed();
  }

  //Step 2) run the stepper forwards about 1cm


  for (previousMillis = currentMillis = millis(); currentMillis - previousMillis <= 500;) {
    Xstepper.setSpeed(200);
    Xstepper.runSpeed();
    currentMillis = millis();
    Serial.println(digitalRead(XlimitSensorPin));
  }

  while ( digitalRead(XlimitSensorPin) == 0) {
    Xstepper.setSpeed(-100);
    Xstepper.runSpeed();
    Serial.println(digitalRead(XlimitSensorPin));
  }

  //Step 4) Set the home position for steppers

  Xhomed = 1;
  Xstepper.setCurrentPosition(0);
  Serial.println("X Homed");

}

void gluingprocess() {

for (int x=0; x < 13; x++) {

  CartsensorState = digitalRead(CartSensorPin);

  if (CartsensorState = 1) { //change high/low depending on laser sensor
  
  
  Zstepper.setMaxSpeed(10000); //move down
  Zstepper.moveTo(2100);
  while (Zstepper.distanceToGo() > 0) {Zstepper.run();}

  Ystepper.setMaxSpeed(10000); //move forward
  Ystepper.moveTo(Ystepper.currentPosition() - 1390);
  while (Ystepper.distanceToGo() < 0) {Ystepper.run();}
  
  digitalWrite(Valvepin, HIGH);
  Rstepper.setMaxSpeed(1200); //open the valve then rotate the nozzle
  Rstepper.setCurrentPosition(0);
  Rstepper.moveTo(1600);
  while (Rstepper.distanceToGo() > 0)   {Rstepper.run();}
  digitalWrite(Valvepin, LOW); // turn off valve
  delay(100);
  Ystepper.moveTo(Ystepper.currentPosition() + 1390); //move back
  while (Ystepper.distanceToGo() < 0) {Ystepper.run();}
 Zstepper.setMaxSpeed(10000); //move up
 Zstepper.moveTo(0);
  while (Zstepper.distanceToGo() < 0) {Zstepper.run();}
  
  Rstepper.setMaxSpeed(10000); //unwind head
  Rstepper.moveTo(0);
  while (Rstepper.distanceToGo() < 0) {Rstepper.run();}
  
 
  }
  if (x<12) {
  Ystepper.setMaxSpeed(10000); //move to the next one
  Ystepper.moveTo(Ystepper.currentPosition() - 8020);
  while (Ystepper.distanceToGo() < 0) {Ystepper.run();}
  }
  
  CartsensorState = 0;
}
  Ystepper.setMaxSpeed(5000); //move to the next one
  Ystepper.moveTo(0);
  while (Ystepper.distanceToGo() < 0) {Ystepper.run();}  
  inByte=0;
}
  
  



  

  void startprocess() {

    //move to the first row in the x direction

    Xstepper.setMaxSpeed(5000); //sets the X max speed
    Xstepper.moveTo( 8850 ); //enter the distance to the fisrt Row
    while (Xstepper.distanceToGo() > 0) {
      Xstepper.run();
    }
    delay (1000);

    //move in the y to check if there is a cartridge

    Ystepper.setMaxSpeed(10000); // sets the y max speed
    Ystepper.moveTo( 109650 ); //enter distance were the laser is right above the first cartridge
    while (Ystepper.distanceToGo() > 0) {
      Ystepper.run();
    }
    delay (1000);
  inByte=0;
  }
  

  void nextprocess() {

    Zstepper.setMaxSpeed(1000);
    Xstepper.setMaxSpeed(1000);

    //Make sure Z is all the way up

    Zstepper.moveTo(0);
    while (Zstepper.distanceToGo() > 0)   {
      Zstepper.run();
    }

    //Move X to next position

    Xstepper.moveTo(Xstepper.currentPosition() + 9294 ); //enter distance to next row
    while (Xstepper.distanceToGo() > 0)   {
      Xstepper.run();
    }
Ystepper.setMaxSpeed(10000); // sets the y max speed
    Ystepper.moveTo( 109000 ); //enter distance were the laser is right above the first cartridge
    while (Ystepper.distanceToGo() > 0) {
      Ystepper.run();
    }
    delay (1000);
  }
  
  void primingprocess() {
  	digitalWrite(Valvepin, HIGH);
  	delay(500);
  	digitalWrite(Valvepin, LOW);
  }


  void loop() { 

  if (Zhomed == 0) {homingprocess();}  //at the end of homingprocess, it sets zhomed = 1

if (Serial.available()) 
    {
    inByte = Serial.read();
    Serial.write(inByte);
    }
if (inByte == 'A') {gluingprocess();} //temporary until I set up the pushbuttons 
if (inByte == 'S') {startprocess();}
if (inByte == 'N') {nextprocess();}
if (inByte == 'H') {homingprocess();}
if (inByte == 'P') {primingprocess();}

  }
