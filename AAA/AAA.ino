
/*
 AAA MACHINE FOR WHITE CLOUD MANUFACTORING 
 Contraced through Baymar Solutions
 Written by Kyle Casteline 

*/

#include <AccelStepper.h>
AccelStepper Ystepper(1, 33, 32); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper Xstepper(1, 31, 30); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper Zstepper(1, 29, 28);
AccelStepper Rstepper(1, 11, 12);
const long interval = 1000; //int for how long to run after tripping sensor
const int XlimitSensorPin = 49; 
const int YlimitSensorPin = 50;
const int ZlimitSensorPin = 51;
const int CartSensorPin = 40;
const int SyrSensorPin = 36;
const int Valvepin = 44;
const int Homepin = 2;
const int Primepin = 3;
const int Startpin = 4;
const int Nextpin = 5;
const int Rowpin = 9;
const int Fullpin = 7;
const int Pausepin = 10;
const int screwpitch = 2; //the pitch of the screw in mm of travel per revolution
const int Steps_per_Rev = 400; // The Steps in one revolution; 200= full step; 400= half step; 800= 1/4step

//variables will change:

int col = 0; 
int row = 0;
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
int y = 0;

void setup() {

  //-----INPUTS-----

  pinMode(XlimitSensorPin, INPUT);
  pinMode(YlimitSensorPin, INPUT);
  pinMode(ZlimitSensorPin, INPUT);
  pinMode(CartSensorPin, INPUT);
  pinMode(SyrSensorPin, INPUT);
pinMode(Homepin, INPUT);
pinMode(Primepin, INPUT);
pinMode(Startpin, INPUT);
pinMode(Nextpin, INPUT);
pinMode(Rowpin, INPUT);
pinMode(Fullpin, INPUT);
pinMode(Pausepin, INPUT);

  //-----OUTPUTS-----

  pinMode(Valvepin, OUTPUT);
  Serial.begin(9600);
  Serial1.begin(9600);
  Xstepper.setMaxSpeed(1350);
  Xstepper.setAcceleration(15000);
  Xstepper.setSpeed(1350);
  Ystepper.setMaxSpeed(3300);
  Ystepper.setAcceleration(15000);
  Ystepper.setSpeed(3300);
  Zstepper.setMaxSpeed(5000);
  Zstepper.setAcceleration(15000); 
  Zstepper.setSpeed(5000);
  Rstepper.setMaxSpeed(300);
  Rstepper.setAcceleration(2000);
  Rstepper.setSpeed(50);

  Stepsmm = (Steps_per_Rev / 2);
}
//---------------------------------------------------------------------HOMING PROCESS---------
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


  

  //Step 4) Set the home position for steppers

  Rhomed = 1;
  Rstepper.setCurrentPosition(0);
  //----------------------------------- FOR Z AXIS --------------------------------

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

  //-------------------------Y AXIS HOMING ----------------------------------

  Serial.println("Begin Y Homing");

  //  Step 1) Run the stepper backwards until it trips the sensor.

  while (digitalRead(YlimitSensorPin) == 0) {
  	Ystepper.setSpeed(-1000);
  	Ystepper.runSpeed();}

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
  


  //-------------------------X AXIS HOMING ------------------------------------

  Serial.println("Begin X Homing");

  //  Step 1) Run the stepper backwards until it trips the sensor.

  while (digitalRead(XlimitSensorPin) == 0) {
    Xstepper.setSpeed(-1000);
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



//---------------------------------------------head homing--------------------------------
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
  
   //unwind head
  Rstepper.moveTo(620);
  while (Rstepper.distanceToGo() > 0) {Rstepper.run();}
  Rstepper.setCurrentPosition(0);
  
}
//------------------------------------------------------------------------------GLUING PROCESS------------
void gluingprocess() {

Rstepper.setMaxSpeed(450);
Rstepper.setCurrentPosition(0);

  CartsensorState = digitalRead(CartSensorPin);
  

  if (CartsensorState == HIGH) {
  //move Right
  Xstepper.moveTo(Xstepper.currentPosition() + 4260);
  while (Xstepper.distanceToGo() > 0) {Xstepper.run();}
  
  //move down
  Zstepper.moveTo(4400);
  while (Zstepper.distanceToGo() > 0) {Zstepper.run();}

  //move forward
  Ystepper.moveTo(Ystepper.currentPosition() - 462);
  while (Ystepper.distanceToGo() < 0) {Ystepper.run();}
  
  digitalWrite(Valvepin, HIGH);
  //open the valve then rotate the nozzle 
  if (Rstepper.currentPosition() ==0) {Rstepper.moveTo(1600);
  while (Rstepper.distanceToGo() > 0) {Rstepper.run();}
 
  }
  else {
  	Rstepper.moveTo(0);
  while (Rstepper.distanceToGo() < 0) {Rstepper.run();}
  	
  }
  digitalWrite(Valvepin, LOW); // turn off valve
  
  
  Ystepper.moveTo(Ystepper.currentPosition() + 462); //move back
  while (Ystepper.distanceToGo() > 0) {Ystepper.run();}
 //move up
 Zstepper.moveTo(2000);
  while (Zstepper.distanceToGo() < 0) {Zstepper.run();}
  
  
 while (digitalRead(Pausepin)==1) {} //if the pause/refil button is pressed, do nothing...
  }
else { //move Right
  Xstepper.moveTo(Xstepper.currentPosition() + 4260);
  while (Xstepper.distanceToGo() > 0) {Xstepper.run();}}
  
  CartsensorState = 0;

while (col < 7) {
	
	CartsensorState = digitalRead(CartSensorPin);
  
  if (CartsensorState == HIGH) {


  Xstepper.setMaxSpeed(60000); //move Right
  Xstepper.moveTo(Xstepper.currentPosition() + 4584);
  while (Xstepper.distanceToGo() > 0) {Xstepper.run();}
  
  //move down
  Zstepper.moveTo(4400);
  while (Zstepper.distanceToGo() > 0) {Zstepper.run();}

   //move forward
  Ystepper.moveTo(Ystepper.currentPosition() - 475);
  while (Ystepper.distanceToGo() < 0) {Ystepper.run();}
  
  digitalWrite(Valvepin, HIGH);
  //open the valve then rotate the nozzle
  if (Rstepper.currentPosition() ==0) {Rstepper.moveTo(1600);
  while (Rstepper.distanceToGo() > 0) {Rstepper.run();}
  
  }
  else {
  	Rstepper.moveTo(0);
  while (Rstepper.distanceToGo() < 0) {Rstepper.run();}
  	
  }
  digitalWrite(Valvepin, LOW); // turn off valve
  delay(100);
  
  Ystepper.moveTo(Ystepper.currentPosition() + 475); //move back
  while (Ystepper.distanceToGo() > 0) {Ystepper.run();}
  //move up
 Zstepper.moveTo(2000);
  while (Zstepper.distanceToGo() < 0) {Zstepper.run();}
 
 while (digitalRead(Pausepin)==1) {} //if the pause/refil button is pressed, do nothing...
  }
 
  else { //move Right
  Xstepper.moveTo(Xstepper.currentPosition() + 4584);
  while (Xstepper.distanceToGo() > 0) {Xstepper.run();}
  
  
  }


  
col = col + 1;
  CartsensorState = 0;
  inByte= 0;

}
   //move up
 Zstepper.moveTo(0);
  while (Zstepper.distanceToGo() < 0) {Zstepper.run();}
   //move back to 0
  Xstepper.moveTo(0);
  while (Xstepper.distanceToGo() < 0) {Xstepper.run();}
  
  col = 0;
}

void contgluingprocess() {
	y = 0;
	while (y<13) {
	   	gluingprocess();
		if (y<12) {nextprocess();}
	y = y + 1;
}
Ystepper.moveTo(0); 
    while (Ystepper.distanceToGo() < 0) {
      Ystepper.run();
}
}
  

 void startprocess() {

    //move to the first row in the x direction

     //sets the X max speed
    Xstepper.moveTo(0 ); //enter the distance to the fisrt Row
    while (Xstepper.distanceToGo() > 0) {
      Xstepper.run();
    }
    delay (1000);

    //move in the y to check if there is a cartridge

     // sets the y max speed
    Ystepper.moveTo( 27340 ); //enter distance were the laser is right above the first cartridge
    while (Ystepper.distanceToGo() > 0) {
      Ystepper.run();
    }
    delay (100);
  inByte=0;
  }
  

  void nextprocess() {

    

    //Make sure Z is all the way up

    Zstepper.moveTo(0);
    while (Zstepper.distanceToGo() > 0)   {
      Zstepper.run();
    }


 // move to the next row
Ystepper.moveTo( Ystepper.currentPosition() - 2135 ); 
    while (Ystepper.distanceToGo() < 0) {
      Ystepper.run();
    }
    delay (10);
 inByte=0; }
  
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
if (inByte == 'B') {contgluingprocess();}
if (inByte == 'S') {startprocess();}
if (inByte == 'N') {nextprocess();}
if (inByte == 'H') {homingprocess();}
if (inByte == 'P') {primingprocess();}
if (digitalRead(Startpin)==1) {startprocess();}
if (digitalRead(Homepin)==1) {homingprocess();}
if (digitalRead(Primepin)==1) {primingprocess();}
if (digitalRead(Nextpin)==1) {nextprocess();}
if (digitalRead(Rowpin)==1) {gluingprocess();}
if (digitalRead(Fullpin)==1) {contgluingprocess();}
  }
