#include <Encoder.h>
#include <PID_v1.h>
#include "DualG2HighPowerMotorShield.h"
DualG2HighPowerMotorShield24v18 md;

int state = 0;


int elevatorUpperSetpoint = 10000;
int elevatorLowerSetpoint = 0;





//-------------------------------------------------------------------------------------------------//
//   _____         ______ ______ _________     __
//  / ____|  /\   |  ____|  ____|__   __\ \   / /
// | (___   /  \  | |__  | |__     | |   \ \_/ / 
//  \___ \ / /\ \ |  __| |  __|    | |    \   /  
//  ____) / ____ \| |    | |____   | |     | |   
// |_____/_/    \_\_|    |______|  |_|     |_| 


const int elevatorBottomPin = 8;
const int elevatorTopPin = 12;
const int ledPin = 13;
int ledState = LOW;
int bottomButtonState, topButtonState;
int lastBottomButtonState = LOW;
int lastTopButtonState = LOW;
unsigned long lastDebounceTime = 0; 
unsigned long debounceDelay = 50;  
boolean turning = false;
unsigned long turnDelay;

// Motor Driver Protection code
void stopIfFault()
{
  if (md.getM1Fault()){
    md.disableDrivers();
  delay(1);
    Serial.println("M1 fault");
    while (1);
  }
  if (md.getM2Fault()){
    md.disableDrivers();
  delay(1);
    Serial.println("M2 fault");
    while (1);
  }
}

//-------------------------------------------------------------------------------------------------//
//  __  __  ____ _______ ____  _____   _____ 
// |  \/  |/ __ \__   __/ __ \|  __ \ / ____|
// | \  / | |  | | | | | |  | | |__) | (___  
// | |\/| | |  | | | | | |  | |  _  / \___ \ 
// | |  | | |__| | | | | |__| | | \ \ ____) |
// |_|  |_|\____/  |_|  \____/|_|  \_\_____/ 
/* Encoder Pins
 * pins 2 and 3 have interrupt capability on Arduino UNO
 * Best Performance: both pins have interrupt capability 
 * Good Performance: only the first pin has interrupt capability
 * Low Performance:  neither pin has interrupt capability */
Encoder encoderTwister(11, 13);
Encoder encoderElevator(3, 5);
long positionTwister  = -999;
long positionElevator = -999;

// set the number of encoder ticks translates to 1 rotation.
int rotationTicks = 200; 

// set the number of encoder ticks translates to 1 inch of elevator movement
int inchTicks = 4000;



//-------------------------------------------------------------------------------------------------//
//  _____ _____ _____  
// |  __ \_   _|  __ \ 
// | |__) || | | |  | |
// |  ___/ | | | |  | |
// | |    _| |_| |__| |
// |_|   |_____|_____/ 

// Set up the PID variables for Twister here:
double KpT=1, KiT=0, KdT=.01;
double twisterPIDInput, twisterPIDOutput, twisterPIDSetpoint;

// Set up the PID variables for Elevator here:
double KpE=15, KiE=0.5, KdE=0.2;
double elevatorPIDInput, elevatorPIDOutput, elevatorPIDSetpoint;

PID twisterPID(&twisterPIDInput, &twisterPIDOutput, &twisterPIDSetpoint, KpT, KiT, KdT, DIRECT);
PID elevatorPID(&elevatorPIDInput, &elevatorPIDOutput, &elevatorPIDSetpoint, KpE, KiE, KdE, DIRECT);


boolean elevatorState = true; // true is bottom going up, false is top going down

//-------------------------------------------------------------------------------------------------//
//   _____ ______ _______ _    _ _____  
//  / ____|  ____|__   __| |  | |  __ \ 
// | (___ | |__     | |  | |  | | |__) |
//  \___ \|  __|    | |  | |  | |  ___/ 
//  ____) | |____   | |  | |__| | |     
// |_____/|______|  |_|   \____/|_|    
 
void setup() {
  pinMode(elevatorBottomPin, INPUT);
  pinMode(ledPin, OUTPUT);

  // Initialize the Motor Driver
  md.init();
  md.calibrateCurrentOffsets();
  delay(10);
  md.flipM1(true);
  md.flipM2(true);

  // set initial LED state
  digitalWrite(ledPin, ledState);
  
  Serial.begin(9600);
  Serial.println("Elevator Motors Encoder Test:");
  twisterPIDInput = 0;
  twisterPIDSetpoint = rotationTicks;
  twisterPID.SetMode(AUTOMATIC);
  
  elevatorPIDInput = 0;
  elevatorPIDSetpoint = 0;
  elevatorPID.SetMode(AUTOMATIC);
}


//-------------------------------------------------------------------------------------------------//
//  _      ____   ____  _____  
// | |    / __ \ / __ \|  __ \ 
// | |   | |  | | |  | | |__) |
// | |   | |  | | |  | |  ___/ 
// | |___| |__| | |__| | |     
// |______\____/ \____/|_|  

void loop() {
  md.enableDrivers();
  delay(1);  // The drivers require a maximum of 1ms to elapse when brought out of sleep mode.
  
  if (state == 0){ //Inital power on
    
    //TODO insert initial button push to home the machine 

    // Button push to advance state
    state = 1;
  }
  if state == 1){ // machine has successfully homes

    //TODO do the 3 pre-twists on the coupler

    // Button push to advance to testing
    state = 2;
  }
  
  if (state == 2){// main motion loop
    
    // check to see if a stop switch has been hit
    int readingBottom = digitalRead(elevatorBottomPin);
    int readingTop = digitalRead(elevatorTopPin);
    if (readingBottom != lastBottomButtonState || readingTop != lastTopButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }
  
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (readingBottom != bottomButtonState) {
        bottomButtonState = readingBottom;
        if (bottomButtonState == HIGH) {
          ledState = !ledState;
        }
      }
      if (readingTop != topButtonState) {
        topButtonState = readingTop;
        if (topButtonState == HIGH){
          ledState = !ledState;
        }
      }
    }
    // Change the LED
    digitalWrite(ledPin, ledState);
    lastBottomButtonState = readingBottom;
    lastTopButtonState = readingTop;
  
  
  //-------------------------------------------------------------------------------------------------//
  
    
    long newTwister, newElevator;
    newTwister = encoderTwister.read();
    newElevator = encoderElevator.read();
    
    // check if either of the encoders have changed
    if (newTwister != positionTwister || newElevator != positionElevator) {
      //Elevator PID control
      elevatorPIDInput = newElevator;

      // check to see if the elevator should be raised
      if (elevatorState){
        
        if (elevatorPIDSetpoint <= elevatorUpperSetpoint - 500){ // ramp up to the upper setpoint to not overload the motor
          elevatorPIDSetpoint += 500;
        }
        
        // check if the elevator has reached the target location
        if (elevatorPIDSetpoint == elevatorUpperSetpoint && newElevator >= elevatorUpperSetpoint - 100 && newElevator <= elevatorUpperSetpoint + 100){ 
          elevatorTurning();
        }
      }

      // check to see if the elevator should be lowered
      if (!elevatorState){
        
        if (elevatorPIDSetpoint >= elevatorLowerSetpoint + 500){ // ramp up to the upper setpoint to not overload the motor
          elevatorPIDSetpoint -= 500;
        }
        
        // check if the elevator has reached the target location and turn around
        if (elevatorPIDSetpoint == elevatorLowerSetpoint && newElevator >= elevatorLowerSetpoint - 100 && newElevator <= elevatorLowerSetpoint + 100){ 
          elevatorTurning();
        }
      }
      Serial.print("ElTarget = ");
      Serial.print(elevatorPIDSetpoint);
      elevatorPID.SetOutputLimits(-40000,40000);
      elevatorPID.Compute();
      Serial.print(", ElPos = ");
      Serial.print(newElevator);
      Serial.print(", ElSpeed = ");
      Serial.print(elevatorPIDOutput);
      Serial.println();
      positionElevator = newElevator;
      
      /* Twister PID
      twisterPIDInput = newTwister;
      twisterPID.SetOutputLimits(-400,400);
      twisterPID.Compute();
      Serial.print(", TwisterPosition = ");
      Serial.print(newTwister);
      Serial.print(", TwisterMotorPower = ");
      Serial.print(twisterPIDOutput);
      Serial.println();
      positionTwister = newTwister;
*/
      
    }
    //Elevator is M1
      md.setM1Speed(elevatorPIDOutput/100);
      stopIfFault();

    
    // if a character is sent from the serial monitor, reset the encoder positions back to zero.
    if (Serial.available()) {
      Serial.read();
      Serial.println("Reset both encoders to zero");
      encoderTwister.write(0);
      encoderElevator.write(0);
    }
  }
}

void elevatorTurning(){
  md.setM1Speed(0);
          
  if (turning == false){ //start turning around without blocking the state machine
    turning = true;
    turnDelay = millis();
    Serial.println();
    Serial.print(" turning");
    Serial.println();
  }
          
  if (turning == true && millis()-turnDelay >= 2000){ //finish turning around and enter the next state
    elevatorState = !elevatorState;
    turning = false;
  }
}
