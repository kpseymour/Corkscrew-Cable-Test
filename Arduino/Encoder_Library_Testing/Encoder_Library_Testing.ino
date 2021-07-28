#include <Encoder.h>
#include <PID_v1.h>
#include "DualG2HighPowerMotorShield.h"
DualG2HighPowerMotorShield18v18 md;


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
int lastBottonButtonState = LOW;
int lastTopButtonState = LOW;
unsigned long lastDebounceTime = 0; 
unsigned long debounceDelay = 50;  



// Motor Driver Protection code
void stopIfFault()
{
  if (md.getM1Fault())
  {
    md.disableDrivers();
  delay(1);
    Serial.println("M1 fault");
    while (1);
  }
  if (md.getM2Fault())
  {
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
Encoder encoderTwister(2, 6);
Encoder encoderElevator(3, 7);
long positionTwister  = -999;
long positionElevator = -999;

// set the number of encoder ticks translates to 1 rotation.
int rotationTicks = 200; 

// set the number of encoder ticks translates to 1 inch of elevator movement
int inchTicks = 75;



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
double KpE=1, KiE=0, KdE=.01;
double elevatorPIDInput, elevatorPIDOutput, elevatorPIDSetpoint;

PID twisterPID(&twisterPIDInput, &twisterPIDOutput, &twisterPIDSetpoint, KpT, KiT, KdT, DIRECT);
PID elevatorPID(&elevatorPIDInput, &elevatorPIDOutput, &elevatorPIDSetpoint, KpE, KiE, KdE, DIRECT);


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

  // set initial LED state
  digitalWrite(ledPin, ledState);
  
  Serial.begin(9600);
  Serial.println("Elevator Motors Encoder Test:");
  twisterPIDInput = 0;
  twisterPIDSetpoint = rotationTicks;
  
  elevatorPIDInput = 0;
  elevatorPIDSetpoint = inchTicks;
  
  twisterPID.SetMode(AUTOMATIC);
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

  // check to see if a stop switch has been hit
  int readingBottom = digitalRead(elevatorBottomPin);
  int readingTop = digitalRead(elevatorTopPin);
  if (readingBottom != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (readingBottom != buttonState) {
      buttonState = readingBottom;
      if (buttonState == HIGH) {
        ledState = !ledState;
      }
    }
  }
  // Change the LED
  digitalWrite(ledPin, ledState);
  lastButtonState = readingBottom;


//-------------------------------------------------------------------------------------------------//

  
  long newTwister, newElevator;
  newTwister = encoderTwister.read();
  newElevator = encoderElevator.read();
  
  // check if either of the encoders have changed
  if (newTwister != positionTwister || newElevator != positionElevator) {
    
    
    // run the PIDs to get the new motor power
    twisterPIDInput = newTwister;
    elevatorPIDInput = newElevator;
    twisterPID.SetOutputLimits(-100,100);
    elevatorPID.SetOutputLimits(-100,100);
    twisterPID.Compute();
    elevatorPID.Compute();
    
    //Print out the variables
    Serial.print("Twister = ");
    Serial.print(newTwister);
    Serial.print(", TwisterMotorPower = ");
    Serial.print(twisterPIDOutput);
    Serial.print(", Elevator = ");
    Serial.print(newElevator);
    Serial.print(", ElevatorMotorPower = ");
    Serial.print(elevatorPIDOutput);
    Serial.println();
    positionTwister = newTwister;
    positionElevator = newElevator;
  }
  // if a character is sent from the serial monitor, reset the encoder positions back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both encoders to zero");
    encoderTwister.write(0);
    encoderElevator.write(0);
  }
}
