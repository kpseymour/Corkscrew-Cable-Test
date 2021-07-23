#include <Encoder.h>
#include <PID_v1.h>
#include "DualG2HighPowerMotorShield.h"
DualG2HighPowerMotorShield18v18 md;

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

/* Encoder Pins
 * pins 2 and 3 have interrupt capability on Arduino UNO
 * Best Performance: both pins have interrupt capability 
 * Good Performance: only the first pin has interrupt capability
 * Low Performance:  neither pin has interrupt capability */
Encoder encoderTwister(2, 6);
Encoder encoderElevator(3, 7);

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
double KpT=5, KiT=.3, KdT=1;
double twisterPIDInput, twisterPIDOutput, twisterPIDSetpoint;

// Set up the PID variables for Elevator here:
double KpE=5, KiE=.3, KdE=1;
double elevatorPIDInput, elevatorPIDOutput, elevatorPIDSetpoint;

PID twisterPID(&twisterPIDInput, &twisterPIDOutput, &twisterPIDSetpoint, KpT, KiT, KdT, DIRECT);
PID elevatorPID(&elevatorPIDInput, &elevatorPIDOutput, &elevatorPIDSetpoint, KpE, KiE, KdE, DIRECT);


//-------------------------------------------------------------------------------------------------//


void setup() {
  Serial.begin(9600);
  Serial.println("Elevator Motors Encoder Test:");
  twisterPIDInput = 0;
  twisterPIDSetpoint = rotationTicks;
  
  elevatorPIDInput = 0;
  elevatorPIDSetpoint = inchTicks;
  
  twisterPID.SetMode(AUTOMATIC);
  elevatorPID.SetMode(AUTOMATIC);
}

long positionTwister  = -999;
long positionElevator = -999;

void loop() {
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
