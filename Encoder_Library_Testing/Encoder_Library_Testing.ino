#include <Encoder.h>
#include <PID_v1.h>

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
double KpT=1, KiT=.1, KdT=.5;
double twisterPIDInput, twisterPIDOutput, twisterPIDSetpoint;

// Set up the PID variables for Elevator here:
double KpE=1, KiE=.1, KdE=.5;
double elevatorPIDInput, elevatorPIDOutput, elevatorPIDSetpoint;

PID twisterPID(&twisterPIDInput, &twisterPIDOutput, &twisterPIDSetpoint, KpT, KiT, KdT, DIRECT);
PID elevatorPID(&elevatorPIDInput, &elevatorPIDOutput, &elevatorPIDSetpoint, KpE, KiE, KdE, DIRECT);


//-------------------------------------------------------------------------------------------------//


void setup() {
  Serial.begin(9600);
  Serial.println("Elevator Motors Encoder Test:");
  
}

long positionTwister  = -999;
long positionElevator = -999;

void loop() {
  long newTwister, newElevator;
  newTwister = encoderTwister.read();
  newElevator = encoderElevator.read();
  if (newTwister != positionTwister || newElevator != positionElevator) {
    Serial.print("Twister = ");
    Serial.print(newTwister);
    Serial.print(", Elevator = ");
    Serial.print(newElevator);
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
