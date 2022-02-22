#include <Encoder.h>
#include <PID_v1.h>
#include "DualG2HighPowerMotorShield.h"
DualG2HighPowerMotorShield24v18 md;




int maxCycles = 10000;




long elevatorUpperSetpoint = 18000;
long elevatorLowerSetpoint = 0;

double elevatorBottomSwitchLocation = 9; //inches from top of coupler
double elevatorTopSwitchLocation = 19.5; //inches from top of coupler
int testBottomPosition = 12; //inches from top of coupler
int testTopPosition = 18; //inches from top of coupler
int preTwists = 3; //rotations

// robot control variables
int speedRampIncrement = 500; // lower this number to decrease acceleration of the motors
int motorTurnsPerTwist = 6000;// 6000 encoder ticks for 1 coupler revolution
int ticksPerInch = 365; // 365 encoder ticks for 1 inch of elevator movement



//-------------------------------------------------------------------------------------------------//
// __      __     _____  _____          ____  _      ______  _____ 
// \ \    / /\   |  __ \|_   _|   /\   |  _ \| |    |  ____|/ ____|
//  \ \  / /  \  | |__) | | |    /  \  | |_) | |    | |__  | (___  
//   \ \/ / /\ \ |  _  /  | |   / /\ \ |  _ <| |    |  __|  \___ \ 
//    \  / ____ \| | \ \ _| |_ / ____ \| |_) | |____| |____ ____) |
//     \/_/    \_\_|  \_\_____/_/    \_\____/|______|______|_____/ 
                                                                 

int state = 0;
int i = 0;
int currentCycles = 0;

const int BottomLimitSwitch = 25;// brown wire is bottom
const int TopLimitSwitch = 24; // brown wire with blue heatshrink is top
const int IoButtonDown = 28;// grey wire turning yellow is Top
const int IoButtonUp = 29;// white wire turning orange is Bottom

long lowerLimitTicks, upperLimitTicks;

// Troubleshooting LED pin
const int ledPin = 13;
int ledState = LOW;

// variabels for limit switches
int bottomSwitchState, topSwitchState;
int lastBottomLimitSwitchState = LOW;
int lastTopLimitSwitchState = LOW;

// variables for I/O buttons
int DownButtonState, UpButtonState;
int lastDownButtonState = LOW;
int lastUpButtonState = LOW;

unsigned long lastDebounceTime = 0; 
unsigned long debounceDelay = 50;  

boolean isElevatorTurning = false;
boolean isHomingRaising = false;
boolean isHoming = true;
boolean isElevatorInHomeLocation = false;
boolean isTwisterInHomeLocation = false;

// variables for the estop checking sequence
long eStopCheckPos;
long newEstopCheckPos;

// variables for turning the motors around
unsigned long turnTimer;
int turnDelay = 20;

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

const unsigned int MAX_MESSAGE_LENGTH = 12;

//-------------------------------------------------------------------------------------------------//
//  __  __  ____ _______ ____  _____   _____ 
// |  \/  |/ __ \__   __/ __ \|  __ \ / ____|
// | \  / | |  | | | | | |  | | |__) | (___  
// | |\/| | |  | | | | | |  | |  _  / \___ \ 
// | |  | | |__| | | | | |__| | | \ \ ____) |
// |_|  |_|\____/  |_|  \____/|_|  \_\_____/ 
/* Encoder Pins
 * pins 2, 3, 18, 19, 20, and 21 have interrupt capability on Arduino MEGA
 * Best Performance: both pins have interrupt capability 
 * Good Performance: only the first pin has interrupt capability
 * Low Performance:  neither pin has interrupt capability */
 
Encoder encoderTwister(18, 19);
Encoder encoderElevator(20, 21);
long positionTwister  = -999;
long positionElevator = -999;



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
int newTwisterPID, newElevatorPID;

// initialize the PID algorithm with the pointers and variables
PID twisterPID(&twisterPIDInput, &twisterPIDOutput, &twisterPIDSetpoint, KpT, KiT, KdT, DIRECT);
PID elevatorPID(&elevatorPIDInput, &elevatorPIDOutput, &elevatorPIDSetpoint, KpE, KiE, KdE, DIRECT);

boolean isElevatorRising = true; // true is bottom going up, false is top going down

//-------------------------------------------------------------------------------------------------//
//   _____ ______ _______ _    _ _____  
//  / ____|  ____|__   __| |  | |  __ \ 
// | (___ | |__     | |  | |  | | |__) |
//  \___ \|  __|    | |  | |  | |  ___/ 
//  ____) | |____   | |  | |__| | |     
// |_____/|______|  |_|   \____/|_|    
 
void setup() {
  pinMode(IoButtonDown, INPUT);
  pinMode(IoButtonUp, INPUT);
  pinMode(BottomLimitSwitch, INPUT);
  pinMode(TopLimitSwitch, INPUT);
  pinMode(ledPin, OUTPUT);

  // Initialize the Motor Driver
  md.init(); // this takes over Digital Pins 2,4,6,7,8,9,10,and 12 on the arduino, they cannot be used for anything else.
  md.calibrateCurrentOffsets();
  delay(10);
  
  // comment these out to flip motor direction
  md.flipM1(true);
  md.flipM2(true);

  // set initial LED state
  digitalWrite(ledPin, ledState);
  
  Serial.begin(9600);
  twisterPIDInput = 0;
  twisterPIDSetpoint = 0;
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

  if (state == 0){//User input cycles and check settings
    if (i == 0){
      Serial.println("Welcome to the Corkscrew Cable Testing Machine. Below is the current configuration for this test:");
      Serial.println();
      Serial.print("Test Bounds: ");
      Serial.print(testBottomPosition);
      Serial.print(" inches to ");
      Serial.print(testTopPosition);
      Serial.println(" inches.");

      Serial.print("Limit Switch Locations:   Bottom Switch: ");
      Serial.print(elevatorBottomSwitchLocation);
      Serial.print(" inches   Top Switch: ");
      Serial.print(elevatorTopSwitchLocation);
      Serial.println(" inches.");

      Serial.print("Cable pre-twists to be done: ");
      Serial.println(preTwists);
      Serial.println();
      
      Serial.print("Cycles to be run: ");
      Serial.println(maxCycles);
      
      Serial.println();
      Serial.println("Press Up Button to initialize Homing Sequence.");
      i++;
    }
    
    
    //TODO insert initial button push to advance to motor testing 

    // Button push to advance state
    int reading = digitalRead(IoButtonUp);
    if (reading == HIGH){
      i=0;
      state = 1;
    }
    
  }

  //-------------------------------------------------------------------------------------------------//

  
  if (state == 1){// check that the motors are operational.
    if(i == 0){
      md.enableDrivers();
      delay(1);  // The drivers require a maximum of 1ms to elapse when brought out of sleep mode.
      eStopCheckPos = encoderTwister.read(); //---------------------------------------UNCOMMENT TO MAKE IT TWISTER
      // eStopCheckPos = encoderElevator.read();
      i++;
    }
    else{
      if (i < 50){
        md.setM2Speed(15); //--------------------------------------------------------------MAKE M2 FOR TWISTER
        delay(10);
        newEstopCheckPos = encoderTwister.read(); //-------------------------------------MAKE TWISTER
        i++;
      }
      else{
        md.setM2Speed(0);
      }
    }
    
    // otherwise state advances automatically if it passes the check
    if (i>=50){
      if (newEstopCheckPos%eStopCheckPos >= 10 || newEstopCheckPos%eStopCheckPos <= -10){
        Serial.println("Motors Check Pass");
        i = 0;
        state = 2;
      }
      else{
        if (i==50){
          Serial.println("Motors Check Fail. Check the eStop and Power Supply. Press up button to re-run");
          i++;
        }
        
        // wait for a button press to restart the check
        int reading = digitalRead(IoButtonUp);
        if (reading ==HIGH){
          i = 0;
          state += 1;
        }
        int readingBack = digitalRead(IoButtonDown);
        if (reading == HIGH){
          i = 0;
          state -= 1;
        }
      }
    }
  }

  //-------------------------------------------------------------------------------------------------//

  
  if (state == 2){ // machine homing sequence
    //relies on isHoming being true and isHomingRising being false at the start of the state
    
    Serial.println("HOMING");
    
    //get the readings from the limit switches
    int lowerLimitSwitch = digitalRead(BottomLimitSwitch);
    int upperLimitSwitch = digitalRead(TopLimitSwitch);
    
    // check to see if the lower limit swith is hit
    if (lowerLimitSwitch == HIGH && isHoming){
      if (i==0){
        md.setM1Speed(0);
        i++;
      }
      lowerLimitTicks = encoderElevator.read();
      Serial.print("Lower Home Position: ");
      Serial.println(lowerLimitTicks);
      isHomingRaising = true;
    }
    if (upperLimitSwitch == HIGH && isHoming){
      if (isHomingRaising){//reject the trigger if it is triggered at the start of the homing sequence
       if (i==1){
        md.setM1Speed(0);
        i++;
       }
        upperLimitTicks = encoderElevator.read();
        Serial.print("Upper Home Position: ");
        Serial.println(upperLimitTicks);
        isHoming = false;
      }
      else{
        Serial.println("Rejected upper press");
      }
    }
    
    if (isHoming){ // default true
      if (isHomingRaising){ // default false
        md.setM1Speed(20);
        // if the machine is homing and is raising then slowly raise the elevator
      }
      else{
        md.setM1Speed(-20);
        // if the machine is not raising then lower the elevator slowly
      }
      
    }
    else{
      i=0;
      //set the points that the elevtaor needs to move to
      elevatorUpperSetpoint = upperLimitTicks - (ticksPerInch * (elevatorTopSwitchLocation - testTopPosition));
      elevatorLowerSetpoint = lowerLimitTicks + (ticksPerInch * (elevatorBottomSwitchLocation - testBottomPosition));
      
      // go to the lower setpoint then set to the next state
      if (isElevatorInHomeLocation == false && isTwisterInHomeLocation == false){
        isElevatorInHomeLocation = goToElevatorSetpoint(elevatorLowerSetpoint);
        isTwisterInHomeLocation = goToTwisterSetpoint(motorTurnsPerTwist * preTwists);
      }
      else{
        md.setM1Speed(0);
        
      }
      
    }
    // elevator homing sequence
    
    
    Serial.print (i);
    md.setM1Speed(0);
    md.disableDrivers();
    delay(20);


    
    while(1);

    // Button push to advance to testing
    state = 3;
  }

  //-------------------------------------------------------------------------------------------------//

  
  if (state == 3){ // machine 3 pre-twists on the cable
    md.setM1Speed(0);
    md.disableDrivers();
    delay(20);
    while(1);

    //TODO do the 3 pre-twists on the coupler

    // Button push to advance to testing
    state = 4;
  }

  //-------------------------------------------------------------------------------------------------//

  
  
  if (state == 4){// main motion loop
    
    newTwisterPID = encoderTwister.read();
    newElevatorPID = encoderElevator.read();
    
    // check if either of the encoders have changed
    if (newTwisterPID != positionTwister || newElevatorPID != positionElevator) {
      //Elevator PID control
      elevatorPIDInput = newElevatorPID;

      // check to see if the elevator should be raised
      if (isElevatorRising){
        elevatorUpperSetpoint -= elevatorUpperSetpoint%speedRampIncrement; // stops the loop from failing to exit properly
        if (elevatorPIDSetpoint <= elevatorUpperSetpoint - speedRampIncrement){ // slowly accelerate to not overload the motor
          elevatorPIDSetpoint += speedRampIncrement;
        }
        
        // check if the elevator has reached the target location
        if (elevatorPIDSetpoint == elevatorUpperSetpoint && newElevatorPID >= elevatorUpperSetpoint - 100 && newElevatorPID <= elevatorUpperSetpoint + 100){ 
          elevatorTurning();
        }
      }

      // check to see if the elevator should be lowered
      if (!isElevatorRising){
        elevatorLowerSetpoint -= elevatorLowerSetpoint%speedRampIncrement; // stops the loop from failing to exit properly
        if (elevatorPIDSetpoint >= elevatorLowerSetpoint + speedRampIncrement){ // slowly accelerate to not overload the motor
          elevatorPIDSetpoint -= speedRampIncrement;
        }
        
        // check if the elevator has reached the target location and turn around
        if (elevatorPIDSetpoint == elevatorLowerSetpoint && newElevatorPID >= elevatorLowerSetpoint - 100 && newElevatorPID <= elevatorLowerSetpoint + 100){ 
          elevatorTurning();
        }
      }
      Serial.print("ElTarget = ");
      Serial.print(elevatorPIDSetpoint);
      elevatorPID.SetOutputLimits(-40000,40000);//set the max and min PID values that the equation can return (100x max motor speed of 400)      
      elevatorPID.Compute();
      Serial.print(", ElPos = ");
      Serial.print(newElevatorPID);
      Serial.print(", ElSpeed = ");
      Serial.print(elevatorPIDOutput);
      Serial.println();
      positionElevator = newElevatorPID;
      
      /* Twister PID
      twisterPIDInput = newTwisterPID;
      twisterPID.SetOutputLimits(-400,400);
      twisterPID.Compute();
      Serial.print(", TwisterPosition = ");
      Serial.print(newTwisterPID);
      Serial.print(", TwisterMotorPower = ");
      Serial.print(twisterPIDOutput);
      Serial.println();
      positionTwister = newTwisterPID;
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
          
  if (isElevatorTurning == false){ //start turning around without blocking the state machine
    isElevatorTurning = true;
    turnTimer = millis();
    Serial.println("^_^");
    Serial.print(" turning");
    Serial.println();
  }
          
  if (isElevatorTurning == true && millis()-turnTimer >= turnDelay){ //finish turning around and enter the next state
    isElevatorRising = !isElevatorRising;
    isElevatorTurning = false;
  }
}

// function that is given the two setpoints to send the motors to, and does all of the math and sets the motor power to move the motor to that position.
// returns true when both motors have reached their setpoints


boolean goToElevatorSetpoint(int elevatorSetpoint){
  //The PID control function for moving the elevator to a certain location
  // return false while moving to position
  // retrun true if reached position
  
  int currentElevatorPosition = encoderElevator.read();
  if (elevatorSetpoint >= currentElevatorPosition){
    isElevatorRising = true;
  }
  else{
    isElevatorRising = false;
  }
  
  // check if either of the encoders have changed
    elevatorPIDInput = currentElevatorPosition;

    // check to see if the elevator should be raised
    if (isElevatorRising){
      elevatorSetpoint -= elevatorSetpoint%speedRampIncrement; // stops the loop from failing to exit properly
      if (elevatorPIDSetpoint <= elevatorSetpoint - speedRampIncrement){ // slowly accelerate to not overload the motor
        elevatorPIDSetpoint += speedRampIncrement;
      }
      
      // check if the elevator has reached the target location
      if (elevatorPIDSetpoint == elevatorSetpoint && currentElevatorPosition >= elevatorSetpoint - 100 && currentElevatorPosition <= elevatorSetpoint + 100){ 
        md.setM1Speed(0);
        Serial.println();
        Serial.println("Made it!");
        Serial.println();
        return true;
      }
    }

    // check to see if the elevator should be lowered
    if (!isElevatorRising){
      elevatorSetpoint -= elevatorSetpoint%speedRampIncrement; // stops the loop from failing to exit properly
      if (elevatorPIDSetpoint >= elevatorSetpoint + speedRampIncrement){ // slowly accelerate to not overload the motor
        elevatorPIDSetpoint -= speedRampIncrement;
      }
      
      // check if the elevator has reached the target location and stop moving
      if (elevatorPIDSetpoint == elevatorSetpoint && currentElevatorPosition >= elevatorSetpoint - 100 && currentElevatorPosition <= elevatorSetpoint + 100){ 
        md.setM1Speed(0);
        Serial.println();
        Serial.println("Made it!");
        Serial.println();
        return true;
      }
    }
    Serial.print("ElTarget = ");
    Serial.print(elevatorPIDSetpoint);
    elevatorPID.SetOutputLimits(-40000,40000);//set the max and min PID values that the algorithm can return (100x max motor speed of 400)      
    elevatorPID.Compute();
    Serial.print(", ElPos = ");
    Serial.print(newElevatorPID);
    Serial.print(", ElSpeed = ");
    Serial.print(elevatorPIDOutput);
    Serial.println();
    
    //Elevator is M1
    md.setM1Speed(elevatorPIDOutput/100);
    stopIfFault();
    return false;
}

boolean goToTwisterSetpoint (int twisterSetpoint){
//  //The PID control function for moving the twister to a certain location
//  // return false while moving to position
//  // retrun true if reached position
//  int currentTwisterPosition = encoderTwister.read();
//  if (twisterSetpoint >= currentTwisterPosition){
//    isTwisterRising = true;
//  }
//  else{
//    isTwisterRising = false;
//  }
//  
//  // check if either of the encoders have changed
//    twisterPIDInput = currentTwisterPosition;
//
//    // check to see if the twister should be raised
//    if (isTwisterRising){
//      twisterSetpoint -= twisterSetpoint%speedRampIncrement; // stops the loop from failing to exit properly
//      if (twisterPIDSetpoint <= twisterSetpoint - speedRampIncrement){ // slowly accelerate to not overload the motor
//        twisterPIDSetpoint += speedRampIncrement;
//      }
//      
//      // check if the twister has reached the target location
//      if (twisterPIDSetpoint == twisterSetpoint && currentTwisterPosition >= twisterSetpoint - 100 && currentTwisterPosition <= twisterSetpoint + 100){ 
//        md.setM2Speed(0);
//        Serial.println();
//        Serial.println("Twister made it!");
//        Serial.println();
//        return true;
//      }
//    }
//
//    // check to see if the twister should be lowered
//    if (!isTwisterRising){
//      twisterSetpoint -= twisterSetpoint%speedRampIncrement; // stops the loop from failing to exit properly
//      if (twisterPIDSetpoint >= twisterSetpoint + speedRampIncrement){ // slowly accelerate to not overload the motor
//        twisterPIDSetpoint -= speedRampIncrement;
//      }
//      
//      // check if the twister has reached the target location and stop moving
//      if (twisterPIDSetpoint == twisterSetpoint && currentTwisterPosition >= twisterSetpoint - 100 && currentTwisterPosition <= twisterSetpoint + 100){ 
//        md.setM2Speed(0);
//        Serial.println();
//        Serial.println("Twister made it!");
//        Serial.println();
//        return true;
//      }
//    }
//    Serial.print("TwistTarget = ");
//    Serial.print(twisterPIDSetpoint);
//    twisterPID.SetOutputLimits(-40000,40000);//set the max and min PID values that the algorithm can return (100x max motor speed of 400)      
//    twisterPID.Compute();
//    Serial.print(", TwistPos = ");
//    Serial.print(newTwisterPID);
//    Serial.print(", TwistSpeed = ");
//    Serial.print(twisterPIDOutput);
//    Serial.println();
//    
//    //Twister is M2
//    md.setM2Speed(twisterPIDOutput/100);
//    stopIfFault();
//    return false;
return true;
}
