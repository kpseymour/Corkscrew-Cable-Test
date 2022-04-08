#include <Encoder.h>
#include <PID_v1.h>
#include "DualG2HighPowerMotorShield.h"
DualG2HighPowerMotorShield24v18 md;


// The test varaibles - these variables will change for different tests to be run

int maxCycles = 40;
int testBottomPosition = 12; //inches from top of coupler
int testTopPosition = 18; //inches from top of coupler

double elevatorBottomSwitchLocation = 9; //inches from top of coupler - below the bottom test position
double elevatorTopSwitchLocation = 19.5; //inches from top of coupler - above the top test position

int preTwists = 3; //rotations
boolean isTwisterTestRotationClockwise = true;















//-------------------------------------------------------------------------------------------------//
// __      __     _____  _____          ____  _      ______  _____ 
// \ \    / /\   |  __ \|_   _|   /\   |  _ \| |    |  ____|/ ____|
//  \ \  / /  \  | |__) | | |    /  \  | |_) | |    | |__  | (___  
//   \ \/ / /\ \ |  _  /  | |   / /\ \ |  _ <| |    |  __|  \___ \ 
//    \  / ____ \| | \ \ _| |_ / ____ \| |_) | |____| |____ ____) |
//     \/_/    \_\_|  \_\_____/_/    \_\____/|______|______|_____/ 
                                                                 

int state, testState = 0;
int homeState, retryCount = 0;
int i,j,k,l,m,n = 0;
int currentCycles = 0;

// variables for the upper and lower test bounds, and limit switch locations
long lowerBoundTicks, lowerSwitchTicks, upperBoundTicks, upperSwitchTicks;
long twisterZeroPos, twisterMaxPos;

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

// logic booleans
boolean isRobotKnownOn = false;
boolean isElevatorTurning = false;
boolean isHomingRaising = false;
boolean isHomed, isRetry = false;
boolean isElevatorInHomeLocation = false;
boolean isTwisterInHomeLocation = false;
boolean isSettingSet = false;
boolean isElevatorRising = true; // true is bottom going up, false is top going down
boolean isTwisterCCW = true;
boolean isLowerLimitKnown, isUpperLimitKnown = false;
boolean isRobotReadyToTest = false;
boolean isFirstTestSegment = true;
boolean isSecondTestSegment = false;
boolean twisterTriggerGo, elevatorTriggerGo = true;
boolean isFirstSegmentElevatorComplete, isFirstSegmentTwistComplete, isSecondSegmentElevatorComplete, isSecondSegmentTwistComplete = false;

// variables for the estop checking sequence
long eStopCheckPos;
long newEstopCheckPos;
unsigned long timeOn;

// variables for turning the motors around
unsigned long turnTimer;
int turnDelay = 20;

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

const int BottomLimitSwitch = 25;// brown wire is bottom
const int TopLimitSwitch = 24; // brown wire with blue heatshrink is top
const int IoButtonDown = 28;// grey wire turning yellow is Top
const int IoButtonUp = 29;// white wire turning orange is Bottom

// encoder ticks to real values translation
const int speedRampIncrementTwister = 5; // lower this number to decrease acceleration of the Twister
const int ticksPerTwist = 5850;// 5850 encoder ticks for 1 coupler revolution
const int speedRampIncrementElevator = 3;// lower this number to decrease acceleration of the Elevator
const int ticksPerInch = 390; // 390 encoder ticks for 1 inch of elevator movement


//-------------------------------------------------------------------------------------------------//
//  _____ _____ _____  
// |  __ \_   _|  __ \ 
// | |__) || | | |  | |
// |  ___/ | | | |  | |
// | |    _| |_| |__| |
// |_|   |_____|_____/ 

// Set up the PID variables for Twister here:
double KpT=30, KiT=3, KdT=2;
double twisterPIDInput, twisterPIDOutput, twisterPIDSetpoint;

// Set up the PID variables for Elevator here:
double KpE=20, KiE=2, KdE=1.5;
double elevatorPIDInput, elevatorPIDOutput, elevatorPIDSetpoint;
int newTwisterPID, newElevatorPID;

// initialize the PID algorithm with the pointers and variables
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
  //md.flipM2(true);

  // set initial LED state
  digitalWrite(ledPin, ledState);
  
  Serial.begin(115200);
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
  // Protection code for the motors and leaving the robot on for more than 2 days
  stopIfFault();
  timeOn = millis();
  if (timeOn > 172800000){
    md.disableDrivers();
    delay(1);
    while(1);
  }

  // 1ms delay to stop the loop from running too quickly and unpredicatably
  delay(1);

  // read the limit switches every cycle to ensure that a trigger is not missed - optimally these would be on interrupt pins
  int lowerLimitSwitch = digitalRead(BottomLimitSwitch);
  int upperLimitSwitch = digitalRead(TopLimitSwitch);
  long currentElevatorPosition = -encoderElevator.read();
  long currentTwisterPosition = encoderTwister.read();


  //-------------------------------------------------------------------------------------------------//
  
  if (state == 0){
        
    // ***
    // This state checks that the motors are operational by very slightly spinning the twister motor
    // and then checking that it has moved from its starting location.
    //
    // After this motor check has passed then this state will be skipped.
    // ***
    
    if (!isRobotKnownOn){
      if(i == 0){
        md.enableDrivers();
        delay(1);  // The drivers require a maximum of 1ms to elapse when brought out of sleep mode.
        eStopCheckPos = encoderTwister.read(); 
        // eStopCheckPos = encoderElevator.read();
        i++;
      }
      else{
        if (i < 50){
          md.setM2Speed(15); //
          delay(10);
          newEstopCheckPos = encoderTwister.read();
          i++;
        }
        else{
          md.setM2Speed(0);
        }
      }
      
      // The State advances automatically if it passes the check
      if (i>=50){
        if (newEstopCheckPos%eStopCheckPos >= 10 || newEstopCheckPos%eStopCheckPos <= -10){
          Serial.println("Motors Check Pass");
          i = 0;
          state = 1;
          isRobotKnownOn = true;
          md.setM2Speed(0);
        }
        else{
          if (i==50){
            Serial.println("Motors Check Fail. Check the eStop and Power Supply. Press up button to re-run");
            i++;
          }
          
          // wait for a button press to restart the check
          int reading = digitalRead(IoButtonUp);
          if (reading == HIGH){
            i = 0;
          }
        }
      }
    } 
    else{
      state = 1;
    }
  }

  //-------------------------------------------------------------------------------------------------//

  
  if (state == 1){
    
    // ***
    // This state acts to allow the user to see (TODO input) the heights and test settings
    // The user can press the down button after the homing sequence to see (TODO edit) these variables
    //
    // After this screen has been seen this state will be skipped
    // ***
    if (!isSettingSet){
      if (i == 0){
      Serial.println("Welcome to the Corkscrew Cable Testing Machine."); 
      Serial.println("Below is the current configuration for this test:");
      Serial.println();
      Serial.print("Test Bounds: ");
      Serial.print(testBottomPosition);
      Serial.print(" inches to ");
      Serial.print(testTopPosition);
      Serial.println(" inches.");

      Serial.print("Switch Locations: Bottom Switch: ");
      Serial.print(elevatorBottomSwitchLocation);
      Serial.print(" inches | Top Switch: ");
      Serial.print(elevatorTopSwitchLocation);
      Serial.println(" inches.");

      Serial.print("Cable pre-twists to be done: ");
      Serial.println(preTwists);

      Serial.print("Cable Twist Direction: ");
      if (isTwisterTestRotationClockwise){
        Serial.println("Clockwise");
        Serial.println();
      }
      else{
        Serial.print("CounterClockwise");
        Serial.println();
      }
      
      Serial.print("Cycles to be run: ");
      Serial.println(maxCycles);
      
      Serial.println();
      Serial.println("Press Up Button to initialize Homing Sequence.");
      i++;
      }
    }

    // Button push to advance state
    int reading = digitalRead(IoButtonUp);
    if (reading == HIGH){
      i=0;
      isSettingSet = true;
      state = 2;
    }
  }
  
  //-------------------------------------------------------------------------------------------------//
  
  if (state == 2){

    // ***
    // This state acts to home the elevator by clicking the elevator against both the top and 
    // the bottom switches and noting their positions. Then by using their known starting heights the
    // zero position for the elevator is set. If the switches are ever hit during the loop operation
    // then the homing sequence will be run again automatically.
    //
    // After the homing is completed this state will be skipped
    // ***
    
    
    
    if (!isHomed){ 
      // check to see that the robot has not completed the homing sequence yet

      if (i==0){
        Serial.println("HOMING!");
        Serial.println();
        i = 1;
      }
      
      if (!isLowerLimitKnown){
        //go to the bottom limit switch and record the position
        md.setM1Speed(-30);
        if (lowerLimitSwitch == HIGH){
          md.setM1Speed(0);
          
          // Set the Lower Test Bounds
          isLowerLimitKnown = true;
          lowerSwitchTicks = currentElevatorPosition;
          delay(50);
          Serial.print("Lower Switch ");
          Serial.println(lowerSwitchTicks);
          lowerBoundTicks = (lowerSwitchTicks) + ((testBottomPosition - elevatorBottomSwitchLocation) * ticksPerInch);
          Serial.print("Lower Bound ");
          Serial.println(lowerBoundTicks);
        }
      }
      
      if (isLowerLimitKnown && !isUpperLimitKnown){
        // go to the top limit switch and record the position
        md.setM1Speed(45);
        if (upperLimitSwitch == HIGH){
          md.setM1Speed(0);
          delay(500);
          
          // Set the Upper Test bounds
          isUpperLimitKnown = true;
          upperSwitchTicks = currentElevatorPosition;
          Serial.print("Upper Switch ");
          Serial.println(upperSwitchTicks);
          upperBoundTicks = (upperSwitchTicks) - ((elevatorTopSwitchLocation - testTopPosition) * ticksPerInch);
          Serial.print("Upper Bound ");
          Serial.println(upperBoundTicks);

          if (!isRetry){
            // Set the Twister Bounds - skip this step on a retry
            positionTwister = encoderTwister.read();
            if (!isTwisterTestRotationClockwise){
            // clockwise testing is negative, ccw testing is positive
              twisterZeroPos = positionTwister + (preTwists * ticksPerTwist) - (200 * preTwists);
              twisterMaxPos = twisterZeroPos + ticksPerTwist;
              Serial.println();
              Serial.print("Twister Zero ");
              Serial.println(twisterZeroPos);
              Serial.print("Twister Max ");
              Serial.println(twisterMaxPos);
              Serial.println();
            }
            else {
              twisterZeroPos = positionTwister - (preTwists * ticksPerTwist) + (200 * preTwists);
              twisterMaxPos = twisterZeroPos - ticksPerTwist;
              Serial.println();
              Serial.print("Twister Zero ");
              Serial.println(twisterZeroPos);
              Serial.print("Twister Max ");
              Serial.println(twisterMaxPos);
              Serial.println();
            }
          } 
        }
      }
      if (isLowerLimitKnown && isUpperLimitKnown){
        // set the elevator to the top test position and complete the homing sequence
        if(currentElevatorPosition > upperBoundTicks){
          md.setM1Speed(-20);
        }
        else{
          md.setM1Speed(0);
          isHomed = true;
          delay(100);
        }
      }
    }
    else{
      if (i!=0){
        i=0;
        if (preTwists == 0){
          Serial.println("Press Up Button to begin testing");
        }
        else{
          if (!isRetry){
            Serial.print("Press Up Button to perform the ");
            Serial.print(preTwists);
            Serial.println(" Pre-Twists");
          }
          else{
            Serial.println("Retry Homing Complete. Beginning Testing.");
            Serial.println();
          }
        }
        
      }
      // wait for the button to be hit to advance the state
      // if there are no pretwists or if this is a limit switch triggered home then skip to testing
      int reading = digitalRead(IoButtonUp);
      if (reading == HIGH || isRetry){
        i = 0;
        if (preTwists == 0 || isRetry){
          isRetry = false;
          state = 4;
          isRobotReadyToTest = true;
        }
        else{
          state = 3;
        }
      }
    }
  }

  //---------------------------------------------------------------------------------------------------//
  
  if (state == 3){
    // ***
    // This state acts to perform the required number of pre-twists on the twisting mechanism
    //  It is only performed if there are twists to do, on a re-home this step is skipped
    // ***

    if (!isRobotReadyToTest){
      if (i==0){
        if (goToTwisterSetpoint(twisterZeroPos)){
          i++;
          delay(100);
          Serial.println("Press Up Button to begin testing");
        }
      }
      int reading = digitalRead(IoButtonUp);
      if (reading == HIGH && i != 0){
        isRobotReadyToTest = true;
      }
    }
    else{
      twisterTriggerGo = true;
      delay(500);
      i = 0;
      state = 4;
    }
  }
  
  //-------------------------------------------------------------------------------------------------//

  
  if (state == 4){
    // ***
    // This state acts to perform the main testing loop for the machine
    // The robot will move the elevator down while performing a twist on the cable
    // Then the robot will move the elvator up while undoing the twist on the cable
    // When both of the processes are complete that will be 1 cycle
    // 
    // If a limit switch is hit at any time the robot will re-home itself and continue testing
    // TODO: Add cableEye functionality so that the test will stop on a cable failure
    // ***
    
    if (lowerLimitSwitch == LOW && upperLimitSwitch == LOW){
      // make sure that neither of the limit switches have been hit during testing
      if (currentCycles < maxCycles){
        
        // first test segment is elevator lowering and twister twisting the cable up
        if (isFirstTestSegment == true){
          if(elevatorTriggerGo == true){
            if (goToElevatorSetpoint(lowerBoundTicks)){
              elevatorTriggerGo = false;
              isFirstSegmentElevatorComplete = true;
            }
          }
          if (twisterTriggerGo == true){
            if (goToTwisterSetpoint(twisterMaxPos)){
               twisterTriggerGo = false;
               isFirstSegmentTwistComplete = true;
            }
          }
          if (isFirstSegmentTwistComplete && isFirstSegmentElevatorComplete){
            delay(100);
            isFirstSegmentTwistComplete = false;
            isFirstSegmentElevatorComplete = false;
            isFirstTestSegment = false;
            isSecondTestSegment = true;
            twisterTriggerGo = true;
            elevatorTriggerGo = true;
          }
        }

        // second test segment is elevator rising and twister untwisting the cable
        if (isSecondTestSegment == true){
          if(elevatorTriggerGo == true){
            if (goToElevatorSetpoint(upperBoundTicks)){
              elevatorTriggerGo = false;
              isSecondSegmentElevatorComplete = true;
            }
          }
          if (twisterTriggerGo == true){
            if (goToTwisterSetpoint(twisterZeroPos)){
               twisterTriggerGo = false;
               isSecondSegmentTwistComplete = true;
            }
          }
          if (isSecondSegmentTwistComplete && isSecondSegmentElevatorComplete){
            delay(100);
            isSecondSegmentTwistComplete = false;
            isSecondSegmentElevatorComplete = false;
            isFirstTestSegment = true;
            isSecondTestSegment = false;
            twisterTriggerGo = true;
            elevatorTriggerGo = true;
            currentCycles++;
            Serial.print("Current Cycle: ");
            Serial.println(currentCycles);
          }
        }
      }
      else{
        md.disableDrivers();
        delay(1);
        Serial.println("TESTING COMPLETE!");
        while(1);
      }
    }
    else{
      // case if one of the limit swtiches has been hit
      // this means that the elevator has drifted outside of the safe testing area and will be homed again
      md.setM1Speed(0);
      md.setM2Speed(0);
      isFirstTestSegment = true;
      isSecondTestSegment = false;
      twisterTriggerGo = true;
      elevatorTriggerGo = true;
      if (lowerLimitSwitch == HIGH){
        Serial.print(retryCount);
        Serial.println(" Lower Limit Switch Hit ---------------------------------------------------------- ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR");
      }
      if (upperLimitSwitch == HIGH){
        Serial.print(retryCount);
        Serial.println(" Upper Limit Switch Hit ---------------------------------------------------------- ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR");
      }
      isRetry = true;
      isLowerLimitKnown = false;
      isUpperLimitKnown = false;
      isHomed = false;
      retryCount++;
      if (retryCount > 10){
        Serial.println("Maximum Retry Reached. Check machine bounds.");
        md.disableDrivers();
        while(1);
      }
      delay(1000);
      state = 2;
    }
  }
  //-------------------------------------------------------------------------------------------------//
}

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

boolean goToElevatorSetpoint(long elevatorSetpoint){
  // The PID control function for moving the elevator to a certain location
  // return false while moving to position
  // retrun true if reached position
  
  long currentElevatorPosition = -encoderElevator.read();
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
      elevatorSetpoint -= elevatorSetpoint%speedRampIncrementElevator; // stops the loop from failing to exit properly
      if (elevatorPIDSetpoint <= elevatorSetpoint - speedRampIncrementElevator){ // slowly accelerate to not overload the motor
        elevatorPIDSetpoint += speedRampIncrementElevator;
      }
      
      // check if the elevator has reached the target location
      if (elevatorPIDSetpoint == elevatorSetpoint && currentElevatorPosition >= elevatorSetpoint - 50 && currentElevatorPosition <= elevatorSetpoint + 50){ 
        md.setM1Speed(0);
        Serial.print("Elevator Up ");
        Serial.println(currentElevatorPosition);
        return true;
      }
    }

    // check to see if the elevator should be lowered
    if (!isElevatorRising){
      elevatorSetpoint -= elevatorSetpoint%speedRampIncrementElevator; // stops the loop from failing to exit properly
      if (elevatorPIDSetpoint >= elevatorSetpoint + speedRampIncrementElevator){ // slowly accelerate to not overload the motor
        elevatorPIDSetpoint -= speedRampIncrementElevator;
      }
      
      // check if the elevator has reached the target location and stop moving
      if (elevatorPIDSetpoint == elevatorSetpoint && currentElevatorPosition >= elevatorSetpoint - 50 && currentElevatorPosition <= elevatorSetpoint + 50){ 
        md.setM1Speed(0);
        Serial.print("Elevator Down ");
        Serial.println(currentElevatorPosition);
        return true;
      }
    }
    //Serial.print("ElTarget = ");
    //Serial.print(elevatorPIDSetpoint);
    elevatorPID.SetOutputLimits(-25000,25000);//set the max and min PID values that the algorithm can return (100x max motor speed of 400)      
    elevatorPID.Compute();
    //Serial.print(", ElPos = ");
    //Serial.print(currentElevatorPosition);
    //Serial.print(", ElSpeed = ");
    //Serial.print(elevatorPIDOutput);
    //Serial.println();
    
    //Elevator is M1
    md.setM1Speed(elevatorPIDOutput/100);
    stopIfFault();
    return false;
}
boolean goToTwisterSetpoint(long twisterSetpoint){
  long currentTwisterPosition = encoderTwister.read();
  if (twisterSetpoint >= currentTwisterPosition){
    isTwisterCCW = true;
  }
  else{
    isTwisterCCW = false;
  }
  if (isTwisterCCW){
    if (twisterSetpoint - currentTwisterPosition > (100*speedRampIncrementTwister) && j >= 200){
      md.setM2Speed(300);
    }
    else if (twisterSetpoint - currentTwisterPosition > 0 || j < 200){
      md.setM2Speed(120);
      j++;
    }
    else{
      md.setM2Speed(0);
      twisterPIDSetpoint = currentTwisterPosition + speedRampIncrementTwister;
      twisterPID.Compute();
      Serial.print("Twister CCW ");
      Serial.println(currentTwisterPosition);
      j = 0;
      return true;
    }
  }
  else{
    if (twisterSetpoint - currentTwisterPosition < (-100*speedRampIncrementTwister) && j >= 200){
      md.setM2Speed(-300);
    }
    else if (twisterSetpoint - currentTwisterPosition < 0 || j < 200){
      md.setM2Speed(-120);
      j++;
    }
    else{
      md.setM2Speed(0);
      twisterPIDSetpoint = currentTwisterPosition - speedRampIncrementTwister;
      twisterPID.Compute();
      Serial.print("Twister Clockwise ");
      Serial.println(currentTwisterPosition);
      j = 0;
      return true;
    }
  }
  return false;
}

boolean goToTwisterSetpoints(long twisterSetpoint){
  // The PID control function for moving the twister to a certain location
  // return false while moving to position
  // retrun true if reached position
  long currentTwisterPosition = encoderTwister.read();
  if (twisterSetpoint >= currentTwisterPosition){
    isTwisterCCW = true;
  }
  else{
    isTwisterCCW = false;
  }
  
  // check if either of the encoders have changed
    twisterPIDInput = currentTwisterPosition;

    // check to see if the twister should be raised
    if (isTwisterCCW){
      twisterSetpoint -= twisterSetpoint%speedRampIncrementTwister; // stops the loop from failing to exit properly
      if (twisterPIDSetpoint <= twisterSetpoint - speedRampIncrementTwister){ // slowly accelerate to not overload the motor
        twisterPIDSetpoint += speedRampIncrementTwister;
        Serial.println(twisterPIDSetpoint);
      }
      
      // check if the twister has reached the target location
      if (twisterPIDSetpoint == twisterSetpoint && currentTwisterPosition >= twisterSetpoint - 30 && currentTwisterPosition <= twisterSetpoint + 30){ 
        md.setM2Speed(0);
        Serial.print("Twister CCW ");
        Serial.println(currentTwisterPosition);
        return true;
      }
    }

    // check to see if the twister should be lowered
    if (!isTwisterCCW){
      twisterSetpoint -= twisterSetpoint%speedRampIncrementTwister; // stops the loop from failing to exit properly
      if (twisterPIDSetpoint >= twisterSetpoint + speedRampIncrementTwister){ // slowly accelerate to not overload the motor
        twisterPIDSetpoint -= speedRampIncrementTwister;
      }
      
      // check if the twister has reached the target location and stop moving
      if (twisterPIDSetpoint == twisterSetpoint && currentTwisterPosition >= twisterSetpoint - 30 && currentTwisterPosition <= twisterSetpoint + 30){ 
        md.setM2Speed(0);
        Serial.print("Twister Clockwise ");
        Serial.println(currentTwisterPosition);
        return true;
      }
    }
    //Serial.print("TwistTarget = ");
    //Serial.print(twisterPIDSetpoint);
    twisterPID.SetOutputLimits(-30000,30000);//set the max and min PID values that the algorithm can return (100x max motor speed of 400 is 40000)      
    twisterPID.Compute();
    //Serial.print(", TwistPos = ");
    //Serial.print(currentTwisterPosition);
    //Serial.print(", TwistSpeed = ");
    //Serial.print(twisterPIDOutput);
    //Serial.println();
    
    //Twister is M2
    int M2Speed = twisterPIDOutput/100;
    if (M2Speed > 400){
      M2Speed = 400;
    }
    if (M2Speed < -400){
      M2Speed = -400;
    }
    md.setM2Speed(M2Speed);
    stopIfFault();
    return false;
}
