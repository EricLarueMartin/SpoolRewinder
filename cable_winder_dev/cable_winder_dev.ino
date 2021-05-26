/*
    Program to run Cable Winder
    Developed by Kevin Bhimani and Esteban Leon with help from Eric Martin.
    Will move winder a number of turns based on input to run command.
    Will move the guide back and fourth at a speed relative to the winder based on the wind pitch setting.
    Guide direction reverses when target upper and lower limits are reached.
    When reversing there are two hysteresis settings.
    First the winder will keep moving and the guide will stop for winderHysteresis steps,
    then the winder will stop and the guide will move for guideHysteresis steps.
    Values passed to and returned from methods are in millimeteres, revolutions, and seconds for easier user interface.
    Values are stored in the class variables in steps and seconds for easier position control.
*/

// Include the AccelStepper library:
#include <AccelStepper.h>
// Define the AccelStepper interface type:
#define MotorInterfaceType 4

// set constants
const int MaxWinderSpeed = 1000;
const int MaxGuideSpeed = 1000;
const int StepsPerRev = 200;
const float LeadScrewPitch = 2.0; // movement of traveling nut per revolution of lead screw, in mm
const float SpoolWidth = 14.8; // width of spool in mm
const long SpoolStepWidth = SpoolWidth * StepsPerRev / LeadScrewPitch; // width of spool in steps on guide stepper motor
const float GuidePositionHysteresis = 2.0; // default distance to move just the guide when reversing directions in mm
const float WinderTurnHysteresis = 0.5; // default revolutions to move just the winder when reversing directions
const float InitialWindPitch = 0.4; // movement of guide per revolution of winder, in mm
const float InitialSpeed = 0.3; // initial winer speed in revolutions per second

// note that these constants are also a bitmap of which steppers are moving, 0x1 is guide and 0x2 is winder
const int MODE_STOPPED = 0; // neither stepper running
const int MODE_GUIDE = 1; // guide motor is running
const int MODE_WINDER = 2; // winder motor running
const int MODE_WINDING = 3; // both motors running to wind cable
const int MODE_REVERSING_GUIDE = 5; // move the guide but not the winder to change directions
const int MODE_REVERSING_WINDER = 6; // move the winder but not the guide to change directions

class cableWinder
{
    // private members, distances are in steps and speeds in steps per second
    AccelStepper *guideStepper, *winderStepper;

    // user adjusted settings and defaults
    float guideToWinderStepRatio = 0.5; // windPitch/LeadScrewPitch;
    float winderSpeed = InitialSpeed * StepsPerRev; // in steps/second
    float guideSpeed = winderSpeed * guideToWinderStepRatio;
    int outputsEnabled = MODE_STOPPED;
    long guideLowerLimit = 0;
    long guideHysteresis = GuidePositionHysteresis * StepsPerRev / LeadScrewPitch; // in steps
    long winderHysteresis = WinderTurnHysteresis * StepsPerRev; // in steps
    long guideUpperLimit = SpoolStepWidth + guideHysteresis;
    long guideTargetPosition; // only used when repositioning guide
    long winderReversePosition; // only used when winding at reverse point 
    int runMode = MODE_STOPPED;
    int resumeMode = MODE_STOPPED; // set this mode when resuming from a pause
    // trying without resume mode by simply starting in reversing mode when resuming
    //    int resumeMode = MODE_STOPPED; // if set to MODE_REVERSING or MODE_WINGING then when winder resumes in this mode

    // public members, return values are in mm of movement on the guide, turns of the winder, and seconds
  public:

    cableWinder(int pins[8]); // constructor sets defaults
    void windCable(float distance); // passing 0.0f keeps current target position
    void moveWinder(float distance); // move the winder a distance in cm
    void moveGuide(float distance); // move the stepper a distance in cm
    void setGuidePosition(float distance) {
      guideStepper->setCurrentPosition(distance * StepsPerRev / LeadScrewPitch);
    }; // sets the current position of the guide stepper in cm
    float getGuidePosition() {
      return guideStepper->currentPosition() * LeadScrewPitch / StepsPerRev;
    }; // get current guide posiiton in mm
    void haltSteppers();
    void pauseSteppers() {
      resumeMode = runMode;
      runMode = MODE_STOPPED;
    };
    void calcSpeed() {
      guideSpeed = ((guideSpeed > 0.0f) ? 1 : -1) * abs(winderSpeed) * guideToWinderStepRatio;
    }; // recalcualte guide speed based on winder speed and ratio
    ; // recalculate guide speed
    void displaySettings();
    void onLoop(); // executes any required run calls for steppers, call this in loop as often as possible
    void setSpoolWidth(float width);
    void calcLimit() {
      guideUpperLimit = guideLowerLimit + guideHysteresis + SpoolStepWidth;
    }; // recalculates upper limit
    float getWinderSpeed() {
      return (winderSpeed / StepsPerRev);
    }; // speed in revolutions per second
    float getGuideSpeed() {
      return (guideSpeed / StepsPerRev);
    }; // speed in revolutions per second
    float setWinderSpeed(float speedInput) {
      winderSpeed = speedInput * StepsPerRev; calcSpeed();
    }; // set winder speed and recalculate guide speed
    void setWindPitch(float windPitch) {
      guideToWinderStepRatio = windPitch / LeadScrewPitch;
      calcSpeed();
    }; // calcualte guide to winder step ratio for wind pitch
    float getWindPitch() {
      return (guideToWinderStepRatio * LeadScrewPitch);
    };
    void setGuideHysteresis(float comp) {
      guideHysteresis = comp * StepsPerRev / LeadScrewPitch;
      calcLimit();
    };
    float getWinderHysteresis() {
      return winderHysteresis / StepsPerRev;
    };
    void setWinderHysteresis(float comp) {
      winderHysteresis = comp * StepsPerRev;
    };
    float getGuideHysteresis() {
      return guideHysteresis * LeadScrewPitch / StepsPerRev;
    };
    int getMode() {
      return runMode;
    };
    void setGuideDirection(int dir = 0) {
      if (dir == 0) guideSpeed = -guideSpeed;
      else if (dir > 0) guideSpeed = abs(guideSpeed);
      else guideSpeed = -abs(guideSpeed);
    }; // set the direction the guide is moving, positive is to right, negative is to left, 0 reverses
    void setResumeMode(int modeIn) {
      resumeMode = modeIn;
    };
};

/* pins for MotorInterfaceType = 4 (FULL4WIRE)
  0-3 - guide Arduino pins 4-7
  4-8 - winder Arduino pins 8-11
*/
cableWinder::cableWinder(int pins[8]) { // constructor sets defaults
  guideStepper = new AccelStepper(MotorInterfaceType, pins[0], pins[1], pins[2], pins[3]);
  winderStepper = new AccelStepper(MotorInterfaceType, pins[4], pins[5], pins[6], pins[7]);

  guideStepper->setMaxSpeed(MaxGuideSpeed);
  winderStepper->setMaxSpeed(MaxWinderSpeed);
  calcSpeed();
}

// checks if anything needs to be done, run as often as possible in main loop
void cableWinder::onLoop() {

  if (runMode & MODE_WINDER) { // if winder is moving for any reason check if it's done
    // check if target position reached
    if ((winderStepper->currentPosition() >= 0 && winderSpeed > 0.0f) // increasing past target position
        || (winderStepper->currentPosition() <= 0 && winderSpeed < 0.0f)) { // decreasing past target position
      Serial.println((runMode & MODE_WINDING) ? "Winding complete." : "Winder movement complete.");
      runMode = MODE_STOPPED;
    }
  }

  // check if guide direction needs to be reversed
  if ((runMode == MODE_WINDING) // only do this when in winding mode
      && ((guideStepper->currentPosition() <= guideLowerLimit && guideSpeed < 0.0f) // moving past lower limit
          || (guideStepper->currentPosition() >= guideUpperLimit && guideSpeed > 0.0f))) { // moving past upper limit
    // reverse direction
    guideSpeed = -guideSpeed;
    Serial.print(F("Setting guide speed to "));
    Serial.print(guideSpeed);
    Serial.println(F("Steps per second."));
    guideStepper->setSpeed(guideSpeed);
    runMode = MODE_REVERSING_WINDER;
    Serial.println(F("Reversing direction, turning winder without moving guide."));
    winderReversePosition=winderStepper->currentPosition(); // save current position to track reversal movement
  }

  // check if winder only movement is complete when reversiong
  if ((runMode == MODE_REVERSING_WINDER) && // check if winder movement hysteresis compensation is complete after reversing
      (abs(winderStepper->currentPosition()-winderReversePosition) >= winderHysteresis)) { // finished winder only movement
    runMode = MODE_REVERSING_GUIDE;
    Serial.println(F("Completed winder only reversal movement. Now performing guide only movement."));
  }


  
  // check if guide only movement is complete when reversing
  if ((runMode == MODE_REVERSING_GUIDE) && // check if guide movement hysteresis compensation is complete after reversing
      ((guideStepper->currentPosition() <= guideUpperLimit - guideHysteresis && guideSpeed < 0.0f) // finished hystersis from upper limit reverse
       || (guideStepper->currentPosition() >= guideLowerLimit + guideHysteresis && guideSpeed > 0.0f))) { // finished hystersis from lower limit reverse
    runMode = MODE_WINDING;
    Serial.println(F("Completed guide only reversal movement. Direction reversal complete."));
  }

  // check if only moving guide and that movement is complete
  if ((runMode == MODE_GUIDE) && // check if guide only movement complete
      ((guideStepper->currentPosition() <= guideTargetPosition && guideSpeed < 0.0f) // decreasing past goal
       || (guideStepper->currentPosition() >= guideTargetPosition && guideSpeed > 0.0f))) { // increasing past goal
    runMode = MODE_STOPPED;
    Serial.println(F("Completed guide positioning movement."));
  }

  // for steppers that should be moving, run them at speed
  if (runMode & MODE_GUIDE) 
    guideStepper->runSpeed();  
  if (runMode & MODE_WINDER) 
    winderStepper->runSpeed();
}

void cableWinder::windCable(float distance = 0.0f) {

  // if a distance was specified then set that as goal by changing winder current position
  // the winder will move until position 0
  if (distance != 0.0f) {
    winderStepper->setCurrentPosition(-distance * StepsPerRev);
    Serial.print(F("Setting winder to wind for "));
    Serial.print(distance);
    Serial.println(F(" turns."));
    // set the winder direction
    if (distance < 0.0f) winderSpeed = -abs(winderSpeed); // set winder to unwind
    else winderSpeed = abs(winderSpeed); // set winder to wind
    runMode = MODE_WINDING; // starting a fresh wind with both steppers
  } else {
    Serial.print(F("Resuming winding for "));
    Serial.print(-winderStepper->currentPosition()/StepsPerRev);
    Serial.println(F(" turns."));
    runMode = resumeMode;
  }
  // Enable outputs
  winderStepper->enableOutputs();
  guideStepper->enableOutputs();
  outputsEnabled = MODE_WINDING;

  // set speeds
  calcSpeed(); // make sure guide speed is right
/*  Serial.print(F("Setting winder speed to "));
  Serial.print(winderSpeed);
  Serial.println(F(" steps per second."));*/
  winderStepper->setSpeed(winderSpeed);
/*  Serial.print(F("Setting guide speed to "));
  Serial.print(guideSpeed);
  Serial.println(F(" steps per second."));*/
  guideStepper->setSpeed(guideSpeed);

}

// move the winder without moving the guide
void cableWinder::moveWinder(float distance) {
  // Enable outputs
  winderStepper->enableOutputs();
  outputsEnabled = MODE_WINDER;

  winderStepper->setCurrentPosition(-distance * StepsPerRev);
  if (distance < 0.0f) winderSpeed = -abs(winderSpeed); // set winder to reverse
  else winderSpeed = abs(winderSpeed); // set winder to advance
  Serial.print(F("Setting winder speed to "));
  Serial.print(winderSpeed);
  Serial.println(F("Steps per second."));
  winderStepper->setSpeed(winderSpeed);

  runMode = MODE_WINDER;
}

// move the guide without moving the winder
void cableWinder::moveGuide(float distance) {
  // Enable outputs
  guideStepper->enableOutputs();
  outputsEnabled = MODE_GUIDE;
  guideTargetPosition = guideStepper->currentPosition() + distance * StepsPerRev / LeadScrewPitch;
  if (distance < 0.0f) guideSpeed = -abs(guideSpeed); // set guide to move left
  else guideSpeed = abs(guideSpeed); // set winder to advance
  Serial.print(F("Setting guide speed to "));
  Serial.print(guideSpeed);
  Serial.println(F(" steps per second."));
  guideStepper->setSpeed(guideSpeed);

  runMode = MODE_GUIDE;
}

// calls pauseSteppers and also turns off drive current to steppers
void cableWinder::haltSteppers()
{
  pauseSteppers();
  winderStepper->disableOutputs();
  guideStepper->disableOutputs();
  outputsEnabled = MODE_STOPPED;
}

// send current settings to serial port
void cableWinder::displaySettings()
{
  Serial.println(F("Speeds in steps per second."));
  Serial.print(F("winderSpeed = "));
  Serial.println(winderSpeed);
  Serial.print(F("guideSpeed = "));
  Serial.println(guideSpeed);
  Serial.print(F("windPitch in mm/revolution = "));
  Serial.println(getWindPitch());
  Serial.print(F("guideHysteresis in steps = "));
  Serial.println(guideHysteresis);
  Serial.print(F("Current run mode is :"));
  switch (runMode) {
    case MODE_STOPPED   : Serial.println(F(" stopped.")); break;
    case MODE_WINDER    : Serial.println(F(" moving winder only.")); break;
    case MODE_GUIDE     : Serial.println(F(" moving guide only.")); break;
    case MODE_WINDING   : Serial.println(F(" winding cable.")); break;
    case MODE_REVERSING_GUIDE : Serial.println(F(" reversing with guide moving only.")); break;
    case MODE_REVERSING_WINDER : Serial.println(F(" reversing winder moving only.")); break;
  }
  Serial.print(F("Current guide posision is "));
  Serial.print(getGuidePosition());
  Serial.println(F(" mm."));
  Serial.print(F("Guide lower limit: "));
  Serial.print(guideLowerLimit);
  Serial.println(F(" mm."));
  Serial.print(F("Guide upper limit: "));
  Serial.print(guideUpperLimit * LeadScrewPitch / StepsPerRev);
  Serial.println(F(" mm."));

}

int WinderPins[8] = {4, 5, 6, 7, 8, 9, 10, 11}; // pins for cable winder

cableWinder *winder;


void displayMenu() {

  if (Serial)
  {
    Serial.println(F("Welcome to the cable winder!"));
    Serial.println(F("R          : Run the winder for (input:float) turns, 0 or blank to continue previous wind."));
    Serial.println(F("M          : Move the winder for (input:float) revolutions in (input:int) direction."));
    Serial.println(F("G          : Move the guide for (input:float) mm."));
    Serial.println(F("S          : Change the winder speed to (input:float) revoutions/second."));
    Serial.println(F("P          : Change the wind pitch (input:float) mm per revolution."));
    Serial.println(F("H          : Halt the winder."));
    Serial.println(F("C          : Set the guide movement hysteresis compensation distance to (input:float) mm."));
    Serial.println(F("W          : Set the reversal winder only movement to (input:float) turns."));
    Serial.println(F("O          : Set the current guide position to (input:float) in mm from left most limit."));
    Serial.println(F("D          : Set the current guide movement direction to (input:int), positive for moving right, negative for moving left, zero to reverse."));
    Serial.println(F("?          : Display current settings."));
  }

}


void setup() {

  // start serial port at 9600 bps
  Serial.begin(9600);

  delay(1500);
  displayMenu();
  winder = new cableWinder(WinderPins);
  winder->calcSpeed();
  winder->calcLimit();
  winder->displaySettings();

}

// check incomming serial for commands
void parseSerial() {

  while (Serial.available()) {
    char arg = Serial.read();
    switch (toUpperCase(arg)) {

      case 'M': // move the winder
        winder->moveWinder(Serial.parseFloat());
        break;

      case 'G': // move the guide
        winder->moveGuide(Serial.parseFloat());
        break;

      case 'R':  // wind cable
        winder->windCable(Serial.parseFloat());
        break;

      case 'S' : // set winder speed
        winder->setWinderSpeed(Serial.parseFloat());
        Serial.print(F("Winder speed set to "));
        Serial.print(winder->getWinderSpeed());
        Serial.println(F(" revolutions/s."));
        winder->displaySettings();
        break;

      case 'P' :  // set pitch
        winder->setWindPitch(Serial.parseFloat());
        Serial.print(F("Wind pitch set to "));
        Serial.print(winder->getWindPitch());
        Serial.println(F(" mm/revolution."));
        winder->displaySettings();
        break;

      case 'C' : // set hysteresis compensation
        winder->setGuideHysteresis(Serial.parseFloat());
        Serial.print(F("Guide hysteresis compensation set to "));
        Serial.print(winder->getGuideHysteresis());
        Serial.print(F(" mm."));
        winder->displaySettings();
        break;

      case 'O' : // set current guide position
        winder->setGuidePosition(Serial.parseFloat());
        Serial.print(F("Guide position set to "));
        Serial.print(winder->getGuidePosition());
        Serial.print(F(" mm."));
        winder->displaySettings();
        break;

      case 'D' : // change guide direction
        winder->setGuideDirection(Serial.parseInt());
        Serial.print(F("Guide speed set to "));
        Serial.print(winder->getGuideSpeed());
        Serial.print(F(" revolustions/second."));
        winder->displaySettings();
        break;

      case 'W' : // change guide direction
        winder->setWinderHysteresis(Serial.parseFloat());
        Serial.print(F("Winder reversal windings to "));
        Serial.print(winder->getWinderHysteresis());
        Serial.print(F(" turns."));
        winder->displaySettings();
        break;

      case 'H' : // halt
        Serial.println(F("Turning off steppers."));
        winder->haltSteppers();
        break;

      case '?' : // display settings
        displayMenu();
        winder->displaySettings();
        break;
    }
  }
}

void loop() {
  parseSerial();
  winder->onLoop();

}
