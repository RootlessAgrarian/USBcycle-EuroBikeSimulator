////-------------------------------------------------------------------
// USBcycle
//
// Leonardo-based Physical Game Controller (game bike, game stepper etc):
// works with anything that has a rotating part you can attach a magnet to,
// and a fixed part that you can attach 3 reed switches to.
//
// WASD v3:
// uses a switch to select Crash Drive 2 (or similar driving game) vs 
// standard WASD walker.
//
// example setups:
// 1) portable stepper (instrumented) plus mouse or wiichuck
// 2) portable stepper plus chair (recumbent emulator), mouse or wiichuck (or both)
// 3) fully instrumented bike, use pedal and steering sensors only, plus mouse or wiichuck
// 
// you may have to tweak some of the parameters for your particular setup -- your top
// pedalling speed might be greater or less than the limit set here, the threshold of 
// effort where you switch from walk to run might need adjusting to feel natural.
//
// WORKING as of April 9 2017, tested with Unity game build.
// As of Oct 2017, wii/mouse no longer working with Unity game build.  Mysterious results from
// testing:  
//     American Truck Sim: yes, works fine
//     Beginner's Guide:   yes
//     Dear Esther:        yes
//     Dr Langeskov (etc): no  (a Unity game)
//     Everything:         no  (a Unity game)
//     Obduction:          yes
//     OffPeak:            no  (a Unity game)
//     Sunless Sea:        yes
// Reported to Unity Answers Oct 9 2017
//
// USBcycle reads 3 reed switches to monitor rotation and derive 
// speed and direction. see
// https://create.arduino.cc/projecthub/Tazling/usbcycle-ride-through-your-virtual-world-8ff961
// This version uses a single Leonardo as a keyboard/mouse for WASD game control.
// This version supports ONLY KEYBOARD/MOUSE functions: NO JOYSTICK EMULATION 
// For full Kbrd/Mouse/Joystick emulation see the EuroBike capable version:
// Leo1_Current.ino and Leo2_Current.ino
// For the simpler WASD version (Portal, Dear Esther) see Leo_WASD_v2.ino
//
// First release version (github) Oct 2017
//
// notes on first CrashDrive test Oct 20 2017
// CrashDrive control physics is quite different from WASD and requires a different algorithm.
// the significant difference is that CrashDrive control is really done by "tapping" or intermittent key pressing.
// if you hold down the Fwd key in CrashDrive, you accelerate.  to maintain a steady speed you have to press and
// release, press and release.  same with steering, you tap-tap rapidly, maybe a half second, quarter second
// of keypress then back off.  
// seems like the algorithm for steering should be something like:
// if (out of deadzone but < next thresh) then keypress left/right arrow .250 sec (every time you check it)
// if (past next thresh) then hold down left/right arrow until back in zone A again
// if neutral, do nothing.  or we might just make it hair-trigger, to make little twitchy steering moves easier.
// and for fwd/reverse --
// if starting to move, press fwd/rev key.  now Moving.
// if Moving, if lastRPM < thisRPM (we are pedalling faster) then keep holding down key.
// if lastRPM > thisRPM (we are slowing down) then let go of key
// this will take a while to work out, but it is quite different from wasd games where holding down the fwd
// key moves you at a *fixed* speed.  it is more like ETS2 actually except that the only 2 throttle settings
// are MASH and BACKOFF :-)  then we could still have an RPM threshold at which we hit TURBO.
// Oct 21 2017:  WASD v3 with crash drive switch (used to be inv mouse y)
//
////-------------------------------------------------------------------
//
// PORTS
//
// Leonardo pins are assigned as follows:
//
// SWITCHES AND LEDs
//
// -- Switches
//
// Notes on the 5 switches (the pin order is a little odd because my
// perma proto board is not wired as intelligently as it should be,
// my bad).  They are all cheap mini slide switches in the prototype version.
//
// wiiChuckSw :  enables wiichuck control using tilt to steer etc.
//               you can use a regular mouse as well of course;  just turn wiichuck off
// bikeSteerSw:  [future feature not implemented yet] map bike steering to mouse movements 
//                (but how to provide buttons etc?)
// hidKillSw:    turns off HID emulation so you can get your computer back
//               if the Leo goes nuts
// invMsySw:  invert mouse Y when using wiichuck -- for those who think airplane yokewise.  
//            many games offer this option in user config menus, so this switch may not be too 
//            useful, but I have kept it just in case...\
//            ** crashDriveSw has replaced invMsySw.  it selects CrashDrive2 mode **
// invSpinSw: if you have a portable stepper that can be used either standing or seated, often
//            it must be rotated 180 between those 2 positions -- which of course reverses the rotation
//            sense.  so this switch simply swaps Fwd and Reverse, so you can use your stepper
//            in either orientation w/o moving wires around.

const int wiiChuckSw = 4;        // sw 1 bottom row
const int bikeSteerSw = 5;       // sw 2 bottom row
const int hidKillSw = 11;
const int crashDriveSw = A3;   // sw 3 bottom row
const int invSpinSw = A2;  // sw 4  bottom row
const int brakePin = A5;

// (reed switches on bike, low when triggered)

const int pinReed0 = 14;        // RJ45 pin 2
const int pinReed1 = 15;        // RJ45 pin 4
const int pinReed2 = 12;        // RJ45 pin 6


// -- LEDs (lots of these)
// 
// lo, med, hi Spd LEDs light to show whether you are in single step, walk, or run mode
// invSpinLED lights when spin direction sense is inverted (stepper orientation change)
// reed LEDs 0 1 2, light each time a reed switch is triggered:  confirmation that your
//          rotation sensors are working
// stopLED lights when rotation has ceased, i.e. you've stopped pedalling long enough to
//          trigger a timeout
// revLED lights when you are pedalling backwards
// hidKillLED lights when hid kill sw is set
// wiiChuckLED lights when wiiChuck switch is set
// bikeSteerLED lights when bike steer switch is set
// invMsyLED lights when mouse Y invert switch is set
// this is now crashDriveLED

// these LEDs are wired to real Leo ports
const int invSpinLED = 6;
const int hiSpdLED = 7;
const int medSpdLED = 8;
const int loSpdLED = 9;
const int hidKillLED = 10;

// these 8 LEDs are attached to the expansion device:
// again the order of pins is a little odd because of wire lengths and poor planning.

const int crashDriveLED = 0;
const int bikeSteerLED = 7;
const int wiiChuckLED = 6;
const int reed2LED = 5;
const int reed1LED = 4;
const int reed0LED = 3;
const int stopLED = 2;
const int reverseLED = 1; 

// -- OTHER INPUTS

// other possible inputs are few:
//    i2c bike steering if selected (not yet implemented)
//    i2c wiichuck if selected
//    (maybe someday) bike front brake lever to use as a game control

// GENERAL FEATURES:
//       LEDs to indicate stop, run, walk  (2x red led) and direction 
//       indicate direction of rotation (led) 
//       wii chuck for mouse input
//       switch for reverse mouse y
//       (future) calibration phase (all red leds on for 10 sec:  pedal as hard as you can to set max spd)


//
////---------------------------- DEFINITIONS ---------------------------------------
//

// LIBRARIES
//
// we need Wire to talk to the nunchuck over i2c
// we need Keyboard to send HID keyboard events.
// we need Hirzel's nifty library to interact with the nunchuck -- it can read pitch and roll, 
//    handy for a separate steering axis or for a different style of mouse steering/looking.
// and we need the 8574 library to use the 8 port i2c i/o expander for this 1-Leo version
//
#include <Wire.h>
#include <Keyboard.h>
#include <Mouse.h>

// init chuck
#include "HirzelWiiChuck.h"
// define chuck object
WiiChuck chuck = WiiChuck();

// init port expander
#include "pcf8574.h"
// adjust address as needed if you have more than one of these
// 38 is default address for stock 8574 with all jumpers set to "-"
// see datasheet
PCF8574 PCF_38(0x38);  // add switches to lines  (used as input)

// ***
// *** definitions for the AS5601 Rotary Hall sensor for steering (an i2c device)
// ***
#include <Encoder.h>
Encoder AS5601enc(18, 19);
// these values are absolutes, i,e, -90 to 90 on the bars;  but we would like more sensitive steering
// steerfactor should be settable by a panel switch!
float steerFactor = .70;
//
int rotaryHallCtr = 1420;                     // this must be empirically determined.  in hindsight, the magnet
                                              // s/b mounted on a rotating jig so the user could set it to a target
                                              // value instead of having to hack the code when the geometry drifts.
int minRotaryHall = rotaryHallCtr - 400;
int maxRotaryHall = rotaryHallCtr + 400;
// centre value is 1483;  this seems to drift a bit, possibly because of jolting to the magnet arm
int rangeRotaryHall = maxRotaryHall - minRotaryHall;
int steerVal = 0;
int rawSteerVal = 0;
int last_distance = 0;

//
// NB: Wire, Encoder, Keyboard and Mouse are stock Ardu libs.  Hirzel is a contrib.
// and so is pfc8574
//

// pretty debug print function from early stages of project
#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(str)    \
   Serial.print(millis());     \ 
   Serial.print(": ");    \
   Serial.print(__FUNCTION__);     \
   Serial.print(':');      \
   Serial.print(__LINE__);     \
   Serial.print(' ');      \
   Serial.print(str); \
   Serial.print(" "); \
   Serial.print(crashDrive); \
   Serial.print(" "); \
   Serial.print(invertSpinDir); \
   Serial.print(" "); \
   Serial.print(avgRPM); \
   Serial.print(" "); \
   Serial.print(walkThresh); \
   Serial.print(" "); \
   Serial.print(runThresh); \
   Serial.print(" "); \
   Serial.println(potVal);
#else
#define DEBUG_PRINT(str)
#endif


//    -- ROTATION TELEMETRY
//
// in theory we only need 2 sensors but in practise the reed switch can bounce
// -- you can get 2 edges for one pass of the magnet.  so I disallow "self" as
// the previous edge...  so you can't use "re encounter self" to indicate a 
// change of direction.  therefore you need 3 switches.  switches are cheap.
//

  const int numSensors = 3;
  int sensorVal;
  int lastval[numSensors];
  long lastoff[numSensors];
  long laston[numSensors];
  int lastrpm[numSensors];
  int reeds[numSensors];
  int lastedge = 0;
  int directionMatrix[numSensors][numSensors];
  //
  const int maxRPM = 96;
  const int minRPM = 20;
  float rpmCap = 120;
  int spinDir;
  int lastdir;
  int timeout;
  bool rpmBuffer = 0;
  
  // boxcar averaging : size of buffer s/b considered carefully, as a large buffer 
  // favours acceleration and masks deceleration.  10 is definitly too soft, big hysteresis
  // 8 may even be too much.  but you need something to counteract the occasional spikes.
  // now that I'm including all sensors, not just pinReed0, 6 seems good enough smoothing.
  
  const int numReadings = 3;         // boxcar history size:  average the last three sensor values
  float readings[numSensors][numReadings];      // the readings from the analog input
  float aDeltas[numReadings];
  int readIndex = 0;                // the index of the current reading 
  float total = 0;                  // the running total
  float avgRPM = 0;                // the average
  float lastAvgRPM = 0;                // we may want to track trend rather than absolute rpm
  float lastSampleAvg = 0;          // last average sampled on 100 ms clock
  float aDelta = 0.0;                // 2nd order terms:  change in averaged rpm?
  float maxaDelta = 0.0;             // we need to calibrate eventually?
  float minaDelta = 1024.0;
  float dtotal = 0.0;                 // total deltas in buffer
  float avgdelta = 0.0;               // averaged change in average
  float deltaThresh = 2.0;           // threshold for change in average to be acknowledged?

//
//
//   -- WIICHUCK
//       parameters for reading the wiichuck:

  boolean chuckExists = 0;
  boolean useChuck = 0;               // user settable flag to request chuck input
  int chuckRange = 40;                // output range of X or Y movement
  int chuckThresh = chuckRange/10;    // resting threshold
  int chuckCentre = chuckRange/2;     // resting position value
  long lastChuck = 0;                 // when did we last read it?
  int chuckRoll;                      // for chuck tilt steering (or other tricks, like maybe sideways motion a d instead of w s)
  int lastRoll;                       // for detecting nonlinearities in chuck tilt
  
//
//   -- GAME CONTROL
//
  int keyInterval = 250;                // time in ms between keystroke process/send
  long lastKeySend = 0;                 // keep track of elap time between keysends
  int footPace = 0;                     // 3 speeds, 0 (slow), 1 (walk), and 2 (run)
  int lastPace = 0;                     // as usual it is transitions that matter
  long lastPaceTime = 0;
  int hysFudge = 2;                     // fudge factor (rpm) by which to take out jitter in
                                        // foot pace decision
  float speedAdjust = 0.0;              // factor by which to map rpm to walk vs run
                                        // or in fly mode, mouse wheel setting
                                        // this "gain" factor is set by a pot
  // 20 rpm is super slow.  30-50 is leisurely. 50-60rpm is a brisk pedal speed.  
  // 100 is very fast.  90 is about the most I can keep up for 30 sec or so.
  // so let's say 25 to 55 is walking, above 55 is running.  
  // and we can scale by the pot, if we install a pot.  but that implies a readout and... sigh...
  // we're out of pins.
  float maxRunSpeed = 100.0;             // I kinda doubt that anyone can pedal faster than this!
  float runThresh = 65;                  // hard wiring this not such a great idea.  ideally set max values
  float walkThresh = 45;                 // and multiply them by the gain pot factor (50 percent as the default).
  boolean slowWalking = 1;               // below a certain rpm, send out just one char per leading edge.
  boolean Stopped = 1;                   // we can stop doing much of anything when we are motionless.
  int motionTimeout = 2000;              // how long must elapse between edges to conclude that we are stopped?
  float potVal = 1.0;                    // this version doesn't have the pot.
  int runCt = 0;                         // number of run speed loops you need to change up 
  int walkCt = 0;                        // number of walk speed loops you need to change up
  uint8_t keyPressed = 0;
  int usbHIDsending = 1;
  boolean reedTrip = 0;          // in slow walking mode, tells us that reed 0 or 2 has tripped so we should take a step
  long stepStart = 0;            // in slow walking mode, this tells us when to release the slow step key
  int stepLength = 350;          // in slow walking mode, this tells us how long to hold the slow step key
  
//  boolean invertMouseY = 0;              // need this for intuitive game look-around control
  boolean crashDrive = 0;                // switch on "drive" rather than "walk" HID output logic
  boolean invertSpinDir = 0;             // you will need this for recumbent bike emulation w/a portable stepper
 
  boolean useBikeSteer = 0;              // by default this is off, but you'll want to use bike steering if you have it
  boolean bikeSteerAvail = 0;            // if useBikeSteer selected but not available, blinky at startup.
  
  // -----------------------------------------------------------------------------------------------------------------
  // CRASH DRIVE 2 (and similar kbrd-controlled driving games) flags & parms
  // this version can use real bike brake lever & steering
  // Crash Drive does not use mouse at all!
  // ctrl keys are up/down arrow = fwd/rev, left/right arrow = steer, 
  // space bar = brake, shift/ctrl = turbo drive, Return = horn/fire
                                          
  boolean turnLeft = 0;          // steering state     
  boolean turnRight = 0;         // steering state
  boolean turnBack = 0;          // steering state
  boolean brakesOn = 0;          // brake state 
  boolean inForward = 0;         // motion state
  boolean inReverse = 0;         // motion state
  float driveThresh = .10;       // aDelta must be larger than this for the difference to count
  boolean turboDrive = 0;        // flag whether turbo drive (shift) is engaged
  float turboThresh = 60;        // rpm threshhold at which to engage turbo drive
  
  // -----------------------------------------------------------------------------------------------------------------
                                       
  int brakeVal = 0;
  int lastBrake = 0;
  int brakeNoise = 20;
  const int numBrakeReads = 10;          // start here:  with 10 samples we can smooth to 6 cts of
  int brakeReads[numBrakeReads];         // noise between averages (pathetic)  what a lousy cheap sensor
  int brakeReadPtr = 0;
  int brakeTot = 0;
  int brakeAvg = 0;
  //                                     // even after smoothing it's ridiculously noisy, as you see
  //                                     // (20 cts of jitter!)
  int brakeMin = 560;                     // these values empirically determined
  int brakeMax = 880;
  // (future feature placeholder)
  // we need to calibrate before running, to get valid maxRPM etc.
  // how to prompt user and do calibration?

  int calibrated = 0;
  long calibrateBegin = 0;
  int calTime = 10000;


// finally!  done with defs


//
////
//////------------------------- SETUP -----------------------------------------
////
//

                                  
void setup() {


  // INIT Serial logger
  //
  Serial.begin(9600);
  delay(100);  
  Serial.println(F("INITIALISING..."));
  // 
  // INIT SWITCHES AND LIGHTS
  //
  Serial.println(F("Initialise Arduino pins"));
  // sensor inputs
  pinMode(pinReed0,INPUT_PULLUP);
  pinMode(pinReed1,INPUT_PULLUP);
  pinMode(pinReed2,INPUT_PULLUP);
  reeds[0] = pinReed0;
  reeds[1] = pinReed1;
  reeds[2] = pinReed2; 

  // switch inputs
  pinMode(hidKillSw, INPUT_PULLUP);
  pinMode(wiiChuckSw, INPUT_PULLUP);
  pinMode(bikeSteerSw, INPUT_PULLUP);
  pinMode(crashDriveSw, INPUT_PULLUP);
  pinMode(invSpinSw, INPUT_PULLUP);
  
  // native indicator lights
  pinMode(invSpinLED,OUTPUT);
  pinMode(hiSpdLED,OUTPUT);
  pinMode(medSpdLED,OUTPUT);
  pinMode(loSpdLED,OUTPUT);
  pinMode(hidKillLED,OUTPUT);

  // *** blink a happy light:  phase 1 OK
  blinky(loSpdLED,6);

  //
  // INIT I/O, various? 
  //
  
  Serial.println(F("Initialise USB HID libs"));

  // Mouse emu, Keyboard emu, and wiichuck if present
  
  Mouse.begin();
  delay(100);
  Keyboard.begin();
  delay(100);

  Serial.println(F("Check I2C bus power"));

  // join i2c bus as master... but only if we see pullup voltage on pins 2 and 3
  int d2 = digitalRead(2);
  int d3 = digitalRead(3);
  if (!(d2&d3)) {
    Serial.println(F("OUCH, no I2C bus power."));
    goCatatonic(hidKillLED);       // fatal, no i2c bus power blink red disaster light forever
  }
  // otherwise... I2C is OK yay!
  Serial.println(F("I2C bus power is OK..."));
  
  // *** blink a happy light:  phase 2 OK
  blinky(medSpdLED,6);
    
  // init i2c bus
  Wire.begin();
  delay(500);

  // first, do we have our port expander?  if we don't, we are dead in the water
  Serial.println(F("Do we have I2C port expander?"));
  bool i2cOK = testI2C(0x38);
  if (!i2cOK) {goCatatonic(loSpdLED);}   //  fatal:  blink annoying white LED forever

  //
  // since it exists, let's talk to the port expander:  set all lights off
  //
  
  uint8_t value = PCF_38.read8();
  Serial.print("  #38:\t");
  Serial.println(value);
  value = 255;
  PCF_38.write8(255);

  // *** blink a happy light to show we are still OK, phase 3 done
  blinky(hiSpdLED,6);
  
  // do we have a wiichuck?  if we don't, we can still operate with a standard mouse;
  // we just have to ignore all chuck logic.
  Serial.println(F("Do we have wii chuck?"));
  i2cOK = testI2C(0x52);
  if (!i2cOK) {
    chuckExists = 0; 
    Serial.println(F("No WiiChuck Available At This Time."));
  } else {
    chuckExists = 1;
  }

  // do we have a bike rotation encoder, as in the full blown EBS version?  if we do, 
  // we should have the option to use it!
// now start up the i2c steering sensor

  Serial.println(F("Do we have rotary Hall effect (steering) sensor?"));
  
  i2cOK = testI2C(0x36);
  if (!i2cOK) {
    Serial.println(F("No Bike Steering Available At This Time."));
    bikeSteerAvail = 0;
  } else {
    bikeSteerAvail = 1;
  }
  
  if (chuckExists) {
    Serial.println(F("Initialise wii chuck since we have one"));
    // init wiichuck on i2c bus (this uses Wire)
    // this would do the Wire(begin) for us if we hadn't already done it
    chuck.begin();
    chuck.update();
    delay(100); 
  }

  if (bikeSteerAvail) {
        Serial.println(F("Initialise rotary steering encoder since we have one"));
        initRotaryHall();
  }

  // INIT direction matrix and history variables
  initTelemetry();
  
  delay(500); 

  // *** woo hoo, we survived init!  big happy light show:
  // riffle through all expander LEDs to signal successful init
  //
  for (int j = 0; j < 4; j++) {
  for (int i=0; i<8; i++)
  {
    // Serial.println(i);
    PCF_38.write(i,0);
    delay(100);
    PCF_38.write(i,1);
  }
  }
  
  // last hurrah
  blinky(invSpinLED,6);
  
  // DONE WITH INIT 
  Serial.println(F("---------- INIT COMPLETE -----------"));
  
  
}

//
////
//////------------------------- MAIN LOOP  -----------------------------------------
////
//

void loop() {
  

  // if hidKill switch is thrown, gimme my keyboard back.
  // this is important in case wiichuck goes nuts or code is bad:
  // having a demonically possessed kbrd and/or mouse makes it hard to recover.
 
  areWeSending();

  if (!crashDrive) {
// first order of business:  if we have a single step outstanding, then cancel
// it if its elapsed time has run out;  if we have not yet taken it, start it.
// but only if in wasd mode, not crashdrive mode!
  if (reedTrip) {
    checkSlowStep();
  }
  }

  // check user switches
  checkUserSwitches();
  
  // read nunchuck and update mouse position as needed
  // lights will blink (above) if you asked for chuck and it wasn't there
  if (chuckExists && useChuck) {
   // Serial.println(F("Check Chuck"));
   checkChuck();
  }
  // the end result of readRotaryHall func call is to set steerVal and rawSteerVal
  // the end result of steerMouseX is to use rawSteerVal to set mouse movement
  // if bikeSteer is selected and not available you'll get a blinky on the select led
  if (bikeSteerAvail && useBikeSteer) {
    readRotaryHall();
    steerMouseX();
  }

  // read the bike reed switches 0, 1, 2
  // Serial.print(F("read sensors ... "));

  for (int i = 0; i < numSensors; i++ ) {
    // Serial.print(i);
    // Serial.print(F(" ... "));
    int val = readSensor(i);
    if (val >= 0) {
      // only write to LEDs when value changes -- value is -1 for no change.
      if (val) { 
        laston[i] = millis();
        PCF_38.write(3+i, 0) ;
      } else {
        lastoff[i] = millis();
        PCF_38.write(3+i, 1) ;
      }
    }
 
  }
  // Serial.println(F(""));
  
  // Calibration place holder
  // future feature, needs Yet Another Switch (sigh)
  // are we calibrated?
  // gives user chance to set RunThresh to their own ability level
  // during calibration we only read sensors and update rpm, skip the rest of the loop
  
  /*
   if (calibrated < 2) {
    bool calib = checkCalibration();
    if (!calib) return;
  }
  */
  
  // Calibration is over... now DO THE MATH
  
  // readSensor has read all 3 sensors, computed direction, computed rpm on sensor 0
  // and determined last on/off.  now we decide what to do with the numbers.

  // -------------------------------------------------------------------------
  // oh dear, the gain pot was actually useful at one time...
  // how shall we set our threshholds?  for now, set reasonable fixed values
  //     potVal = analogRead(gainPot) / 1023.0;
  // we express the pot reading as a percentage...
  // the normal position was 50 percent or centred, so we'll just adjust the run/walk thresholds
  // hardwired in the defs.  experience with the EuroBike version suggests 50-60 is a comfy
  // pace to keep up, above 60 is definitely a sprint.
  
  // potVal = .50;
  // a crude way to do this -- we can do better.
  //---------------------------------------------------------------------------


  // if we're playing CrashDrive then we'll use the bike brake to send Spacebar (Brake)
  // to the game.  checkBrake sets brakeVal...
  // this maybe should not be done so often, will have to try it and see.
  
  if (crashDrive && bikeSteerAvail && useBikeSteer) {
    
   checkBrake();
// Z axis:  brake
// the Hall proximity sensor is super jittery for whatever reasons, it's fluctuating
// by 30 counts or more;  we hide this filtering in the brakeRead function
// we preserve the raw value and convert at the last minute (only on demand) to 0-255.
// we also clip "close to zero" to prevent slight brake dragging at low values
// I'm saying arbitrarily that Space is the brake key (it is for CrashDrive, and I
// use it also for ETS2 in WASD mode.
  if (brakeVal != lastBrake) {
    if ((brakeVal - brakeMin) <= brakeNoise) {
      brakeVal = brakeMin;
      if (brakesOn) {
      Serial.println(F("Brakes OFF"));
      Keyboard.release(32);
      brakesOn = 0;
      }
    } else {
      int newval = map(brakeVal,brakeMin,brakeMax,0,255);
      if (!brakesOn) {
      Serial.println(F("BRAKES ON!"));
      Keyboard.press(32);
      brakesOn = 1;
      }
    }
  }
  
  }

  // here we do the motion keyboard inputs:  WASD walking, or driving type controls
  // we do this no more frequently than every keyInterval ms.
  long now = millis();
  int wdelta = now - lastKeySend;
  if (wdelta >= keyInterval) {
    if (crashDrive) {
      doDrive(now);
    } else {
      doWalk(now);
    }
    lastKeySend = millis();
  }
    
  // and we are done w/main loop!  yay!
  
}
 
  
  
//
////
//////------------------------- FUNCTIONS  -----------------------------------------
////
//

// 
////  HID  functions:  key presses, i2c inputs, all the good stuff
//

void areWeSending () {

  // Kill Switch reads 0 when engaged (kill HID)
  // but the board is upside down now w/ref to my original layout idea so I'm reversing this sense
  // so that switches move to the right for On.
  
  int val = digitalRead(hidKillSw);
  
  //Serial.print("Are We Sending? switch: ");
  //Serial.println(val);

  // so 0 = true (send HID events) and anything greater is false (don't send) 
  if (val) {
    if (usbHIDsending) {
    //Keyboard.end();
    //Mouse.end();
    usbHIDsending = 0;
    Serial.println(F("HID emulation OFF"));
    digitalWrite(hidKillLED,HIGH); 
    }
    // Serial.println("\nKEYBOARD END");
  } else { 
    if (! usbHIDsending) {
      //Keyboard.begin();
      //Mouse.begin(); 
      usbHIDsending = 1;
      Serial.println(F("HID emulation ON"));
      digitalWrite(hidKillLED,LOW);
    }
  }
  
}


void checkUserSwitches() {
  
  // switches are 0 true (input_pullup)
  
  int val = digitalRead(crashDriveSw);
  if (val != crashDrive) {
  if (val) {
    crashDrive = 1;
    PCF_38.write(crashDriveLED,0);
    Serial.println(F("CrashDrive Logic"));
  } else {
    crashDrive = 0;
    Serial.println(F("Regular Walking"));
    PCF_38.write(crashDrive,1);
  }
  }

  // check spin dir invert mode
  val = digitalRead(invSpinSw);
  if (val != invertSpinDir) {
  if (val) {
    invertSpinDir = 1;
    Serial.println(F("Inverted SpinDir"));
    digitalWrite(invSpinLED,HIGH);
  } else {
    invertSpinDir = 0;
    Serial.println(F("Normal SpinDir"));
    digitalWrite(invSpinLED,LOW);
  }
  }

  // check whether we want to use the wiichuck
  val = digitalRead(wiiChuckSw);
  if (val != useChuck) {
  if (val) {
    useChuck = 1;
    PCF_38.write(wiiChuckLED,0);
    if (!chuckExists) {
      PCF_blinky(wiiChuckLED,4);
    }
    Serial.println(F("Use WiiChuck"));
  } else {
    useChuck = 0;
    Serial.println(F("Ignore WiiChuck"));
    PCF_38.write(wiiChuckLED,1);
  }
  }

    // check whether we want to use the wiichuck
  val = digitalRead(bikeSteerSw);
  if (val != useBikeSteer) {
  if (val) {
    useBikeSteer = 1;
    PCF_38.write(bikeSteerLED,0);
    if (!bikeSteerAvail) {
      PCF_blinky(bikeSteerLED,4);
    }
    Serial.println(F("Use Bike Steering"));
  } else {
    useBikeSteer = 0;
    Serial.println(F("Ignore BikeSteering"));
    PCF_38.write(bikeSteerLED,1);
  }
  }

  if (useChuck && !chuckExists) {
    PCF_blinky(wiiChuckLED,4);
  }
  if (useBikeSteer && !bikeSteerAvail) {
    PCF_blinky(bikeSteerLED,4);
  }
  
}


// check chuck for mouse emulation activity
// and any other inputs we can get out of it :-)

void checkChuck () {

  // Serial.println("Check Chuck!");
  // if HID is disabled, do nothing.
  
  if (!usbHIDsending) {
    return;
  }
  
  // Serial.println("Check Chuck Time");
  // check chuck approx on multiples of 20 ms
  long now = millis();
  if ((now - lastChuck) < 20) {
    return;
  }
  
  lastChuck = now;
  chuck.update();

  //Serial.println("Get Chuck Data!");
  // read chuck data
  // nunchuck_get_data();

  //Serial.println("Get Button data");
  //right and left click control
    int leftState = chuck.buttonC;
    if (leftState) Mouse.press(MOUSE_LEFT); else Mouse.release(MOUSE_LEFT);
    int rightState = chuck.buttonZ;
    // if (rightState) Mouse.press(MOUSE_RIGHT); else Mouse.release(MOUSE_RIGHT);
    // for now we will make right button our Jump key in wasd mode
    // and you can map Space to Brake in ETS2 and get a brake action
    if (rightState) Keyboard.press(' '); else Keyboard.release(' ');
    
  //Serial.println("Get Axis data");
  // read the x axis
    int xReading = chuck.joyX;
    // Serial.print("Chuck X: ");
    // Serial.println(xReading);
    xReading = map(xReading, 38, 232, 0, chuckRange);
    int xDistance = xReading - chuckCentre;
    if (abs(xDistance) < chuckThresh) {
      xDistance = 0;
    }
 
  // read the y axis
    int yReading = chuck.joyY;
    // Serial.print("Chuck Y: ");
    // Serial.println(yReading);
    yReading = map(yReading, 38, 232, 0, chuckRange);
    int yDistance = yReading - chuckCentre;
    if (abs(yDistance) < chuckThresh) {
      yDistance = 0;
    } 

  // move the mouse  
  // but ONLY IN Y if bike steering is enabled
  if (bikeSteerAvail && useBikeSteer) {
    if (yDistance != 0) {
      Mouse.move(0, -yDistance, 0); 
      delay(1);
    }
  } else { 
    if ((xDistance != 0) || (yDistance != 0)) {
      Mouse.move(xDistance, -yDistance, 0); 
      delay(1);
       Serial.print("Move mouse by X Y : ");
       Serial.print(xDistance);
       Serial.print(" ");
       Serial.println(yDistance);      
       delay(1);      
     }
  }
    
    //  chuckRoll varies bt -170 (roll left) and +170 (roll right)
    //  pushing it past those limits puts you at risk of rollover.
    chuckRoll = chuck.readRoll();
    // if you get a sudden wild swing in value from one extreme to the other,
    // ignore it.  max would be 340.  320 should be adequate to detect.
    if (abs(lastRoll - chuckRoll) > 320) {
      chuckRoll = lastRoll;
    } else {
     lastRoll = chuckRoll;
    }
    // Serial.println(chuckRoll); 

}

// if we have a real bike brake, check the brakes
void checkBrake() {

  lastBrake = brakeVal;

  // turn raw jittery value into a somewhat less bouncy average
  int rawval = analogRead(brakePin);
  brakeTot = brakeTot - brakeReads[brakeReadPtr];
  brakeReads[brakeReadPtr] = rawval;
  brakeTot = brakeTot + brakeReads[brakeReadPtr];
  brakeReadPtr++;
  if (brakeReadPtr >= numBrakeReads) {
    brakeReadPtr = 0;
  }
  brakeAvg = brakeTot / numBrakeReads;
  // Serial.print("Brake Average bin ");  Serial.print(numBrakeReads);
  // Serial.print("  :  ");Serial.println(brakeAvg);

  int val = brakeAvg;
  
  // if more than Noise from last value, recognise a change, update official value
  // this is so noisy that it mucks up the log.
  if (abs(val - brakeVal) > brakeNoise) {
//    Serial.print("Last Brake: "); Serial.print(brakeVal);
//    Serial.print("    New Brake: "); Serial.println(val);
    brakeVal = val;
 //   Serial.print("  Set LastBrake to "); Serial.print(lastBrake);
 //   Serial.print("  Set BrakeVal to "); Serial.println(brakeVal);
  }
  
  // later in the code we will have to map this to 0-255
  
}

void steerMouseX() {

//  after reading the rotary Hall you have a new rawSteerVal.
//  map it to a reasonable degree of mouse movement, and move the mouse in X.
//  but only if HID sending is enabled of course.
//  this code adapted from Arduino example JoystickMouseControl
  
  if (!usbHIDsending) {return;}

  int distance = 0;
  // parameters for mapping analog sensor to mouse:
  int range = 48;                // output range of X or Y movement
  int responseDelay = 5 ;        // response delay of the mouse, in ms
  int threshold = 3;             // resting threshold / deadband -- make this pretty twitchy for CD2
  int thresh2 = 9;
  int centre = range / 2;        // resting position value
  // note that these values have to work (at present) for both mouse and
  // keyboard steering, i.e. both WASD world and CrashDrive world.  
  // if you make range too large, your mouse steering in WASD games will get crazy.
  // if you make range too small, you will have difficulty tuning your deadband in CD2.
  // I found 96 was too large, 48 about right. ymmv.

  // if the output reading is outside from the rest position threshold, use it:
  int mappedval = map(rawSteerVal, minRotaryHall, maxRotaryHall, 0, range);
  distance = mappedval - centre;
  
  /*
  Serial.println(F("Distance gt threshold, make a turn"));
  Serial.print(F("Map raw steer val ")); Serial.print(rawSteerVal); 
  Serial.print(F(" range ")); Serial.print(minRotaryHall); Serial.print(F(" - ")); Serial.print(maxRotaryHall); 
  Serial.print(F("  to mapped value range 0 - "));  Serial.print(range);  Serial.print(F(" -> ")); 
  Serial.println(mappedval);
  */
    
  // Serial.print(F("mappedval distance from centre val ")); Serial.print(centre); Serial.print(F(" is ")); Serial.println(distance);
  
  // move mouse appropriate distance in X
  // OR if in special CrashDrive mode, press left and right arrows
  
  /* 
   Serial.print(F("SteerMouseX... raw "));  Serial.print(rawSteerVal); Serial.print(F("   map "));
        Serial.print(mappedval);  Serial.print(F("   thresh ")); Serial.print(threshold);
        Serial.print(F("   distance "));  Serial.print(distance); 
        Serial.print(F("   lastdist ")); Serial.println(last_distance);
   */
  if (!crashDrive) {
    Mouse.move(-distance,0,0);
    delay(responseDelay);
    return;
  }

  // the rest of this code is all crashdrive steering madness

  // if we are in the deadband, as we cross the threshhold cancel any steering
  if (abs(distance) < threshold) {
//    Serial.print(F("In Deadband, cancel all steering   raw "));  Serial.println(rawSteerVal);
    if (abs(last_distance) >= threshold) {
    Serial.print(F("In Deadband, cancel all steering   raw "));  Serial.println(rawSteerVal);
    Keyboard.release(KEY_LEFT_ARROW);
    Keyboard.release(KEY_RIGHT_ARROW);
    turnRight = 0;
    turnLeft = 0;
    }
    last_distance = distance;
    return;
   }

  // if the user is not moving the bars, do nothing.

  if (abs(distance) == abs(last_distance)) {
    last_distance = distance;
    return;
  }

  // distance is decreasing
  // if we are just starting to turn the bars back towards centre, stop the keypress right now
  if (abs(distance) < abs(last_distance)) {
    if (!turnBack) {
      Serial.println(F("Turning back, release steering keys"));
      Keyboard.release(KEY_LEFT_ARROW);
      Keyboard.release(KEY_RIGHT_ARROW);
      turnBack = 1;
      turnRight = 0;
      turnLeft = 0;
    }
    last_distance = distance;
    return;
  }

  // so you only get here if distance is increasing.
 //    Serial.println(F("+motion, turnBack = 0"));
     turnBack = 0;
  
 //  simplify, simplify, simplify.
//   Serial.print(F("In Steer Zone, raw "));  Serial.println(rawSteerVal);
   if (distance > 0) {
      if (!turnLeft) {
        Serial.println(F(" <-- "));
        Keyboard.press(KEY_LEFT_ARROW);
        turnLeft = 1;
      }
      } else {
       if (!turnRight) {
        Serial.println(F(" -->  "));
        Keyboard.press(KEY_RIGHT_ARROW);
        turnRight = 1;
       }
      }
    last_distance = distance;
}


//
////  handy bits for determining direction and smoothing out blips (boxcar avg)
//

void initTelemetry () {
  
  directionMatrix[0][1] = 1;
  directionMatrix[1][2] = 1;
  directionMatrix[2][0] = 1;
  directionMatrix[0][2] = -1;
  directionMatrix[1][0] = -1;
  directionMatrix[2][1] = -1;
  
  for (int i = 0; i < numSensors; i++ ) {
        lastoff[i] = 0;
        laston[i] = 0;
        lastval[i] = 0;
        lastrpm[i] = 0;
        zeroBoxcar(i);
  }
  
  lastedge = -1;
  spinDir = 0;
  lastdir = 0;
  
}

void zeroBoxcar (int i) {
  //Serial.print("ZERO BOXCAR:  ");
  //Serial.println(i);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[i][thisReading] = 0;
  }
  total = 0;
  rpmBuffer = 0;
  readIndex = 0;
 
}

// this calibration code is not in use yet:  future feature

boolean checkCalibration() {

  // we only get in here if calibration < 2
  // so it can be only 0 or 1
  
 if (!calibrated) {
    digitalWrite(loSpdLED,HIGH);
    digitalWrite(medSpdLED,HIGH);
    digitalWrite(hiSpdLED,HIGH);
    calibrateBegin = millis();
    calibrated = 1;
    return(0);
  }
  
    long elap = millis() - calibrateBegin;
    // have we waited long enough for calibration?
    if (elap < calTime) {
      return(0) ;
    } else {
      calibrated = 2;
      // if the user pedalled during calibration period then set threshold values
      // walk speed is 1/3 of max rpm, run speed is 2/3 (more or less).
      if (maxRPM > 0) {
      maxRunSpeed = maxRPM * 1.10;
      runThresh = (maxRunSpeed * .60);
      walkThresh = (maxRunSpeed * .25);
      }
      digitalWrite(loSpdLED,LOW);
      digitalWrite(medSpdLED,LOW);
      digitalWrite(hiSpdLED,LOW);
      
      DEBUG_PRINT("Done with calibration.");

      return (1) ;
    }

  }
  
//
//// The guts of the tachometer:  read sensors, calculate RPM each time
//// by calculating rpm on each sensor and averaging, we smooth out variations in rotation
//

int readSensor(int i) {
    
  int retval = 0;
  String msg = "foo";
  String msg2 = "bar";
  //val = analogRead(i);
  
  sensorVal = digitalRead(reeds[i]);
  // reverse sense so 1 is true and 0 is false because these are dragging a pullup input to ground
  sensorVal = !sensorVal;

  // 0 now becomes HIGH and HIGH becomes 0, so we can keep our ugly old code.

  long now = millis();
  float rpm = 0.0;
  
  // has state changed?  if so, it's either a rising or falling edge.
  // we count rising edges
  
  if (sensorVal !=  lastval[i]) {
    
    Stopped = 0;
    PCF_38.write(stopLED,1);
    retval = sensorVal;
 
  // is it a leading edge event (went high)?  if so, compute rpm
  
  if (sensorVal) {
    
    // delta is in millisec
    // for now, compute rpm ONLY on pinReed0
    // we are sensor i, who was last sensor?
    // ignore it if it was "self", as we do get bounce from time to time
    // update direction 
    // if we just changed direction then force average rpm to 0

    if (lastedge != i) {
      spinDir = directionMatrix[i][lastedge];
      if (invertSpinDir) spinDir = 0-spinDir;
      if (spinDir != lastdir) {
        msg = "*** CHANGE DIR: " ;
        msg2 = msg + lastedge + " --> " + i + " last " + lastdir + " now " + spinDir;
        DEBUG_PRINT("msg2");
         zeroBoxcar(i);
         zeroBoxcar(0);
      }
      lastdir = spinDir;
    }

    // now we get to the heart of the matter:  calculate rpm
    
    long tdelta = now - laston[i];
    rpm = (1000.0 / tdelta) * 60.0;
    // filter out bogus rpm (switch bounces? causing super high values)
    if (rpm <= rpmCap) {
        updateRpm(rpm * spinDir);
        // aDelta is change in averaged rpm.  neg if falling, pos if rising
    }
    lastedge = i;

    // if we are in slow walk mode, set up to emit one keystroke per leading edge on even reeds only
    // this avoids the jinkiness of uneven spacing of reed 1
    // if we are not in slow walk mode we wd just hold down the appropriate key (in doWASD).
    // I think one keystroke is not enough here.  should probably be a 100 or 200 ms press.
    // for now I'm leaving it as is, but for gaming it should be improved.
    // reedTrip will be unset when the keystroke is emitted.
    if ((i == 2) || (i == 0)) {
    if (slowWalking) {
//      Serial.print(F("SlowWalking, set reedTrip 1, saw reed "));  Serial.println(i);
      reedTrip = 1;
     }
    }
  }     // if sensor val positive
  //update lastval for each sensor, not just A0
  lastval[i] = sensorVal;
  
  } else {
    
    // sensor value is same as last time:
    // no fresh edge on this sensor.  how long has it been? > 1 sec avg:  call it a stop
    // this timeout should get longer as potval gets smaller.
    // pot set to .50 by default.  motionTimeout set to 2000, so default is 1000
    // when pot value is .25, timeout expands to 1500+100:  1600
    // when pot value is .75, timeout shrinks to 500+100:    600
    // these may not be the numbers we want but it's a start.  it scales.
    int dt = now - laston[i];
    // how long ago was last rising edge
    // timeout = round((motionTimeout * (1.0 - potVal)) + 100);
    // having no pot in this simple version I'm just going to set it to 1.5sec
    // which seems generous
      timeout = 1500;
    // if it has been timeout ms since most recent rising edge, then we are stopped
    if (!Stopped) {
    if (dt > timeout) { 
      msg = "Timeout on " ;
      msg2 = msg + i + " " + dt + " > " + timeout;
      DEBUG_PRINT(msg2);
      stopTheWorld(i); 
    }
    }
    retval = -1;
  } // end of else block, no edge detected
 
  return (retval);
  
}


void stopTheWorld(int i) {
    
      // set Stopped flag and clean up other details
      float rpm = 0.0;
      avgRPM = 0;
      lastAvgRPM = 0;
      zeroBoxcar(i);
      
      // updateRpm(rpm);
      
      // lastval[i] = 0;
      laston[i] = millis();
      lastoff[i] = millis();
      
      // update last edge time so you don't end up with tremendous rpm on first motion
      
      // if not already stopped, stop the machinery, kill the lights.
      // this is the only place where the Stopped flag is tested.
      if (!Stopped) {
      digitalWrite(loSpdLED,LOW);
      digitalWrite(medSpdLED,LOW);
      digitalWrite(hiSpdLED,LOW);
      DEBUG_PRINT("STOP DETECTED, set slowWalking 1");
      Stopped = 1;
      slowWalking = 1;
      PCF_38.write(stopLED,0);
      Keyboard.releaseAll();
      restoreState();
      }
      
}

void restoreState() {
      if (brakesOn) {Keyboard.press(32);}
      if (turnRight) {Keyboard.press(KEY_RIGHT_ARROW);}
      if (turnLeft) {Keyboard.press(KEY_LEFT_ARROW);}
}

// updateRPM -- do boxcar smoothing of measured RPM and set global val "avgRPM"

void updateRpm(float rpm)  {

      // do boxcar averaging of rpm, record average, step buffer
      
      // subtract the last reading (which should be zero on first iteration):
      total = total - readings[0][readIndex];
      // read from the sensor:
      readings[0][readIndex] = rpm;
      // add the reading to the total:
      total = total + readings[0][readIndex];
      // do not try to average rpm till you have at least numReadings in buffer.
      if (!rpmBuffer) {
        readIndex = readIndex + 1;
        if (readIndex == numReadings) {
          readIndex = 0;
          rpmBuffer = 1;
        }
        return;
      }
      // calculate the average:
      avgRPM = total / numReadings;
      // if (avgRPM > maxRPM) { maxRPM = avgRPM;}
      // clip the speed to predefined max.  should have a warning light for this.
      if (avgRPM > maxRPM) {avgRPM = maxRPM;}
      // how is it different from last avg?  neg if less, pos if more
      aDelta = abs(avgRPM) - abs(lastAvgRPM);
      float abdelta = abs(aDelta);
      // this next diagnostic print is a bit noisy
      /*
      Serial.print(rpm);
      Serial.print(" ");
      Serial.println(avgRPM);
      */
      // advance to the next position in the array:
      //  advance buffer pointer
      readIndex = readIndex + 1;

      // if we're at the end of the array...
      if (readIndex == numReadings) {
         // ...wrap around to the beginning:
         readIndex = 0;
      }

      lastAvgRPM = avgRPM;
 
}


void doWalk(long now) {
  
  // WASD output
  // we are in wasdland, so we use absolute rpm (higher than X is run, lower is walk)
  // and spinDir tells us whether to send W or S -- do this every 1/10th sec
  // we don't do this too often because it's expensive and jittery.

    if (!usbHIDsending) {return;}

    int slope = avgRPM - lastSampleAvg; // neg if slowing down, pos if speeding up
    int fudge = 0;
    if (slope >= 0) {
      fudge = hysFudge;
    } else {
      fudge = -hysFudge;
    }
    
    // case 1:  we are walking slower than normal walk speed:  slow walking
    // this is a special mode:  we issue individual keystrokes (see reed switch code and mainloop)
    
    if ((abs(avgRPM) < (walkThresh*potVal + fudge))) {
      footPace = 0;
      if (lastPace != footPace) {
        Keyboard.releaseAll();
        restoreState();
        slowWalking = 1;
        Serial.print(F("Changing to Slow Walk Mode @ "));
        Serial.println(avgRPM);
        digitalWrite(loSpdLED,HIGH);
        digitalWrite(medSpdLED,LOW);
        digitalWrite(hiSpdLED,LOW);
        lastPace = footPace;
        lastPaceTime = now;
      }
    }
    
    // case 2:  we are walking at normal walk speed, less than run speed
    // we hold down a key (w if forward and s if back).
    
    if ((abs(avgRPM) >= (walkThresh*potVal + fudge)) && (abs(avgRPM) < (runThresh*potVal + fudge))) {

      footPace = 1;
 
      if (lastPace != footPace) {
        slowWalking = 0;
        Serial.print(F("Changing to Regular Walk Mode @ "));
        Serial.println(avgRPM);
        digitalWrite(loSpdLED,LOW);
        digitalWrite(medSpdLED,HIGH);
        digitalWrite(hiSpdLED,LOW);
      // if we are slowing down from a run, release the shift;  if we are
      // speeding up from a tentative tapping walk, hold down the direction key
      if (lastPace > footPace) {
        Keyboard.release(KEY_LEFT_SHIFT);   
      } else {
        // cancel any outstanding step
        reedTrip = 0;
        stepStart = 0;      
        if (spinDir > 0) {
          PCF_38.write(reverseLED,1);
            Keyboard.press('w');
              // Serial.print("w");
        } else {
          PCF_38.write(reverseLED,0);
          // Serial.print("s");
            Keyboard.press('s');
        }
      }
      lastPace = footPace;
      lastPaceTime = now;
      }
    }
    
    // case 3:  we are walking at run speed
    // so add a shift key to whatever is already going on
    
    if (abs(avgRPM) >= (runThresh*potVal + fudge)) {
      footPace = 2;
      if (lastPace != footPace) {
      digitalWrite(loSpdLED,LOW);
      digitalWrite(medSpdLED,LOW);
      digitalWrite(hiSpdLED,HIGH);
      Serial.print(F("Changing to Fast Walk (Run) Mode @"));
      Serial.println(avgRPM);
      // if we leapt from Slow to Run without going through Walk... turn on walk key first
      // I doubt this could ever happen (how fast can you spin up your trainer?) 
      // with a reasonable config, but if someone set their pace boundaries 
      // very narrowly spaced, it might.
      if (!lastPace) {
        if (spinDir > 0) {
          PCF_38.write(reverseLED,1);
            Keyboard.press('w');
        } else {
          PCF_38.write(reverseLED,0);
          // Serial.print("s");
            Keyboard.press('s');
        } 
      }
      Serial.println(F("RUN MODE"));
      Keyboard.press(KEY_LEFT_SHIFT);
      // you cannot see this shift effect in an xterm but video game sees it OK

      lastPace = footPace;
      lastPaceTime = now;
      }
    }
    // record last avgRPM that we actually considered
    lastSampleAvg = avgRPM;
}

void checkSlowStep() {

//  WASD mode only!
//  we are here because reedTrip was set.
//  readTrip was set in the readSensor code, because we are in slow walk mode and a reed 0 or 2 was detected.
//  so we're trying to take one step.
//  this is outside the regular doWASD clock.
//  if reedTrip was set and stepStart is not set, then we need to commence a step.
//  if reedTrip was set and stepStart is set, then we are in mid-step and need to check our duration and
//  if necessary, terminate our step.

      if (stepStart > 0) {
        long now = millis();
        if ((now - stepStart) >= stepLength) {
        // END STEP
          Keyboard.release('w');
          Keyboard.release('s');
          reedTrip = 0;
          stepStart = 0;
          Serial.println(F("In checkSlowStep end step"));
        }
      } else {
        if (spinDir > 0) {
          Serial.println(F("In checkSlowStep go FWD press w"));
          Keyboard.press('w');
        } else {
          Serial.println(F("In checkSlowStep go REV press s"));
          Keyboard.press('s');
        }
        stepStart = millis();        
      }
}


void doDrive(long now) {

//  what we need to know:
//  aDelta plus state (forward, reverse, or stopped)
//  avgRPM (current avg rpm)
//  spinDir (direction of pedalling)
//  aDelta is the diff between abs(avgRPM) and abs(lastAvgRPM)
//  its sign indicates change of magnitude not direction

    if (!usbHIDsending) {return;}
    
    // Serial.print(F("DO_DRIVE...   RPM: ")); Serial.print(avgRPM);  Serial.print(F("   aDelta: ")); Serial.println(aDelta);
    if (Stopped) {
      Keyboard.release(KEY_UP_ARROW);
      Keyboard.release(KEY_DOWN_ARROW);
      inForward = 0;
      inReverse = 0;
      return;
    }

//  simple version:  not Stopped and pedalling forward?  hit the gas!

//  case FORWARD
    if (spinDir > 0) {
      // aDelta is ge required threshhold and pedals are moving Forward
      // if we were previously in reverse, cancel reverse motion
      // if we are not yet in forward, initiate forward motion
      if (inReverse) {
        Keyboard.release(KEY_DOWN_ARROW);
        inReverse = 0;
      }
      Serial.print(F("ACCELeration detected, Fwd, UpArrow press, DELTA : ")); Serial.println(aDelta);
      Keyboard.press(KEY_UP_ARROW);
      inForward = 1;
    } else {
//  case REVERSE
      // aDelta is ge required threshhold and pedals are moving Backward
      // if we were previously in forward, cancel forward motion
      // if we are not yet in reverse, initiate reverse motion
      if (inForward) {
        Keyboard.release(KEY_UP_ARROW);
        inForward = 0;
      }
      if (!inReverse) {
      Serial.print(F("ACCELeration detected, Rev, DnArrow press, DELTA : ")); Serial.println(aDelta);
      Keyboard.press(KEY_DOWN_ARROW);
      inReverse = 1;
      }
    }

    if (abs(avgRPM) >= turboThresh) {
      if (!turboDrive) {
      Serial.print(F("Speed over TURBO ON threshhold, hold Shift key, RPM : ")); Serial.println(avgRPM);
      Keyboard.press(KEY_LEFT_SHIFT);
      turboDrive = 1;
      }
    } else {
      if (turboDrive) {
       Serial.print(F("Speed below TURBO OFF threshhold, release Shift key, RPM : "));  Serial.println(avgRPM);
       Keyboard.release(KEY_LEFT_SHIFT);
       turboDrive = 0;   
       }
    }
}

// some basic startup funcs...

boolean testI2C(byte addr) {
  
  boolean success = 0;
  for (int attempt = 0; attempt < 10; attempt++) {
    Serial.print(F("  Try to transmit to address "));
    Serial.println(addr);
  Wire.beginTransmission (addr);
  byte error = Wire.endTransmission();
  if (error == 0)
  {
     Serial.print(F("  Found slave alive and listening on try: "));
     Serial.println(attempt);
     success = 1;
     break;
  } else {
     Serial.print(F("  Slave contact attempt: "));
     Serial.println(attempt);
     attempt++;
     delay(1000);
  }
  }
  
  return(success);

}

//
// Stuff stolen from Leo1_Current to set up rotary Hall encoder
// we set up the Rotary Hall sensor with desired parameters and read one value

void initRotaryHall() {

  // approx range 90-2016
  // SetupQuadrature resolution to max (2048)
 
  Wire.beginTransmission(0x36);       // transmit to device as5601
  Wire.write(byte(0x09));             // sets register pointer to ABN 
  Wire.write(byte(0x0F));             //resolution 2048 
  byte result = Wire.endTransmission();    // stop transmitting
  //************************************************************************
  // Check SetupQuadrature resolution to max (2048) 
  Wire.beginTransmission(0x36);       // transmit to device as5601
  Wire.write(byte(0x09));             // sets register pointer to ABN 
  result = Wire.endTransmission();    // stop transmitting
  Wire.requestFrom(0x36, 1);          // request 2 bytes from as5601
  int reading = Wire.read();
    Serial.print(F("Qres: "));
// receive high byte 
  Serial.print(reading,BIN);   
  Serial.println(F("")); 
}

void readRotaryHall () {
  
// Quadrature enc with the AS5601 Hall position.
  Wire.beginTransmission(0x36);       // transmit to device as5601
  Wire.write(byte(0x0E));             // sets register pointer 
  byte result = Wire.endTransmission();    // stop transmitting
//
  Wire.requestFrom(0x36, 2);         // request 2 bytes from as5601
  int reading = Wire.read();             // receive high byte 
  reading = reading << 8;            // shift high byte to be high 8 bits
  reading |= Wire.read();            // receive low byte as lower 8 bits
  // Serial.print(F("First RHall angle: "));
  rawSteerVal = reading/2;  

  // clamp steering input within sensible limits
  if (rawSteerVal > maxRotaryHall) {
    rawSteerVal = maxRotaryHall;
  }
  if (rawSteerVal < minRotaryHall) {
    rawSteerVal = minRotaryHall;
  }
  
  steerVal = map(rawSteerVal,minRotaryHall,maxRotaryHall,-180,180);
  // Serial.print(F("RAW Steer: ")); Serial.print(rawSteerVal);
  // Serial.print(F("  JOY Steer: ")); Serial.println(steerVal);
  
  //Serial.print(F("  Status: "));        // remove // if like to display the bits
  Wire.beginTransmission(0x36);       // transmit to device as5601
  Wire.write(byte(0x0b));             // sets register pointer 
  result = Wire.endTransmission();    // stop transmitting

  Wire.requestFrom(0x36, 1 );         // request 1 bytes from as5601
  reading = Wire.read();              // receive high byte
  //Serial.print(reading, BIN);       // remove // if like to display the bits
  
  if (bitRead(reading, 5)!=1) { Serial.print(F(" Status: ")); 
  if (bitRead(reading, 4)==1) Serial.println(F("too high "));
  if (bitRead(reading, 3)==1) Serial.println(F("too low "));}
  
   //************************************************************************
// AGC  ( we don't need the AGC)
/*  
  Serial.print(F("AGC: "));
  Wire.beginTransmission(0x36);       // transmit to device as5601
  Wire.write(byte(0x1a));             // sets register pointer 
  result = Wire.endTransmission();    // stop transmitting

  Wire.requestFrom(0x36, 1);          // request 1 bytes from as5601
  reading = Wire.read();              // receive high byte
  Serial.print(reading, DEC);             
  Serial.println(F(""));
 //************************************************************************
*/

}

//
//  basic LED signalling
//

void blinky(int which, int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(which,HIGH);
    delay(100);
    digitalWrite(which,LOW);
    delay(150);
  }
}

void PCF_blinky(int which, int times) {
  for (int i = 0; i < times; i++) {
    PCF_38.write(which,0);
    delay(100);
    PCF_38.write(which,1);
    delay(150);
  }
}

void goCatatonic(int which) {
  // you can't recover from this condition.  just blink the selected light and hope for rescue.
  while(1) {
    blinky(which,2);
    delay(500);
  }
}

