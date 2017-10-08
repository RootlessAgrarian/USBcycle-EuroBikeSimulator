////-------------------------------------------------------------------
// USBcycle
//
// Leonardo-based Physical Game Controller (game bike, game stepper etc):
// works with anything that has a rotating part you can attach a magnet to,
// and a fixed part that you can attach 3 reed switches to.
//
// Basic dirt-simple WASD version
//
// WORKING as of April 9 2017, tested with Unity game build.
//
// USBcycle reads 3 reed switches to monitor rotation and derive 
// speed and direction.
// This version uses a single Leonardo as a keyboard/mouse for WASD game control.
// This version supports ONLY KEYBOARD/MOUSE functions: NO JOYSTICK EMULATION 
// For full Kbrd/Mouse/Joystick emulation see the EuroBike capable version:
// Leo1_Current.ino and Leo2_Current.ino
//
// First release version (github) Oct 2017
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
//            useful, but I have kept it just in case...
// invSpinSw: if you have a portable stepper that can be used either standing or seated, often
//            it must be rotated 180 between those 2 positions -- which of course reverses the rotation
//            sense.  so this switch simply swaps Fwd and Reverse, so you can use your stepper
//            in either orientation w/o moving wires around.

const int wiiChuckSw = 4;        // sw 1 bottom row
const int bikeSteerSw = 5;       // sw 2 bottom row
const int hidKillSw = 11;
const int invMsySw = A3;   // sw 3 bottom row
const int invSpinSw = A2;  // sw 4  bottom row

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

// these LEDs are wired to real Leo ports
const int invSpinLED = 6;
const int hiSpdLED = 7;
const int medSpdLED = 8;
const int loSpdLED = 9;
const int hidKillLED = 10;

// these 8 LEDs are attached to the expansion device:
// again the order of pins is a little odd because of wire lengths and poor planning.

const int invMsyLED = 0;
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

//
// NB: Wire, Keyboard and Mouse are stock Ardu libs.  Hirzel is a contrib.
// and so is pfc8574
//

// pretty debug print function from early stages of project
//#define DEBUG

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
   Serial.print(invertMouseY); \
   Serial.print(" "); \
   Serial.print(invertSpinDir); \
   Serial.print(" "); \
   Serial.print(average); \
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
  float maxRPM = 0.0;
  float rpmCap = 240;
  int lastedge = 0;
  int directionMatrix[numSensors][numSensors];
  int spindir;
  int lastdir;
  int timeout;
  bool rpmBuffer = 0;
  
  // boxcar averaging : size of buffer s/b considered carefully, as a large buffer 
  // favours acceleration and masks deceleration.  10 is definitly too soft, big hysteresis
  // 8 may even be too much.  but you need something to counteract the occasional spikes.
  // now that I'm including all sensors, not just pinReed0, 6 seems good enough smoothing.
  
  const int numReadings = 3;         // boxcar history size:  average the last three sensor values
  float readings[numSensors][numReadings];      // the readings from the analog input
  float adeltas[numReadings];
  int readIndex = 0;                // the index of the current reading 
  float total = 0;                  // the running total
  float average = 0;                // the average
  float lastavg = 0;                // we may want to track trend rather than absolute rpm
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


  int footPace = 0;                     // 3 speeds, 0 (slow), 1 (walk), and 2 (run)
  int lastPace = 0;                     // as usual it is transitions that matter
  long lastPaceTime = 0;
  int hysFudge = 2;                     // fudge factor (rpm) by which to take out jitter in
                                        // foot pace decision
  float speedAdjust = 0.0;              // factor by which to map rpm to walk vs run
                                        // or in fly mode, mouse wheel setting
                                        // this "gain" factor is set by a pot
  // 30 rpm is very slow.   60rpm is comfortable walking speed.  120 is very fast.
  // so let's say 45 to 80 is walking, above 80 is running.  and we can scale by the gainpot.
  float maxRunSpeed = 180.0;            // I kinda doubt you can run faster than that
  float runThresh = 160;                // hard wiring this not such a great idea.  we'll set max values
  float walkThresh = 90;                // and multiply them by the gain pot factor (50 percent is default).
  boolean slowWalking = 1;                  // below a certain rpm, send out one char per leading edge
  boolean Stopped = 1;                      // so we can stop doing much of anything when we are motionless
  int motionTimeout = 2000;              // how long must elapse between edges to conclude that we are stopped?
  float potVal = 1.0;
  int runCt = 0;                        // number of run speed loops you need to change up 
  int walkCt = 0;                       // number of walk speed loops you need to change up
  uint8_t keyPressed = 0;
  int usbHIDsending = 1;

  boolean invertMouseY = 0;             // need this for intuitive game look-around control
  boolean invertSpinDir = 0;            // you will need this for recumbent bike emulation

  // (future feature placeholder)
  // we need to calibrate before running, to get valid maxRPM etc.
  // how to prompt user and do calibration?

  int calibrated = 0;
  long calibrateBegin = 0;
  int calTime = 10000;

  boolean chuckExists = 0;

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


  // 
  // INIT SWITCHES AND LIGHTS
  //
  
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
  pinMode(invMsySw, INPUT_PULLUP);
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

  // Mouse emu, Keyboard emu, and wiichuck if present
  
  Mouse.begin();
  delay(1);
  Keyboard.begin();
  delay(1);

  // join i2c bus as master but only if we see pullup voltage on pins 2 and 3
  int d2 = digitalRead(2);
  int d3 = digitalRead(3);
  if (!(d2&d3)) {
    Serial.println(F("OUCH, no I2C bus power."));
    goCatatonic(hidKillLED);       // most severe, no i2c bus power red disaster light
  }
  // otherwise I2C is OK

  // *** blink a happy light:  phase 2 OK
  blinky(medSpdLED,6);
    
  // init i2c bus
  Wire.begin();
  delay(500);

  // first do we have our port expander?  if we don't, we are dead in the water
  
  bool i2cOK = testI2C(0x38);
  if (!i2cOK) {goCatatonic(loSpdLED);}   //  most severe:  blink annoying white LED forever

  //
  // since it exists, let's talk to the port expander:  set all lights off
  //
  
  uint8_t value = PCF_38.read8();
  Serial.print("#38:\t");
  Serial.println(value);
  value = 255;
  PCF_38.write8(255);

  // *** blink a happy light to show we are still OK, phase 3 done
  blinky(hiSpdLED,6);
  
  // do we have a wiichuck?  if we don't, we can still operate with a standalone mouse
  // we just have to ignore all chuck operations.
  
  i2cOK = testI2C(0x52);
  if (!i2cOK) {
    chuckExists = 0; 
  } else {
    chuckExists = 1;
  }
  
  if (chuckExists) {
  // init nunchuck on i2c bus (this uses Wire)
  // this presumably does Wire(begin) for us.
  chuck.begin();
  chuck.update();
  delay(1); 
  }

  // INIT direction matrix and history variables
  initTelemetry();
  
  delay(1000); 

  // *** woo hoo we survived init!  big happy light show:
  // riffle through expander LEDs to signal successful init
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
  
  blinky(invSpinLED,6);
  
  // DONE WITH INIT 
  Serial.println(F("INIT COMPLETE"));
  
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

  // check user switches

  // cgeck mouse invert mode
  // switches are 0 true (input_pullup)
  
  int val = digitalRead(invMsySw);
  if (val != invertMouseY) {
  if (val) {
    invertMouseY = 1;
    PCF_38.write(invMsyLED,0);
    Serial.println(F("Inverted Mouse Y"));
  } else {
    invertMouseY = 0;
    Serial.println(F("Default Mouse Y"));
    PCF_38.write(invMsyLED,1);
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

  // one day we will check for "request real bike steering"
  
   // read nunchuck and update mouse position as needed
   // lights will blink (above) if you asked for chuck and it wasn't there
   if (chuckExists & useChuck) {
   checkChuck();
   }

// read the bike reed switches

  for (int i = 0; i < numSensors; i++ ) {
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

  // oh dear, this pot was actually useful at one time...
  // how shall we set our threshholds?  take the default value
  //     potVal = analogRead(gainPot) / 1023.0;
  // we express the pot reading as a percentage
  // the normal position is 50 percent or centred
  
  potVal = .50;
  
  long now = millis();

  // WASD output
  // we are in wasdland, so we use absolute rpm (higher than X is run, lower is walk)
  // and spindir tells us whether to send W or S -- do this every 1/10th sec
  // we don't do this too often because it's expensive and jittery.
  
  if (!(now % 100)) {
    
    int slope = average - lastSampleAvg; // neg if slowing down, pos if speeding up
    int fudge = 0;
    if (slope >= 0) {
      fudge = hysFudge;
    } else {
      fudge = -hysFudge;
    }
    
    // case 1:  we are walking slower than normal walk speed:  slow walking
    // this is a special mode:  we issue individual keystrokes (see reed switch code)
    
    if ((abs(average) < (walkThresh*potVal + fudge))) {
      footPace = 0;
      if (lastPace != footPace) {
        Keyboard.releaseAll();
        slowWalking = 1;
        digitalWrite(loSpdLED,HIGH);
        digitalWrite(medSpdLED,LOW);
        digitalWrite(hiSpdLED,LOW);
        lastPace = footPace;
        lastPaceTime = now;
      }
    }
    
    // case 2:  we are walking at normal walk speed, less than run speed
    // we hold down a key (w if forward and s if back).
    
    if ((abs(average) >= (walkThresh*potVal + fudge)) && (abs(average) < (runThresh*potVal + fudge))) {
      footPace = 1;
      if (lastPace != footPace) {
      slowWalking = 0;
      digitalWrite(loSpdLED,LOW);
      digitalWrite(medSpdLED,HIGH);
      digitalWrite(hiSpdLED,LOW);
      
      if (lastPace > footPace) {
        Keyboard.release(KEY_LEFT_SHIFT);   
      } else {      
        if (spindir > 0) {
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
    
    if (abs(average) >= (runThresh*potVal + fudge)) {
      footPace = 2;
      if (lastPace != footPace) {
      digitalWrite(loSpdLED,LOW);
      digitalWrite(medSpdLED,LOW);
      digitalWrite(hiSpdLED,HIGH);

      // if we leapt from Slow to Run without going through Walk... turn on walk key
      if (!lastPace) {
        if (spindir > 0) {
          Keyboard.press('w');
          PCF_38.write(reverseLED,1);
        } else {
          Keyboard.press('s');
          PCF_38.write(reverseLED,0);
        }
      }
      Keyboard.press(KEY_LEFT_SHIFT);
      // you cannot see this shift effect in an xterm but video game sees it OK

      lastPace = footPace;
      lastPaceTime = now;
      }

    }
    // record last average that we actually considered
    lastSampleAvg = average;

  }

  // and we are done w/main loop!
  
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
    } else {
      if (invertMouseY) {
      yDistance = -yDistance;
      }
    }  

  // move the mouse
 
    if ((xDistance != 0) || (yDistance != 0)) {
      Mouse.move(xDistance, -yDistance, 0); 
      delay(1);
       Serial.print("Move mouse by X Y : ");
       Serial.print(xDistance);
       Serial.print(" ");
       Serial.println(yDistance);
       
       delay(1);
      
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
  spindir = 0;
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

    // NOTE that in the first draft of this code, these switches pulled their analog pins HIGH.
    // In this version, the pins are set INPUT_PULLUP and the switches pull them LOW.
    // and we have gone from analog to digital so values are 1 vs 0 not a range from 0 to 1023.
    // 
  int retval = 0;
  String msg = "foo";
  String msg2 = "bar";
  //val = analogRead(i);
  sensorVal = digitalRead(reeds[i]);
  // reverse sense so 1 is true and 0 is false
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
      spindir = directionMatrix[i][lastedge];
      if (invertSpinDir) spindir = 0-spindir;
      if (spindir != lastdir) {
        msg = "*** CHANGE DIR: " ;
        msg2 = msg + lastedge + " --> " + i + " last " + lastdir + " now " + spindir;
        DEBUG_PRINT("msg2");
         zeroBoxcar(i);
         zeroBoxcar(0);
      }
      lastdir = spindir;
    }

    // now we get to the heart of the matter:  calculate rpm
    
    long tdelta = now - laston[i];
    rpm = (1000.0 / tdelta) * 60.0;
    // filter out bogus rpm (switch bounces? causing super high values)
    if (rpm <= rpmCap) {
        updateRpm(rpm * spindir);
        // adelta is change in averaged rpm.  neg if falling, pos if rising
    }
    lastedge = i;

    // if we are in slow walk mode, emit one keystroke per leading edge on even reeds only
    // this avoids the jinkiness of uneven spacing of reed 1
    // if we are not in slow walk mode we just hold down the appropriate key (in main loop).
    // I think one keystroke is not enough here.  should probably be a 100 or 200 ms press.
    // for now I'm leaving it as is, but for gaming it should be improved.
    if ((i == 2) || (i == 0)) {
    if (slowWalking) {
      Keyboard.releaseAll();
        if (spindir > 0) {
          Keyboard.write('w');
        } else {
          Keyboard.write('w');
        }
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
    timeout = round((motionTimeout * (1.0 - potVal)) + 100);
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
      average = 0;
      lastavg = 0;
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
      DEBUG_PRINT("STOP DETECTED");
      Stopped = 1;
      slowWalking = 1;
      PCF_38.write(stopLED,0);
      Keyboard.releaseAll();
      }
      
}

// updateRPM -- do boxcar smoothing of measured RPM and set global val "average"

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
      average = total / numReadings;
      if (average > maxRPM) { maxRPM = average;}
      // how is it different from last avg?  neg if less, pos if more
      aDelta = abs(average) - abs(lastavg);
      float abdelta = abs(aDelta);
      /*
      Serial.print(rpm);
      Serial.print(" ");
      Serial.println(average);
      */
     
      // advance to the next position in the array:
      //  advance buffer pointer
      readIndex = readIndex + 1;

      // if we're at the end of the array...
      if (readIndex == numReadings) {
         // ...wrap around to the beginning:
         readIndex = 0;
      }

      lastavg = average;
 
}


// some basic startup funcs

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

