

// This tab contains all the global declarations and code
// for the game controller aspects of the sketch.
// Such as keyboard related definitions.
// Such as computations from raw value to joystick values,
// joystick send, etc.



#include <Keyboard.h>
#include <Mouse.h>


// *** Extended Keyboard Definitions
//     you need these defs in order to send any keypad keys via Keyboard.press(val)
//     and you need keypad keys for my ETS2 control map; though I suppose I could 
//     eliminate them I prefer to leave the keymap familiar in case I revert to using
//     a usb keyboard at any time.  Why these are not in Keyboard.h I have no idea.
const byte KEYPAD_0 = 234;
const byte KEYPAD_1 = 225;
const byte KEYPAD_2 = 226;
const byte KEYPAD_3 = 227;
const byte KEYPAD_4 = 228;
const byte KEYPAD_5 = 229;
const byte KEYPAD_6 = 230;
const byte KEYPAD_7 = 231;
const byte KEYPAD_8 = 232;
const byte KEYPAD_9 = 233;
const byte KEY_ENTER = 176;
const byte KEYPAD_ENTER = 224;


// there are some issues to work out with Due as joystick.  new library? //

#include <Joystick.h>
//-------------------------------------------------------------------
//
// Create the Joystick
 
  Joystick_ Joystick(0x10, 
  JOYSTICK_TYPE_JOYSTICK, 12, 0,
  true, true, true, false, false, false,
  false, false, false, false, false);

//  Joystick_ Joystick;
// create a JOYSTICK type controller that looks just exactly like a driving force gt
// with three axes 0 1 2, a hat, and a bunch of buttons
// looks like you cannot be a multiaxis controller, you have to be a JOYSTICK type ctrlr.
// if you choose MULTI_AXIS, OSX will go mad and start pushing the mouse pointer around.
//

//-------------------------------------------------------------------
// ***
// *** definitions for the AS5601 Rotary Hall sensor for steering (an i2c device)
// ***
// #include <Encoder.h>
// Encoder AS5601enc(18, 19);
// these values are absolutes, i,e, -90 to 90 on the bars;  but we would like more sensitive steering
// steerfactor can be set from potVal by a button press.
// I find that .80 is pretty good so I'm pre-setting it to that.
// NOTE these have been replaced by State.val.steerT and State.val.turbo.
float steerFactor = .80;
boolean logThrottle = 0;
// centre value is determined at startup; we allow rotation of +400 and -400
// increase that number and you can get more degrees of steering action (turn bars further, get 
// more encoder count range).  but the range is mapped to a fixed joystick range.  so it's less
// sensitive if you make the steerLock value larger.
// NOTE:  it's important to have the bars centred at boot time!
//   not as of v2: we have Steering Centre which sets centrepoint to current position
// early calibration:  +/- 90 on the bars is 1900 to 500, range +/- 700
// Jan 1 2019:  new sensor (3v3, new breakout)
//   fortunately, rotation looks good.  I'm getting +/- 500 from 90 to 90.
//   our initial values are now as follows:
int maxRotaryHall = 1320;
int minRotaryHall = 320;
int steerCentre = 820;
int steerLock = 500;
int steerVal = 0;
int rawSteerVal = 0;
int lastSteer = 0;
// 
//-------------------------------------------------------------------

// all this also could be in a shared .h file
float rpm = 0.0;
const int maxRPM = 180;     // that is superhuman pedaling, 3x my target rate
const int minRPM = 20;      // dead slow!
float slowThresh = .30 * maxRPM;
float medThresh = .50 * maxRPM;                
float fastThresh = .75 * maxRPM; 
boolean Stopped = 1;
float potVal;
int magEnc = 750;

int brakeVal = 0;                     // getting this (real bike version) from A5, hall proximity sensor
int lastBrake = 0;
int brakeNoise = 6;                   // jitter in hall effect sensor analog value -- ouch!
                                       // 30+ cts  seems awfully noisy;  maybe we can smooth?
const int numBrakeReads = 10;          // start here:  with 10 samples we can smooth to 6 cts of
int brakeReads[numBrakeReads];         // noise between averages (pathetic)  what a lousy cheap sensor
int brakeReadPtr = 0;
int brakeTot = 0;
int brakeAvg = 0;
//                                     // even after smoothing it's ridiculously noisy, as you see
//                                     // (20 cts of jitter!)
int brakeMin = 9500;                     // these values empirically determined
int brakeMax = 13850;
int tval = 0;


const int numSensors=3;
int reeds[numSensors]; 



//-------------------------------------------------------------------
//
//    *** ROTATION TELEMETRY
//
// in theory we only need 2 sensors but in practise the reed switch can bounce
// -- you can get 2 edges for one pass of the magnet.  so I disallow "self" as
// the previous edge...  so you can't use "re encounter self" to indicate a 
// change of direction.  therefore you need 3 switches.  switches are cheap.
//
  int sensorVal;
  int lastval[numSensors];
  long lastoff[numSensors];
  long laston[numSensors];
  int lastrpm[numSensors];
  // float maxRPM = 0.0;
  float rpmCap = 240;
  int lastedge = 0;
  int directionMatrix[numSensors][numSensors];
  int spindir;
  int lastdir;
  int timeout;
  bool rpmBuffer = 0;

  int motionTimeout = 3500;              // how long must elapse between edges to conclude that we are stopped?

  // boxcar averaging : size of buffer s/b considered carefully, as a large buffer 
  // favours acceleration and masks deceleration.  10 is definitly too soft, big hysteresis
  // 8 may even be too much.  but you need something to counteract the occasional spikes.
  // now that I'm including all sensors, not just reed0, 6 seems good enough smoothing.
  
  const int numReadings = 3;         // boxcar history size:  average the last three sensor values
  float readings[numReadings];      // the readings from the analog input
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
//  boolean Stopped = 1;

// ================  unused Calibration setup ====================
  boolean calibrated = 0;
  long calibrateBegin = 0;
  int calTime = 5000;
  int lastRpm;

//  boolean logThrottle = 0;     // method by which to map throttle input.  1 means log.
  float throtFactor = 0.75;      // default is .75
  float throtRange = .70;
  
// throtRange default value is .70 -- Leo1 sets it via trellis button and potval.
// here's how it works.  working range is 20 to maxRPM (200), diff is 180.
// we multiply that diff by throtRange and add it to minRPM.
// .50 would give a range of 20 to 90+20, or 20 to 110 (very easy)
// so the calculation for the value map is (minRPM + ((maxRPM - minRPM) * throtRange))
// (Leo2 does this) so we can stretch the range and make it harder to go fast.
//-------------------------------------------------------------------

// *** EUROBIKE SPECIFIC DATA STRUCTURE
// ***
//
// goto destinations:  nice bike rides.  push a button and teleport to the start point.
// our pot setting method should allow us to choose among 11 rides.  it would be nice
// to do this by configuration file, incorporating an SD card reader into the gizmo.
// and to have an unlimited ride menu -- scrolling through on display.

char *rides[] = { "-29532.7;50.963;27231.3;3.10388;-0.0189607",
       "-10385.2;5.25531;-56288.3;0.397946;-0.0189382",
       "-13262.6;53.1382;20004.9;-3.05266;-0.0190093",
       "-69321.1;7.90307;-93818.3;3.06417;-0.124047",
       "-42607.9;43.9425;37947.7;2.25146;-0.109437",
       "-45018;94.3745;-51746.7;0.459837;0.00802962",
       "44940.3;15.1884;-108305;2.30959;0.00702866",
       "-47950.2;34.809;36218.6;2.7468;-0.00187658",
       "-10688.8;14.8132;-47160.5;-2.55101;-0.00185663",
       "53708.5;-17.7884;-104711;1.45764;-0.00185776",
       "38823.6;61.8028;-86857.7;-0.570165;-0.00283337"};
char *ridenames[] = { "FRENCH FOREST", 
       "BERGEN TUNNELS", 
       "BERN/MILANO ALPS",
       "ICELAND SOUTH",
       "PYRENEES",
       "SCOTLAND",
       "KIRKENES DIRT",
       "PAU/HUESCA",
       "STAVANGER",
       "MURMANSK",
       "FINLAND EPIC"};

//
//  Lost options:  Mouse reverse Y (settings)
//  High beams h   Wipers?  but I never use cab view.
//  Wipers are mapped to a joy button in controls_osx.sii (game file).
//

char keyMap1[15] = {
  'i',             // Ign
  'x',             // TA Page 'x'
  'b',             // Brake  'b'
  KEY_UP_ARROW,             // Shift Up  KEY_UP_ARROW
  KEY_ESC,             // ESC
  // end row 1
  'l',             // Lights 'l'
  'h',             // Hi Beams 'h'
  '1',             // Cab View  '1'
  0x01,             // NULL (go tomorrow)
  0x01,             // NULL (fix weather)
  // end row 2
  KEYPAD_9,             // Photo  KP9?  KEYPAD_9
  'z',             // TA zoom 'z'
  '5',             // Bike View  '5'
  KEY_DOWN_ARROW,             // Shift Down  KEy_DOWN_ARROW
  'm',             // Map  'm'
  // end row 3
};

char keyMap2[20] = {
  'c',             // Signal Left  'c'
  '.',             // TA Mode?  (fix page 2)
  'x',             // TA Page  'x'
  'z',             // TA Zoom 
  'v',             // Signal Right  'v'
  // end row 1
  'i',             // Ign
  't',             // Trailer 't'
  'b',             // Brake 'b'
  KEY_UP_ARROW,             // Shift Up
  KEY_ESC,             // ESC
  // end row 2
  'l',             // Lights  'l'
  0x01,             // Hazard Spinner -- special, mapped to a joy button
  'y',             // Mirror  'y'
  0x01,             // NULL (Go tomorrow)
  0x01,             // NULL (Fix weather)
  // end row 3
  KEYPAD_9,             // Photo
  'h',             // BLANK  (high beam?)  'h'
  ' ',             // Camera Cycle  (space)
  KEY_DOWN_ARROW,             // Shift Down
  'm',             // Map
  // end row 4

};


char *keyMaps[2] = {
  keyMap1,
  keyMap2,
};

boolean enterKey=0;       // state of Enter key (on or off)                                    
