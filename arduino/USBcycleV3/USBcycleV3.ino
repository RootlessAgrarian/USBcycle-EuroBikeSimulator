//
// Due-based Physical Game Controller (game bike, game walker etc).
//
// This is a whole new version of USBcycle.  Development started
// in late Spring 2018, stopped for sailing season and other reasons,
// and started again in Fall 2018.
//
// the first version documented here
// https://www.hackster.io/Tazling/usbcycle-ride-through-your-virtual-world-8ff961
// used 2 Leonardos, a zillion switches, some fast footwork with i2c, and took up
// a lot of space.
//
// This version goes upmarket a bit (!) with 
//   one Due (NOTE this is a 3.3v Arduino which changes everything!)
//   one Nextion 4.3 inch touch screen (5v 1A, ouch)
//   one RTC module (Chronodot), 
//   one ADC to keep the noise down, 
//   one FRAM for saving state, and 
//   one level converter to glue the 5v Nextion to the 3.3v Due.
// It should accept all the connections from the existing bike.
// though the Hall sensor may have to be replaced by a 3.3v model.
// There may be an issue with the AS5601 rotary encoder which 
// runs on 5v.  Might have to adapt it to 3.3v or build a new one.
//
// The price tag is somewhat higher of course.  A Nextion 4.3 (not Enhanced) runs
// about $65 CAD.  The Chronodot alone is another $20.  A Due is not cheap at $50 US,
// $65 CAD.  OTOH there is so much less wiring and fooling about than there was
// with version 1, that the savings in labour easily pays for the spiffy new
// hardware.
//
// There is a 7 page UI on the Nextion, which replaces all the physical switches and
// buttons on the "USBcycle v2" version.  Except the ENTER button which I feel should
// be tactile (and a nice bright colour).  Or maybe a touch sensor?
// I tried to keep the Nextion dumb, *really* dumb, so that as little code as possible
// would be marooned in that proprietary environment.
// ONE Arduino now runs the whole show.
// Although this version involves more code, it's actually simpler hardware and will
// fit in a smaller package.

// Unanswered questions:
//   which port does the Due do its USB HID on?
//   ANSWER:  the second (Native) port
// Missing features:
//   * set time (not just tomorrow) in game
//     hacked in via page 6 clock-set
//   * traffic on/off
//     added to pg 3
//   * adjust screen brightness 
//     AGH -- we are out of screen real estate
//   * better fonts?
//     improved the ride list anyway
//
// This version therefore is USBcycle v3.
//
// For the first time, the code's been split into tabs to make it easier
// to edit this large sketch.  There are 3 .h files and a whole bunch of ino
// files.  You don't need to include the old USBcycle library, because it's
// now part of this sketch in the ToolBox tab.
//
// I've tried to divvy up the code by functional area.
//   First Tab:   essential declarations and instantiations and comments
//   Second Tab:  include for Nextion specific stuff
//   Third Tab:  include for USB game controller stuff
//   Fourth Tab:  include for musical scale on piezo
// -- now the real code begins --
//   Fifth Tab:  Setup function
//   Sixth Tab:  Loop function
//   Seventh and Subsequent:  function definitions, various, by category
//

// setting up for production mode, no serial output so we can unplug prog port

#define DEBUG true //set to true for debug output, false for no debug ouput

#if DEBUG == false
#define Serial if(DEBUG)Serial
#endif

const char compile_date[] = __DATE__ " " __TIME__;

//
////
//////======================  GLOBAL VARS INCLUDES DEFS AND DECLARATIONS =====================
////
//

//
// =======================================
// init for MEMORY introspection
// =======================================

#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>

extern char _end;
extern "C" char *sbrk(int i);
char *ramstart=(char *)0x20070000;
char *ramend=(char *)0x20088000;
char *heapend=sbrk(0);
register char * stack_ptr asm ("sp");
struct mallinfo mi=mallinfo();

//=========================================
// init Wire and i2c DEVICES:
// RTC, FRAM, ADC, rot hall sensor
// =======================================

#include <Wire.h>

// Just for grins shall we add an RPM direct display
// for the front panel? 
//
// RTC for keeping time
#include "Chronodot.h"
Chronodot RTC1;
DateTime rtc;
//
// FRAM for saving state
#include "Adafruit_FRAM_I2C.h"
Adafruit_FRAM_I2C fram     =   Adafruit_FRAM_I2C();
uint16_t          framAddr = 0;
//
// 16 bit ADC
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;
// warning, mod needed to ADS1015 library:  increase delay
// before read to 10ms.
//
//
// AS5601 Rotary Hall sensor for steering
#include <Encoder.h>
Encoder AS5601enc(21, 20);
// SCL,SDA (was 18,19 on Leo)
//

#define TONEpin    12                                 // piezo buzzer pin


//=========================================
// include the specialised .h files
// =======================================

#include "AA_NextionConfig.h"
#include "AB_Ctrl.h"
#include "AC_ToneScale.h"

// NOTE:  it is not recommended to use Software Serial with Nextion.  
// So on a Zero for example, one would use a dedicated SerialN port 
//  (not the programming port).
// Our Due also has a dedicated Serial1 hardware port so we are using it.

// the logic for keeping in touch with the Nextion should be something like...
// 1) read the nexti incoming buffer on every loop
// 2) whatever is to be read and move on, don't wait for complete message
// 3) write received bytes into message buffer
// 4) count contiguous FFs.  when you get three in a row, EOM.
// 5) on EOM, process Message
// 6) on processMessage check message type (event or value).  this should be stuff like:
// 7)   PAGE=n;  BUTTON_PRESS=b
// 8) dispatch routine for BUTTON_PRESS: same as for Trellis (one dispatch per page?)
//    plus a chirp or LED blink confirm, 'cos the nexti touch screen is a bit stiff.
//    We keep essential state in a global struct (see below) that everyone can see.
// do we even need PAGE=N since all events contain a PAGE marker?  why bother?
// answer:  as a sanity check.
//
// =======================================
//
////  ======== IMPORTANT State structure, heart of the machine ===============
//
// =======================================

boolean StateChange = 0;

struct systemState {
  // TIME/STAT STUFF
  int elap;     // elap time since boot, in seconds
  int hh;       // current time, hours
  int mm;       // current time, minutes
  int freemem; // last estimate of free mem
  int active;   // active pct of samples as int
  int idle;     // idle pct of samples time as int
  int avgrpm;   // avg rpm over elap time as int
  int maxrpm;   // max rpm during elap time
  int actsam;   // number of rpm/actv samples
  // USER PREFERENCES
  int rideCt;  // count of predefined rides
  int ridePtr; // pointer to current predef ride
  int warp;   // warp adjust (these three are pct * 100)
  int steerT; // steering tuning (sensitivity)
  int throtT; // throttle tuning
  boolean turbo;// turbo on/off
  boolean traf; // traffic on/off
  boolean Yinv; // invert Mouse Y (not used unless we are emulating mouse)
  boolean Rmou; // true if we have a real mouse (unused)
  // SENSOR READINGS
  int RPM;    // current smoothed RPM (previously known as "average" in v2)
  int rBrake; // current raw brake value
  int rSteer; // current raw steering value
  int Throt;  // current throttle axis value
  int Steer;  // current steering axis value
  int Brake;  // current brake axis value
};

union StateData {
systemState val;
byte bytes[sizeof(systemState)];
};

StateData State;  // current state
StateData LState; // last state, so we can check for changes


// if our compiler supports it we can just do this:  LState = State;
// if our compiler does not support that, we can do 
//   memcpy(&LState,&State,sizeof(systemState));
// memcpy dest source count

// you would use this union as follows:
// refer to state.val.elap if you want the elap int.
// but refer to state.bytes[10] if you want the 11th byte of the blob
// by this means we can SAVE our State to FRAM every so often.

// =======================================
//
////============================= TIME, TIMING, & TIMERS ==============================
//
// =======================================

// millis times (for event spacing)
long start_time;
long elap = 0;
int loopct=0;
byte loopdir=0;
long looptick=0;

// loop timer, elapsed
int timer = 0;
int last_min;
time_t tzero;
time_t prez;
bool tick;
bool epicFail=0;  
int timeDigits[4];
//
//
////  DEBUG VARIABLES, early testing, not really needed in production
//
// analog read
int rawVal = 0;
int lastVal = 0;

// ====================================================
// IMPORTANT NOTES on SERIAL setup for NEXTION
// =======================================
//
// pin 8 Uno tx, pin 7 Uno rx:
//
//   pin 8 goes to Yellow line from Nextion
//     on the Due Serial1, this would be pin 18 TX
//
//   pin 7 goes to Blue line from Nextion
//     on the Due Serial1, this would be pin 19 RX
//
// BUT remember that you MUST go through a level converter first!
// the red and black lines from Nextion go to 5v and GND.
// do not run the Nextion undervoltage, you can hurt it.
// and NEVER connect any 5v signal to a Due port, you can blow it up.
//
// Nov 16 2018 WORKING, basic sketch.  Now for the real challenge :-)
// if you must test with a Uno, use sketch NextionSanityUno.
// SoftwareSerial nextion(7, 8);  
//
// Dec 2:  i2c equipped version includes RTC (Chronodot), ADC
// for noise free analog input, FRAM for storing configuration
// and statistics.

// now load in the specialty code

// =======================================
////
////// DEFINE PINS
////
// =======================================

// DUE:  Pins 18/19 are dedicated to Serial1.
//       Pins 20/21 are dedicated to i2c.
//       For the moment I have grabbed odd numbers 33-51 for leds and other
//        front panel stuff
//       Pin D8 is attached to the piezo, in case we want beeping.
//          (there is a Tone equiv for Due).
// 

// on digital header 2
const int cTouch = 12;       // cap touch Enter switch on R side of box
const int reed0 = 11;       // reed switches on bike
const int reed1 = 9;       // for rotation sensing
const int reed2 = 10;        // they come in on RJ45 on L side of box

const int piezoPin = 8;     // pin to drive noisemaker

// on "extra" pins at bottom of board

const int emuSwitch = 35;   // switch to turn off HID
bool hidKill = 1;           // associated state var

const int ylo1 = 51;        // indicator LEDs
const int ylo2 = 49;
const int ylo3 = 47;
const int wLED = 45;
const int bLED = 43;
const int gLED = 41;
const int yLED = 39;
const int rLED = 37;

// NOTES on indicator LEDs
// they are used at boot time to show what phase of boot we are in (see AD_Setup.ino)
// and they are used at run time to indicate various conditions:
//  RED = Stopped
//  YELLOW = AS5601 gap too big
//  GREEN = AS5601 gap too small
//  BLUE =
//  WHITE = steering calibration in progress
//  YLO1,2,3 = momentary reed switch 
//

// =======================================
// CONGRATS YOU ARE NOW READY TO RUN SETUP
// =======================================




