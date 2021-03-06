//
// Leonardo-based Physical Game Controller (game bike, game walker etc).
//
// this is Leo 1, the core controller.  It shows up as HIDF under OSX (in Arduino IDE menus)
//
// This is a purely diagnostic sketch for Leo 1.  It writes to the serial monitor & blinks LEDs.
// That is all it does.  It's intended strictly for testing the i2c bus & attached sensors/devices.
// I decided to make a separate sketch rather than obfuscate and clutter the 
// production code with masses of ifdefs.
//
const char compile_date[] = __DATE__ " " __TIME__;


// ***
// ****
// *****
// ******
// ******* ENOUGH VERBIAGE.  CODE BEGINS HERE
// ******
// *****
// ****
// ***

// ************************************************************************************************
// *** INCLUDES, GLOBALS AND CONSTANTS
// ************************************************************************************************

//
// we need Wire to talk to the nunchuck over i2c. and to talk to Leo 2.
// we need Keyboard to send HID keyboard events.
// we need Joystick to send HID game controller events (that's Leo2)
// we need Hirzel's library to interact with the nunchuck -- it reads pitch and roll
// EasyTransferI2C enables us to share a struct with a slave duino
// Adafruit 7 seg LED library, Adafruit Trellis keypad library
// Leo2 uses TM1637 simple 7seg library to drive a dumb breakout
//

// *** BASIC DUINO LIBRARIES

#include <Wire.h>
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

// ***
// *** definitions for the AS5601 Rotary Hall sensor for steering (an i2c device)
// ***
#include <Encoder.h>
Encoder AS5601enc(18, 19);
// these values are absolutes, i,e, -90 to 90 on the bars;  but we would like more sensitive steering
// steerfactor should be settable by a panel switch!
float steerFactor = .70;
int minlimRotaryHall = 1083;
int maxlimRotaryHall = 1883;
// centre value is 1483, we allow rotation of +400 and -400
int maxRotaryHall = maxlimRotaryHall;
int minRotaryHall = minlimRotaryHall;
int steerVal = 0;
int rawSteerVal = 0;
int lastSteer = 0;

// ***
// ***  definitions for Adafruit GFX
// ***
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
// initialise a fancy adafruit 7 seg led display
Adafruit_7segment matrix = Adafruit_7segment();

// ***
// ***  definitions for Adafruit Trellis 4x4 light-up keypads
// ***
#include "Adafruit_Trellis.h"
// we have 2 of these keypads.  wired up correctly they appear as a continuous sequence of button numbers
// to the adafruit library
Adafruit_Trellis matrix0 = Adafruit_Trellis();
Adafruit_Trellis matrix1 = Adafruit_Trellis();
#define MOMENTARY 0
#define LATCHING 1
// set the mode here
#define MODE MOMENTARY 
Adafruit_TrellisSet trellis =  Adafruit_TrellisSet(&matrix0, &matrix1);
#define NUMTRELLIS 2
#define numKeys (NUMTRELLIS * 16)

int trellisPress=-1;    // -1 is the default or No Press value
unsigned int trellisDown = 0;   // bit mask that shows if a trellis key is being held down

// ***
// ***  This is the init for the LinkSprite, just in case you want to use one
// ***
/*
#include "Adafruit_MCP23017.h"
Adafruit_MCP23017 mcp;
*/
// but this version doesn't, so we don't have to do all this:
/*
const byte  mcp_address=0x20;      // I2C Address of MCP23017 Chip
const byte  GPIOA=0x12;            // Register Address of Port A
const byte  GPIOB=0x13;            // Register Address of Port B
const byte  IODIRA=0x00;            // IODIRA register
const byte  IODIRB=0x01;            // IODIRB register 
*/

// ***
// *** now prepare to share data over I2C with Leo2.
// ***

#include "I2C_Anything.h"

//
// I think this struct is limited to 28 bytes max size
struct __attribute__ ((packed)) SHARE_DATA {
  //put your variable definitions here for the data you want to send/receive 
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  float rpm = 0;      // Leo2 writes this and the master reads it
  float potval = 0.0; // Leo2 writes this and master reads it
  int steer = 0;      // master shares with Leo2 the steering sensor position
                      // which could be from chuck or from linear Hall depending on
                      // mode switch (Leo1)
  int mask = 0;       // master shares with Leo2 some switch status if needed
};

//give a local name to the blob of shared data
SHARE_DATA share_data;

// ***
// *** define slave i2c address
// *** this is the address of Leo2;  there can only be one master on the i2c bus and it is Leo1
// ***
#define I2C_SLAVE_ADDRESS 8
const byte LEO2 = 8;       // i2c address of Leo2, Joystick controller

//
// NB: Wire, Keyboard and Mouse are stock Ardu libs
// Hirzel, Adafruit, I2C_Anything are contribs.
// Mega hat tip and kudos and general warm fuzzies to Arduino community, 
// a generous & talented bunch of hackers.
//
// ***
// *** if you turn this on you can generate verbose debug msgs using DEBUG_PRINT
// ***
//#define DEBUG

// NOTE:  ALWAYS use the F macro to wrap any string literals in Serial print and println statements.  Why?
//        'cos too many unwrapped literals leads to the dreaded "low memory" message and possibly a bricked duino
//        upon upload.  You have been warned :-)

#ifdef DEBUG
#define DEBUG_PRINT(str)    \
   Serial.print(millis());     \ 
   Serial.print(F(": "));    \
   Serial.print(__FUNCTION__);     \
   Serial.print(':');      \
   Serial.print(__LINE__);     \
   Serial.print(' ');      \
   Serial.print(str); \
   Serial.print(F(" ")); \
   Serial.print(rpm); \
   Serial.print(F(" ")); \
   Serial.print(invertSpinDir); \
   Serial.print(F(" ")); \
   Serial.print(rpm); \
   Serial.print(F(" ")); \
   Serial.print(walkThresh); \
   Serial.print(F(" ")); \
   Serial.print(runThresh); \
   Serial.print(F(" ")); \
   Serial.println(potVal);
#else
#define DEBUG_PRINT(str)
#endif
//
//
// ***
// ***    -- PIN ins and outs
// ***
//
// pins D0-D3 are dedicated to USB and I2C.
// mainboard  lights are on 4 5 6 7
const int bluLight = 4;
const int grnLight = 5;
const int yloLight = 6;
const int redLight = 7;
const int whtLight = 8;
const int hidSwitch = 9;

// D10, D11, D12 unused.  D13 native onboard LED.

const int dispSelSw = 14;
const int realMouseSw = A0;
const int invMYsw = A1;
const int startEngSw = A2;
const int escKeySw = A3;

// hidSwitch is a safety feature that gets your mouse and kbrd back if Leo1 goes nuts
// it will stop HID transmission.

// *** 
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
char *ridenames[] = { "French Forest", 
       "Bergen Tunnels", 
       "Bern/Milano Alps",
       "Iceland South",
       "Pyrenees",
       "Scotland",
       "Kirkenes Dirt",
       "Pau/Huesca",
       "Stavanger",
       "Murmansk",
       "Finland Epic"};
int rideCt = 11;


// ***
// *** never used this, place holder for future feature
// ***
float showvals[11];     // place holder for one day displaying various values in 7seg

// actually I think a subsequent version would use an LCD backlight panel instead of 7 seg disp.
// or even a pixel addressable panel with graphics.

// ***
// ***   -- LIGHTS 
// ***
// Leo2 uses 3 generic leds to show the leading edge of each sensor plus a stop light
// Leo2 uses 4 rainbow leds to show speed
// Leo1 also has 4 rainbow leds and uses them to echo the speed:
//    blue is slow, green is normal, yellow is fast, and red is stopped.
// both Leos use their LEDs for a little confirmation display during boot,
//      so user knows they are alive and sane:  this is done by LEDs and 7 seg.
// the onboard LED light is hidden after the duino is packaged in a box, so it becomes another
// panel LED I guess -- it can be doubled to the front panel
 
const int ledLight = 13;  // this is the built in LED light (debug only)

// *** SPEED
// *** Parameters for speed computation
// *** we may no longer need these as Leo2 is doing this now.
const int maxRPM = 96;     // that is pedaling all out
const int minRPM = 20;      // dead slow
float rpm = 0.0; 
float slowThresh = .25 * maxRPM;
float medThresh = .50 * maxRPM;                
float fastThresh = .75 * maxRPM;                
boolean Stopped = 1;
float potVal;
int magEnc = 750;

boolean invertSpinDir = 0;    // this only applies to the portable stepper version
boolean showPotVal = 0;       // another future feature
//

//    -- for the chuck lib
//       parameters for reading the joystick:
//    only applies if you want to use the wiichuck as a mouse replacement
  int chuckRange = 40;                // output range of X or Y movement
  int chuckThresh = chuckRange/10;    // resting threshold
  int chuckCentre = chuckRange/2;     // resting position value
  long lastChuck = 0;                 // when did we last read it?
  
  
//
//   GAME CONTROL
//
                                        
  uint8_t keyPressed = 0;
  int usbHIDsending = 1;

  boolean invertMouseY = 0;           // need this for intuitive game look-around control
                                      // actually you don't.  ETS2 has this option.
  boolean realMouse = 0;              // this s/b True if we have a real usb mouse plugged in
                                      // (but is not yet used anywhere)
  boolean noTraffic = 1;              // this is an ETS2 settings variable


// ***
// *** HOUSEKEEPING
// ***

  long lastI2C = 0;                   // last time we did an i2c transfer
  long lasTrellis = 0;                // last time we checked the trellis for input
  boolean epicFail = 0;               // this is our distress flag if we encounter a
                                      // condition we just cannot handle

//
// finally!  done with definitions.  Time for setup.
//
// ***
// ****
// *****
// ******
// ******* MAIN consists of setup() and loop()
// ******
// *****
// ****
// ***
//
//

// ************************************************************************************************
// ***
// *** SETUP
// ***
                                  
void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(2000);    // give yourself 2 whole sec to start the serial monitor

  Serial.println(F("INIT input/output pins"));
  
  // INIT PINS 
  
  pinMode(hidSwitch,INPUT_PULLUP);

// short of pins, we are using A0-3 as digital inputs for 4 buttons on panel.
// these are (in order)
  pinMode(A0,INPUT_PULLUP);     // yellow button:  "." or "cycle trip advisor display"
  pinMode(A1,INPUT_PULLUP);     // blue button:  invert mouse y (chuck) (unused at this time)
  pinMode(A2,INPUT_PULLUP);     // green button:  start engine (I)
  pinMode(A3,INPUT_PULLUP);     // red button:  ESC (escape to main menu)

  // we are also using the spi header pin 14
  // for the mysterious "select what to display in 7 seg led" switch.
  // this switch is hidden underneath the panel, for debug use only...
  // and is NOT YET IMPLEMENTED.
  pinMode(14,INPUT_PULLUP);
  
  for (int i=bluLight; i<=whtLight; i ++) {
  pinMode(i,OUTPUT);
  }
  // roll those pins
  for (int i = bluLight; i<= whtLight; i++) {
    digitalWrite(i,HIGH);
  }
  for (int i = bluLight; i<= whtLight; i++) {
      digitalWrite(i,LOW);
  }

  Serial.println(F("Pin init complete, blink blue light 4x then wait 1/2 sec"));
  
  blinky(bluLight,4);   // Yes I live!
  
  delay(500);            // waiting for Leo2 to wake up, among other things
  
  // the code below is not what you want.  
  // this does in fact wait for the serial MONITOR to be connected
  // http://forum.arduino.cc/index.php?topic=322260.0
  // This waits and does nothing until the serial port is opened. 
  // It's useful for demonstration and debugging purposes, so you can see the 
  // initial serial output, but it means that it won't actually run until you 
  // open the serial monitor window.  Which you don't, in production.

//  if (!Serial) {
//       goCatatonic(0);       // most severe error, not even a Serial connection: red
//  }
  
  // Serial.println(F("Yippee yi yay!"));

  Serial.println(F("Pins are initialised, now test i2c bus"));
  
// join i2c bus as master but only if we see pullup voltage on pins 2 and 3
  int d2 = digitalRead(2);
  int d3 = digitalRead(3);
  if (!(d2&d3)) {
    Serial.println(F("OUCH, no I2C bus power."));
    goCatatonic(1);       // next most severe, no i2c bus power:  yellow
  }
  blinky(grnLight,3);
  Serial.println(F("I2C bus is OK, start Wire library and wait 1/2 sec"));
  
  Wire.begin();
  delay(500);
  

// now start up the i2c steering sensor

  Serial.println(F("Init rotary Hall effect (steering) sensor"));
  
  bool i2cOK = testI2C(0x36);
  if (!i2cOK) {goCatatonic(3);}   // next most severe:  individual slave missing:  blue
  initRotaryHall();
  
  blinky(grnLight,3);
  
// now start up the numeric display so we can say "Init" and other things
// we will hang here forever if we don't get a response from matrix.
 Serial.println(F("Init 7 segment matrix"));
 
  i2cOK = testI2C(0x71);
  if (!i2cOK) {goCatatonic(3);}   // next most severe:  individual slave missing:  blue

  blinky(grnLight,3);
  
 // init 7 segment LED matrix (i2c again)
  matrix.begin(0x71);
  blinky(grnLight,3);
  
  matrixInitMsg();
  
  Serial.println(F("Init 2 Trellis Keypads"));

  i2cOK = testI2C(0x76);
  if (!i2cOK) {goCatatonic(3);}   // next most severe:  individual slave missing:  blue
  i2cOK = testI2C(0x77);
  if (!i2cOK) {goCatatonic(3);}   // next most severe:  individual slave missing:  blue
  
  trellis.begin(0x76, 0x77);
  // do a highly visible victory roll on the trellis LEDs
  for (uint8_t i=0; i<numKeys; i++) {
    trellis.setLED(i);
    trellis.writeDisplay();    
    delay(50);
  }
    // then turn them off
  for (uint8_t i=0; i<numKeys; i++) {
    trellis.clrLED(i);
    trellis.writeDisplay();    
    delay(50);
  }

// init port expander

   blinky(bluLight,6);

  // init pot val
  // potVal = analogRead(gainPot)/1023.0;
  // not any more, Leo2 owns this pot now.

  //
  // INIT Mouse and Keyboard HID functions 
  //
  blinky(yloLight,3);  
  Serial.println(F("Mouse and keyboard begin"));
//  Mouse.begin();
//  delay(1);
  Keyboard.begin();
  delay(1);

  blinky(yloLight,3);

  //
  // init adafruit keypad 1
  // init adafruit keypad 2

  // total number of i2c devices we will be talking to:  5
  // Leo 2
  // adafruit keypads (2, can be treated as 1)
  // i/o expander board
  // magnetic rotary encoder
  // led 7 seg display
  
// notes on writing 7 segment messages
// Location #0 is all the way to the left, location #2 is the colon dots 
// location #4 is all the way to the right. 
// If you want a decimal point, call writeDigitNum(location, number, true) 
//    which will paint the decimal point. 
// To draw the colon, use drawColon(true or false)
// If you want even more control, you can call writeDigitRaw(location,bitmask) 
// to draw a raw 8-bit mask (as stored in a uint8_t) to that location.
//  matrix.print(0.0 0);
//  matrix.writeDisplay();

  blinky(redLight,4);
  
  matrixDoneMsg();
  // give us a chance to read it
  delay(2000);
  
  DEBUG_PRINT("INIT COMPLETE");

  // this is the point at which we would check to make sure that Leo2 is online...
  // and if it is not, we would loop waiting for it (10 sec should do it), 
  // blinking a status light.
  // if it did not come online w/in 10 seconds we would light up a red FAIL light
  // and set a fail flag and do an empty main loop.
  // aargh we should be doing this for all our i2c slaves.  more thought needed.
    
   Serial.println(F("Lastly, check for Leo2 on bus -- it's had time to boot up"));

   i2cOK = testI2C(LEO2);
   if (!i2cOK) {goCatatonic(3);}

//  lastly set steering limits
  setSteeringScale();
   
  blinky(grnLight,6);    // show a green status light for Leo1, yay!
  Serial.println(F("*** DONE with setup! ***"));
  
} 

// ************************************************************************************************
// ***
// *** LOOP
// ***

void loop() {
  
  // put your main code here, to run repeatedly:
  
  // if something has gone horribly wrong, yell for help and do nothing.
  // right now nothing triggers this... it's just there in case we need it someday.
  if (epicFail) {goCatatonic(0);}

//  handing off rpm indicator to Leo1, I guess we will do without these for now;
//  a multi line display would come in handy
//  digitalWrite(bluLight,invertMouseY);
//  digitalWrite(yloLight, realMouse);
  
  // Leo2 reads this pot for us now
  // potVal = analogRead(gainPot)/1023.0;

  // ctrlMode = analogRead(modeSwitch);
  // Serial.println(ctrlMode);
  // we use these strings to build debug messages.  clumsy, oh well.
  
  String msg = "foo";
  String msg2 = "bar";
  int tilt = 0;

  int switchMask = 0;
  
  long now = millis();

  if (Stopped) {
    
  // "Stopped" at this point means "stopped pedalling".  The truck can still be rolling.
  
  // Oct 23 '17:  now this is disturbing.  when I press startEngSw I'm getting both the startEng 
  // and realMouse messages.  when I press the realMouse (yellow) switch I get nothing.  so
  // something has gone wrong with that little switch matrix.  It can be replaced (modular)...
  // or maybe 2 wires shorted underneath?
  
  int val = digitalRead(startEngSw);
  if (!val) {
  Serial.println(F("Saw Engine Start Switch"));
  delay(100);
  
  }

  //  this should have a status light
  //  I am temporarily stealing this switch and mapping it to '.'
  //  which in turn is mapped to "cycle through trip advisor modes"
  if (!val) {
  val = digitalRead(realMouseSw);
  Serial.println(F("Saw TA mode cycle switch"));
  delay(100);
  }

  val = digitalRead(escKeySw);
  if (!val) {
    Serial.println(F("Saw ESC switch"));
    delay(100);
  }
  }
  
// this will read the i2c rotary encoder and set rawSteerVal and steerVal

    readRotaryHall();

 // do the i2c transfer stuff every 1/10 sec
 // I think we are having timing issues now.
 // reducing the polling of switches via i2c (above) and
 // changing the rpm indicator lights to local vs i2c
 // seems to have improved the update speed...
 // keep an eye on this when we add yet more i2c devices!
 // 
 
    if ((now - lastI2C) > 100) {
       
    // Serial.println(F("I2C time!"));
    // read data from slave, get current rpm
    getLeo2data();
    
    // Serial.print(F("Got Leo Data?  RPM: "));
    rpm = share_data.rpm;
    potVal = share_data.potval;
    if (rpm == 0) {
      Stopped = 1;
      // Serial.println(F("Stop"));
    } else {
      // Serial.println(F("Start"));
      Stopped = 0;
      showPotVal = 0;           // when moving, we ALWAYS show rpm??
    }
    // Serial.println(rpm);
    delay(2);
    
    // now overwrite slave data with our updated values
    // not sure leo 2 really needs to know ctrlMode, since we are making the steering
    // decisions;  but that could change.
    if (steerVal != lastSteer) {
      Serial.print(F("New Steering Value.... raw: "));  Serial.print(rawSteerVal);  Serial.print(F("   cooked: "));  Serial.println(steerVal);
      lastSteer = steerVal;
    }
    
    share_data.steer = steerVal;
    share_data.mask = 12345;
    putLeo2data();
    
    // Serial.println(F("Put Leo Data"));
     
    // update display  with received rpm OR with potVal depending on which is
    // selected.
    
    if (showPotVal) {
      matrix.print(potVal);
    } else {
      matrix.print(rpm);
    }
    matrix.writeDisplay();
    displaySpeed();
    lastI2C = now;
    
    }
    
 // done with i2c update

   if ((now - lasTrellis) > 200) {
      readTrellis();
      lasTrellis = now;
   }
 
  }
  
  
// ************************************************************************************************
// FUNCTIONS
// ************************************************************************************************

// high level ETS2 specific functions
// about those backspaces:  hey, I dunno.  when I do it from the keyboard,
// it works.  when I do it via HID, for some reason a spurious sgl quote is
// prepended to the ext after the backquote that gets me into the dev con.
// ETS2 then complains that "'warp .21" is not a valid command.
// ETS2 is doing this -- xev does not show the spurious char.
// tore hair for about 30 minutes and gave up.
// turns out that I can delete the invasive backspace before entering my cmd.
// who knows what evil lurks in the heart of ETS2 devcon?


void setWarp(float val) {
  Serial.print(F("Set Warp to "));
  Serial.println(val); 
}

void goTomorrow() {
  Serial.println(F("Set time to Tomorrow 4am"));
}

void fixWeather() {
  Serial.println(F("Set weather to improve gradually"));
}


void goPlace(float val) {
  int ptr = intify(val);
  // don't accept values off end of array!
  if (ptr >= rideCt) {ptr = rideCt - 1;}
  String coords = rides[ptr];
  String rname = ridenames[ptr];
  Serial.print(F("GOTO place# ")); Serial.print(ptr); Serial.print(F(" : "));
  Serial.print(rname); Serial.print(F(" ("));
  Serial.print(coords);  Serial.println(F(")"));
  
}

void goTime(float val) {
  //  multiplier has values 0 through 10
  // so we multiply it by 2 and add to 3 am.  
  // getting 0300 0500 0700 0900 1100 1300 1500 1700 1900 2100 2300
  // cmd is g_set_time 5  g_set_time 15  etc
  int multiplier = intify(val);
  int hours = 3 + 2 * multiplier;
  String hourstr = String(hours);
  Serial.print(F("Set time to : ")); Serial.println(hourstr);
}

void toggleTraffic() {
  if (noTraffic) {
    noTraffic = 0;
  } else {
    noTraffic = 1;
  }
  Serial.print(F("Set NoTraffic flag to "));  Serial.println(noTraffic);
}

int intify(float val) {
  // get a percentage type value 0.0 to 1.0 and turn it into
  // an int:  you get values 0 through 10 from your pot.
  // if I were cleverer I would supply another param N that determines
  // how many values in one pot turn.
  int v = round(val * 10);
  return(v);
}

//
//// User notification
//
void matrixInitMsg () {
  matrix.writeDigitRaw(0,B00110000);      // i
  matrix.writeDigitRaw(1,B01010100);      // n
  matrix.writeDigitRaw(3,B00110000);      // i
  matrix.writeDigitRaw(4,B01111000);      // t
  matrix.writeDisplay();
}
void matrixDoneMsg () {
  matrix.writeDigitRaw(0,B01011110);    // d
  matrix.writeDigitRaw(1,B01011100);    // o
  matrix.writeDigitRaw(3,B01010100);    // n
  matrix.writeDigitRaw(4,B01111001);    // e
  matrix.writeDisplay();
}


// 
////  HID  functions:  key presses, joystick inputs, other axes etc.
//

void areWeSending () {
   
   int val = digitalRead(hidSwitch);
 
  // pull this value to g round when switch is engaged.
  // so 0 = true (send HID events) and anything g reater is false (don't send) 
  if (val) {
    if (usbHIDsending) {
      usbHIDsending = 0;
      Serial.println(F("HID emulation OFF"));
      digitalWrite(whtLight,HIGH);          // warning light (s/b red but oh well)
    } 
  } else {
    if (! usbHIDsending) {
      usbHIDsending = 1;
      Serial.println(F("HID emulation ON"));
      digitalWrite(whtLight,LOW);
    }
  }
  
}

void mouseDance(int steps) {
  if (!usbHIDsending) {
    return;
  }
  for (int i=1; i< steps; i++) {
    Mouse.move(1,1,0);
    delay(10);
  }
  for (int i=1; i< steps; i++) {
    Mouse.move(-1,-1,0);
    delay(10);
  }
  
}


void getLeo2data() {

  Wire.requestFrom(I2C_SLAVE_ADDRESS, sizeof(share_data));
  I2C_readAnything(share_data);
  
}

void putLeo2data() {
  
  Wire.beginTransmission(I2C_SLAVE_ADDRESS);
  Wire.write ((uint8_t*) &share_data, sizeof(share_data));
  Wire.endTransmission();
  
}


void blinky(int which, int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(which,HIGH);
    delay(100);
    digitalWrite(which,LOW);
    delay(150);
  }
}

/*
void blinky2(int which, int times) {
  for (int i = 0; i < times; i++) {    
    mcp.digitalWrite(which,HIGH);
    delay(100);
    mcp.digitalWrite(which,LOW); 
    delay(150);
  }
}
*/

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

void goCatatonic(int which) {
  // you can't recover from this condition.  just blink a light and hope for rescue.
  // which can be 0 through 3 -- counting downward from Red.
  while(1) {
    blinky((redLight-which),2);
    delay(500);
  }
}

void readTrellis() {

  // we will be in momentary mode, I think;  game saves state, not us
  // If any button was just pressed or released...
  // now we have to honour the switches.
  // no chording!  we only accept *one switch at a time*
  // last one wins!
  // selected buttons can be held down for continuous action:  godcam ctrls QWEASD, and ENTER :
  // see trellisExec
  
  if (trellis.readSwitches()) {
    trellisPress = -1;
    // go through every button
    for (uint8_t i=0; i<numKeys; i++) {
    // if it was pressed, turn it on
      if (trellis.justPressed(i)) {
        Serial.print(F("v")); Serial.println(i);
        trellis.setLED(i);
        if (!trellisDown) {
        trellisPress = i;
        }
      } 
    // if it was released, turn it off
      if (trellis.justReleased(i)) {
        Serial.print(F("^")); Serial.println(i);
        trellis.clrLED(i);
        Keyboard.releaseAll();
        trellisDown = 0;
      }
    }
    // tell the trellis to set the LEDs we requested & exec the appropriate command 
    trellis.writeDisplay();
    
  }
  
}


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
    Serial.print(F("Rotary Hall Encoder Qres: "));
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

void displaySpeed() {

  
  if (rpm < slowThresh) {
    digitalWrite(redLight,HIGH);
    digitalWrite(grnLight,LOW);
    digitalWrite(yloLight,LOW);
    digitalWrite(bluLight,LOW);
  }
  if (rpm >= slowThresh && rpm < medThresh) {
    digitalWrite(bluLight,HIGH);
    digitalWrite(yloLight,LOW);
    digitalWrite(redLight,LOW);
    digitalWrite(grnLight,LOW);
  }
  if (rpm >= medThresh && rpm < fastThresh) {
    digitalWrite(grnLight,HIGH);
    digitalWrite(redLight,LOW);
    digitalWrite(yloLight,LOW);
    digitalWrite(bluLight,LOW);
  }
  if (rpm >= fastThresh) {
    digitalWrite(yloLight,HIGH);
    digitalWrite(bluLight,LOW);
    digitalWrite(grnLight,LOW);
    digitalWrite(redLight,LOW);
  }
  
}

// we have:  maxlimRotaryHall, minlimRotaryHall, steerFactor (scale the range)
void setSteeringScale () {
  
// range is the actual physical range of the encoder as installed
// midpoint should in theory be "straight ahead"
// scale range to steerFactor of physical range, and set limits for lock/lock joystick
// the smaller steerFactor is, the more sensitive the steering
  int range = maxlimRotaryHall - minlimRotaryHall;
  int midpoint = minlimRotaryHall + range/2;
  maxRotaryHall = midpoint + (range/2 * steerFactor);
  minRotaryHall = midpoint - (range/2 * steerFactor);
  
}

