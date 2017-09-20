//
// Leonardo-based Physical Game Controller (game bike, game walker etc).

// WORKING BETA1 saved Apr 27 2017, preparing for next big step.
// BETA VERSION 2 (current) compiled first time Apr 28 2017
//
// this is Leo 1, the core controller.  
//
// Version 1:  It reads the reed switches to monitor rotation
//   and functions as a keyboard/mouse for WASD control;  it also reports rpm to a slave
//   Leo 2 which functions as a joystick and sends appropriate axis commands to driving 
//   or other games.
// Version 2 (current):  Leo 1 functions as i2c bus master.  Leo 2 reads reed switches
//   and shar es rpm data with Leo 1 via I2C_Anything.  Leo 2 runs reed edge detect LEDs.
//   Leo 1 functions as kbrd/mouse, controls most function lights & push buttons
//   Leo 2 reads bike telemetry (except steering which is i2c).
//   ETS2 control is done via keyboard function map with keystrokes (thus Leo 1, i2c button
//   matrix reader, can handle almost all ETS2 interactions).
//   Leo 2 only provides joystick axis input to ETS2 (accel, brake, and steering).
//   A Mode switch defines system behaviour:  steer by chuck vs steer by bike
//   MAJOR CHANGE:  split one-Leo WASD version completely from joystick 2-Leo version.
//                  this cleans up a lot of mess.
//                  install 16 port I/O expander shield (I2C) on Leo1.  Reassign all pins.
//
// Leo 1 is now equipped with a LinkSprite i2c i/o port expander adding 16 digital i/o ports.
// This should enable it to handle all the LED indicator lighting, function buttons, etc.
//
//   NEW PIN MAP  May 17 2017:
//    Native Analog:  used as digital in
//            A0:    yellow button (real mouse)
//            A1:    blue button (invert mouse Y)
//            A2:    green button (start engine (i))
//            A3:    red button (ESC key)
//    Native Digital:
//            D0-3:     (usurped by i2c and usb function)
//            D4-8:     LEDs blue green yellow red white
//            D9:       hid kill switch
//            D10-12:   unused

// Brief History:
// First Running v2 Apr 29 4pm, everything working including 2 way i2c!
// Apr 30 dev con cmds seem to be working
//
// ETS2 game settings:  BikeView mod, warp .6, rich-start profile (provided)
// user settings provided in final version via physical buttons:
//  set warp  (pot plus button)
//  turn traffic on/off
//  change weather to fair
//  change clock to tomorrow morning (set time by pot?)
//  goto selected excellent bike rides on the map

// ENOUGH VERBIAGE.  CODE BEGINS HERE

// LIBRARIES
//
// we need Wire to talk to the nunchuck over i2c. and to talk to Leo 2.
// we need Keyboard to send HID keyboard events.
// we need Joystick to send HID game controller events (that's Leo2)
// we need Hirzel's library to interact with the nunchuck -- it reads pitch and roll
// EasyTransferI2C enables us to share a struct with a slave duino
// Adafruit 7 seg LED library, Adafruit Trellis keypad library
// Leo2 uses TM1637 simple 7seg library to drive a dumb breakout
//
#include <Wire.h>
#include <Keyboard.h>
#include <Mouse.h>

// you need these defs in order to send any keypad keys via Keyboard.press(val)
// and you need keypad keys for my ETS2 control map; though I suppose I could 
// eliminate them I prefer to leave the keymap familiar in case I revert to using
// a usb keyboard at any time.
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

// definitions for the Rotary Hall sensor for steering (AS5601)
#include <Encoder.h>
Encoder AS5601enc(18, 19);
int minRotaryHall = 1000;
int maxRotaryHall = 1980;
int steerVal = 0;
int rawSteerVal = 0;
  
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
// initialise a fancy adafruit 7 seg led display
Adafruit_7segment matrix = Adafruit_7segment();

#include "Adafruit_Trellis.h"
// and a fancy adafruit 16 button keypad or two
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

// #include "HirzelWiiChuck.h"

// define chuck object
// WiiChuck chuck = WiiChuck();
#define MAXANGLE 90
#define MINANGLE -90
// int angleStart, currentAngle;
// double angle;

// at present we are NOT using an i2c i/o expander because we can just barely squeeze
// all the ports onto 2 leos and would like to avoid any extra i2c overhead.  the trellis
// devices are quite expensive enough to monitor.
// an earlier version used the i2c i/o expander shielf from LinkSprite
// be aware that the silkscreen is wrong on these boards and the GPIOA ports
// are labelled backwards, i.e. D7 is really D0 and vice versa.  other than that
// it tested out just fine.  And it works with Adafruit's generic library, which is kewl.
/*
#include "Adafruit_MCP23017.h"
Adafruit_MCP23017 mcp;
*/
// which means that we don't have to do all this:
/*
const byte  mcp_address=0x20;      // I2C Address of MCP23017 Chip
const byte  GPIOA=0x12;            // Register Address of Port A
const byte  GPIOB=0x13;            // Register Address of Port B
const byte  IODIRA=0x00;            // IODIRA register
const byte  IODIRB=0x01;            // IODIRB register 
*/


// now prepare to share data over I2C with Leo2.

#include "I2C_Anything.h"

//
// I think this struct is limited to 28 bytes
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

//define slave i2c address
// this would be the address of Leo2
#define I2C_SLAVE_ADDRESS 8
const byte LEO2 = 8;       // i2c address of Leo2, Joystick controller

//
// NB: Wire, Keyboard and Mouse are stock Ardu libs
// Hirzel, Adafruit, I2C_Anything are contribs.
// Mega hat tip and kudos and general warm fuzzies to Arduino community, 
// a generous & talented bunch of hackers.
//
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
   Serial.print(rpm); \
   Serial.print(" "); \
   Serial.print(invertSpinDir); \
   Serial.print(" "); \
   Serial.print(rpm); \
   Serial.print(" "); \
   Serial.print(walkThresh); \
   Serial.print(" "); \
   Serial.print(runThresh); \
   Serial.print(" "); \
   Serial.println(potVal);
#else
#define DEBUG_PRINT(str)
#endif
//
//
//    -- PIN ins and outs
//
// mainboard  lights are on 4 5 6 7
const int bluLight = 4;
const int grnLight = 5;
const int yloLight = 6;
const int redLight = 7;
const int whtLight = 8;
const int hidSwitch = 9;

const int dispSelSw = 14;
const int realMouseSw = A0;
const int invMYsw = A1;
const int startEngSw = A2;
const int escKeySw = A3;

// hidSwitch is a safety feature that gets your mouse and kbrd back if Leo1 goes nuts

// we need some more switches (about 6 I think).  the trellis devices give us 32,
// but a lot of those are dedicated ETS2 functions.
//
//
// goto destinations

char *rides[] = { "-29532.7;50.963;27231.3;3.10388;-0.0189607",
       "-10385.2;5.25531;-56288.3;0.397946;-0.0189382",
       "-13262.6;53.1382;20004.9;-3.05266;-0.0190093",
       "-69321.1;7.90307;-93818.3;3.06417;-0.124047",
       "-42607.9;43.9425;37947.7;2.25146;-0.109437",
       "-45018;94.3745;-51746.7;0.459837;0.00802962",
       "44940.3;15.1884;-108305;2.30959;0.00702866"};
char *ridenames[] = { "France 1", 
       "Bergen", 
       "Bern/Milano",
       "Iceland South",
       "Pyrenees",
       "Scotland",
       "Kirkenes Dirt"};
int rideCt = 7;


float showvals[11];     // place holder for one day displaying various values in 7seg

//
//    -- LIGHTS
//
// Leo2 uses 3 generic leds to show the leading edge of each sensor plus a stop light
// Leo2 uses 4 rainbow leds to show speed
// Leo1 also has 4 rainbow leds and uses them to echo the speed
// both Leos should also have some kind of calibration code, a little confirmation display
//      so user knows they are alive and sane:  this is done by LEDs and 7 seg
// the onboard LED light is not useful after the unit is packaged, so it becomes another
// panel LED I guess
 
const int ledLight = 13;  // this is the built in LED light (debug only)


// Parameters for speed computation
// we may no longer need these as Leo2 is doing this now.
const int maxRPM = 96;     // that is pedaling all out
const int minRPM = 20;      // dead slow
float rpm = 0.0; 
float slowThresh = .25 * maxRPM;
float medThresh = .50 * maxRPM;                
float fastThresh = .75 * maxRPM;                
boolean Stopped = 1;
float potVal;
int magEnc = 750;

boolean invertSpinDir = 0;
boolean showPotVal = 0;
//

//    -- for the chuck lib
//       parameters for reading the joystick:
  int chuckRange = 40;                // output range of X or Y movement
  int chuckThresh = chuckRange/10;    // resting threshold
  int chuckCentre = chuckRange/2;     // resting position value
  long lastChuck = 0;                 // when did we last read it?
  
  
//
//   GAME CONTROL
//

// do we have a real bike or a pseudo bike?
// if real bike then steering is read from Hall Effect
// if fake bike then steering is by wiichuck.
// two different releases of the code; it gets messy otherwise.
                                        
  uint8_t keyPressed = 0;
  int usbHIDsending = 1;

  boolean invertMouseY = 0;           // need this for intuitive game look-around control
  boolean realMouse = 0;              // this s/b True if we have a real usb mouse plugged in
  int chuckRoll;                      // for chuck steering
  int lastRoll;                       // for detecting nonlinearities in chuck tilt
  
  boolean noTraffic = 1;              // this is an ETS2 settings variable

  long lastI2C = 0;                   // last time we did an i2c transfer
  long lasTrellis = 0;                // last time we checked the trellis for input
  boolean epicFail = 0;               // this is our distress flag if we encounter a
                                      // condition we just cannot handle

//
// finally!  done with definitions.  Time for setup.
//

//
// MAIN
//
                                  
void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);    // give yourself a sec to start the serial monitor
  
  // INIT PINS 
  
  pinMode(hidSwitch,INPUT_PULLUP);

// short of pins, we are using A0-3 as digital inputs for 4 buttons on panel.
// these are (in order)
  pinMode(A0,INPUT_PULLUP);     // yellow button:  real mouse
  pinMode(A1,INPUT_PULLUP);     // blue button:  invert mouse y (chuck)
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
  blinky(bluLight,4);   // Yes I live!
  
  delay(500);            // waiting for Leo2 to wake up, among other things
  
  // the code below is not what you want.  
  // this does in fact wait for the serial MONITOR to be connected
  // http://forum.arduino.cc/index.php?topic=322260.0
  // This waits and does nothing until the serial port is opened. 
  // It's useful for demonstration and debugging purposes, so you can see the 
  // initial serial output, but it means that it won't actually run until you 
  // open the serial monitor window.

//  if (!Serial) {
//       goCatatonic(0);       // most severe error, not even a Serial connection: red
//  }
  
  Serial.println("Yippee yi yay!");

  Serial.println("Pins are initialised");
  
// join i2c bus as master but only if we see pullup voltage on pins 2 and 3
  int d2 = digitalRead(2);
  int d3 = digitalRead(3);
  if (!(d2&d3)) {
    Serial.println("OUCH, no I2C bus power.");
    goCatatonic(1);       // next most severe, no i2c bus power:  yellow
  }
  blinky(grnLight,3);
  
  Wire.begin();
  delay(500);
  

// now start up the i2c steering sensor

  Serial.println("Init rotary Hall effect (steering) sensor");
  
  bool i2cOK = testI2C(0x36);
  if (!i2cOK) {goCatatonic(3);}   // next most severe:  individual slave missing:  blue
  initRotaryHall();
  
  blinky(grnLight,3);
  
// now start up the numeric display so we can say "Init" and other things
// we will hang here forever if we don't get a response from matrix.
 Serial.println("Init 7 segment matrix");
 
  i2cOK = testI2C(0x71);
  if (!i2cOK) {goCatatonic(3);}   // next most severe:  individual slave missing:  blue

  blinky(grnLight,3);
  
 // init 7 segment LED matrix (i2c again)
  matrix.begin(0x71);
  blinky(grnLight,3);
  
  matrixInitMsg();
  
  Serial.println("Init 2 Trellis Keypads");

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
  Serial.println("Mouse and keyboard begin");
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
    
   Serial.println("Check for Leo2 on bus");

   i2cOK = testI2C(LEO2);
   if (!i2cOK) {goCatatonic(3);}
   
  blinky(grnLight,6);    // show a green status light for Leo1, yay!
  Serial.println("*** Done with setup! ***");
  
} 


void loop() {
  
  // put your main code here, to run repeatedly:
  
  // if something has gone horribly wrong, yell for help and do nothing.
  // right now nothing triggers this... it's just there in case we need it someday.
  if (epicFail) {goCatatonic(0);}

  digitalWrite(bluLight,invertMouseY);
  digitalWrite(yloLight, realMouse);
  
  // Leo2 reads this pot for us now
  // potVal = analogRead(gainPot)/1023.0;

  // ctrlMode = analogRead(modeSwitch);
  // Serial.println(ctrlMode);
  // we use these strings to build debug messages.  clumsy, oh well.
  
  String msg = "foo";
  String msg2 = "bar";
  int tilt = 0;

  int switchMask = 0;
  
  // if switch is thrown, gimme my keyboard back.
  // why is this not working?
  areWeSending();

  // (hack) set mouse invert mode

  // this is an i2c interaction with the magnetic rotation encoder
  // int steerVal = getSteering();
 
  // this is an i2c interaction with the switch matrix
  // int switchMask = getSwitches();

  long now = millis();
  
  // this will be handled by switch matrix in future w/a single i2c t'fer I hope.
  // but for now we are using various switches on the test bed.
  // and for performance we will only check these sw if we are stopped.
  // hope to fix this later.
  
  if (Stopped) {

  int val = digitalRead(startEngSw);
  if (!val) {
    Keyboard.print('i');
    delay(100);
  }
  //  this should have a status light
  val = digitalRead(realMouseSw);
  if (!val) {
      realMouse = !realMouse;
      Serial.print("Set Real Mouse to:  ");
      Serial.println(realMouse);
      delay(100);
  }

  val = digitalRead(escKeySw);
   if (!val) {
    Keyboard.write(KEY_ESC);
    delay(100);
  }

  }
  
  
   // read nunchuck and update mouse position as needed
   // this sets chuckRoll, which varies from -170 to +170
   // and can be used for steering if real steering is not available.
   // ctrlMode determines what is used.  steering axis is set to range -180, +180 on Leo2
   // oh no, another button!  or could we detect this by unplugging the chuck?
   // or by detecting the mag enc?
   

// this will read the i2c rotary encoder and set rawSteerVal and steerVal
// or the i2c wiichuck

//  it doesn't make sense to maintain chuck backward compatibility.
//  forget ctrlMode!
//  this sets rawSteerVal and steerVal

    readRotaryHall();

 // do the i2c transfer stuff every 1/10 sec
 // I think we are having timing issues now.
 // reducing the polling of switches via i2c (above) and
 // changing the rpm indicator lights to local vs i2c
 // seems to have improved the update speed...
 // keep an eye on this when we add yet more i2c devices!
 // 
 
    if ((now - lastI2C) > 100) {
       
    // Serial.println("I2C time!");
    // read data from slave, get current rpm
    getLeo2data();
    // Serial.print("Got Leo Data?  RPM: ");
    rpm = share_data.rpm;
    potVal = share_data.potval;
    if (rpm == 0) {
      Stopped = 1;
      // Serial.println("Stop");
    } else {
      // Serial.println("Start");
      Stopped = 0;
      showPotVal = 0;           // when moving, we ALWAYS show rpm??
    }
    // Serial.println(rpm);
    delay(2);
    
    // now overwrite slave data with our updated values
    // not sure leo 2 really needs to know ctrlMode, since we are making the steering
    // decisions;  but that could change.
    share_data.steer = steerVal;
    share_data.mask = 12345;
    putLeo2data();
    
    // Serial.println("Put Leo Data");
     
    // update display  with received rpm OR with potVal depending on which is
    // selected.
    if (showPotVal) {
      matrix.print(potVal);
    } else {
      matrix.print(rpm);
    }
    matrix.writeDisplay();
    lastI2C = now;
    
    }
    
 // done with i2c update

   if ((now - lasTrellis) > 200) {
      readTrellis();
      lasTrellis = now;
   }
 
  }
 
  
  

// -------------------------------------------------------------------------
// FUNCTIONS
// -------------------------------------------------------------------------

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
  Keyboard.write('`');
  Keyboard.write(KEY_BACKSPACE);
  delay(100);
  Keyboard.print("warp ");
  Keyboard.print(val);
  Keyboard.println(" \n");
  delay(100);
  Keyboard.write('`');
  Serial.print("Set Warp to ");
  Serial.println(val); 
}

void goTomorrow() {
  Keyboard.write('`'); 
  Keyboard.write(KEY_BACKSPACE);
  delay(100);
  Keyboard.println("g_set_time 4 \n");
  delay(100);
  Keyboard.write('`');
  Serial.println("Set time to Tomorrow 4am");
}

void fixWeather() {
  // no code here yet
  Keyboard.write('`');
  Keyboard.write(KEY_BACKSPACE);
  delay(100); 
  Keyboard.println("g_set_weather 0 i \n");
  delay(100);
  Keyboard.write('`');
  Serial.println("Set weather to improve gradually");
}


void goPlace(float val) {
  int ptr = intify(val);
  // don't accept values off end of array!
  if (ptr >= rideCt) {ptr = rideCt - 1;}
  String coords = rides[ptr];
  String rname = ridenames[ptr];
  Keyboard.write('`');            // enter dev con
  Keyboard.write(KEY_BACKSPACE);  // why do we have to do this?  delete backquote.
  delay(50);
  Keyboard.print("goto ");
  Keyboard.print(coords);
  Keyboard.println("\n");       // go to coordinates
  delay(50);
  Keyboard.write('`');          // exit dev con
  Keyboard.press(KEYPAD_2);     // press key to teleport truck to location
  delay(10);
  Keyboard.release(KEYPAD_2);
  delay(50);
  Keyboard.write('5');    // switch back to camera 5
  Serial.print("GOTO place# "); Serial.print(ptr); Serial.print(" : ");
  Serial.print(rname); Serial.print(" (");
  Serial.print(coords);  Serial.println(")");
  
}


void toggleTraffic() {

  Keyboard.write('`');
  Keyboard.write(KEY_BACKSPACE);
  delay(100);
  if (noTraffic) {
    Keyboard.println("g_traffic 1 \n");
    noTraffic = 0;
  } else {
    Keyboard.println("g_traffic 0 \n");
    noTraffic = 1;
  }
  delay(100);
  Keyboard.write('`');
  Serial.print("Set NoTraffic flag to ");  Serial.println(noTraffic);
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

/*
void victoryRoll() {
  
  for (int i = 0; i < 2; i++) {
  mcp.digitalWrite(bluLight2,HIGH);
   delay(100);
  mcp.digitalWrite(grnLight2,HIGH);
   delay(100);
  mcp.digitalWrite(yloLight2,HIGH);
   delay(100);
  mcp.digitalWrite(redLight2,HIGH);
   delay(100);

  mcp.digitalWrite(redLight2,LOW);
    delay(100);
  mcp.digitalWrite(yloLight2,LOW);
    delay(100);
  mcp.digitalWrite(grnLight2,LOW);
    delay(100);
  mcp.digitalWrite(bluLight2,LOW);
    delay(100);
  }
  
}
*/
// 
////  HID  functions:  key presses, joystick inputs, other axes etc.
//

void areWeSending () {
   
  int val = digitalRead(hidSwitch);
  // Serial.print("Are We Sending? switch: ");
  // Serial.println(val);
 
  // pull this value to g round when switch is engaged.
  // so 0 = true (send HID events) and anything g reater is false (don't send) 
  if (val) {
    if (usbHIDsending) {
    //Keyboard.end();
    //Mouse.end();
    usbHIDsending = 0;
    Serial.println("HID emulation OFF");
    digitalWrite(whtLight,HIGH);          // warning light (s/b red but oh well)
    } 
    // Serial.println("\nKEYBOARD END");
  } else {
    if (! usbHIDsending) {
      //Keyboard.begin();
      //Mouse.begin(); 
      usbHIDsending = 1;
      Serial.println("HID emulation ON");
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
    Serial.print("  Try to transmit to address ");
    Serial.println(addr);
  Wire.beginTransmission (addr);
  byte error = Wire.endTransmission();
  if (error == 0)
  {
     Serial.print("  Found slave alive and listening on try: ");
     Serial.println(attempt);
     success = 1;
     break;
  } else {
     Serial.print("  Slave contact attempt: ");
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
  // no chording!  we only accept one switch at a time.
  // last one wins.
  
  if (trellis.readSwitches()) {
    trellisPress = -1;
    // go through every button
    for (uint8_t i=0; i<numKeys; i++) {
    // if it was pressed, turn it on
      if (trellis.justPressed(i)) {
        Serial.print("v"); Serial.println(i);
        trellis.setLED(i);
        trellisPress = i;
      } 
    // if it was released, turn it off
      if (trellis.justReleased(i)) {
        Serial.print("^"); Serial.println(i);
        trellis.clrLED(i);
      }
    }
    // tell the trellis to set the LEDs we requested
    trellis.writeDisplay();
    if (trellisPress > -1) {
    trellisExec();
    }
  }


}
 
void trellisExec() {

// these switch values match the "Cyclist" profile osx_controls.sii
// you can change the characters emitted by editing this code...
// or you can change the mappings of those characters in ETS2

  switch (trellisPress) {
    case 0:
    //    left turn signal -- C
    Keyboard.write('c');
      break;
    case 1:
    //    parking brake -- B
      Keyboard.write('b');
      break;
    case 2:
    //    trailer attach/detach -- T
      Keyboard.write('t');
      break;
    case 3:
    //    right turn signal -- V
      Keyboard.write('v');
      break;
    case 4:
    //    light mode toggle -- L
      Keyboard.write('l');
      break;
    case 5:
    //    hi beam toggle -- H
      Keyboard.write('h');
      break;
    case 6:
    //    mirror toggle -- Y
      Keyboard.write('y');
      break;
    case 7:
    //    ENTER
      Keyboard.write(KEY_RETURN);
      break;
    case 8:
    //    page through trip advisor (?)  -- x
        Keyboard.write('x');
      break;
    case 9:
    //    camera 1  -- 1
      Keyboard.write('1');
      break;
    case 10:
    //    shift up -- UpArrow
      Keyboard.write(KEY_UP_ARROW);
      break;
    case 11:
    //    take snapshot -- KP9 (have to avoid common letters 'cos it works in devcon!)
      Keyboard.write(KEYPAD_9);
      break;
    case 12:
    //    zoom trip advisor map (?) -- z
          Keyboard.write('z' ); 
      break;
    case 13:
    //    camera 5 -- 5
      Keyboard.write('5');
      break;
    case 14:
    //    shift down -- DnArrow
      Keyboard.write(KEY_DOWN_ARROW);
      break;
    case 15:
    //    map view -- M
      Keyboard.write('m');
      break;
    case 16:
    //    LH keypad, fly commands, Q
      Keyboard.write('q');
      break;
    case 17:
    //    fly commands, W
      Keyboard.write('w');
      break;
    case 18:
    //    fly commands, E
      Keyboard.write('e');
      break;
    case 19:
    //    godcam invoke -- KP1
     Keyboard.write(KEYPAD_1);
      break;
    case 20:
    //    fly commands, A
      Keyboard.write('a');
      break;
    case 21:
    //    fly commands, S
      Keyboard.write('s');
      break;
    case 22:
    //    fly commands, D
      Keyboard.write('d');
      break;
    case 23:
    //    teleport to location -- KP2
      Keyboard.write(KEYPAD_2);
      break;
    case 24:
    //    go tomorrow
      goTomorrow();
      break;
    case 25:
    //    make weather better 
      fixWeather(); 
      break; 
    case 26:
    //    set mode?  what mode?  this + rpm scale could be retarder up/down
      Keyboard.write('=');
      break;
    case 27:
    //    goto preselect ride (via pot val):  up to 11 rides in this version
      goPlace(potVal);
      break;
    case 28:
    //    SAVE -- Space?  (no I think Space is brake, in chuck mode) -- /
      Keyboard.write('/');
      break;
    case 29:
    //    traffic toggle on/off (starts traffic 0)
      toggleTraffic();
      break;
    case 30:
    //    rpm scale factor (via pot val)?  should this be retarder up/down? yup.
      Keyboard.write('-');
      break;
    case 31:
    //    set warp value (via pot val)
      setWarp(potVal);
      break;
 
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
    Serial.print("Qres: ");
// receive high byte 
  Serial.print(reading,BIN);   
  Serial.println(""); 
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
  // Serial.print("First RHall angle: ");
  rawSteerVal = reading/2;  
  // clamp values 
  /*
  if (rawSteerVal > maxRotaryHall) {
    rawSteerVal = maxRotaryHall;
  }
  if (rawSteerVal < minRotaryHall) {
    rawSteerVal = minRotaryHall;
  }
  */
  steerVal = map(rawSteerVal,minRotaryHall,maxRotaryHall,-180,180);
  Serial.print("RAW Steer: "); Serial.print(rawSteerVal);
  Serial.print("  JOY Steer: "); Serial.println(steerVal);
  
  //Serial.print("  Status: ");        // remove // if like to display the bits
  Wire.beginTransmission(0x36);       // transmit to device as5601
  Wire.write(byte(0x0b));             // sets register pointer 
  result = Wire.endTransmission();    // stop transmitting

  Wire.requestFrom(0x36, 1 );         // request 1 bytes from as5601
  reading = Wire.read();              // receive high byte
  //Serial.print(reading, BIN);       // remove // if like to display the bits
  
  if (bitRead(reading, 5)!=1) { Serial.print(" Status: "); 
  if (bitRead(reading, 4)==1) Serial.print("too high ");
  if (bitRead(reading, 3)==1) Serial.print("too low ");}
  
   //************************************************************************
// AGC  ( we don't need the AGC)
/*  
  Serial.print("AGC: ");
  Wire.beginTransmission(0x36);       // transmit to device as5601
  Wire.write(byte(0x1a));             // sets register pointer 
  result = Wire.endTransmission();    // stop transmitting

  Wire.requestFrom(0x36, 1);          // request 1 bytes from as5601
  reading = Wire.read();              // receive high byte
  Serial.print(reading, DEC);             
  Serial.println("");
 //************************************************************************
*/

  
}

