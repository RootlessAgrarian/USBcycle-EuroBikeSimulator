

// This is Leo2, the ETS2 (and other driving games?) side of the USB HID controller.
// BETA VERSION 2 compiled first time Apr 28 2017
// RELEASE VERSION June 20 2017
// Testing:  using external editor (vim) with IDE.
//
// Leo2 is a slave on the i2c bus.  It shows up as HIDM under OSX (in Arduino IDE menus)
// Leo 2 will do the reading of reed switches and analog inputs from the bike (everything
// except the i2c rotary hall sensor, in other words).
// So that means 3 reed switches, plus the brake proximity sensor.
// It gets the steering value via I2C from Leo1, which is monitoring the rotary Hall encoder.
// Then in normal operation it will send HID data for 3 joystick axes:  steering, brake, accelerator.
// Leo1 is also in charge of the "value pot" and its associated 2 digit 7seg display.
// It will report any changes to this value back to Leo1 via I2C.  Leo1 will keep this
// current value available and use it to set parameters chosen by the user (by button press).
// That is, if you want to set game warp speed, set the pot value (Leo2 reads it and reports to Leo1)
// then press one of the Trellis buttons (Leo1 reads the button and uses the pot value reported by Leo2
// to generate a command to send to the game engine).
//
// I2C startup gotchas:
// Note that it takes 8 seconds before the Wire handshake is complete, also that 
// the Uno (sparkfun) I used for this test resets itself twice.  
// And that Leo1 (master) may hang in init if Leo2 (slave) is not powered up first.
// So the last thing that Leo1 checks for is Leo2 being awake on the I2C bus.
//
// To recap:
// Leo1 reads rotation off the AS5601 and passes it to Leo2;
// Leo2 reads rpm and brake directly.

// Via the Joystick library we can set everything except the hat.  The hat is always 
// pointing due north for some reason (need to work on that).  X and Y axes work fine
// and so do buttons.  So for now, no hat.  It's OK.

// We will use keyboard keys (Leo1) for all the in-game commands, leaving the joystick
// buttons alone (I have them all set up for my GT Driving Force wheel).  We'll just use
// three joystick axes.

// INCLUDE:  Wire.h for i2c, I2C_Anything.h for 2 way data transfer Leo1/Leo2, Joystick.h
//           for joystick emulation, and TM1637 library for the 2 digit 7seg dumb display.
#include "USBcycle.h"
#include <Wire.h>
#include "I2C_Anything.h"
#include <Joystick.h>
#include <TM1637Display.h>

//-------------------------------------------------------------------
//
// create the basic 7 seg display (crude stuff)
//
#define CLK 14
#define DIO 15

const uint8_t SEG_DONE[] = {
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,           // d
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
  SEG_C | SEG_E | SEG_G,                           // n
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G            // E
  };

// 2 digit cheapie 7deg breakout board
TM1637Display display(CLK, DIO);

//
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

// pretty debug print lines

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
   Serial.print(rpm); \
   Serial.print(" "); \
   Serial.print(rpm); \
   Serial.print(" "); \
   Serial.println(potVal);
#else
#define DEBUG_PRINT(str)
#endif
// strings for constructing debug printfs
String msg = "foo";
String msg2 = "bar";

//-------------------------------------------------------------------
// I2C data sharing (this is not peer/peer but master/slave, and Leo2 is an I2C slave).

// prepare to share data with Leo1 over I2C using I2C_Anything lib
// note that this struct MUST be identical on each device and should probably
// be defined in a shared .h file.  for now, just make sure they match.
struct __attribute__ ((packed)) SHARE_DATA {
  //put your variable definitions here for the data you want to send/receive 
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  float rpm = 0;      // we write this and the master reads it
  float potval = 0.0; // we write this and master reads it
  int steer = 0;      // master shares with us the steering sensor position
                      // which could be from chuck or from linear hall sensor
                      // depending on ctrlMode (a Leo1 switch)
  boolean throtl = 0; // copy of logThrottle, set by Leo1 (who owns the trellis keypad)
  float throtr = 0.0; // copy of throtRange, which stretches throttle min max, set by Leo1
};

//-------------------------------------------------------------------

// all this also could be in a shared .h file
float rpm = 0.0;
const int maxRPM = 200;     // that is superhuman pedaling 
const int minRPM = 20;      // dead slow!
float slowThresh = .30 * maxRPM;
float medThresh = .50 * maxRPM;                
float fastThresh = .75 * maxRPM; 

int swMask = 0;                         // in case we need to know switch readings from master
int steerVal = 0;                       // from master, one way or another (ctrlMode)
int lastSteer = 0;
int brakeVal = 0;                     // getting this (real bike version) from A5, hall proximity sensor
int lastBrake = 0;
int brakeNoise = 20;                   // jitter in hall effect sensor analog value -- ouch!
                                       // 30+ cts  seems awfully noisy;  maybe we can smooth?
const int numBrakeReads = 10;          // start here:  with 10 samples we can smooth to 6 cts of
int brakeReads[numBrakeReads];         // noise between averages (pathetic)  what a lousy cheap sensor
int brakeReadPtr = 0;
int brakeTot = 0;
int brakeAvg = 0;
//                                     // even after smoothing it's ridiculously noisy, as you see
//                                     // (20 cts of jitter!)
int brakeMin = 560;                     // these values empirically determined
int brakeMax = 880;
int tval = 0;
                                      
//give a name to the group of data we share
volatile SHARE_DATA share_data;
volatile boolean gotData = 0;  // this is set once, the first time we get data from master
volatile int dataRcvd = 0;    // this is 0 if all is well, else unexpected byte count

#define I2C_SLAVE_ADDRESS 8

// hardware definitions
// we read one brake sensor (Linear Hall), one trim pot, 3 reed switches
const int defled = 13;
const int trimPot = A0;       // Leo2 is monitoring the trim pot and updating its display via TM1637
//
const int numSensors=3;
int reeds[numSensors];

const int bluLight = A1;    // using our analog pins for output may be creating the noise on A5
const int grnLight = A2;    // even though A4 is almost never lit
const int yloLight = A3;
const int redLight = A4;

const int brakePin = A5;

const int rled0 = 4;    // yellow 1
const int rled1 = 5;    // yellow 2
const int rled2 = 6;    // yellow 3
const int whtLED1 = 7;  // white

const int sw1 = 8;      // it is now our responsibility to detect reverse direction
const int reed0 = 9;
const int reed1 = 10;
const int reed2 = 11;

const int stopLED = 12; // red
const int whtLED2 = 13;  // ganged to the now invisible LED 13 on board
const int diagLED = whtLED1;

float potVal = 0.0;
int potPct = 0;

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
  boolean Stopped = 1;

  boolean calibrated = 0;
  long calibrateBegin = 0;
  int calTime = 5000;
  int lastRpm;

  boolean logThrottle = 0;     // method by which to map throttle input.  1 means log.
  float throtFactor = 0.75;      // default is .75
  float throtRange = .70;
// throtRange default value is .70 -- Leo1 sets it via trellis button and potval.
// here's how it works.  working range is 20 to maxRPM (200), diff is 180.
// we multiply that diff by throtRange and add it to minRPM.
// .50 would give a range of 20 to 90+20, or 20 to 110 (very easy)
// so the calculation for the value map is (minRPM + ((maxRPM - minRPM) * throtRange))
// (Leo2 does this) so we can stretch the range and make it harder to go fast.
//-------------------------------------------------------------------

//
////
////// SETUP
////
//

void setup() {

  Serial.begin(9600);           // start serial for output
  delay(1000);

  zeroBrake();
  
  pinMode(trimPot,INPUT);       // A0 is where we read the hall effect sensor for brake action

  pinMode(brakePin,INPUT);      // A5 is where we read the hall effect sensor for brake action

  pinMode(sw1,INPUT_PULLUP);

  
  // turn off all led lights

  pinMode(stopLED,OUTPUT);
  
  digitalWrite(stopLED,LOW);

  // basic coloured LEDs
  for (int i = A1; i <= A4; i++) {
    pinMode(i,OUTPUT);
    digitalWrite(i,LOW);
  }
  
  for (int i=4; i<=7; i++) {
    pinMode(i,OUTPUT);
    digitalWrite(i,LOW);
  }

  blinky(stopLED,6);

  // reed switches when closed connect pin to ground
  pinMode(reed0,INPUT_PULLUP);
  pinMode(reed1,INPUT_PULLUP);
  pinMode(reed2,INPUT_PULLUP);
  
  reeds[0] = reed0;
  reeds[1] = reed1;
  reeds[2] = reed2;

  // what would this switch do?
  // pinMode(offSwitch,INPUT_PULLUP);
  
  // tell the world we are alive and struggling towards consciousness
  blinky(diagLED,6);
  delay(100);
  
  // 
  // now try to join the I2C bus and hope that the bus master is awake by now
  //
  log_print(F("Join I2C bus"),1);
  // join i2c bus as slave but only if we see pullup voltage on pins 2 and 3
  int d2 = 0;
  int d3 = 0;
  while(1) {
    d2 = digitalRead(2);
    d3 = digitalRead(3);
    if (!(d2&d3)) {
      log_print(F("OUCH, no I2C bus power."),1);
      blinky(stopLED,6);
    } else {
      log_print(F("I2C bus power is OK, join bus as slave #8"),1);
      break;
    }
  delay(100);
  }
  blinky(diagLED,6);
  Wire.begin(I2C_SLAVE_ADDRESS);                // join i2c bus with address #8
  // there may be a long delay here
  Wire.onReceive(receiveEvent); // register event for receive from master
  Wire.onRequest(requestEvent); // register event for request from master
  // the master will set the timing for shared data exchanges
  // so we should never get interrupted again in the middle of our ISR

  //
  // startup the joystick emulation
  //
  log_print(F("Begin Joystick emulation!"),1);
  Joystick.begin();
  delay(100);
  
  // set value ranges for accelerator (Xaxis, or axis 0 to ETS2) and steering (axis 1)
  // now adding Brake (Z axis)... map oddball brake values to 0-255
  Joystick.setXAxisRange(0,255);
  Joystick.setYAxisRange(-180,180);
  Joystick.setZAxisRange(0,255);
  // Joystick.setHatSwitch(0,JOYSTICK_HATSWITCH_RELEASE);
  delay (100);

  log_print(F("Init telemetry"),1);
  initTelemetry();

  lastBrake = analogRead(brakePin);
  
  digitalWrite(diagLED,LOW);
  //
  
}

//-------------------------------------------------------------------

//
////
////// LOOP
////
//

void loop() {

//  Leo 2 owns the Hall proximity sensor on the brake cable.
//  first, check the brake.
    checkBrake();

//  next, read the trim pot
    int potRaw = analogRead(trimPot);
    potVal = potRaw / 1023.0;               // percentage, between 0 and .99 or so
//  for some reason turning this pot up interferes with RPM -- still scratching head
//  over this one.  for now, the fix is to turn the pot down to 0 after setting a 
//  user param.

  // Serial.print("POT RAW: "); Serial.println(potRaw);
  // Serial.print("POT VAL: "); Serial.println(potVal);

// then read the reed switches to determine rpm
// we are using the rainbow LEDs to show reed switch actuation because one of our
// set of monochrome (yellow) LEDs is dead.  So the LEDs are grn ylo red for reeds 0 1 2

   for (int i = 0; i < numSensors; i++ ) {
    int val = readSensor(i);
    if (val >= 0) {
      // only write to LEDs when value changes -- value is -1 for no change.
      if (val) { 
        laston[i] = millis();
        digitalWrite(grnLight+i, HIGH) ;
 //       Serial.print("Reed ");  Serial.print(i); Serial.println(" ON");
      } else {
        lastoff[i] = millis();
        digitalWrite(grnLight+i, LOW) ;
 //        Serial.print("Reed ");  Serial.print(i); Serial.println(" OFF");
      }
    }
 
  }
  
  // write our contribution to shared data after reading
  rpm = average;

  long now = millis();
  
  /*
    Serial.print("Received: ");
    Serial.print(rpm);
    Serial.print(" , ");
    Serial.print(steer);
    Serial.print("   @ ");
    Serial.println(now);
  */
   
// Y axis:  steering
// if steering has changed, respond;  Leo1 will send us steering values
// beta version steering looks jerky ... need to smooth it out somehow... (done)
  
  if (steerVal != lastSteer) {
  // Serial.print(F("New Steer Val:  setYaxis = "));
  // Serial.println(steerVal);
  Joystick.setYAxis(steerVal);
  lastSteer = steerVal;
  }

// Z axis:  brake
// the Hall proximity sensor is super jittery for whatever reasons, it's fluctuating
// by 30 counts or more;  we hide this filtering in the brakeRead function
// we preserve the raw value and convert at the last minute (only on demand) to 0-255.
// we also clip "close to zero" to prevent slight brake dragging at low values
 
  if (brakeVal != lastBrake) {
    if ((brakeVal - brakeMin) <= brakeNoise) {
    brakeVal = brakeMin;
//    Serial.println("Brakes OFF");
    }
    int newval = map(brakeVal,brakeMin,brakeMax,0,255);
    Joystick.setZAxis(newval);
  }

// X axis:  accelerator
// if rpm has changed, respond
  if (rpm != lastRpm) {
    tval = 0;         // time for a new throttle value
    // we only honour throttle values gt minRPM which right now is set to 20
    // at rpm lower than this, it's hard to figure out what is going on.
    if (rpm >= minRPM) {
      Stopped = 0;
      // map raw FP rpm to integer joystick value 0-255, with dynamic range adjust set by user
      // default range is 20 to 200, so 180 rpm difference.
      // throtRange of .50 would yield a range of 20 to 110 (20 + diff * .50)
      // .70 would be 20 to 146  (default)
      // .90 would be 20 to 182  (challenging!)
      // so the bigger this number is,the harder you have to work for a given throttle setting.
      // so... that determines your cadence when you are cruising at 60 kph on the flat 
      // for example.  
      float maxr = minRPM + ((maxRPM - minRPM) * throtRange);
      float mrpm = rpm;
      // and here we do a shameless hack to goose the acceleration, to overcome the mass of
      // the truck (someday we'll have telemetry to assist in this algorithm, right now we
      // just brute-force it.
      if (logThrottle && (aDelta > 1.0) && (rpm < 55)) {
            // mrpm = rpm * 3.0;
            // floor it!
            mrpm = maxr;
      }
      tval = map(round(mrpm),minRPM,maxr,0,255);

      constrain(tval,0,255);
      Joystick.setXAxis(tval);

    /*
    log_print(F("RPM = "),0);
    Serial.print(rpm);
    Serial.print(F("    MRPM = "));
    Serial.print(mrpm);
    Serial.print("   TVAL = ");
    Serial.print(tval);
    Serial.print("   STEER = ");
    Serial.println(steerVal);
    */
    
    } else {
      digitalWrite(bluLight,HIGH);    // we've stopped pedalling
    }
  }
  
// when you have done all the really important stuff, update the pot readout
// every .20 seconds or so

 if (!(now % 200)) {
  potPct = round(potVal * 100);           // strictly for display: x100 and round it to int
  // Serial.print("POT INT: "); Serial.println(potPct);
  uint8_t data[] = { 0xff, 0xff, 0xff, 0xff };
  display.setBrightness(0x0f);
  // display.setSegments(data);
  delay(5);
  display.showNumberDec(potPct, false);
  delay(5);
  }

  // there is something very odd about the pot.  if it is set to max value .99, then
  // rpm goes to zero and stays there despite pedalling.  
  // if it is set to min value 0 then rpm seems correct.
  // if it is set in between somewhere then rpm repeatedly and randomly drops to zero 
  // and then recovers, the dropouts being more frequent at higher values.
  // not sure if this means that the reeds are not being read, or that i2c is being 
  // interfered with... or...?  
  // I notice that the reed edge detect LEDs are still blinking when the dropout happens,
  // so we are detecting the reed switches OK.  more research needed.
  
  // lastly check for I2C data
  
  // always do this after first data received.
  if (!gotData) {
    // Serial.println(F("No data from Leo1"));
    return;
  }

  if (dataRcvd) {
    log_print(F("I2C Error, expected "),0);
    Serial.print(sizeof(share_data));
    Serial.print(F("bytes from Leo1 but got "));
    Serial.println(dataRcvd);
  }
  
  // we turn off interrupts very, very briefly so that we aren't trying to read
  // share_data when the interrupt routine is writing it.
  noInterrupts();
  dataRcvd = 0;
  steerVal = share_data.steer;
  logThrottle = share_data.throtl;
  throtRange = share_data.throtr;
  interrupts();
  
  // note that swMask is not actually being used, but if it were used it could pass
  // switch settings from Leo1 to Leo2 and vice versa.

  // note that we can ONLY read the elements we do NOT write.
  // also that the master controls the timing, so there should be no conflict.

  
// END OF MAINLOOP

}

//-------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------

//-------------------------------------------------------------------
// function that executes whenever data is received from master
// this function is registered as an event, see setup()
// in EasyTransfer, this function is null
 
void receiveEvent(int howMany) {
    // requestEvent is confused with data without this test
    if(howMany==sizeof(share_data)) {  
      I2C_readAnything(share_data);
      gotData = 1;
      // Serial.print(F("Data Received from Leo1: steerval "));
      // Serial.println(share_data.steer);
      dataRcvd = 0;
    } else {
      // Serial.print(F("Wrong number of bytes received.  Expecting "));
      // Serial.print(sizeof(share_data));
      // Serial.print(F(" but got "));
      // Serial.println(howMany);
      dataRcvd = howMany;
    }
    
}

void requestEvent() {
  
  // Serial.print("Data Requested by Leo1, rpm = ");
  // Serial.println(rpm);
  share_data.rpm = average;
  share_data.potval = potVal;
  Wire.write ((uint8_t*) &share_data, sizeof(share_data));
  
}

//-------------------------------------------------------------------

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
//-------------------------------------------------------------------

void zeroBrake() {
    // null out brake boxcar array
  for(int thisReading = 0; thisReading < numBrakeReads; thisReading++) {
    brakeReads[thisReading] = 0;
  }
}
//-------------------------------------------------------------------

void zeroBoxcar (int i) {
  //Serial.print("ZERO BOXCAR:  ");
  //Serial.println(i);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  total = 0;
  rpmBuffer = 0;
  readIndex = 0;
}
//-------------------------------------------------------------------

int readSensor(int i) {

    // In this version, the reed pins are set INPUT_PULLUP and the switches pull them LOW.
    // values are 1 vs 0 and we invert them so that closed = True.
    // 
  int retval = 0;

  //val = analogRead(i);
  sensorVal = digitalRead(reeds[i]);
 
  // reverse sense so 1 is true and 0 is false
  sensorVal = !sensorVal;
  
  // 0 now becomes HIGH and HIGH becomes 0, so we can stay intuitive
  long now = millis();
  float rpm = 0.0;
  
  // has state changed?  if so, it's either a rising or falling edge.
  // we count rising edges
  
  if (sensorVal !=  lastval[i]) {
  /*
  Serial.print("Sensor change on ");
  Serial.print(i);
  Serial.print(" = ");
  Serial.println(sensorVal);
  */
    if (Stopped) {digitalWrite(stopLED,LOW);}
    Stopped = 0;
    retval = sensorVal;
 
  // is it a leading edge event (went high)?  if so, compute rpm
  
  if (sensorVal) {
    
    // delta is in millisec
    // for now, compute rpm ONLY on reed0
    // we are sensor i, who was last sensor?
    // ignore it if it was "self", as we do get bounce from time to time
    // update direction 
    // if we just changed direction then force average rpm to 0

    if (lastedge != i) {
      
      spindir = directionMatrix[i][lastedge];
      /*
      Serial.print("This edge ");
      Serial.print(i);
      Serial.print ("; last edge ");
      Serial.print(lastedge);
      Serial.print("  --->   Spin dir: ");
      Serial.println(spindir);
      */
      if (spindir != lastdir) {
        log_print(F("*** CHANGE DIR: "),0) ;
        Serial.print(lastedge);  Serial.print(F(" --> "));  Serial.print(i);
        Serial.print(F(" lastdir "));  Serial.print(lastdir); 
        Serial.print(F(" now ")); Serial.println(spindir);
         zeroBoxcar(i);
         zeroBoxcar(0);
      }
      lastdir = spindir;
    }

    // now we get to the heart of the matter:  calculate rpm
    
    long tdelta = now - laston[i];
    rpm = (1000.0 / tdelta) * 60.0;
    /*
    Serial.print("Reed");
    Serial.print(i);
    Serial.print("  Last on: ");
    Serial.print(laston[i]);
    Serial.print("  NOW: ");
    Serial.print(now);
    Serial.print("  Delta: ");
    Serial.print(tdelta);
    Serial.print("   --->  Raw RPM = ");
    Serial.println(rpm);
    */
 
    // filter out bogus rpm (switch bounces? causing super high values)
    if (rpm <= rpmCap) {
        updateRpm(rpm * spindir);
        // adelta is change in averaged rpm.  neg if falling, pos if rising
    }
    lastedge = i;
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
      log_print(F("Timeout on "),0);
      Serial.print(i); Serial.print(F(" ")); Serial.print(dt);
      Serial.print(F(" > ")); Serial.println(timeout);
      stopTheWorld(i); 
    }
    }
    retval = -1;
  } // end of else block, no edge detected
 
  return (retval);
  
}

//-------------------------------------------------------------------

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
      log_print(F("STOP DETECTED"),1);
      Joystick.setXAxis(0);
      digitalWrite(rled0,LOW);
      digitalWrite(rled1,LOW);
      digitalWrite(rled2,LOW);
      Stopped = 1;
      digitalWrite(stopLED,HIGH);
      }
      
}

//-------------------------------------------------------------------

void updateRpm(float rpm)  {

      // do boxcar averaging of rpm, record average, step buffer
      /*
      Serial.print("updateRpm got rpm ");
      Serial.println(rpm);
      */
      // subtract the last reading (which should be zero on first iteration):
      total = total - readings[readIndex];
      // read from the sensor:
      readings[readIndex] = rpm;
      // add the reading to the total:
      total = total + readings[readIndex];
      /*
      Serial.print("New total: ");
      Serial.println(total);
      */
      // do not try to average rpm till you have at least numReadings in buffer.
      if (!rpmBuffer) {
        // Serial.println("Don't have 3 readings yet");
        readIndex = readIndex + 1;
        if (readIndex == numReadings) {
          readIndex = 0;
          rpmBuffer = 1;
        }
        return;
      }
      // calculate the average:
      average = total / numReadings;
      // log_print(F("New Average RPM"),1);

//      if (average > maxRPM) { maxRPM = average;}
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


//-------------------------------------------------------------------
// not actually using this right now.
// place holder for future feature allowing user to recalibrate rpm range

boolean checkCalibration() {

  // we only get in here if calibration < 2
  // so it can be only 0 or 1
  
 if (!calibrated) {
    digitalWrite(rled0,HIGH);
    digitalWrite(rled1,HIGH);
    digitalWrite(rled2,HIGH);
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
//      if (maxRPM > 0) {
//      maxRunSpeed = maxRPM * 1.10;
//      runThresh = (maxRunSpeed * .60);
//      walkThresh = (maxRunSpeed * .25);
//      }
      digitalWrite(rled0,LOW);
      digitalWrite(rled1,LOW);
      digitalWrite(rled2,LOW);
      
      log_print(F("Done with calibration."),1);

      return (1) ;
    }

  }
  
//-------------------------------------------------------------------

// read the brake sensor and if it has changed by more than Noise, update saved value
// otherwise do nothing
// I am not happy with this sensor or with this code, but guess it can be refined later.
// the analog signal is noisy as hell and my smoothing is very crude.  the axis response
// is rather digital encoder like, steppy.  but it works well enough for a beta.

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



//-------------------------------------------------------------------
//-----------------------------END-----------------------------------
//-------------------------------------------------------------------

