//
////
//////================================ SETUP =================================
////
//


void setup() {

//// INIT SERIAL
// start 2 serial ports
// one for Nextion (Serial1) the other for Programming Port

  Serial.begin(9600);
  Serial1.begin(19200);

  delay(1000);   // give some time to start Serial Monitor in IDE

//
//// INIT PINS
//
// initialise pins first, so we can drive those LEDs
// this just inits the reeds array, which is silly -- FIX LATER
  initSensorData();

  pinMode(TONEpin, OUTPUT);
  pinMode(emuSwitch,INPUT_PULLUP);  // this switch turns off all HID output;  gets pulled to ground
  pinMode(cTouch,INPUT); // this is HIGH true.

// initialise the LED lines
  for (int i = 37; i < 52; i+=2) {
    pinMode(i,OUTPUT);
    digitalWrite(i,1);
    delay(50);
    digitalWrite(i,0);
    delay(50);
  }
// initialise the reed switch input lines -- they pull to ground when closed
  for (int i = 0; i < 3; i++) {
    pinMode(reeds[i],INPUT_PULLUP);
  }
  // blinky(ylo1,3);   // and so it begins!

  log_print(F("BEGIN INIT"),1);
  showMem(1);

  // we will init Joystick here also when the time comes
  log_print(F("Keyboard..."),1);  
  Keyboard.begin();       // start Kbrd HID emulation
  log_print(F("Joystick..."),1);
  Joystick.begin();
  delay(10);
   // set value ranges for accelerator (Xaxis, or axis 0 to ETS2) and steering (axis 1)
  // now adding Brake (Z axis)... map oddball brake values to 0-255
  Joystick.setXAxisRange(0,255);
  Joystick.setYAxisRange(-180,180);
  Joystick.setZAxisRange(0,255);
  // Joystick.setHatSwitch(0,JOYSTICK_HATSWITCH_RELEASE);
  
  delay(100);

// ** LIGHT 1
  digitalWrite(ylo1,1);   // and leave the first light on.


//
////
////// ===== INIT I2C BUS 
////
//

  // workaround for known i2c bus hang 
  // which can happen if you reboot during an i2c handshake...
  // notify the slaves that the bus is alive again.
  // sort of an "ahem" from the bus master.
  pinMode(21, OUTPUT);
  for (int i = 0; i < 8; i++) {
    digitalWrite(21, HIGH);
    delayMicroseconds(3);
    digitalWrite(21, LOW);
    delayMicroseconds(3);
  }
  delay(10);
  // now let's just check and see if all is well
  pinMode(20,INPUT);
  pinMode(21,INPUT);
//  blinky(ylo2,3);
  bool foo = digitalRead(20);
  if (!foo) {
    Serial.println(F("Warning:  pin 20 SDA doesn't read High, check pullups?"));
  }
  foo = digitalRead(21);
  if (!foo) {
    Serial.println(F("Warning:  pin 21 SCL doesn't read High, check pullups?"));
  }
  // IRL we would stop dead here and blinky for help if these lines are low
// ** LIGHT 2
  digitalWrite(ylo2,1);   // and leave the 2nd light on

// now it is time to start the i2c bus.
 
  Wire.begin();
  delay(500);        // to let the bus settle

//
////
//////  ================ ROLL CALL OF I2C SLAVES ==============
////
//
// check for existence of i2c slaves
// there should be 4 slaves on bus:
//   rotary Hall encoder (AS 5601),
//   ADC (ADS1115),
//   FRAM,
//   Chronodot RTC

  // blinky(ylo3,3);
  
  boolean i2cOK;
  i2cOK = testI2C(0x36);
  if (!i2cOK) {
    Serial.println(F("WARNING, can't talk to Rotary Encoder!"));
  } else {   // next most severe:  individual slave missing:  blue
    Serial.println(F("  ...Rotary Hall Encoder AS5601"));
    initRotaryHall();
  }
  
  i2cOK = testI2C(0x48);
  if (!i2cOK) {
    Serial.println(F("WARNING, can't talk to ADC i2c device!"));
  } else {
        Serial.println(F("  ...16 bit ADC ADS1015"));
  }
  i2cOK = testI2C(0x50);
  if (!i2cOK) {
    Serial.println(F("WARNING, can't talk to FRAM i2c device!"));
  } else {
        Serial.println(F("  ...Adafruit FRAM device"));
  }
  i2cOK = testI2C(0x68);
  if (!i2cOK) {
    Serial.println(F("WARNING, can't talk to Chronodot RTC device!"));
  } else {
        Serial.println(F("  ...Chronodot RTC device"));
  }

// *** LIGHT 3
  digitalWrite(ylo3,1);  // and leave the 3rd light on
  
// ===============================  NOW INIT DEVICES


  
//  blinky(wLED,3);
  Serial.println(F("Initializing Chronodot."));
  RTC1.begin();
  if (! RTC1.isrunning()) {
    Serial.println("  RTC is NOT running!\n  Set date/time from sketch.");
    // following line sets the RTC to the date & time this sketch was compiled
    // but once you have initialised, it should be OK.
     RTC1.adjust(DateTime(__DATE__, __TIME__));
  } else {
    Serial.println(F("  RTC is running..."));
  }
  
  digitalWrite(wLED,1);    // white LED is RTC
  rtc_check();  
  rtc = RTC1.now();
  tzero = rtc.unixtime();
  delay(500);

  Serial.println(F("Initialising FRAM"));
  fram.begin(0x50);
  fram_check();

  digitalWrite(bLED,1);   // blue LED is FRAM
  delay(500);

//  blinky(rLED,3);
  Serial.println(F("Initialising ADC"));
  ads.begin();
  digitalWrite(gLED,1);   // green LED is ADC

// STATE REFRESH?

  // here we might read out some data from FRAM, like... the Rides list maybe.  
  // User preferences.  Cumulative statistics.  State.  Whatever.
  
  start_time = millis();
  randomSeed(millis());

//// ==================================== INIT NEXTION

// Let's all get on the same page :-)
// there is no test here to make sure the nextion is connected.
// Add this to the TODO list!  We need "expect" :-)
// how about a global nexti_expect set to the hex code we
// would see -- 0x66 for a page change
// it could be -1 when we are not expecting.

  delay(500);
  Serial1.print("page 0");
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  nexti_page = 0;
  // nexti_expect = 0x66;
  delay(100);
  // damn this expect mechanism will not work with a subsequent command
  // messing up the buffer before we have a chance to read.  more thought needed.
  // pity it is not an ISR...

  // This is a hard coded version number which should probably be more clever.
  // I update it when I have a more or less stable release or a new feature.
  // YY.MM.DD
  
  Serial1.print("p[0].b[1].txt=\"18.12.15\"");
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  digitalWrite(yLED,1);    // yellow LED is Nextion
  
  delay (2000);   // pause so we can notice the lights

  // showMem again to see if we leaked any
  
  showMem(1);

  // do a victory roll on the coloured LEDs

  log_print(F("LED victory roll..."),1);
  
  for (int i = 37; i < 52; i+=2) {
      digitalWrite(i,LOW);
  }
  delay(250);
  for (int i = 37; i < 52; i+=2) {
    digitalWrite(i,1);
    delay(25);
    digitalWrite(i,0);
    delay(25);
  }
  for (int i = 51; i > 36; i-=2) {
    digitalWrite(i,1);
    delay(25);
    digitalWrite(i,0);
    delay(25);
  }
  
  log_print(F("Init telemetry"),1);
  initTelemetry();
  // we might not need all this smoothing if the ADC works.
  zeroBrake();
  
  log_print(F("RESTORE STATE"),1);
  
  stateRestore();
  // stateErase();            // only do this if you've mucked up your FRAM
  State.val.rideCt = 11;      // it doesn't hurt to do this always, whether you erased or not
                              // ugly though.  ride save and restore NEEDS WORK.
  
  Serial.println(F("Play a tune..."));
  playTune(piezoPin, 300);
  
  Serial.print(F("ridePtr = "));  Serial.println(State.val.ridePtr);

  looptick = millis();
// you need these the first time you ever start up.
//  State.val.rideCt = 11;
//  State.val.ridePtr = 0;

// always start with zero values
State.val.RPM = 0;
State.val.Throt = 0;
State.val.Brake = 0;
State.val.Steer = 0;
statePush();

    
  // CONGRATULATIONS, YOU ARE NOW READY TO RUN LOOP
  
}

