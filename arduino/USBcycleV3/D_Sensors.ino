
//
////
////// =============== SENSORS & LEDs ===============
////
//
//
// notes on sensors:
// there are three axes we need to send:  throttle, brake, and steering.
// * throttle is a function of RPM.  RPM is measured by watching a set of 3
//    reed switches triggered by a magnet on the pedal assembly (digital)
// * brake is measured by a hall linear sensor (analog)
// * steering is measured by a rotary hall sensor (i2c)
//
// the reed switches will use up 3 digital pins.  they are low-true.
// the brake will be attached to an ADC because the Due's analog pins are
//    ridiculously noisy.
// the rotary hall sensor attaches via i2c and I hope it will run on 3.3v.


// NOTE that if you use the Adafruit library for ADS 1115 
// you should increase the CONVERSIONDELAY factor to 10 ms.  
// 8 is not enough, at least not for the Chinese Cheapies.
// We could have up to four sensors talking to this ADC.
// Brake is all we need for now.
//
// there may also be a momentary pushbutton for Enter.


// LEDS:  we have always had diagnostic LEDs for low-level sanity checking.
// at the very least, we have 3 LEDs that light up as the reed switches close.
// we've also traditionally had LEDs for monitoring the boot process.  So I
// think a set of 4 coloured LEDs and 3 monochrome LEDs would be ideal.
// these LEDs can be run on 3.3v because we don't need a lot of brightness,
// but we should avoid overtaxing the 3.3v supply.
// A 2-voltage supply for the box as a whole might be a good idea, with
// power brought to both the Nextion and the Due.
// lights should be lit immediately, not wait for a refresh cycle.

////================================================================= BRAKE
////
// ADC read for analog (Hall linear) sensor  BRAKE
////

int adcget(uint8_t which) {
  
  int16_t foo;

  if (which > 3) {return(-999);}

  foo = ads.readADC_SingleEnded(which);
  
  return(foo);
  
}

// we may not need to boxcar the brake readings with an ADC
// that would save a few cycles

void zeroBrake() {
    // null out brake boxcar array
  for(int thisReading = 0; thisReading < numBrakeReads; thisReading++) {
    brakeReads[thisReading] = 0;
  }
}

// read the brake sensor and if it has changed by more than Noise, update saved value
// otherwise do nothing
// I am not happy with this sensor or with this code, but guess it can be refined later.
// the analog signal is noisy as hell and my smoothing is very crude.  the axis response
// is rather digital encoder like, steppy.  but it works well enough for a beta.

void checkBrake() {
  
  // turn raw jittery value into a somewhat less bouncy average
  // just ignore it unless we have more than 5 cts diff.

  
  int  rawval = adcget(uint8_t(3));
  if (abs(rawval - State.val.rBrake) <= 5) {
      return;
  }
  
  LState.val.rBrake = State.val.rBrake;
  State.val.rBrake = rawval;
  StateChange = 1;
  /*
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
    State.val.rBrake = val;
 //   Serial.print("  Set LastBrake to "); Serial.print(lastBrake);
 //   Serial.print("  Set BrakeVal to "); Serial.println(brakeVal);
  }
  */
  
  // later in the code we will have to map this to 0-255
  
}


////======================================================================= STEER
////
// Rotary Hall sensor:  INIT and READ 
////


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
  log_print(F("Qres: "),0);
// receive high byte 
  Serial.print(reading,BIN);   
  Serial.println(F("")); 
}

void readRotaryHall () {

// we actually do two reads here.  the first one gets the rotation angle.
// the second checks our fly height.

  // Suppose steerFactor was .75, we would take 25 pct off the range, or
  // 12.5 pct off each end.  This would make our steering more sensitive
  // as a smaller range of motion would be mapped to the standard +/- 180
  // joystick value.
  // Larger values of steerT means more sensitivity, intuitive.
  // this is now implemented, late in the process:  UI is used to set
  // steerFactor.
  int steerRange = maxRotaryHall - minRotaryHall;
  int adjust = int(steerRange * (100 - State.val.steerT) / 200.0);   // this is zero normally, steerFactor 1.0
  int steerBegin = minRotaryHall + adjust;
  int steerEnd = maxRotaryHall - adjust;
  
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
  
  LState.val.rSteer = State.val.rSteer;
  State.val.rSteer = reading/2;  

  // clamp steering input within sensible limits
  if (State.val.rSteer > steerEnd) {
    State.val.rSteer = steerEnd;
  }
  if (State.val.rSteer < steerBegin) {
    State.val.rSteer = steerBegin;
  }

  if (LState.val.rSteer == State.val.rSteer) {
    return;
  }

  StateChange = 1;
  
  LState.val.Steer = State.val.Steer;
  // map into joystick axis val
  State.val.Steer = map(State.val.rSteer,steerBegin,steerEnd,-180,180);

  /*
  if (steerVal != lastSteer) {
  Serial.print(F("RAW Steer: ")); Serial.print(State.val.rSteer);
  Serial.print(F("  JOY Steer: ")); Serial.println(steerVal);
  lastSteer = steerVal;
  }
  */
  //Serial.print(F("  Status: "));        // remove // if like to display the bits
  Wire.beginTransmission(0x36);       // transmit to device as5601
  Wire.write(byte(0x0b));             // sets register pointer 
  result = Wire.endTransmission();    // stop transmitting

  Wire.requestFrom(0x36, 1 );         // request 1 bytes from as5601
  reading = Wire.read();              // receive high byte
  //Serial.print(reading, BIN);       // remove // if like to display the bits

  // we should light a trouble light for too high or too low, with appropriate colour
  //  we're taking over the rainbow lights for this purpose... maybe we need more?
  
  digitalWrite(gLED,LOW);
  digitalWrite(yLED,LOW);
  
  if (bitRead(reading, 5)!=1) { 
    log_print(F("AS5601 Status: "),0); 
    if (bitRead(reading, 4)==1) {
      Serial.println(F("too high "));
      digitalWrite(yLED,HIGH);
    } else if (bitRead(reading, 3)==1) {
      digitalWrite(gLED,HIGH);
      Serial.println(F("too low "));
    } else {
      Serial.println(reading, HEX);
    }
  }
  
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

void centreSteering () {
  
// range is the actual physical range of the encoder as installed
// midpoint should in theory be "straight ahead"
// scale range to steerFactor of physical range, and set limits for lock/lock joystick
// the smaller steerFactor is, the more sensitive the steering
// assume the bars are set to dead centre.
// NOTE that the initial orientation of the sensor is all-important.
// it can be re-zeroed 3x and that is it.
  readRotaryHall();
  steerCentre = State.val.rSteer;
  maxRotaryHall = steerCentre + steerLock;
  minRotaryHall = steerCentre - steerLock;
  blinky(wLED,4);
  log_print(F("Steering re-centred at raw value "),0);
  Serial.println(State.val.rSteer);
  
}


////======================================================================== THROTTLE
////
//   Reed Switches:  read, get rotation, compute rpm
////

// this is silly, but we can clean it up later.

void initSensorData() {
  
  reeds[0] = reed0;
  reeds[1] = reed1;
  reeds[2] = reed2;
  
}

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
    if (Stopped) {digitalWrite(rLED,LOW);}
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
      
      spindir = directionMatrix[lastedge][i];
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
      digitalWrite(ylo1,LOW);
      digitalWrite(ylo2,LOW);
      digitalWrite(ylo3,LOW);
      Stopped = 1;
      digitalWrite(rLED,HIGH);
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
//    log_print(F("New Average RPM"),1);
//    this is the stub for dynamic range setting
//      if (average > maxRPM) { maxRPM = average;}

      // how is it different from last avg?  neg if less, pos if more
      aDelta = abs(average) - abs(lastavg);
      float abdelta = abs(aDelta);
      
      // and set state!
      State.val.RPM = average;
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
 
}

//========================================================================
//-------------------------------------------------------------------
// not actually using this right now.
// place holder for future feature allowing user to recalibrate rpm range

boolean checkCalibration() {

  // we only get in here if calibration < 2
  // so it can be only 0 or 1
  
 if (!calibrated) {
    digitalWrite(ylo1,HIGH);
    digitalWrite(ylo2,HIGH);
    digitalWrite(ylo3,HIGH);
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
      digitalWrite(ylo1,LOW);
      digitalWrite(ylo2,LOW);
      digitalWrite(ylo3,LOW);
      
      log_print(F("Done with calibration."),1);

      return (1) ;
    }

  }
  


