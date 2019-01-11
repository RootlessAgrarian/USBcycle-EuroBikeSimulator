
//
////
//////================================ MAINLOOP ===================================
////
//



void loop() {

//////-----------------------------------------------------------
//////  DECLARE LOOP VARS

  boolean nstat;
  String logmsg;
  long looptime;

// ======= FIRST, ALWAYS CHECK FOR MANUAL SWITCHES

  bool b = digitalRead(emuSwitch);      // switch HIGH is default, LOW is pressed.  
                                         // Press to enable HID, release to disable
  if (b != hidKill) {
    Serial.print(F("Setting HIDKILL to "));  Serial.println(b);
    hidKill = b;
  }

  b = digitalRead(cTouch);
  if (b != enterKey) {
    enterKey = b;
    Serial.print(F("Enter Key is "));  Serial.println(b);
    keyEnter(b);
  }

  int foo;
  foo = random(100);    // in case you need a random number for testing

  long mil = millis();
  elap = mil - start_time;
  looptime = mil - looptick;
  

////////-----------------------------------------------------------
//////
////  READ SENSORS
//
//

// we still have place holder code here because not yet connected
// to the real bike sensors.

//
////
////// BRAKE
////
//


    checkBrake();
    
//    analog Hall input is connected to pin A3 of ADC
//    rawVal = adcget(uint8_t(3));
 
  // DEBUG early testing only:
  // wow, the page selected is having an effect on the adc output?  RF?
  // and it appears to be chirping the piezo too!  Yikes.  May need some tinfoil.
  // Jan 1 2019:  no signal on ADC.  Bad Hall?  Hall only works on +5 nov 3v3?
  //              broke something?  haha, no wire!  easy fix :-)
  // jitter is now about 5 counts.  Range of values: 9700 to 13910
  // EXCEPT when there is physical vibration of the bike, then the noise gets
  // large!  yikes.  Like 60 cts.  argh.  is it the sensor or the ctrl box?
  // lifting the lid on the box makes it go nuts also.
  // big improvement over Leo ADC with 20 counts of jitter.
  /*
  if (abs(rawVal - lastVal) >= 6) {
    Serial.print(F("NEW BRAKE VAL = "));
    Serial.println(rawVal);
    lastVal = rawVal;
    int pct = round(100 * rawVal / 18000.0);    // this is an ADC now and the range is
                                                // 0-3.3v <=> 0-17500 more or less
//  State.val.steerT = 100 - pct;
    LState.val.rBrake = State.val.rBrake;
    State.val.rBrake = pct;
    // Serial.print(F("New Pot Value: "));  Serial.println(rawVal);
    StateChange = 1;
  }
 */

//
////
////// STEERING
////
//

  // this will read the i2c rotary encoder and set rawSteerVal and steerVal

    readRotaryHall();

//  timer = 0;
//  timer++;

//
////
////// PEDALS
////
//

// now read the reed switches
   for (int i = 0; i < numSensors; i++ ) {
    int val = readSensor(i);
    if (val >= 0) {
      // only write to LEDs when value changes -- value is -1 for no change.
      // LED numbering system has changed a LOT since v2 :-)
      if (val) { 
        laston[i] = millis();
        digitalWrite(ylo1-2*i, HIGH) ;
 //       Serial.print("Reed ");  Serial.print(i); Serial.println(" ON");
      } else {
        lastoff[i] = millis();
        digitalWrite(ylo1-2*i, LOW) ;
 //        Serial.print("Reed ");  Serial.print(i); Serial.println(" OFF");
      }
    }
  }
  
////////-----------------------------------------------------------
//////
//
////  COMPUTE AND SEND AXES :  Steering, Brake, Throttle
//

// now you have the raw values:  brake, steering, rpm
// you can compute and send the joystick axis values
//
//// Y axis:  steering
//
  if (State.val.Steer != LState.val.Steer) {
  // Serial.print(F("New Steer Val:  setYaxis = "));
  // Serial.println(State.val.Steer);
    Joystick.setYAxis(State.val.Steer);
     // LState.val.Steer = State.val.Steer;  NO not here!  omg how do we do this.  at end of loop I guess
  }

//
//// Z axis:  brake
//
// the Hall proximity sensor is super jittery for whatever reasons, it's fluctuating
// by 30 counts or more;  we hide this filtering in the brakeRead function
// we preserve the raw value and convert at the last minute (only on demand) to 0-255.
// we also clip "close to zero" to prevent slight brake dragging at low values
// to cut down on useless joystick writes, we probably should scale this to units of 
// joystick axis input.
// which would be 0-255, I think, scaled to 4200 counts.  so 17 cts required
// to make a 1-ct change to Joystick Steer val.  
// we pin it to min when within Noise of min to avoid slight brake drag.
// but this is not really necessary now that the noise is down to 5 cts
  if (LState.val.rBrake != State.val.rBrake) {
    if ((State.val.rBrake - brakeMin) <= brakeNoise) {
     State.val.rBrake = brakeMin;
//    Serial.println("Brakes OFF");
    }
    State.val.Brake = map(State.val.rBrake,brakeMin,brakeMax,0,255);
    if (LState.val.Brake != State.val.Brake) {
        Serial.print(F("New Brake Val:  raw = ")); Serial.print(State.val.rBrake); Serial.print(F("   ->  setZaxis = "));
        Serial.println(State.val.Brake);
        Joystick.setZAxis(State.val.Brake);
        StateChange = 1;
    }
  }
  
//
//// X axis:  accelerator (throttle)
//
// if rpm has changed, respond:  figure out what is going on

  if (LState.val.RPM != State.val.RPM) {
    
    Serial.print(F("New RPM Val: "));
    Serial.println(State.val.RPM);
    State.val.Throt = 0;         // time for a new throttle value
    // we only honour throttle values gt minRPM which right now is set to 20
    // at rpm lower than this, it's hard to figure out what is going on.
    
    if (State.val.RPM >= minRPM) {
      Stopped = 0;
      digitalWrite(bLED,LOW);
      // map raw FP rpm to integer joystick value 0-255, with dynamic range adjust set by user
      // default range is 20 to 200, so 180 rpm difference.
      // throtRange of .50 would yield a range of 20 to 110 (20 + diff * .50)
      // .70 would be 20 to 146  (default)
      // .90 would be 20 to 182  (challenging!)
      // so the bigger this number is,the harder you have to work for a given throttle setting.
      // so... that determines your cadence when you are cruising at 60 kph on the flat 
      // for example.  
      float maxr = minRPM + ((maxRPM - minRPM) * State.val.throtT/100);
      float mrpm = State.val.RPM;
      // and here we do a shameless hack to goose the acceleration, to overcome the mass of
      // the truck (someday we'll have telemetry to assist in this algorithm, right now we
      // just brute-force it.
      if (State.val.turbo && (aDelta > 1.0) && (State.val.RPM < 55)) {
            // mrpm = rpm * 3.0;
            // floor it!
            mrpm = maxr;
      }
      State.val.Throt = map(round(mrpm),minRPM,maxr,0,255);

      constrain(State.val.Throt,0,255);
      if (LState.val.Throt != State.val.Throt) {
        Serial.print(F("New Throt Val:  setXaxis = "));
        Serial.println(State.val.Throt);
//      Joystick.setXAxis(State.val.Throt);
      }

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
      digitalWrite(bLED,HIGH);    // we've stopped pedalling
    }
  }
  
  // if in basic debug mode, no Nextion, return from here
  // to avoid Serial1 read timeouts
  
//  loopct++;
//  return;

////////-----------------------------------------------------------
//////
//
////  READ NEXTION
//

  // nstat returns 1 if it has assembled a complete message from the Nextion.
  // better be careful about incomplete messages!
  
  nstat = listen();

  // now, if we are clever we will update our widget inventory based on
  // the dump data sent from the Nextion?  NO. too dangerous as it means
  // resizing the gizmos array.  do this .h file codegen stuff ONLY in the Basic sketch,
  // once per widget inventory change to the Nextion UI.
  
  if (nstat) {
    logmsg = logMessage();
    if (nexti_dump) {
      boolean r = parseDump(logmsg);
      if (r) {
          Serial.println(F("Error parsing line of component dump output from Nextion"));
      }
    }
    Serial.println(logmsg.c_str());
    nexti_msg = "";
  }


////////-----------------------------------------------------------
//////  
//
////  READ CLOCK
//

  tick = 0;
  
  rtc = RTC1.now();

  int mi = rtc.minute();
  
  // Serial.print("MINUTE = ");  Serial.print(mi);  Serial.print("   LAST WAS:  ");  Serial.println(last_min);
  
  if (mi != last_min) {
    // showTime();
    LState.val.hh = State.val.hh;
    LState.val.mm = State.val.mm;
    State.val.hh = rtc.hour();
    State.val.mm = mi;
    last_min = mi;
    tick = 1;
    StateChange = 1;
//    log_print(F("SAVE STATE"),1);
//    Serial.print(F("ridePtr = "));  Serial.println(State.val.ridePtr);
    stateSave();
  }

////////-----------------------------------------------------------
//////  
//
////  MONITOR MEMORY
//

  showMem(0);

////////-----------------------------------------------------------
//////  
//
////  UPDATE NEXTION 
//



  // if any event during the loop triggered a change of visible state,
  // something that might require an update, then do a page refresh.
  // tick is a special case.
  // we will check for prev and current values in all other instances.
  // but tick will always drive a clock update.
  // if we have just changed pages, we call refreshPage with arg 1, which
  // tells it to force all updates.
  
  if (StateChange) {
  refreshPage(0);
  statePush();         // do all the state push in one place, stupid!
  StateChange = 0;
  }


  return;
  
  
}

// END MAIN LOOP (whew!)


