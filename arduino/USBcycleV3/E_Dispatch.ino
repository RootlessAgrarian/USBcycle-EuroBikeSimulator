
void touchDispatch(int page, int id, int which, int x, int y) {

  // which is 1 for press and 0 for release
  // usually we don't care, but someday we might.
  // x and y are left 0 for a simple (button) touch.
  
  int gindex;

// if this is a button PRESS rather than RELEASE then we are
// waiting for geometry from a sendxy

  if (which) return;

  gindex = findGizmo(page, id);
  int wx = gizmos[gindex].x;
  int wy = gizmos[gindex].y;
  int ww = gizmos[gindex].w;
  int wh = gizmos[gindex].h;
  String wn = gizmos[gindex].alias;
  int per = floor (((x - wx) / float(ww)) * 100);
  
  Serial.println("\nGot a touch on " + String(page) + "." + String(id) + " at " + String(x) + "," + String(y) + " (" + String(which) + ")");
  Serial.println("This item is at " + String(wx) + "," + String(wy) + " and is " + String(ww) + " x " + String(wh));
  if (x) {Serial.println("This touch is at " + String(per) + "% of the item's width");}

  // if it is a simple button to keystroke map, then send char and we are done
  // see Ctrl.h for the key map arrays
    char key = key_char(page,id);
    if (key > 0x01) {
        send_char(key);
        return;
    } 
  // if it is more complicated than that, do the big switch stmt:
    
  // Page 1 is the main Cycling Page with most-used buttons and a
  // goto at top for Page 6
  
  if (page == 1) {
    switch(id) {
      case 11:            // goto tomorrow
        goTomorrow();
        break;
      case 12:            // fix the weather
        fixWeather(0);
        break;      
      default:
        break;
    }
    return;

  }
  
//  Page 2 is the Truck Driving Page

  if (page == 2) {
    switch(id) {
      case 16:      // go to tomorrow
        goTomorrow();
        break;
      case 17:      // fix the weather
        fixWeather(0);
        break;
      case 19:        // Blank key
        break;
      default:
        break;
    }
    return;
  }

// Page 3 is the Game Settings page where user preferences can be set

  if (page == 3) {
    switch(id) {
      case 2:
        LState.val.turbo = State.val.turbo;
        if (State.val.turbo) {
          Serial.println(F("Try to turn Turbo off"));
          flipRegion(page,id,0);
          State.val.turbo = 0;
        } else {
          Serial.println(F("Try to turn Turbo on"));
          flipRegion(page,id,1);
          State.val.turbo = 1;
        }
        StateChange = 1;
        break;
      case 3:
        Serial.println(F("We should scroll DOWN one predefined ride."));
        LState.val.ridePtr = State.val.ridePtr;
        State.val.ridePtr--;        
        if (State.val.ridePtr < 0) {State.val.ridePtr = State.val.rideCt - 1;}
        StateChange = 1;
        chirp(2);
        break;
      case 4:
        Serial.println(F("We should scroll UP one predefined ride."));
        LState.val.ridePtr = State.val.ridePtr;
        State.val.ridePtr++;
        if (State.val.ridePtr == State.val.rideCt) {State.val.ridePtr = 0;};
        Serial.print(F("New Ride Number: "));  Serial.println(State.val.ridePtr);
        StateChange = 1;
        chirp(1);
        break;
      case 5:
        Serial.println(F("We should Teleport to the selected ride."));
        goPlace();
        break;
      case 6:
        LState.val.warp = State.val.warp;
        Serial.print(F("We should set the warp to "));  Serial.print(per);  Serial.println(F(" pct of max."));
        State.val.warp = per;
        setWarp();
        StateChange = 1;
        break;
      case 7:
        LState.val.throtT = State.val.throtT;
        Serial.print(F("We should set the throttle gain to "));  Serial.print(per);  Serial.println(F(" pct of max."));
        State.val.throtT = per;
        StateChange = 1;
        break;
      case 8:     // Steering sensitivity
        LState.val.steerT = State.val.steerT;
        Serial.print(F("We should set the steering gain to "));  Serial.print(per);  Serial.println(F(" pct of max."));
        State.val.steerT = per;
        StateChange = 1;
        break;
      case 13:
        LState.val.traf = State.val.traf;
        if (State.val.traf) {
          Serial.println(F("Try to turn Traffic off"));
          setTraffic(0);
          flipRegion(page,id,0);
          State.val.traf = 0;
        } else {
          Serial.println(F("Try to turn Traffic on"));
          setTraffic(1);
          flipRegion(page,id,1);
          State.val.traf = 1;
        } 
        StateChange = 1;
        break;       
      default:
        break;
    }
    return;
  }

// Page 4 is the Diagnostics Page where we can see encoders in real time, also
// cooked joystick output

  if (page == 4) {
    switch(id) {
      case 7:
        Serial.println(F("We should Recalibrate the Steering!"));
        centreSteering();
        break;
      default:
        break;
    }
    return;
  }

// Page 5 is just the credits page which does nothing interesting at all

// Page 6 is the Statistics Page where we can see elap time, average rpm and 
// similar statistics.  We may also be able to set the time from there.
// and because we are out of room, we'll stash the mouse invert there (argh!!)

  if (page == 6) {
    switch(id) {
      case 12:
        // this sets the RTC, which is displayed on the Nextion
        setTime();
        break;
      case 14:
        timeAdjust(0,1);
        break;
      case 15:
        timeAdjust(1,1);
        break;
      case 16:
        timeAdjust(2,1);
        break;
      case 17:
        timeAdjust(3,1);
        break;
      case 18:
        timeAdjust(0,-1);
        break;
      case 19:
        timeAdjust(1,-1);
        break;
      case 20:
        timeAdjust(2,-1);
        break;
      case 21:
        timeAdjust(3,-1);
        break;
      case 22:        // I don't think we actually use this any more...
         if (State.val.Yinv) {
          Serial.println(F("Try to turn Mouse Inv Y off"));
          flipRegion(page,id,0);
          State.val.Yinv = 0;
        } else {
          Serial.println(F("Try to turn Mouse Inv Y on"));
          flipRegion(page,id,1);
          State.val.Yinv = 1;
        }
        break;
      case 23:
        goTime();
        break;
      default:
        break;
    }
    return;
  }
  
  Serial.println(F("THIS CAN'T HAPPEN, NONEXISTENT PAGE!"));
  
}


byte key_char(int pg, int id) {

   byte c;
   
  if ((pg < 1) || (pg > 2)) {
    return(0);
  }
// return the key code for this button
// this is another unwieldy array (sigh).
// the array is numbered like Gizmos? with some nulls?
// or is it N arrays for N pages?  Or is it (gasp) just
// a long string stored in a char array with N elements one
// per page?

//  our first widget id is 3, but our first array element is 0
   int i = id - 3;
   
   char *cp = keyMaps[pg-1];
   c = cp[i];
   
//   if (pg == 1) {
//    c = keyMap1[i];
//   } else {
//    c = keyMap2[i];
//   }
   
   return(c);
   

}

// this is sloppy coding, I know, I know... mea culpa.
// but we never chord.  if we are holding down Enter then we are doing nothing
// else during that time (filling tank with diesel).  if we hit Enter, we're busy.
// so we can afford to clear all keys when Enter is released.
// so when we get a change to 1, we press the key.  a change to 0, release all keys.
void keyEnter(bool b) {
  if (hidKill) return;
  if (b) {
      Keyboard.press(KEY_RETURN);
  } else {
      Keyboard.releaseAll();
  }
}

void setTime () {

   DateTime dt;
   rtc = RTC1.now();

   int hh = (10 * timeDigits[0]) + timeDigits[1];
   int mm = (10 * timeDigits[2]) + timeDigits[3];
   
   RTC1.adjust(DateTime(rtc.year(), rtc.month(), rtc.day(), hh, mm, 0));
  
   return;

}

