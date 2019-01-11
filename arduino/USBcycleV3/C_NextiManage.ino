//
////
//////==================== More advanced communication functions. r/w Nextion
////
//

// refreshPage is a key point in our mainloop cycle.
// it checks which page is current before doing anything.
// setBarItem has a wee problem:  right now the value is
// a int percent (as in 50 means 50 percent), but in the
// case of rpm and throt we might also want to know the
// raw value.

void refreshPage (boolean force) {

// what to do when you load a new page
// after a decent interval?
    // setBarItem(1,23,pct,1);
    // setBarItem(1,22,1-pct,1);
    // setBarItem(3,6,pct,0);
    // setBarItem(3,8,1-pct,0);
   int pg = nexti_page;

 switch(pg) {
  case 1:                             // bike page
    if (tick || force) {showTime();}
    if ((State.val.RPM != LState.val.RPM) || force) {
      setBarItem(1,23,State.val.RPM,1,force);
    }
    if ((State.val.Throt != LState.val.Throt) || force) {
      setBarItem(1,22,State.val.Throt,1,force);
    }
    break;
  case 2:                             // truck page
    showTime();
    break;
  case 3:                             // options page
    if ((State.val.warp != LState.val.warp) || force) {
      setBarItem(3,6,State.val.warp,0,force);
    }
    if ((State.val.throtT != LState.val.throtT) || force) {
      setBarItem(3,7,State.val.throtT,0,force);
    }
    if ((State.val.steerT != LState.val.steerT) || force) {
      setBarItem(3,8,State.val.steerT,0,force);
    }
    if (force) {
        if (State.val.turbo) {
          Serial.println(F("Turbo is on"));
          flipRegion(3,2,1);
        }
        if (State.val.traf) {
          Serial.println(F("Traffic is on"));
          flipRegion(3,13,1);
        }
    }
    if ((State.val.ridePtr != LState.val.ridePtr) || force) {
        String rname = ridenames[State.val.ridePtr];
        nexti_cmd = "p[3].b[9].txt=\"" + rname + "\"";  
        send_cmd();
    }
    break;
  case 4:                             // monitor page
    int pct;
    // set the raw value bar widgets
    if ((State.val.rBrake != LState.val.rBrake) || force) {
      pct = map(State.val.rBrake,brakeMin,brakeMax,0,100);
      setBarItem(4,12,pct,0,force);
      nexti_cmd = "p[4].b[4].txt=\"" + String(State.val.rBrake) + "\"";                // raw brakeval
      send_cmd();
    }
    if ((State.val.rSteer != LState.val.rSteer) || force) {
      pct = map(State.val.rSteer,minRotaryHall,maxRotaryHall,0,100);
      setBarItem(4,13,pct,0,force);
      nexti_cmd = "p[4].b[5].txt=\"" + String(State.val.rSteer) + "\"";                // raw steerval
      send_cmd();
    }
    if ((State.val.RPM != LState.val.RPM) || force) {
      pct = map(State.val.RPM,minRPM,maxRPM,0,100);
      setBarItem(4,14,pct,0,force);
      nexti_cmd = "p[4].b[6].txt=\"" + String(State.val.RPM) + "\"";                // raw RPM
      send_cmd();
    }
    // set the joystick sending values
    if ((State.val.Brake != LState.val.Brake) || force) {
      nexti_cmd = "p[4].b[1].txt=\"" + String(State.val.Brake) + "\"";                // Joy brakeval
      send_cmd();
    }
    if ((State.val.Steer != LState.val.Steer) || force) {
      nexti_cmd = "p[4].b[2].txt=\"" + String(State.val.Steer) + "\"";                // Joy steerval
      send_cmd();
    }
    if ((State.val.Throt != LState.val.Throt) || force) {
      nexti_cmd = "p[4].b[3].txt=\"" + String(State.val.Throt) + "\"";                // Joy throtval
      send_cmd();
    }
    break;
  case 6:                             // special stats page
    if (force) {paintTime();}         // we do not update this time.  we can use it to 
                                      // set the rtc (rarely) or we can set the game clock.
    nexti_cmd = "p[6].b[6].txt=\"" + String(0) + "\"";                // Avg RPM
    send_cmd();
    nexti_cmd = "p[6].b[7].txt=\"" + String(0) + "\"";                // Max RPM
    send_cmd();
    nexti_cmd = "p[6].b[8].txt=\"" +  String(TimeToString(State.val.elap,0)) + "\"";   // Elap Time
    send_cmd();
    nexti_cmd = "p[6].b[9].txt=\"" + String(0) + "\"";                // Idle Time
    send_cmd();
    nexti_cmd = "p[6].b[10].txt=\"" + String(50) + "\"";             // Active Pct
    send_cmd();
    nexti_cmd = "p[6].b[11].txt=\"" + String(State.val.freemem) + "\"";  // Free Memory
    send_cmd();
    break;
  default:
    break;
   
 } // end switch
 
}

// at the moment we only try to show time on Page 1.  this may change.
// I think we are going about this all wrong.  surely we should just
// write relevant values to a great big State struct and then pull out
// the values we need whenever we go to a new page.

void showTime() {
  
  // Serial.println(F("ShowTime!"));
  // we only show time on certain pages, so don't waste cycles
  // if we are not on the right page.
  
  if ((nexti_page < 1) || (nexti_page > 2)) {return;}
  
  rtc = RTC1.now();
  String ftime;
  String etime;

  // ELAP time
  time_t prez = rtc.unixtime();
  long elap = prez - tzero;
  State.val.elap = elap;
  int eh=elap/3600;
  int secsRemaining=elap%3600;
  int em=secsRemaining/60;
  // int es=secsRemaining%60;

  if (eh < 10) {etime = "0";}
  etime += String(eh);
  etime += ":";
  if (em < 10) {etime += "0";}
  etime += String(em);

  // CLOCK time
  int hr; int mi;
  hr = rtc.hour();
  mi = rtc.minute();

  State.val.hh = hr;
  State.val.mm = mi;

  if (hr < 10) {ftime = "0";}
  ftime += String(hr);
  ftime += ":";
  if (mi < 10) {ftime += "0";}
  ftime += String(mi);
  // ftime += ":";
  // if (se < 10) {ftime += "0";}
  // ftime += String(se);

  switch(nexti_page) {
    case 1:
      nexti_cmd = "p[1].b[1].txt=\"" + String(etime) + "\"";
      send_cmd();
      nexti_cmd = "p[1].b[2].txt=\"" + String(ftime) + "\"";
      send_cmd();
      break;
    case 2: 
      nexti_cmd = "p[2].b[1].txt=\"" + String(ftime) + "\"";
      send_cmd();
      nexti_cmd = "p[2].b[2].txt=\"" + String(etime) + "\"";
      send_cmd();
      break;
    default:
      break;
  }
  return;
  
}

//    MCU does the math and figures out how the value maps to percent.
//    setBarItem routine figures out the geometry and how to map that
//    percent to screen coords, and sends the picq command:
//    picq x,y,w,h,picid
//    Warp is item 5 on page 3.  How do we remember that???
//    widget id's change all the time.  this is why names would be nice.
//    but names can't be extracted.  grmph.  unless we can parse the hmi or tft files.
//    the Full image for page 3 is pic 4 (s/b an array lookup)  (now it is:  Images)
//    empty image is pic 5
//    so, let's write a function that IF we are on page 3, sets item 5 to x pct.
//    args:  Page, Id, Percent, Orientation
//    orientation is 0 for horizontal, 1 for vertical
    
//
//     setBarItem(3,5,5,4,pct);
//     picq x,y,w,h,picid

void setBarItem(int pg, int id, int ipct, int ori, bool force) {

    int epic;
    int fpic;
   
    int i;
    float pct = ipct/100.0;

//  don't waste your time & scramble your display trying to do this on the wrong page!
    if (pg != nexti_page) {
      // Serial.print(nexti_page); Serial.print("... is wrong page for ");  Serial.print(pg);  Serial.print(".");  Serial.println(id);
      return;
    }
    
    epic=Images[pg][0];
    fpic=Images[pg][1];
    // Serial.print("For Page "); Serial.print(pg); Serial.print("  Empty Image: ");  Serial.print(epic);  
    // Serial.print("  Full Image:"); Serial.println(fpic);
    
    // get item geometry
    for (i = 0; i < gizmo_ct; i++) {
    int p = gizmos[i].page;
    int g = gizmos[i].id;
    if ((p != pg) || (g != id)) {continue;}
    break;
    }
    
    int x = gizmos[i].x;
    int y = gizmos[i].y;
    int w = gizmos[i].w;
    int h = gizmos[i].h;

    float old = gizmos[i].val;
    if (force) {old = 0.0;}

    // surely we can generalise better than this.  suppose we know the old value, call it old.
    // and we know the new val, pct.  So, we get dir automagically without anyone having to
    // tell us.  AND we should be able to figure out the smallest possible rectangle to repaint.
    // IF old val is less than new val, we can repaint only the region from oldval to newval 
    // with the ON image.
    // IF old val is MORE than new val, then we can repaint only (ha!) the region from oldval
    // to newval with the OFF image.

    // YOU ARE HERE
    
    // Serial.print("NEW VALUE: "); Serial.print(pct);  Serial.print("   OLD VALUE: ");  Serial.println(old);
 
    if (!ori) {
      
      int nw = round(w * pct);
      int ow = round(w * old);
      
      if (nw > ow) {
        nexti_cmd = String("picq " + String(x+ow-2) + "," + String(y) + "," + String(nw-ow+2) + "," + String(h) + "," + String(fpic));
      } else {
        nexti_cmd = String("picq " + String(x+nw) + "," + String(y) + "," + String(ow-nw+2) + "," + String(h) + "," + String(epic));
      }
      
    } else {
      
       int nh = round(h * pct);
       int oh = round(h * old);
       int base = y+h;
       
       if (nh > oh) {
         nexti_cmd = String("picq " + String(x) + "," + String(base-nh) + "," + String(w) + "," + String(nh-oh+2) + "," + String(fpic));
       } else {
         nexti_cmd = String("picq " + String(x) + "," + String(base-oh-2) + "," + String(w) + "," + String(oh-nh) + "," + String(epic));
       }
    }

    send_cmd();
    
    gizmos[i].val = pct;
    

}

//

void flipRegion(int pg, int id, int which) {
    
    int pic, i;

    // if which is 1, flip pixels to the "on" image.
    // if which is 0, flip pixels to the "off" image.
    // if we had more sophisticated graphics we might need more than 2 states.
    // this one works fine for on-off or radio buttons.
    
    pic = Images[pg][which];
    
    // get item geometry
    for (i = 0; i < gizmo_ct; i++) {
    int p = gizmos[i].page;
    int g = gizmos[i].id;
    if ((p != pg) || (g != id)) {continue;}
    break;
    }
    
    int x = gizmos[i].x;
    int y = gizmos[i].y;
    int w = gizmos[i].w;
    int h = gizmos[i].h;
    
    nexti_cmd = String("picq " + String(x) + "," + String(y) + "," + String(w) + "," + String(h) + "," + String(pic));
    send_cmd();
     
}

void paintTime() {

  // paintTime is very specific and only applies to page 6.
  // there are tiny widgets over on the RHS that we need to
  // manage digit by digit.
  // the top row of up arrows are 14 15 16 17
  // the bottom row of down arrows are 18 19 20 21
  // the digits are 1 2 3 4 5 (3 is the colon);
  
  nexti_cmd = "p[6].b[3].txt=\":\"";                // colon
  send_cmd();
  DateTime rtc = RTC1.now();
  int hr = rtc.hour();
  int mm = rtc.minute();
  int d1,d2,d3,d4;
  d1 = floor(hr/10.0);
  d2 = hr%10;
  d3 = floor(mm/10.0);
  d4 = mm%10;
  // set global array
  timeDigits[0] = d1;
  timeDigits[1] = d2;
  timeDigits[2] = d3;
  timeDigits[3] = d4;
  
  nexti_cmd = "p[6].b[1].txt=\"" + String(d1) + "\"";                // digit 1
  send_cmd();
  nexti_cmd = "p[6].b[2].txt=\"" + String(d2) + "\"";                // digit 2
  send_cmd();
  nexti_cmd = "p[6].b[4].txt=\"" + String(d3) + "\"";                // digit 3
  send_cmd();
  nexti_cmd = "p[6].b[5].txt=\"" + String(d4) + "\"";                // digit 4
  send_cmd();

}

void timeAdjust (int dig, int adj) {

// dig is "which digit of the global timeDigit array"
// adj is what to add to it (+1 or -1)
   int lim[4] = {2,9,6,9};
   
   int n = timeDigits[dig] + adj;
   if (n < 0) {n = lim[dig];}
   if (n > lim[dig]) {n = 0;}
   timeDigits[dig] = n;
   
   switch (dig) {
    case 0:
        nexti_cmd = "p[6].b[1].txt=\"" + String(n) + "\"";                // digit 1
        break;
    case 1:
        nexti_cmd = "p[6].b[2].txt=\"" + String(n) + "\"";                // digit 2
        break;    
    case 2:
        nexti_cmd = "p[6].b[4].txt=\"" + String(n) + "\"";                // digit 3
        break;
    case 3:
        nexti_cmd = "p[6].b[5].txt=\"" + String(n) + "\"";                // digit 4
        break;   
     default:
        break;
     }
     send_cmd();
     
}

