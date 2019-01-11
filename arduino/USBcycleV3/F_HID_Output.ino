void send_char(char c) {

// sends exactly one key code to Kbrd service
// For now, we will send it to Serial
  Serial.print("\"");Serial.write(c);Serial.print("\"");
  Serial.print(", hex: ");
  // prints value as string in hexadecimal (base 16):
  Serial.println(c, HEX);
  
  if (!hidKill) {Keyboard.write(c);}
  
  return;
  
}


void pressEnter() {

  if (hidKill) {return;}
  delay(50);
  Keyboard.press(KEY_RETURN);       // how long to hold Enter?
  delay(100);
  Keyboard.releaseAll();            // Keyboard.release(KEY_RETURN) doesn't seem to work.
  delay(50);
}


void setWarp() {
  //  
  float val = State.val.warp / 100.0;
  if (!hidKill) {
  Keyboard.write('`');
  delay(50);
  Keyboard.print("warp ");
  Keyboard.print(val);
  pressEnter();
  Keyboard.write('`');
  // end write to game
  }
  log_print(F("Set Warp to "),0);
  Serial.println(val); 
}


void goTomorrow() {
  // go to 5AM tomorrow morning
  //
  if (!hidKill) {
  Keyboard.write('`'); 
  delay(50);
  Keyboard.print("g_set_time 5");
  pressEnter();
  Keyboard.write('`');
  }
  // end write to game
  log_print(F("Set time to Tomorrow 5am"),1);
}

void fixWeather(int how) {
  // make the weather gradually improve
  //
  if (!hidKill) {
    Keyboard.write('`');
    delay(50); 
    if (!how) {
       Keyboard.print("g_set_weather 0 i");
    } else {
      Keyboard.print("g_set_weather 0 f");
    }
    pressEnter();
    Keyboard.write('`');
  }
  //
  log_print(F("Set weather to improve "),0);
  if (how) {
    Serial.println(F("Immediately!"));
  }else {
    Serial.println(F("Gradually..."));
  }
}

// you get here from Settings page (3)
void goPlace() {
  
  int ptr = State.val.ridePtr;
  // don't accept values off end of array!
  if (ptr >= State.val.rideCt) {ptr = State.val.rideCt - 1;}
  String coords = rides[ptr];
  String rname = ridenames[ptr];
  //
  if (!hidKill) {
    Keyboard.write('`');            // enter dev con
    delay(50);
    Keyboard.print("goto ");
    Keyboard.print(coords);
    pressEnter();
    Keyboard.write('`');          // exit dev con
    //
    delay(50);
    Keyboard.press(KEYPAD_2);     // press key to teleport truck to location
    delay(20);
    Keyboard.release(KEYPAD_2);
    delay(50);
    Keyboard.write('5');    // switch back to camera 5
  }
  log_print(F("GOTO place# "),0); 
  Serial.print(ptr); Serial.print(F(" : "));
  Serial.print(rname); Serial.print(F(" ("));
  Serial.print(coords);  Serial.println(F(")"));

  return;
  //
}

void goTime() {
  // must be on Page 6 to get here.
  // get strings from text on page 6, make time string,
  // build cmd.
  String cmd = ("g_set_time ") + String(State.val.hh) + String(" ") + String(State.val.mm);
  if (!hidKill) {
    Keyboard.write('`');            // enter dev con
    delay(50);
    Keyboard.print(cmd);
    pressEnter();
    Keyboard.write('`');
  }
  //
  Serial.println(cmd);
  //
}


void setTraffic(bool b) {
  
  // turn traffic on and off
  if (!hidKill) {
    Keyboard.write('`');
    delay(100);
    if (b) {
      Keyboard.print("g_traffic 1");
    } else {
      Keyboard.println("g_traffic 0");
    }
    pressEnter();
    Keyboard.write('`');
  }
  //
  log_print(F("Set traffic to "),0);  Serial.println(b);
  return;
  
}


