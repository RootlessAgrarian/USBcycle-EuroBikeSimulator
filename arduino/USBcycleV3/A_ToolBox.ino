//
////
//////  GENERALLY USEFUL FUNCTIONS
////
//


/*
 *      USBcycle library that was shared by Leo1 and Leo2 and any other
 *      USBcycle controllers, now part of Toolbox
 */

int intify(float val) {
  // get a percentage type value 0.0 to 1.0 and turn it into
  // an int:  you get values 0 through 10 from your pot.
  // if I were cleverer I would supply another param N that determines
  // how many values in one pot turn.
  int v = round(val * 10);
  return(v);
}

// t is time in seconds = millis()/1000;
char * TimeToString(unsigned long t, bool b)
{
 t = t/1000;
 static char str[12];
 long h = t / 3600;
 t = t % 3600;
 int m = t / 60;
 int s = t % 60;
 if (b) {
   sprintf(str, "%04ld:%02d:%02d", h, m, s);
 } else {
   sprintf(str, "%02d:%02d", h, m);
 }
 return str;
}

void log_print(const __FlashStringHelper *str, boolean flag) {

   if(!DEBUG) return;
  
   Serial.print(TimeToString(millis(),1));      // timestamp
   Serial.print(F(" --  "));                  // formatting
   Serial.print(str); 
   // if bool flag then end line, otherwise leave it open and add more stuff later
   if (flag) Serial.println(F(""));
   
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

void goCatatonic(int which) {
  // you can't recover from this condition.  just blink a light and hope for rescue.
  // which can be any one of 33 through 47 (odd only) -- counting upward from ylo1.
  while(1) {
    blinky(which, 2);
    delay(500);
  }
}



void showMem(bool flag) {

// if flag, be verbose.
  if (flag) {
  Serial.print("\nDynamic ram used: "); Serial.println(mi.uordblks);
  Serial.print("Program static ram used: "); Serial.println(&_end - ramstart); 
  Serial.print("Stack ram used: "); Serial.println(ramend - stack_ptr); 
  Serial.print("My guess at free mem: ");  Serial.println(stack_ptr - heapend + mi.fordblks);
  }
  State.val.freemem =  stack_ptr - heapend + mi.fordblks;
  if (LState.val.freemem != State.val.freemem) { StateChange = 1;}
}

// pick values out of csv or other sep string

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

//
////
//////========================== BASIC COMMUNICATIONS ==============================
////
//

// flush buffers if need be

void flushSerial(){
  Serial.flush();
  Serial1.flush();
}//end flush


// check whether an i2c device is on the bus

boolean testI2C(byte addr) {
  
  boolean success = 0;
  for (int attempt = 0; attempt < 4; attempt++) {
    Serial.print(F("  Try to transmit to address "));
    Serial.print(addr,HEX);
    Serial.print(F("  dec "));
    Serial.println(addr);
    
  Wire.beginTransmission (addr);
  
//  0:success
//  1:data too long to fit in transmit buffer
//  2:received NACK on transmit of address
//  3:received NACK on transmit of data
//  4:other error

  byte error = Wire.endTransmission();
  if (error == 0)
  {
     Serial.print(F("  Found slave alive and listening on try: "));
     Serial.println(attempt);
     success = 1;
     break;
  } else {
     Serial.print(F("  Slave contact attempt: "));
     Serial.print(attempt);
     Serial.print(F("  ... ERROR: "));
     Serial.println(error);
     delay(1000);
  }
  }
  
  return(success);

}


// sanity check your RTC!

void rtc_check() {
  
    rtc = RTC1.now();
    
    Serial.print(rtc.year(), DEC);
    Serial.print('/');
    if(rtc.month() < 10) Serial.print("0");
    Serial.print(rtc.month(), DEC);
    Serial.print('/');
    if(rtc.day() < 10) Serial.print("0");
    Serial.print(rtc.day(), DEC);
    Serial.print(' ');
    if(rtc.hour() < 10) Serial.print("0");
    Serial.print(rtc.hour(), DEC);
    Serial.print(':');
    if(rtc.minute() < 10) Serial.print("0");
    Serial.print(rtc.minute(), DEC);
    Serial.print(':');
    if(rtc.second() < 10) Serial.print("0");
    Serial.print(rtc.second(), DEC);
    Serial.println();

    Serial.print("Unix time: ");  Serial.println(rtc.unixtime(),DEC);

    Serial.print(rtc.tempC(), 1);
    Serial.println(" degrees Celsius");
    Serial.print(rtc.tempF(), DEC);
    Serial.println(" degrees Fahrenheit\n");
    
}



void fram_check () {

   DateTime lastd;
   
 // tzero should already be set due to RTC init code in setup
   union timebytes {
    unsigned char tbytes[4];
    unsigned int dt;   } 
    tbi;
   

   
  // Read the first 5 bytes
  uint8_t test = fram.read8(0x0);
  for (int i=0; i<4; i++) {
    tbi.tbytes[i] = fram.read8(i+1);
  }

  lastd = tbi.dt;

  // We could put this in a page somewhere (state page?  metadata?)
  Serial.print("Restarted "); Serial.print(test); Serial.println(" times");
  Serial.print("Last restart: ");  
  Serial.print(lastd.year()); Serial.print("-"); Serial.print(lastd.month());  Serial.print("-"); Serial.print(lastd.day()); Serial.print("  ");
  if (lastd.hour() < 10) {Serial.print(0);}
  Serial.print(lastd.hour()); Serial.print(":"); 
  if (lastd.minute() < 10) {Serial.print(0);}
  Serial.println(lastd.minute());
  
  // Test write ++
  // zero it out before you run for real
  // fram.write8(0x0, 0);

  tbi.dt = tzero;

// make sure our 1 byte counter wraps

  if (test == 255) {
    fram.write8(0x0,0);
  } else {
    fram.write8(0x0, test+1);
  }
  
  for (int i=0; i<4; i++) {
    fram.write8(i+1,tbi.tbytes[i]);
  }
  
  // dump the first 32 words of memory!
  uint8_t value;
  for (uint16_t a = 0; a < 32; a++) {
    value = fram.read8(a);
    if ((a % 16) == 0) {
      Serial.print("\n 0x"); Serial.print(a, HEX); Serial.print(": ");
    }
    Serial.print("0x"); 
    if (value < 0x1) 
      Serial.print('0');
    Serial.print(value, HEX); Serial.print(" ");
  }
  Serial.println("\n");
  }



void adc_check () {
  
  int16_t adc0, adc1, adc2, adc3;

  int16_t i=0;
  for (i=0; i< 4; i++) {
  adc0 = ads.readADC_SingleEnded(i);
  delay(10);
  adc1 = ads.readADC_SingleEnded(i);
  delay(10);
  adc2 = ads.readADC_SingleEnded(i);
  Serial.print(i); Serial.print("...read0 (bogus): "); Serial.println(adc0);
  Serial.print(" ...read1 (real ): "); Serial.println(adc1);
  Serial.print(" ...read2 (again): "); Serial.println(adc2);
  }
  Serial.println(" ");

  delay(1000);

  }

  void stateSave () {
    // FRAM bytes 5-END are available for saving data.
    // we would like to save our State.
    int framStart = 5;
    for (int i=0; i < sizeof(systemState); i++) {
      fram.write8(i+framStart,State.bytes[i]);
    }
  }

  void stateRestore () {
    // FRAM bytes 5-END are available for saving data.
    // we would like to restore our State.
    // but only after the first 9 values!
    int framStart = 5;
    byte b;
    for (int i=36; i < sizeof(systemState); i++) {
      b=fram.read8(i+framStart);
      State.bytes[i] = b;
    }
  }

void stateErase () {
  for (int i=0; i< sizeof(systemState); i++) {
    State.bytes[i] = 0;
  }
}

void statePush () {
  for (int i=0; i< sizeof(systemState); i++) {
    LState.bytes[i] = State.bytes[i];
  }
}

