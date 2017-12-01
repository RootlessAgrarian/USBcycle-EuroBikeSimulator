/*
 * 	USBcycle library shared by Leo1 and Leo2 and any other
 * 	USBcycle controllers
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
char * TimeToString(unsigned long t)
{
 t = t/1000;
 static char str[12];
 long h = t / 3600;
 t = t % 3600;
 int m = t / 60;
 int s = t % 60;
 sprintf(str, "%04ld:%02d:%02d", h, m, s);
 return str;
}

void log_print(const __FlashStringHelper *str, boolean flag) {
  
   Serial.print(TimeToString(millis()));     \ 
   Serial.print(F(" --  "));    \
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
  // which can be 0 through 3 -- counting downward from Red.
  while(1) {
    blinky(which, 2);
    delay(500);
  }
}


