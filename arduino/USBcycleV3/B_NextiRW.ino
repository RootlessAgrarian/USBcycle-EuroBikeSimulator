//
////
////// BASIC SERIAL COMMUNICATION FUNCTIONS
////
//
//
//  basic listen function
//

boolean listen(){
  
  char _bite;
  char _end = 0xff;//end of file x3
  // int countEnd = 0;

  while(Serial1.available()>0){
          _bite = Serial1.read();
          nexti_msg += _bite;
          if(_bite == _end){
                  nexti_ffct++;
          } else {
                  nexti_ffct = 0;

          }
  }
  if(nexti_ffct == 3){
        nexti_ffct = 0;
        return(1);
  } else {
        return(0);
  }
}

void send_cmd() {
  // Serial.println(cmd);
  Serial1.print(nexti_cmd);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  ackWaiting = 1;           // we are not using this (yet)
}//end sendCommand


String logMessage() {

  char _end = 0xff;//end of file x3
  int countEnd = 0;
  /*
   here are the codes:
   -- ACKS (nextion defined):
   x00 invalid instruction ack
   x01 successful exec ack
   x02 invalid component id
   x03 invalid page id
   x04 invalid picture id
   x05 invalid font id
   x11 invalid baud rate
   x12 invalid curve ctrl or channel num
   x1A invalid var name
   x1B invalid var operation
   x1C failed to assign attribute
   x1D EEPROM operation failed
   x1E invalid parameter count
   x1F I/O operation failed
   x20 undefined escape chars
   x23 variable name too long
   -- EVENTS AND REQUESTED DATA RETURNS (nextion defined)
   x65 touch event, next bytes are: PageId, ItemId, Touch Event bool, END
   x66 current page id: one byte pageid plus END
   x67 touch coord data, Xhi, Xlo, Yhi, Ylo, Touch Event State bool, END
   x68 touch event in sleep mode: xhi, xlo, yhi, ylo, State bool, END
   x70 string variable data: N bytes of ASCII plus END
   x71 num variable data: 4 bytes little endian, END
   x86 entering sleep mode:  END
   x87 wake up: END
   x88 System startup success: END
   x89 starting SD card read: END
   -- CUSTOM (defined by me for my app)
   x72 touch event *plus* data:  page id, item id, item value 4 bytes little endian, END
   x73 starts a long text message (component dump output begin)
   x74 ends the long text message (component dump output end)
   */

  // handy union for decoding numeric values from Nexti such as coords
  String temp;
  String temp2;
  byte page;
  union two_byte_int {
    unsigned int ii; //to gain access to bytes of int
    unsigned char cc[4]; //as many as needed to cover the largest other element
  } tbi;
  unsigned int x;
  unsigned int y;

// here is where I guess we would check for Expect
// and make complaining noises if we don't get the byte
// we are looking for?
  if (nexti_expect != '?') {
  if (nexti_msg[0] != nexti_expect) {
    Serial.println(F("Warning, did not get expected message from Nextion"));
  }
  nexti_expect = '?';
  }

  switch (nexti_msg[0]) {
  case '^'://0x88, boot OK
        temp = "x88 Successful Boot";
        return temp;
        break;
  case 'r'://0x72, my own addition to the protocol
        temp = "x72 button " + String(nexti_msg[1], DEC) + ":" + String(nexti_msg[2],DEC) + " = " + String(nexti_msg[3],DEC);
        return temp;
        break;
  case 'f'://0x66
        //Serial.print(String(nexti_msg[1], HEX));
        temp = "\nx66 Page " + String(nexti_msg[1], DEC);
        page = nexti_msg[1];
        nexti_page = page;
        tick = 1;
        chirp(1);
        delay(20);        // give it a chance to settle
        refreshPage(1);
        return temp;
        break;
  case 'g'://0x67
  //    Nextion sends bytes in order High, Low.
        tbi.cc[1] = nexti_msg[1];
        tbi.cc[0] = nexti_msg[2];
        x = tbi.ii;
        tbi.cc[1] = nexti_msg[3];
        tbi.cc[0] = nexti_msg[4];
        y = tbi.ii;
        temp = "x67 XY Touch at " + String(nexti_msg[1], DEC) + "," + String(nexti_msg[2], DEC) + "," + String(nexti_msg[3], DEC) + "," + String(nexti_msg[4], DEC) +","+ String(nexti_msg[5], DEC) + " = " + x + "," + y;
        touchDispatch(nexti_page,nexti_id,nexti_msg[5],x,y);
        return temp;
        break;
  case 'h'://0x68
        temp = "x68 SleepTouch at " + String(nexti_msg[2], DEC) + "," + String(nexti_msg[4], DEC) +","+ String(nexti_msg[5], DEC);
        return temp;
        break;
  case 'p'://0x70
        temp = "x70 String Data: " + nexti_msg.substring(1, nexti_msg.length()-3);
        return temp;
        break;
  case 'e'://0x65   Same as default -.-
        temp = "x65 Simple Touch " + String(nexti_msg[1], DEC) + ":" + String(nexti_msg[2],DEC) + " = " + String(nexti_msg[3],DEC);
        nexti_page = nexti_msg[1];
        nexti_id = nexti_msg[2];
        chirp(0);
        touchDispatch(nexti_msg[1],nexti_msg[2],nexti_msg[3],0,0);
        return temp;
        break;
  case 's':       // this is my own addition to the protocol
        Serial.println(F("\n0x73 BEGIN dump data from nextion!"));
        nexti_dump = 1;
        nexti_msg[0] = 0x20;
        break;
  case 't':       // this is my own addition to the protocol
        Serial.println(F("0x74 END dump data from nextion!"));
        nexti_dump = 0;
        break;
  default:
        if ((nexti_msg[0] <= 0x23) && (!nexti_back[int(nexti_msg[0])].msg.equals(""))) {
          // Serial.println(nexti_back[nexti_msg[0]]);
          return nexti_back[int(nexti_msg[0])].msg;
        }
        if (!nexti_dump) {
          Serial.println(F("Got non-standard start byte from nextion!"));
        } 
        for(uint8_t i = 0; i<nexti_msg.length(); i++){
          if(nexti_msg[i] == _end) {
            countEnd++;
          } else {
            temp2 += nexti_msg[i];
          }
          temp += String(nexti_msg[i], HEX);//add hexadecimal value
          if (countEnd == 3) {
                // Serial.println(F("End of nexti msg found"));
                if (nexti_dump) {
                  return temp2;
                } else {
                  return temp;
                }
          }//end if
          temp += " ";// For easy visualization of hex sequence
        }//end for
        return temp;  // in case you for some reason have an incomplete transmission?
        break;
  }//end switch 
  return "";
}//end logMessage

