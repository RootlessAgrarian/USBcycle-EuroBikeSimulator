// beep types are key click, upTone, downTone
const int beeps[3] = {
NOTE_CS4,NOTE_CS5,NOTE_CS3
};

// tone starting value, Hz
// notes in the melody:
// shave and a haircut
int melody[] = { 262,196, 196, 220, 196, 0,  247, 262
  /*NOTE_C4, NOTE_G3,NOTE_G3, NOTE_A3, NOTE_G3,0, NOTE_B3, NOTE_C4*/ };
int notdur[] = {  4, 8, 8, 4,4,4,4,4 };
// I want to ride my bicycle
int melody2[] = {NOTE_C4, NOTE_E4, NOTE_F4, NOTE_F4, NOTE_G4, NOTE_G4, NOTE_C5, NOTE_C5,
                 NOTE_C4, NOTE_E4, NOTE_F4, NOTE_F4, NOTE_G4, NOTE_G4};
int notdur2[] = {4,4,4,4,4,4,4,4,4,4,4,4,4,1};
// note durations: 4 = quarter note, 8 = eighth note, etc.:

// timers TC0 TC1 TC2   channels 0-2 ids 0-2  3-5  6-8     AB 0 1
// use TC1 channel 0 
#define TONE_TIMER TC1
#define TONE_CHNL 0
#define TONE_IRQ TC3_IRQn

// TIMER_CLOCK4   84MHz/128 with 16 bit counter give 10 Hz to 656KHz
//  piano 27Hz to 4KHz

static uint8_t pinEnabled[PINS_COUNT];
static uint8_t TCChanEnabled = 0;
static boolean pin_state = false ;
static Tc *chTC = TONE_TIMER;
static uint32_t chNo = TONE_CHNL;

volatile static int32_t toggle_count;
static uint32_t tone_pin;

// frequency (in hertz) and duration (in milliseconds).

void tone(uint32_t ulPin, uint32_t frequency, int32_t duration)
{
    const uint32_t rc = VARIANT_MCK / 256 / frequency; 
    tone_pin = ulPin;
    toggle_count = 0;  // strange  wipe out previous duration
    if (duration > 0 ) toggle_count = 2 * frequency * duration / 1000;
     else toggle_count = -1;

    if (!TCChanEnabled) {
      pmc_set_writeprotect(false);
      pmc_enable_periph_clk((uint32_t)TONE_IRQ);
      TC_Configure(chTC, chNo,
        TC_CMR_TCCLKS_TIMER_CLOCK4 |
        TC_CMR_WAVE |         // Waveform mode
        TC_CMR_WAVSEL_UP_RC ); // Counter running up and reset when equals to RC
  
      chTC->TC_CHANNEL[chNo].TC_IER=TC_IER_CPCS;  // RC compare interrupt
      chTC->TC_CHANNEL[chNo].TC_IDR=~TC_IER_CPCS;
       NVIC_EnableIRQ(TONE_IRQ);
                         TCChanEnabled = 1;
    }
    if (!pinEnabled[ulPin]) {
      pinMode(ulPin, OUTPUT);
      pinEnabled[ulPin] = 1;
    }
    TC_Stop(chTC, chNo);
                TC_SetRC(chTC, chNo, rc);    // set frequency
    TC_Start(chTC, chNo);
}

void noTone(uint32_t ulPin)
{
  TC_Stop(chTC, chNo);  // stop timer
  digitalWrite(ulPin,LOW);  // no signal on pin
}

// timer ISR  TC1 ch 0
void TC3_Handler ( void ) {
  TC_GetStatus(TC1, 0);
  if (toggle_count != 0){
    // toggle pin  TODO  better
    digitalWrite(tone_pin,pin_state= !pin_state);
    if (toggle_count > 0) toggle_count--;
  } else {
    noTone(tone_pin);
  }
}


void playTune(int pinid, int tempo) {

    // this shouldn't really be hardwired.
    // the tune should be a struct including its length
    // in fact there should be an array of pointers to structs
    // tempo if larger than 0 indicates an amount to subtract
    // from the default note duration basis of 1000 ms.
    // to double the pace, subtract 500.  crude, but it'll do for now.
    for (int thisNote = 0; thisNote < 14; thisNote++) {
    // to calculate the note duration, take one second 
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = (1000 - tempo)/notdur2[thisNote];
    tone(pinid, melody2[thisNote],noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.40;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(pinid);
  }
  delay(3000);
}

void chirp(int which) {
  // flavours of chirps:
  // high, low (up, down)
  // regular (click)
  int pinid = piezoPin;
  int durat = 1000/9;
  if (!which) {
    durat = 1000/15;
  }

  int chirpTone = beeps[which];
  
  tone(pinid,chirpTone,durat);
  
}

