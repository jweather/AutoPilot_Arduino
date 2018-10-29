/*
 * Jeremy Weatherford <jweather@xidus.net>
 * Developed for PropWash Simulation
 */

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F, 16, 2); // 16x2 on pins 2+3 (SDA + SCL)

// IO pins
const int nEnc = 2;
const int pinEncA[nEnc] = {15, 16};  // coarse, then fine
const int pinEncB[nEnc] = {14, 10};  // coarse, then fine

const int nSwitch = 11;
const int pinSwitch[nSwitch]    = { 0, 4, 5, 6, 7, 8, 9, 21, 20, 19, 18 };
const char *nameSwitch[nSwitch]  = { "switch", "AP", "ALT", "BARO", "ARM", "UP", "DOWN", "REV", "APR", "NAV", "HDG" };

const int pinLEDArm = 1;
const int nLEDs = 11; // actual LED and LCD indicators

const int pinLED = 30;

// state
int swLast[nSwitch], swDebounce[nSwitch];
char ledStates[nLEDs];

// encoder decoding
const int8_t encStates[] = {0,0,1,0,1,0,0,-1,0,0,0,0,0,0,-1,0}; // x2 counting (for PropWash Simulation dual-encoder)
int8_t oldEnc[4] = {0, 0, 0, 0};
int8_t rotaryDelta(byte idx) {
  oldEnc[idx] <<= 2;
  oldEnc[idx] |= (digitalRead(pinEncA[idx])<<1 | digitalRead(pinEncB[idx]));
  return encStates[oldEnc[idx] & 0x0f];
}

void setup() {
  pinMode(pinLED, OUTPUT);
  Serial.begin(115200);

  for (int i=0; i<nSwitch; i++) {
    pinMode(pinSwitch[i], INPUT_PULLUP);
    swLast[i] = digitalRead(pinSwitch[i]);
  }

  for (int i=0; i<nEnc; i++) {
    pinMode(pinEncA[i], INPUT_PULLUP);
    pinMode(pinEncB[i], INPUT_PULLUP);
  }
  pinMode(pinLEDArm, OUTPUT);

  // setup LCD
  lcd.init();
  lcd.backlight();
  lcd.home (); // go home
  lcd.print("PropwashSim APv1");
}

// encoders
#define DIR_CCW 0x10
#define DIR_CW 0x20
#define R_START 0x0

#define R_CCW_BEGIN 0x1
#define R_CW_BEGIN 0x2
#define R_START_M 0x3
#define R_CW_BEGIN_M 0x4
#define R_CCW_BEGIN_M 0x5
const byte ttable[6][4] = {
  {R_START_M,            R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
  {R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},
  {R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},
  {R_START_M,            R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
  {R_START_M,            R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},
  {R_START_M,            R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW},
};

byte state[nEnc];

byte encoderDelta(int _i) {
  byte pinstate = (digitalRead(pinEncA[_i]) << 1) | digitalRead(pinEncB[_i]);
  state[_i] = ttable[state[_i] & 0xf][pinstate];
  return (state[_i] & 0x30); // click?
}

int blink;
int lastBlink = 0;

void loop() {
  uint32_t val;

  // time-keeping and blink
  int now = millis();
  if (now - lastBlink > 1000) {
    digitalWrite(pinLED, blink);
    blink = !blink;
    lastBlink = now;
    Serial.write("id=2\r");
  }

 // rotary encoders
  for (int enc=0; enc<nEnc; enc++) {
    byte delta = encoderDelta(enc);
    
    if (delta == DIR_CW) {
      if (enc % 2 == 0) {
        Serial.write("+10\r");
      } else {
        Serial.write("+1\r");
      }
    } else if (delta == DIR_CCW) {
      if (enc % 2 == 0) {
        Serial.write("-10\r");
      } else {
        Serial.write("-1\r");
      }
    }
  }

  // switches
  for (int i=0; i<nSwitch; i++) {
    val = digitalRead(pinSwitch[i]);
    if (swLast[i] == 1 && val == 0) {
      if (swDebounce[i] == 0 || now - swDebounce[i] > 200) {
        Serial.write(nameSwitch[i]); Serial.write("\r");
        swDebounce[i] = now;
      }
    }
    swLast[i] = val;
    if (swDebounce[i] != 0 && now - swDebounce[i] > 200)
      swDebounce[i] = 0;
  }

  // =A123.45\r or =B678.90\r
  if (Serial.available() > 3) {
    while (Serial.read() != '=' && Serial.available()) {}
    char ch = Serial.read();

    switch (ch) {
      case 'L':
        Serial.readBytesUntil('\r', ledStates, nLEDs);
        digitalWrite(pinLEDArm, ledStates[0] == '1');

        lcd.clear();
        lcd.setCursor(13, 1); if (ledStates[9]=='1') lcd.print("YAW");  else lcd.print("   ");
        lcd.setCursor(12, 0); if (ledStates[1]=='1') lcd.print("APPR"); else lcd.print("    ");
        lcd.setCursor(0, 1); if (ledStates[2]=='1') lcd.print("HDG");  else lcd.print("   ");
        lcd.setCursor(4, 1); if (ledStates[3]=='1') lcd.print("NAV");  else lcd.print("   ");
        lcd.setCursor(8, 1); if (ledStates[4]=='1') lcd.print("ALT");  else lcd.print("   ");
        
        lcd.setCursor(0, 0); 
        if (ledStates[5]=='1') 
          lcd.print("FLIGHT DIR "); 
        else if (ledStates[10]=='1')
          lcd.print("AUTOPILOT  "); 
        else 
          lcd.print("           ");
        break;
    }
  }
}
