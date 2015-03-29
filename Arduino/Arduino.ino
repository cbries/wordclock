/*
 * Copyright (C) 2015 Christian Benjamin Ries
 * Contact: mail@christianbenjaminries.de
 * Web: http://www.christianbenjaminries.de
 * Source: https://github.com/cbries/wordclock
 * License: MIT
 */

#include <Wire.h>
#include "LedControl.h"
#include "RTClib.h"

#define ADDRESS_SENSOR_BMP180 0x77

// Used for BMP180
int16_t  ac1, ac2, ac3, b1, b2, mb, mc, md; // Store sensor PROM values from BMP180.
uint16_t ac4, ac5, ac6;                     // Store sensor PROM values from BMP180.
float T, P;                                 // Set global variables for temperature and pressure.

RTC_DS1307 rtc;

int clkpin = 13;
int datapin = 12;
int latchpin = 8;

LedControl lc = LedControl(datapin, clkpin, latchpin, 2);

// P R O T O T Y P E S
void toggleWord(byte *ptr, byte len, bool state=true);
void init_SENSOR();
float pressure(int32_t b5);
int32_t temperature();
uint16_t read_2_bytes(uint8_t code);
// ----

const uint8_t oss = 3;                      // Set oversampling setting
const uint8_t osd = 26;                     // with corresponding oversampling delay


#define SHOW_TEMP_AFTER_MILLIS 30000 // milliseconds
unsigned long __temperatureWallTime = 0;

void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  rtc.begin();
  
  init_SENSOR();
  
  delay(100);
  
  int devices = lc.getDeviceCount();
  for(int address=0; address < devices; ++address)
  {
    lc.shutdown(address, false);
    lc.setIntensity(address, 15);
    lc.clearDisplay(address);
  }
  
  createTimeSentence(0, 0);
  
  __temperatureWallTime = millis() + SHOW_TEMP_AFTER_MILLIS;
}

void clearDisplay()
{
  lc.clearDisplay(0);
  lc.clearDisplay(1);
}

void showFailure()
{
  while(true) {
    lc.setLed(1, 0, 0, false);
    delay(250);
    lc.setLed(1, 0, 0, true);
    delay(250);
  }
}

byte TEMP_ONE_L[]   = {  5,  0,0,2,  0,0,3,  0,0,4,  0,0,5,  0,0,6 };
byte TEMP_TWO_L[]   = {  9,  1,2,2,  1,3,2,  0,0,2,  0,0,3,  0,0,4,  1,3,5,  1,2,6,  1,3,6,  0,0,6 };
byte TEMP_THREE_L[] = { 10,  1,2,2,  1,3,2,  0,0,2,  0,0,3,  0,0,4,  0,0,5,  0,0,6,  1,3,6,  1,2,6,  1,3,4 };
byte TEMP_FOUR_L[]  = {  7,  1,2,2,  1,2,3,  1,2,4,  1,3,4,  0,0,4,  1,3,5,  1,3,6 };

byte TEMP_ZERO_R[]  = { 12,  0,4,2,  0,4,3,  0,4,4,  0,4,5,  0,4,6,  0,3,6,  0,2,6,  0,2,5,  0,2,4,  0,2,3,  0,2,2,  0,3,2 };
byte TEMP_ONE_R[]   = {  5,  0,4,2,  0,4,3,  0,4,4,  0,4,5,  0,4,6 };
byte TEMP_TWO_R[]   = {  9,  0,2,2,  0,3,2,  0,4,2,  0,4,3,  0,4,4,  0,3,5,  0,2,6,  0,3,6,  0,4,6 };
byte TEMP_THREE_R[] = { 10,  0,2,2,  0,3,2,  0,4,2,  0,4,3,  0,4,4,  0,4,5,  0,4,6,  0,3,6,  0,2,6,  0,3,4 };
byte TEMP_FOUR_R[]  = {  7,  0,2,2,  0,2,3,  0,2,4,  0,3,4,  0,4,4,  0,3,5,  0,3,6 };
byte TEMP_FIVE_R[]  = { 12,  0,4,2,  0,3,2,  0,2,2,  0,2,3,  0,2,4,  0,3,4,  0,4,4,  0,4,5,  0,4,6,  0,3,6,  0,3,6,  0,2,6 };
byte TEMP_SIX_R[]   = { 10,  0,2,2,  0,2,3,  0,2,4,  0,2,5,  0,2,6,  0,3,6,  0,4,6,  0,4,5,  0,4,4,  0,3,4 };
byte TEMP_SEVEN_R[] = {  7,  0,2,2,  0,3,2,  0,4,2,  0,4,3,  0,4,4,  0,3,5,  0,2,6 };
byte TEMP_EIGHT_R[] = { 13,  0,2,2,  0,3,2,  0,4,2,  0,4,3,  0,4,4,  0,4,5,  0,4,6,  0,3,6,  0,2,6,  0,2,5,  0,2,4,  0,2,3,  0,3,4 };
byte TEMP_NINE_R[]  = { 10,  0,2,2,  0,3,2,  0,4,2,  0,4,3,  0,4,4,  0,4,5,  0,4,6,  0,2,3,  0,2,4,  0,3,4 };

#define TEMP_NUMBERS_SIZE 50
byte *TEMP_NUMBERS[TEMP_NUMBERS_SIZE][2] {
  { NULL, TEMP_ZERO_R }, { NULL, TEMP_ONE_R }, { NULL, TEMP_TWO_R },
  { NULL, TEMP_THREE_R }, { NULL, TEMP_FOUR_R }, { NULL, TEMP_FIVE_R },
  { NULL, TEMP_SIX_R }, { NULL, TEMP_SEVEN_R }, { NULL, TEMP_EIGHT_R },
  { NULL, TEMP_NINE_R },
  { TEMP_ONE_L, TEMP_ZERO_R }, { TEMP_ONE_L, TEMP_ONE_R }, { TEMP_ONE_L, TEMP_TWO_R },
  { TEMP_ONE_L, TEMP_THREE_R }, { TEMP_ONE_L, TEMP_FOUR_R }, { TEMP_ONE_L, TEMP_FIVE_R },
  { TEMP_ONE_L, TEMP_SIX_R }, { TEMP_ONE_L, TEMP_SEVEN_R }, { TEMP_ONE_L, TEMP_EIGHT_R },
  { TEMP_ONE_L, TEMP_NINE_R },
  { TEMP_TWO_L, TEMP_ZERO_R }, { TEMP_TWO_L, TEMP_ONE_R }, { TEMP_TWO_L, TEMP_TWO_R },
  { TEMP_TWO_L, TEMP_THREE_R }, { TEMP_TWO_L, TEMP_FOUR_R }, { TEMP_TWO_L, TEMP_FIVE_R },
  { TEMP_TWO_L, TEMP_SIX_R }, { TEMP_TWO_L, TEMP_SEVEN_R }, { TEMP_TWO_L, TEMP_EIGHT_R },
  { TEMP_TWO_L, TEMP_NINE_R },
  { TEMP_THREE_L, TEMP_ZERO_R }, { TEMP_THREE_L, TEMP_ONE_R }, { TEMP_THREE_L, TEMP_TWO_R },
  { TEMP_THREE_L, TEMP_THREE_R }, { TEMP_THREE_L, TEMP_FOUR_R }, { TEMP_THREE_L, TEMP_FIVE_R },
  { TEMP_THREE_L, TEMP_SIX_R }, { TEMP_THREE_L, TEMP_SEVEN_R }, { TEMP_THREE_L, TEMP_EIGHT_R },
  { TEMP_THREE_L, TEMP_NINE_R },
  { TEMP_FOUR_L, TEMP_ZERO_R }, { TEMP_FOUR_L, TEMP_ONE_R }, { TEMP_FOUR_L, TEMP_TWO_R },  
  { TEMP_FOUR_L, TEMP_THREE_R }, { TEMP_FOUR_L, TEMP_FOUR_R }, { TEMP_FOUR_L, TEMP_FIVE_R },  
  { TEMP_FOUR_L, TEMP_SIX_R }, { TEMP_FOUR_L, TEMP_SEVEN_R }, { TEMP_FOUR_L, TEMP_EIGHT_R },  
  { TEMP_FOUR_L, TEMP_NINE_R }
};

byte TEMP_DOT[]     = { 4,  0,6,1,  0,6,0,  0,5,0,  0,5,1 };
byte TEMP_CELCIUS[] = { 6,  0,7,3,  0,6,3,  0,6,4,  0,6,5,  0,6,6,  0,7,6 };

bool showTemperature(byte temperature, int showForSecs=5)
{
  if(temperature < 0 || temperature >= TEMP_NUMBERS_SIZE)
  {
    return false;
  }
  
  clearDisplay();
  
  byte *pdot = TEMP_DOT; toggleWord(pdot+1, TEMP_DOT[0], true);
  byte *pcelcius = TEMP_CELCIUS; toggleWord(pcelcius+1, TEMP_CELCIUS[0], true);
    
  byte *pleft = TEMP_NUMBERS[temperature][0];
  byte *pright = TEMP_NUMBERS[temperature][1];
      
  if( pleft != NULL) toggleWord( pleft+1,  *pleft, true);
  if(pright != NULL) toggleWord(pright+1, *pright, true);
  delay(showForSecs*1000);
      
  if( pleft != NULL) toggleWord( pleft+1,  *pleft, false);
  if(pright != NULL) toggleWord(pright+1, *pright, false);
  
  toggleWord(pdot+1, TEMP_DOT[0], false);
  toggleWord(pcelcius+1, TEMP_CELCIUS[0], false);

  delay(5); 

  return true;  
}

/*
[](device, row, column)

I(1,0,0) T(1,1,0) T(1,2,0) I(1,3,0) | S(0,0,0) I(0,1,0) T(0,2,0) W(0,3,0) E(0,4,0) N(0,5,0) T(0,6,0) Y(0,7,0)
Q(1,0,1) U(1,1,1) A(1,2,1) R(1,3,1) | T(0,0,1) E(0,1,1) R(0,2,1) H(0,3,1) A(0,4,1) L(0,5,1) F(0,6,1) M(0,7,1)
T(1,0,2) E(1,1,2) N(1,2,2) F(1,3,2) | I(0,0,2) V(0,1,2) E(0,2,2) E(0,3,2) P(0,4,2) A(0,5,2) S(0,6,2) T(0,7,2)
T(1,0,3) O(1,1,3) A(1,2,3) T(1,3,3) | W(0,0,3) E(0,1,3) L(0,2,3) V(0,3,3) E(0,4,3) O(0,5,3) N(0,6,3) E(0,7,3)
T(1,0,4) W(1,1,4) O(1,2,4) T(1,3,4) | H(0,0,4) R(0,1,4) E(0,2,4) E(0,3,4) F(0,4,4) O(0,5,4) U(0,6,4) R(0,7,4)
F(1,0,5) I(1,1,5) V(1,2,5) E(1,3,5) | S(0,0,5) I(0,1,5) X(0,2,5) S(0,3,5) E(0,4,5) V(0,5,5) E(0,6,5) N(0,7,5)
E(1,0,6) I(1,1,6) G(1,2,6) H(1,3,6) | T(0,0,6) N(0,1,6) I(0,2,6) N(0,3,6) E(0,4,6) T(0,5,6) E(0,6,6) N(0,7,6)
N(1,0,7) E(1,1,7) L(1,2,7) E(1,3,7) | V(0,0,7) E(0,1,7) N(0,2,7) D(0,3,7) D(0,4,7) A(0,5,7) T(0,6,7) E(0,7,7)

T(1,4,7) I(1,4,6), O(1,4,5) '(1,4,4) C(1,4,3) L(1,4,2) O(1,4,1)   C(1,4,0)   K(1,5,0) M(1,5,1) E(1,5,2) A(1,5,3)
*/    

/* IT */        byte nIT = 2;       byte IT[]      = { 1,0,0,  1,1,0 };
/* IS */        byte nIS = 2;       byte IS[]      = { 1,3,0,  0,0,0 };
/* TWENTY */    byte nTWENTY = 6;   byte TWENTY[]  = { 0,2,0,  0,3,0,  0,4,0,  0,5,0,  0,6,0,  0,7,0 };
/* QUARTER */   byte nQUARTER = 7;  byte QUARTER[] = { 1,0,1,  1,1,1,  1,2,1,  1,3,1,  0,0,1,  0,1,1,  0,2,1 };
/* HALF */      byte nHALF = 4;     byte HALF[]    = { 0,3,1,  0,4,1,  0,5,1,  0,6,1 };
/* TEN */       byte nTEN = 3;      byte TEN[]     = { 1,0,2,  1,1,2,  1,2,2 };
/* FIVE */      byte nFIVE = 4;     byte FIVE[]    = { 1,3,2,  0,0,2,  0,1,2,  0,2,2 };
/* PAST */      byte nPAST = 4;     byte PAST[]    = { 0,4,2,  0,5,2,  0,6,2,  0,7,2 };
/* TO */        byte nTO = 2;       byte TO[]      = { 1,0,3,  1,1,3 };
/* TWELVE */    byte nTWELVE = 6;   byte TWELVE[]  = { 1,3,3,  0,0,3,  0,1,3,  0,2,3,  0,3,3,  0,4,3 };
/* ONE */       byte nONE = 3;      byte ONE[]     = { 0,5,3,  0,6,3,  0,7,3 };
/* TWO */       byte nTWO = 3;      byte TWO[]     = { 1,0,4,  1,1,4,  1,2,4 };
/* THREE */     byte nTHREE = 5;    byte THREE[]   = { 1,3,4,  0,0,4,  0,1,4,  0,2,4,  0,3,4 };
/* FOUR */      byte nFOUR = 4;     byte FOUR[]    = { 0,4,4,  0,5,4,  0,6,4,  0,7,4 };
/* FIVE */      byte nFIVE2 = 4;    byte FIVE2[]   = { 1,0,5,  1,1,5,  1,2,5,  1,3,5 };
/* SIX */       byte nSIX = 3;      byte SIX[]     = { 0,0,5,  0,1,5,  0,2,5 };
/* SEVEN */     byte nSEVEN = 5;    byte SEVEN[]   = { 0,3,5,  0,4,5,  0,5,5,  0,6,5,  0,7,5 };
/* EIGHT */     byte nEIGHT = 5;    byte EIGHT[]   = { 1,0,6,  1,1,6,  1,2,6,  1,3,6,  0,0,6 };
/* NINE */      byte nNINE = 4;     byte NINE[]    = { 0,1,6,  0,2,6,  0,3,6,  0,4,6 };
/* TEN */       byte nTEN2 = 3;     byte TEN2[]    = { 0,5,6,  0,6,6,  0,7,6 };
/* ELEVEN */    byte nELEVEN = 6;   byte ELEVEN[]  = { 1,1,7,  1,2,7,  1,3,7,  0,0,7,  0,1,7,  0,2,7};
/* AT */        byte nAT = 2;       byte AT[]      = { 0,5,7,  0,6,7 };
/* O'CLOCK */   byte nOCLOCK = 7;   byte OCLOCK[]  = { 1,4,5,  1,4,4,  1,4,3,  1,4,2,  1,4,1,  1,4,0, 1,5,0 };

#define MAXWORDS 6

byte numberOfWords = 5;
byte *tn[MAXWORDS] = { &nIT, &nIS, &nHALF, &nPAST, &nEIGHT, NULL};
byte *tstr[MAXWORDS] = { IT, IS, HALF, PAST, EIGHT, NULL };
byte recentHour = -1;
byte recentMinutes = -1;
bool recentTimeChanged = false;

void clearSentence()
{
  numberOfWords = 0;
  for(byte i=0; i < MAXWORDS; ++i)
  {
    tn[i] = NULL;
    tstr[i] = NULL;
  }
}

void appendWord(byte *ptrWord, byte *ptrLength)
{
  if(numberOfWords >= MAXWORDS) { showFailure(); }  
  if(ptrWord == NULL || ptrLength == NULL) { showFailure(); }
  
  tn[numberOfWords] = ptrLength;
  tstr[numberOfWords] = ptrWord;
  
  ++numberOfWords;
}

void address(byte *ptr, int *device, int *row, int *col)
{
  if(ptr == NULL) 
  {
    showFailure();
  }
  
  if(ptr != NULL)
  {
      *device = *ptr;
      *row    = *(ptr + 1);
      *col    = *(ptr + 2);
  }
}

void toggleWord(byte *ptr, byte len, bool state)
{
  for(byte j=0; j < len; ++j)
  {
    int device, row, col;
    address(ptr + j*3, &device, &row, &col);
    lc.setLed(device, row, col, state);
  }
}

bool ptrToHour(byte hour, byte **ptrHour, byte **ptrLength)
{
  switch(hour)
  {
    case 0: case 12: case 24: *ptrHour = TWELVE; *ptrLength = &nTWELVE; return true;
    case 1: case 13:          *ptrHour = ONE;    *ptrLength = &nONE;    return true;
    case 2: case 14:          *ptrHour = TWO;    *ptrLength = &nTWO;    return true;
    case 3: case 15:          *ptrHour = THREE;  *ptrLength = &nTHREE;  return true;
    case 4: case 16:          *ptrHour = FOUR;   *ptrLength = &nFOUR;   return true;
    case 5: case 17:          *ptrHour = FIVE2;  *ptrLength = &nFIVE2;  return true;
    case 6: case 18:          *ptrHour = SIX;    *ptrLength = &nSIX;    return true;
    case 7: case 19:          *ptrHour = SEVEN;  *ptrLength = &nSEVEN;  return true;
    case 8: case 20:          *ptrHour = EIGHT;  *ptrLength = &nEIGHT;  return true;
    case 9: case 21:          *ptrHour = NINE;   *ptrLength = &nNINE;   return true;
    case 10: case 22:         *ptrHour = TEN2;   *ptrLength = &nTEN2;   return true;
    case 11: case 23:         *ptrHour = ELEVEN; *ptrLength = &nELEVEN; return true;
  }
  *ptrHour = NULL;
  *ptrLength = NULL;
  return false;
}

void createTimeSentence(byte hour, byte minute)
{
  if(hour == recentHour && minute == recentMinutes)
  {
    recentTimeChanged = false;
    return ;
  }
  else
  {
    recentTimeChanged = true;
    recentHour = hour;
    recentMinutes = minute;
  }
  
  hideTimeSentence();
  clearSentence();
  
  byte *ptrHour = NULL, *ptrHourLength = NULL;
  bool res = ptrToHour(hour, &ptrHour, &ptrHourLength);
  
  if(ptrHour == NULL)
  {
    showFailure();
    return ; 
  }

  appendWord(IT, &nIT);
  appendWord(IS, &nIS);
    
  // IT IS FIVE PAST nnn
  if(minute > 3 && minute <= 8) {
    appendWord(FIVE, &nFIVE);
    appendWord(PAST, &nPAST);
    appendWord(ptrHour, ptrHourLength);
  }
  // IT IS TEN PAST nnn
  if(minute > 8 && minute <= 13) { 
    appendWord(TEN, &nTEN);
    appendWord(PAST, &nPAST);
    appendWord(ptrHour, ptrHourLength);
  }
  // IT IS QUARTER PAST nnn
  if(minute > 13 && minute <= 18) { 
    appendWord(QUARTER, &nQUARTER);
    appendWord(PAST, &nPAST);
    appendWord(ptrHour, ptrHourLength);
  }
  // IT IS TWENTY PAST nnn
  if(minute > 18 && minute <= 23) { 
    appendWord(TWENTY, &nTWENTY);
    appendWord(PAST, &nPAST);
    appendWord(ptrHour, ptrHourLength);
  }
  // IT IS TWENTY FIVE PAST nnn
  if(minute > 23 && minute <= 28) { 
    appendWord(TWENTY, &nTWENTY);
    appendWord(FIVE, &nFIVE);
    appendWord(PAST, &nPAST);
    appendWord(ptrHour, ptrHourLength);
  }
  // IT IS HALF PAST nnn
  if(minute > 28 && minute <= 33) { 
    appendWord(HALF, &nHALF);
    appendWord(PAST, &nPAST);
    appendWord(ptrHour, ptrHourLength);
  }
  // IT IS TWENTY FIVE TO nnn
  if(minute > 33 && minute <= 38) {
    ptrToHour(hour+1, &ptrHour, &ptrHourLength);
    appendWord(TWENTY, &nTWENTY);
    appendWord(FIVE, &nFIVE);
    appendWord(TO, &nTO);
    appendWord(ptrHour, ptrHourLength);
  }
  // IT IS TWENTY TO nnn
  if(minute > 38 && minute <= 44) {
    ptrToHour(hour+1, &ptrHour, &ptrHourLength);
    appendWord(TWENTY, &nTWENTY);
    appendWord(TO, &nTO);
    appendWord(ptrHour, ptrHourLength);
  }
  // IT IS QUARTER TO nnn
  if(minute > 44 && minute <= 48) { 
    ptrToHour(hour+1, &ptrHour, &ptrHourLength);
    appendWord(QUARTER, &nQUARTER);
    appendWord(TO, &nTO);
    appendWord(ptrHour, ptrHourLength);
  }
  // IT IS TEN TO nnn
  if(minute > 48 && minute <= 53) {
    ptrToHour(hour+1, &ptrHour, &ptrHourLength);
    appendWord(TEN, &nTEN);
    appendWord(TO, &nTO);
    appendWord(ptrHour, ptrHourLength);
  }
  // IT IS FIVE TO nnn
  if(minute > 53 && minute <= 58) {
    ptrToHour(hour+1, &ptrHour, &ptrHourLength);
    appendWord(FIVE, &nFIVE);
    appendWord(TO, &nTO);
    appendWord(ptrHour, ptrHourLength);
  }
  // IT IS nnn O'CLOCK
  if(minute >= 59 || minute <= 3)
  {
    if(minute >= 59)
    {
      ptrToHour(hour+1, &ptrHour, &ptrHourLength);
    }

    appendWord(ptrHour, ptrHourLength);
    appendWord(OCLOCK, &nOCLOCK);
  }
}

void showTimeSentence(bool visiblestate=true)
{
  for(byte i=0; i < numberOfWords; ++i)
  {
    byte n = *(tn[i]);
    byte *p = tstr[i];
    
    toggleWord(p, n, visiblestate);
  }
}

void hideTimeSentence()
{
  showTimeSentence(false);
}

inline bool timeSentenceChanged() { return recentTimeChanged; }

bool checkSerialForTime(int *hour, int *minutes)
{
  String in = "";
    
  int n = 0;
    
  while(Serial.available() > 0)
  {
    byte ch = Serial.read();
    if(isDigit(ch))
    {
      in += (char) ch;
    }
    
    if(ch == ',' || ch == '\r' || ch == '\n')
    {
      switch(n)
      {
        // hour
        case 0: *hour = in.toInt(); break;
        // minutes
        case 1: *minutes = in.toInt(); break;
      }
      
      in = "";  ++n;
    }
  }
  
  Serial.flush();
  
  return (*hour != 0 && *minutes != 0);
}

bool checkSerialForAdjust()
{
  while(Serial.available() > 0)
  {
    byte c = Serial.read();
    Serial.println((char) c);
    if(c == 's')
    {
      unsigned int tt = (unsigned int) Serial.parseInt();
      Serial.println(tt);
      if(tt > 0)
      {
        rtc.adjust(DateTime(tt));
        
        DateTime rtcdt = rtc.now();
        
        Serial.print("DateTime adjusted -> ");
        Serial.print(rtcdt.hour()); Serial.print(":"); Serial.print(rtcdt.minute()); Serial.print(" @ ");
        Serial.print(rtcdt.day()); Serial.print("."); Serial.print(rtcdt.month()); Serial.print("."); Serial.println(rtcdt.year());
      }
      
      return true;
    }
  }
  
  return false;
}

int __blinkN = 0;
void blinkSeconds()
{
  // N(1,0,7)    E(0,7,7)
  
  if(__blinkN == 10)
  {
    __blinkN = 0;
    lc.setLed(1, 0, 7, true);
    delay(50);
    lc.setLed(1, 0, 7, false);
  }
  
  ++__blinkN;
  
  delay(100);
}

void loop() 
{ 
  bool useRtc = rtc.isrunning();
  
  if(useRtc == false)
  {
    rtc.begin();
    delay(1000);
    useRtc = rtc.isrunning();
    if(useRtc == false)
    {
      Serial.println("RTC is NOT running!");
    }
  }
  
  bool ret = checkSerialForAdjust();
  if(ret == true)
  {
    ;
  }
  
  if(useRtc == true)
  {
    DateTime now = rtc.now();
    int hour = now.hour();
    int minutes = now.minute();
    int seconds = now.second();
    
    Serial.print(hour); Serial.print(":"); 
    Serial.print(minutes); Serial.print(":");
    Serial.println(seconds);
    Serial.flush();
    
    createTimeSentence((byte) hour, (byte) minutes);
    if(timeSentenceChanged())
    {
      showTimeSentence();
    }
  }  
  else
  {
    int hour = 0, minutes = 0;
    bool res = checkSerialForTime(&hour, &minutes);
    if(res == true)
    {
      Serial.print(hour); Serial.print(":"); Serial.println(minutes);
      Serial.flush();
      createTimeSentence((byte) hour, (byte) minutes);
      if(timeSentenceChanged())
      {
        showTimeSentence();
      }
    }
  }
  
  if(millis() > __temperatureWallTime)
  {
    int32_t b5 = temperature();
             P = pressure(b5);
    
    /*
      Temperature:   T  °C
      Temperature:   1.8 * T + 32.0  Fahrenheit
      
      Pressure:  P  mbar
      Pressure:  P * 0.0295299830714  inHg
    */
    
    Serial.print("Temperature: ");
    Serial.print(T, 2);
    Serial.print(" C, ");
    Serial.print(1.8 * T + 32.0, 2);
    Serial.println(" F");
    
    showTemperature((byte) T + 1, 3 /* seconds */);
    
    __temperatureWallTime = millis() + SHOW_TEMP_AFTER_MILLIS;
  
    clearDisplay();
    showTimeSentence();
  }
  
  //blinkSeconds();
  
  delay(250);
}

// #################################################################################################
//  BMP180
//  Stolen from somewhere :-) 
//  Used in every uC project.
// #################################################################################################

// Ultra Low Power       OSS = 0, OSD =  5ms
// Standard              OSS = 1, OSD =  8ms
// High                  OSS = 2, OSD = 14ms
// Ultra High Resolution OSS = 3, OSD = 26ms

void init_SENSOR()
{
  ac1 = read_2_bytes(0xAA);
  ac2 = read_2_bytes(0xAC);
  ac3 = read_2_bytes(0xAE);
  ac4 = read_2_bytes(0xB0);
  ac5 = read_2_bytes(0xB2);
  ac6 = read_2_bytes(0xB4);
  b1  = read_2_bytes(0xB6);
  b2  = read_2_bytes(0xB8);
  mb  = read_2_bytes(0xBA);
  mc  = read_2_bytes(0xBC);
  md  = read_2_bytes(0xBE);
}

/**********************************************
  Calcualte pressure readings
 **********************************************/
float pressure(int32_t b5)
{
  int32_t x1, x2, x3, b3, b6, p, UP;
  uint32_t b4, b7; 

  UP = read_pressure();                         // Read raw pressure

  b6 = b5 - 4000;
  x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;
  b3 = (((uint32_t)(ac1 * 4 + x3) << oss) + 2) >> 2;
  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;
  b7 = ((uint32_t)UP - b3) * (50000 >> oss);
  if(b7 < 0x80000000) { p = (b7 << 1) / b4; } else { p = (b7 / b4) << 1; } // or p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  return (p + ((x1 + x2 + 3791) >> 4)) / 100.0f; // Return pressure in mbar
}

/**********************************************
  Read uncompensated temperature
 **********************************************/
int32_t temperature()
{
  int32_t x1, x2, b5, UT;

  Wire.beginTransmission(ADDRESS_SENSOR_BMP180); // Start transmission to device 
  Wire.write(0xf4);                       // Sends register address
  Wire.write(0x2e);                       // Write data
  Wire.endTransmission();                 // End transmission
  delay(5);                               // Datasheet suggests 4.5 ms
  
  UT = read_2_bytes(0xf6);                // Read uncompensated TEMPERATURE value

  // Calculate true temperature
  x1 = (UT - (int32_t)ac6) * (int32_t)ac5 >> 15;
  x2 = ((int32_t)mc << 11) / (x1 + (int32_t)md);
  b5 = x1 + x2;
  T  = (b5 + 8) >> 4;
  T = T / 10.0;                           // Temperature in celsius 
  return b5;  
}

/**********************************************
  Read uncompensated pressure value
 **********************************************/
int32_t read_pressure()
{
  int32_t value; 
  Wire.beginTransmission(ADDRESS_SENSOR_BMP180);   // Start transmission to device 
  Wire.write(0xf4);                         // Sends register address to read from
  Wire.write(0x34 + (oss << 6));            // Write data
  Wire.endTransmission();                   // SEd transmission
  delay(osd);                               // Oversampling setting delay
  Wire.beginTransmission(ADDRESS_SENSOR_BMP180);
  Wire.write(0xf6);                         // Register to read
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS_SENSOR_BMP180, 3);      // Request three bytes
  if(Wire.available() >= 3)
  {
    value = (((int32_t)Wire.read() << 16) | ((int32_t)Wire.read() << 8) | ((int32_t)Wire.read())) >> (8 - oss);
  }
  return value;                             // Return value
}

/**********************************************
  Read 1 byte from the BMP sensor
 **********************************************/
uint8_t read_1_byte(uint8_t code)
{
  uint8_t value;
  Wire.beginTransmission(ADDRESS_SENSOR_BMP180);         // Start transmission to device 
  Wire.write(code);                               // Sends register address to read from
  Wire.endTransmission();                         // End transmission
  Wire.requestFrom(ADDRESS_SENSOR_BMP180, 1);            // Request data for 1 byte to be read
  if(Wire.available() >= 1)
  {
    value = Wire.read();                          // Get 1 byte of data
  }
  return value;                                   // Return value
}

/**********************************************
  Read 2 bytes from the BMP sensor
 **********************************************/
uint16_t read_2_bytes(uint8_t code)
{
  uint16_t value;
  Wire.beginTransmission(ADDRESS_SENSOR_BMP180);         // Start transmission to device 
  Wire.write(code);                               // Sends register address to read from
  Wire.endTransmission();                         // End transmission
  Wire.requestFrom(ADDRESS_SENSOR_BMP180, 2);            // Request 2 bytes from device
  if(Wire.available() >= 2)
  {
    value = (Wire.read() << 8) | Wire.read();     // Get 2 bytes of data
  }
  return value;                                   // Return value
}
// #################################################################################################
//  BMP180    END   END   END   END
// #################################################################################################

