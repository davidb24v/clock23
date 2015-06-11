const int ShiftPWM_latchPin=8;
const bool ShiftPWM_invertOutputs = false;
const bool ShiftPWM_balanceLoad = false;

#include <ShiftPWM.h>   // include ShiftPWM.h after setting the pins!

unsigned char maxBrightness = 255;
unsigned char pwmFrequency = 100;
unsigned int numRegisters = 4;
unsigned int numOutputs = numRegisters*8;

unsigned int ldr[5];
int ldrNext = 0;
int currentBrightness = 0;
const int minBrightness = 1;

// Values for each digit
byte digits[11]= {63, 6, 91, 79, 102, 109, 125, 7, 127, 111, 0};

// Hold 4 digits to set time
String inputTime = "";         // a string to hold incoming data
boolean timeComplete = false;  // whether the string is complete

// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include <Wire.h>
#include "RTClib.h"
RTC_DS1307 rtc;

unsigned int brightness() {
  ldr[ldrNext] = analogRead(A0)/4;
  ldrNext = (ldrNext+1) % 5;
  return max((ldr[0]+ldr[1]+ldr[2]+ldr[3]+ldr[4])/5, minBrightness);
}

// Routines to map numbers into the display
int segment = 0;
void resetDisplay() {
  segment = 0;
}

void displayNextDigit(int singleDigit) {
  int digit = digits[singleDigit];
  unsigned int bitMask = 1;
  for (int i=segment; i<segment+8; i++) {
    unsigned int val = digit & bitMask;
    if ( val > 0 ) val = 1;
    ShiftPWM.SetOne(i, val*currentBrightness);
    bitMask *= 2;
  }
  segment += 8;
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the string if it is a digit
    if ( inChar >= '0' && inChar <= '9' ) {}
      inputTime += inChar;
    }
    if (inputTime.length() == 4) {
      timeComplete = true;
  }
}

volatile bool tick = false;

void ticktock() {
  tick = true;
}

void setup(){
  while(!Serial){
    delay(100); 
  }
  Serial.begin(115200);

  inputTime.reserve(4);

  Wire.begin();
  rtc.begin();

  // Sets the number of 8-bit registers that are used.
  ShiftPWM.SetAmountOfRegisters(numRegisters);
  ShiftPWM.Start(pwmFrequency, maxBrightness);

  // Get a light reading (discard very first one)
  ldr[0] = analogRead(A0);
  for (int i=0; i<5; i++) {
    delay(25);
    ldr[i] = analogRead(A0)/4;
  }

  // Initial clock setup (not very accurate)
  if (! rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Interrupt once every second
  rtc.writeSqwPinMode(SquareWave1HZ);
  attachInterrupt(0, ticktock, FALLING);
}

void loop() {
  if ( timeComplete ) {
    timeComplete = false;
    int h = 10*(inputTime[0]-'0') + (inputTime[1]-'0');
    int m = 10*(inputTime[2]-'0') + (inputTime[3]-'0');
    rtc.adjust(DateTime(2015, 1, 1, h, m, 0));
    Serial.print(h, DEC);
    Serial.print(":");
    Serial.println(m, DEC);
    inputTime = "";
  }

  if ( tick ) {
    tick = false;
    currentBrightness = brightness();

    DateTime now = rtc.now();

    int hh = now.hour();
    int h1 = hh/10;
    int h2 = hh % 10;

    int mm = now.minute();
    int m1 = mm/10;
    int m2 = mm % 10;

    Serial.print(hh, DEC);
    Serial.print(":");
    Serial.println(mm, DEC);

    // Set the digits
    resetDisplay();
    // Use blank for leading zero
    if ( h1 == 0 ) h1 = 10;
    displayNextDigit(h1);
    displayNextDigit(h2);
    displayNextDigit(m1);
    displayNextDigit(m2);

  }
}


