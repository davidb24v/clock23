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

// Values for each digit
byte digits[11]= {63, 6, 91, 79, 102, 109, 125, 7, 127, 111, 0};

// Hold 4 digits to set time
String inputTime = "";         // a string to hold incoming data
boolean timeComplete = false;  // whether the string is complete
boolean printTime = false;
boolean showBrightness = false;
unsigned long brightnessStartTime;
unsigned int minBrightness = 1;

// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include <Wire.h>
#define DS3231 1
#include "RTClib.h"
RTC_DS1307 rtc;

#include <EEPROM.h>

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

    // Write current time to serial monitor if we get a '#'
    if ( inChar == '#' ) {
      printTime = true;
      return;
    }

    // Use '+'/'-' to change minBrightness and '=' to display
    if ( inChar == '=' ) {
      showBrightness = true;
      brightnessStartTime = millis();
      return;
    }
    if ( inChar == '+' ) {
      showBrightness = true;
      brightnessStartTime = millis();
      if ( minBrightness < 254) {
        minBrightness += 1;
      }
      return;
    }
    if ( inChar == '-' ) {
      showBrightness = true;
      brightnessStartTime = millis();
      if ( minBrightness > 0) {
        minBrightness -= 1;
      }
      return;
    }

    // add it to the string if it is a digit
    if ( inChar >= '0' && inChar <= '9' ) {
      inputTime += inChar;
    }
    if (inputTime.length() == 4) {
      timeComplete = true;
    }
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
  Serial.begin(19200);

  Serial.println(F("clock23 v1.0"));
  Serial.println(F("============"));
  Serial.println(F("for DS3231 using ShiftPWM and modified AdaFruit RTClib"));
  Serial.println(" ");
  Serial.println(F("Type '+', '-' or '=' to increase/decrease/show min brightness"));
  Serial.println(F("Type '#' to show current time"));
  Serial.println(F("Type 4 digits to set time to that hour, minute and 00 seconds"));
  Serial.println(F("e.g. 0957 will set time to 09:57:00 when last digit is typed"));
  Serial.println(" ");

  inputTime.reserve(4);

  Wire.begin();
  rtc.begin();

  // Sets the number of 8-bit registers that are used.
  ShiftPWM.SetAmountOfRegisters(numRegisters);
  ShiftPWM.Start(pwmFrequency, maxBrightness);

  // Get minimum brightness from EEPROM
  minBrightness = EEPROM.read(0);
  // If not set (== 0xFF) then set to 0x01
  if ( minBrightness == 255 ) {
    minBrightness = 1;
    EEPROM.write(0, minBrightness);
  }

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

  // Interrupt once every second (specify control register for DS3231)
  // Passing in the control register address requires a modification
  // to the Adafruit RTClib:
  /*
diff --git a/RTClib.cpp b/RTClib.cpp
index ad0f0c6..2d2dc63 100644
--- a/RTClib.cpp
+++ b/RTClib.cpp
@@ -265,11 +265,11 @@ DateTime RTC_DS1307::now() {
   return DateTime (y, m, d, hh, mm, ss);
 }
 
-Ds1307SqwPinMode RTC_DS1307::readSqwPinMode() {
+Ds1307SqwPinMode RTC_DS1307::readSqwPinMode(uint8_t control) {
   int mode;
 
   WIRE.beginTransmission(DS1307_ADDRESS);
-  WIRE._I2C_WRITE(DS1307_CONTROL);
+  WIRE._I2C_WRITE(control);
   WIRE.endTransmission();
   
   WIRE.requestFrom((uint8_t)DS1307_ADDRESS, (uint8_t)1);
@@ -279,9 +279,9 @@ Ds1307SqwPinMode RTC_DS1307::readSqwPinMode() {
   return static_cast<Ds1307SqwPinMode>(mode);
 }
 
-void RTC_DS1307::writeSqwPinMode(Ds1307SqwPinMode mode) {
+void RTC_DS1307::writeSqwPinMode(Ds1307SqwPinMode mode, uint8_t control) {
   WIRE.beginTransmission(DS1307_ADDRESS);
-  WIRE._I2C_WRITE(DS1307_CONTROL);
+  WIRE._I2C_WRITE(control);
   WIRE._I2C_WRITE(mode);
   WIRE.endTransmission();
 }
diff --git a/RTClib.h b/RTClib.h
index 6d6b361..424920e 100644
--- a/RTClib.h
+++ b/RTClib.h
@@ -4,6 +4,8 @@
 #ifndef _RTCLIB_H_
 #define _RTCLIB_H_
 
+#define DS1307_CONTROL  0x07
+
 class TimeSpan;
 
 // Simple general-purpose date/time class (no TZ / DST / leap second handling!)
@@ -64,8 +66,8 @@ public:
     static void adjust(const DateTime& dt);
     uint8_t isrunning(void);
     static DateTime now();
-    static Ds1307SqwPinMode readSqwPinMode();
-    static void writeSqwPinMode(Ds1307SqwPinMode mode);
+    static Ds1307SqwPinMode readSqwPinMode(uint8_t control=DS1307_CONTROL);
+    static void writeSqwPinMode(Ds1307SqwPinMode mode, uint8_t control=DS1307_CONTROL);
     uint8_t readnvram(uint8_t address);
     void readnvram(uint8_t* buf, uint8_t size, uint8_t address);
     void writenvram(uint8_t address, uint8_t data);

  */
  rtc.writeSqwPinMode(SquareWave1HZ, 0x0E);
  attachInterrupt(0, ticktock, FALLING);
}

void loop() {
  if ( timeComplete ) {
    timeComplete = false;
    int h = 10*(inputTime[0]-'0') + (inputTime[1]-'0');
    int m = 10*(inputTime[2]-'0') + (inputTime[3]-'0');
    rtc.adjust(DateTime(2015, 1, 1, h, m, 0));
    printTime = true;
    inputTime = "";
  }

  if ( printTime ) {
    printTime = false;
    DateTime now = rtc.now();
    int hh = now.hour();
    int h1 = hh/10;
    int h2 = hh % 10;

    int mm = now.minute();
    int m1 = mm/10;
    int m2 = mm % 10;

    int ss = now.second();
    int s1 = ss/10;
    int s2 = ss % 10;

    if ( h1 == 0 ) {
      Serial.print(" ");
    } else {

      Serial.print(h1);
    }
    Serial.print(h2);
    Serial.print(":");
    Serial.print(m1);
    Serial.print(m2);
    Serial.print(":");
    Serial.print(s1);
    Serial.println(s2);
  }

  if ( showBrightness ) {
    if ( millis() - brightnessStartTime < 5000 ) {
      currentBrightness = 128;
      resetDisplay();
      if ( minBrightness < 10 ) {
        displayNextDigit(10);
        displayNextDigit(10);
        displayNextDigit(10);
        displayNextDigit(minBrightness);
        return;
      }
      if ( minBrightness < 100 ) {
        displayNextDigit(10);
        displayNextDigit(10);
        displayNextDigit(minBrightness/10);
        displayNextDigit(minBrightness % 10);
        return;
      } else {
        displayNextDigit(10);
        int b1 = minBrightness / 100;
        int b2 = (minBrightness-100*b1) / 10;
        int b3 = minBrightness - 100*b1 - 10*b2;
        displayNextDigit(b1);
        displayNextDigit(b2);
        displayNextDigit(b3);
        return;
      }
    } else {
      showBrightness = false;
      // Store the minimum brightness
      EEPROM.write(0, minBrightness);
    }
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


