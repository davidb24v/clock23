const int ShiftPWM_latchPin=8;
const bool ShiftPWM_invertOutputs = false;
const bool ShiftPWM_balanceLoad = false;

#include <ShiftPWM.h>   // include ShiftPWM.h after setting the pins!

unsigned char maxBrightness = 255;
unsigned char pwmFrequency = 100;
unsigned int numRegisters = 4;
unsigned int numOutputs = numRegisters*8;

int currentBrightness = 50;

// Values for each digit "0"-"9", " ", "."
byte digits[12]= {63, 6, 91, 79, 102, 109, 125, 7, 127, 111, 0, 128};

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


void setup(){
  // Sets the number of 8-bit registers that are used.
  ShiftPWM.SetAmountOfRegisters(numRegisters);
  ShiftPWM.Start(pwmFrequency, maxBrightness);
}

int count = 0;

void loop() {
  resetDisplay();
  for(int i=0; i<numRegisters; i++) {
    displayNextDigit(count);
  }
  count = ++count % 12;
  //currentBrightness += 10;
  //currentBrightness = max(currentBrightness % 200, 10);
  delay(500);
}


