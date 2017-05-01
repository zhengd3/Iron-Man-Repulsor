#include "SPI.h"
#include "RimsIdent.h"
#include "LiquidCrystal.h"

double currentTemp, ssrControl, settedTemp;

LiquidCrystal lcd(8,9,4,5,6,7);
UIRimsIdent myUI(&lcd,0,10);
RimsIdent myIdent(&myUI,1,11,&currentTemp,&ssrControl,&settedTemp);

void setup() {
  Serial.begin(115200);
  float steinhartCoefs[4] = { 
  0.0006 , 0.0003 , -0.000007 , 0.0000003 
  };
  myIdent.setThermistor(steinhartCoefs,10000.0);
}
void loop() {
  myIdent.run();
};
