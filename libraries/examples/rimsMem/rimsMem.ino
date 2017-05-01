/*
 * Rims flash mem basic exemple. Based on Winbond W25Q80BV (1 Mbytes) 
 * SPI flash mem.
 * 
 * UNCOMMENT LINE :
 * #define WITH_W25QFLASH
 * IN "Rims.h" TO MAKE IT COMPILE  
 * 
 */

#include "SPI.h"
#include "LiquidCrystal.h"
#include "Rims.h"

double currentTemp, ssrControl, settedTemp;

LiquidCrystal lcd(8,9,4,5,6,7);
UIRims myUI(&lcd,0,10);
Rims myRims(&myUI,1,11,&currentTemp,&ssrControl,&settedTemp);

void setup() {
  Serial.begin(115200);
  float steinhartCoefs[4] = { 
  0.0006 , 0.0003 , -0.000007 , 0.0000003 
  };
  myRims.setThermistor(steinhartCoefs,10000.0);
  myRims.setTuningPID(2000,5,-150000,80,  20); //(Kc,Ki,Kd,Tf,Vol)
  
  myRims.setMemCSPin(A5);      // mem chip select pin (CS or SS)

  myRims.checkMemAccessMode(); // enter in USB mem acces if 
                               // KEYSELECT is held at startup
}
void loop() {
  myRims.run();
}
