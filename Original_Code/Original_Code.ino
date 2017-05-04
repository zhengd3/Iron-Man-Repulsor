/*
* Example 4
* Bend Sensor/Wave shield
* This example uses a bend sensor as a trigger to fade a LED with sound effect
* using a Wave shield and then activate a servo
* Honus 2010
* Modified from Knock Sensor code created 25 Mar 2007 by David Cuartielles 
* and modified 4 Sep 2010 by Tom Igoe
*/

#include "Servo.h"  // include the servo library

 Servo servo1; // creates an instance of the servo object to control a servo

  
// these constants won't change:
    const int servoPin1 = 9; // control pin for servo
    const int triggerSensor = 1; // the sensor is connected to analog pin 1
    const int threshold = 400;  // threshold value to decide when the sensor input triggers
    const int ledPin = 11;
    int soundPin1 = 10;  // control pin for sound board
  
   // these variables will change:
    int sensorReading = 0;  // variable to store the value read from the sensor pin
    int ledState = LOW;  // variable used to store the last LED status, to toggle the light

  void setup() {
   Serial.begin(9600);       // use the serial port
   servo1.attach(servoPin1);   // attaches the servo on pin 9 to the servo object
   pinMode(soundPin1, OUTPUT);  // sets the sound pin as output
      digitalWrite(soundPin1, LOW);
 
  }
 
   void loop()  {
     servo1.write(20); // move the servo to 20 degree position
    
    // read the sensor and store it in the variable sensorReading:
  sensorReading = analogRead(triggerSensor);   
 
  // if the sensor reading is greater than the threshold:
  if (sensorReading >= threshold) {
   
   digitalWrite(soundPin1, HIGH); // turn the sound on
   delay(10); // wait ten milliseconds
   digitalWrite(soundPin1, LOW); // turn the sound off
  
    // fade in from min to max in increments of 5 points:
  for(int fadeValue = 0 ; fadeValue <= 255; fadeValue +=5) {
    // sets the value (range from 0 to 255):
    analogWrite(ledPin, fadeValue);        
    // wait for 30 milliseconds to see the dimming effect   
    delay(40);                           
  }

  // fade out from max to min in increments of 5 points:
  for(int fadeValue = 255 ; fadeValue >= 0; fadeValue -=5) {
    // sets the value (range from 0 to 255):
    analogWrite(ledPin, fadeValue);        
    // wait for 30 milliseconds to see the dimming effect   
    delay(40);
    // send the string "trigger!" back to the computer, followed by newline
    Serial.println("trigger!");         
  }
 
  servo1.write(160);  // move the servo to 160 degree position
   delay(3000);   // wait 3 seconds
   servo1.write(20);  // move the servo to 20 degree position
  }
  
  delay (3000);  //  three second delay to avoid overloading the serial port buffer
  }
 
