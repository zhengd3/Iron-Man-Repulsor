#include <Stepper.h>
#define STEPS_PER_MOTOR_REVOLUTION 32
#define STEPS_PER_OUTPUT_REVOLUTION 32 * 64  //2048

Stepper small_stepper(STEPS_PER_MOTOR_REVOLUTION, 8, 11, 12, 13);

int  Steps2Take;
int button = 1;
int value;

void setup()
{
  pinMode(button, INPUT);
}

void loop()
{
  value = digitalRead(button);
  Serial.println(value);
  if (value == HIGH)
  {
    Steps2Take  =  STEPS_PER_OUTPUT_REVOLUTION ;
    small_stepper.setSpeed(1000);
    small_stepper.step(Steps2Take);
    delay(1000);

    Steps2Take  =  - STEPS_PER_OUTPUT_REVOLUTION;
    small_stepper.setSpeed(1000);
    small_stepper.step(Steps2Take);
    delay(1000);
    //value = LOW;
  }
}

