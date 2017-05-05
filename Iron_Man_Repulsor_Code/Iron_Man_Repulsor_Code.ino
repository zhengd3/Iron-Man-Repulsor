//Neopixel Ring
#include <Adafruit_NeoPixel.h>
#define PIN 6

Adafruit_NeoPixel strip = Adafruit_NeoPixel(12, PIN, NEO_GRB + NEO_KHZ800);

//Stepper Motor
#include <Stepper.h>
#define STEPS_PER_MOTOR_REVOLUTION 32
#define STEPS_PER_OUTPUT_REVOLUTION 32 * 64  //2048

Stepper small_stepper(STEPS_PER_MOTOR_REVOLUTION, 8, 12, 11, 13);  //Stepper Motor Pins

int  Steps2Take;
int button = 7;
int buttonValue;

//Adafruit Wave Shield//
#include <FatReader.h>
#include <SdReader.h>
#include <avr/pgmspace.h>
#include "WaveUtil.h"
#include "WaveHC.h"

#define REPULSOR "Repulsor.WAV"
#define MISSILE "LaserM.WAV"

SdReader card;    // This object holds the information for the card
FatVolume vol;    // This holds the information for the partition on the card
FatReader root;   // This holds the information for the filesystem on the card
FatReader file;

uint8_t dirLevel; // indent level for file/dir names    (for prettyprinting)
dir_t dirBuf;     // buffer for directory reads

WaveHC wave;      // This is the only wave (audio) object, since we will only play one at a time

// Function definitions (we define them here, but the code is below)
void lsR(FatReader &d);
void play(FatReader &dir);

//Flex Sensor//
const int flexSensor = 1;
int sensor = 0;
const int threshold = 40;

//State Machine//
const int S_Idle = 2;
const int S_Bend = 3;
const int S_Fire = 4;
const int S_Recharge = 5;
const int S_Missile = 6;

static int SM_States;
static int Next_States;
static int counter = 0;




void setup()
{
  Serial.begin(9600);       // use the serial port

  pinMode(button, INPUT);   //set pushbutton as an input

  strip.begin();
  strip.setBrightness(64);  //Set Neopixel Ring's Brightness
  strip.show();

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //                                     Audio Setup Code                                             //
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  putstring_nl("\nWave test!");  // say we woke up!

  putstring("Free RAM: ");       // This can help with debugging, running out of RAM is bad
  Serial.println(freeRam());

  // Set the output pins for the DAC control. This pins are defined in the library
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  //  if (!card.init(true)) { //play with 4 MHz spi if 8MHz isn't working for you
  if (!card.init()) {         //play with 8 MHz spi (default faster!)
    putstring_nl("Card init. failed!");  // Something went wrong, lets print out why
    sdErrorCheck();
    while (1);                           // then 'halt' - do nothing!
  }

  // enable optimize read - some cards may timeout. Disable if you're having problems
  card.partialBlockRead(true);

  // Now we will look for a FAT partition!
  uint8_t part;
  for (part = 0; part < 5; part++)
  { // we have up to 5 slots to look in
    if (vol.init(card, part))
      break;                             // we found one, lets bail
  }
  if (part == 5)
  { // if we ended up not finding one  :(
    putstring_nl("No valid FAT partition!");
    sdErrorCheck();      // Something went wrong, lets print out why
    while (1);                           // then 'halt' - do nothing!
  }

  // Lets tell the user about what we found
  putstring("Using partition ");
  Serial.print(part, DEC);
  putstring(", type is FAT");
  Serial.println(vol.fatType(), DEC);    // FAT16 or FAT32?

  // Try to open the root directory
  if (!root.openRoot(vol))
  {
    putstring_nl("Can't open root dir!"); // Something went wrong,
    while (1);                            // then 'halt' - do nothing!
  }

  // Whew! We got past the tough parts.
  putstring_nl("Files found:");
  dirLevel = 0;
  // Print out all of the files in all the directories.
  lsR(root);
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////

  while (1)
  {
    SM_States = Next_States;
    Next_States = S_Idle;
    Funct_States();
  }
}

void loop()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     State Machine                                                //
//////////////////////////////////////////////////////////////////////////////////////////////////////
void Funct_States()
{
  sensor = analogRead(flexSensor);
  buttonValue = digitalRead(button);

  switch (SM_States)
  {
    case S_Idle:
      if (sensor < threshold) //if flex sensor is bent
        Next_States = S_Bend;
      else
        Next_States = S_Idle;
      break;

    case S_Bend:
      if (sensor >= threshold) //if flex sensor is not bent
        Next_States = S_Fire;
      else if (sensor < threshold && buttonValue == HIGH) //if flex sensor is bent and button is pressed
        Next_States = S_Missile;
      else
        Next_States = S_Bend;
      break;

    case S_Fire:
      if (sensor < threshold) //if flex sensor is not bent
        Next_States = S_Bend;
      else
        Next_States = S_Recharge;
      break;

    case S_Recharge:
      if (sensor < threshold) //if flex sensor is bent
        Next_States = S_Bend;
      else
      {
        Next_States = S_Recharge;
        if (counter == 10)  //while at recharge state, a counter is incremented by 1, if counter hits 10, move to idle state.
          Next_States = S_Idle;
      }
      break;
    case S_Missile: 
      Next_States = S_Bend;
      break;
  }

  switch (SM_States)
  {
    case S_Idle:
      rainbowCycle(1);
      counter = 0;
      break;

    case S_Bend:
      strip.setPixelColor(0, 0, 0, 0);
      strip.setPixelColor(1, 0, 0, 0);
      strip.setPixelColor(2, 0, 0, 0);
      strip.setPixelColor(3, 0, 0, 0);
      strip.setPixelColor(4, 0, 0, 0);
      strip.setPixelColor(5, 0, 0, 0);
      strip.setPixelColor(6, 0, 0, 0);
      strip.setPixelColor(7, 0, 0, 0);
      strip.setPixelColor(8, 0, 0, 0);
      strip.setPixelColor(9, 0, 0, 0);
      strip.setPixelColor(10, 0, 0, 0);
      strip.setPixelColor(11, 0, 0, 0);
      strip.show();
      counter = 0;
      break;

    case S_Fire:
      // fade in from min to max in increments of 5 points:
      for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5)
      {
        // sets the value (range from 0 to 255):
        strip.setPixelColor(0, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(1, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(2, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(3, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(4, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(5, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(6, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(7, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(8, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(9, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(10, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(11, fadeValue, fadeValue, fadeValue);
        strip.show();

        // wait for 10 milliseconds to see the dimming effect
        delay(10);
      }

      playcomplete(REPULSOR);  //Plays Repulsor.WAV file

      // fade out from max to min in increments of 5 points:
      for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5)
      {
        // sets the value (range from 0 to 255):
        strip.setPixelColor(0, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(1, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(2, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(3, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(4, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(5, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(6, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(7, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(8, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(9, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(10, fadeValue, fadeValue, fadeValue);
        strip.setPixelColor(11, fadeValue, fadeValue, fadeValue);
        strip.show();

        // wait for 30 milliseconds to see the dimming effect
        delay(30);
      }
      counter = 0;
      break;

    case S_Recharge:
      for (int i = 0; i < strip.numPixels(); i++)
      {
        strip.setPixelColor(i, strip.Color(255, 0, 0));
        strip.show();
      }
      delay (1);

      //blink off
      for (int j = 64; j > 0; j--)
      {
        for (int i = 0; i < strip.numPixels(); i++)
        {
          strip.setPixelColor(i, strip.Color(j, 0, 0));
          strip.show();
        }
        delay (2);
      }
      if (counter == 10) //If counter is greater than 10, set state to Idle.
        counter = 0;
      counter++;
      break;

    case S_Missile:
      //Stepper Motor Rotation(Opens Forearm Missle Compartment)
      Steps2Take  =  STEPS_PER_OUTPUT_REVOLUTION ;
      small_stepper.setSpeed(1000);
      small_stepper.step(Steps2Take);
      delay(1000);

      playcomplete(MISSILE); //Plays Missile.WAV file

      //Stepper Motor Reverse Rotation (Closes Forearm Missle Compartment)
      Steps2Take  =  - STEPS_PER_OUTPUT_REVOLUTION;
      small_stepper.setSpeed(1000);
      small_stepper.step(Steps2Take);
      delay(1000);
      break;
  }
  delay(100);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Neopixel Ring Code                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////////

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256; j++) {
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 * 1; j++) { // 1 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

uint32_t Wheel(byte WheelPos) {
  if (WheelPos < 85) {
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     Audio Code                                                   //
//////////////////////////////////////////////////////////////////////////////////////////////////////

int freeRam(void)
{
  extern int  __bss_end;
  extern int  *__brkval;
  int free_memory;
  if ((int)__brkval == 0)
  {
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
  }
  else
  {
    free_memory = ((int)&free_memory) - ((int)__brkval);
  }
  return free_memory;
}

/*
   print error message and halt if SD I/O error, great for debugging!
*/
void sdErrorCheck(void)
{
  if (!card.errorCode()) return;
  putstring("\n\rSD I/O error: ");
  Serial.print(card.errorCode(), HEX);
  putstring(", ");
  Serial.println(card.errorData(), HEX);
  while (1);
}
/*
   print dir_t name field. The output is 8.3 format, so like SOUND.WAV or FILENAME.DAT
*/
void printName(dir_t &dir)
{
  for (uint8_t i = 0; i < 11; i++)
  { // 8.3 format has 8+3 = 11 letters in it
    if (dir.name[i] == ' ')
      continue;         // dont print any spaces in the name
    if (i == 8)
      Serial.print('.');           // after the 8th letter, place a dot
    Serial.print(dir.name[i]);      // print the n'th digit
  }
  if (DIR_IS_SUBDIR(dir))
    Serial.print('/');       // directories get a / at the end
}
/*
   list recursively - possible stack overflow if subdirectories too nested
*/
void lsR(FatReader &d)
{
  int8_t r;                     // indicates the level of recursion

  while ((r = d.readDir(dirBuf)) > 0)
  { // read the next file in the directory
    // skip subdirs . and ..
    if (dirBuf.name[0] == '.')
      continue;

    for (uint8_t i = 0; i < dirLevel; i++)
      Serial.print(' ');        // this is for prettyprinting, put spaces in front
    printName(dirBuf);          // print the name of the file we just found
    Serial.println();           // and a new line

    if (DIR_IS_SUBDIR(dirBuf))
    { // we will recurse on any direcory
      FatReader s;                 // make a new directory object to hold information
      dirLevel += 2;               // indent 2 spaces for future prints
      if (s.open(vol, dirBuf))
        lsR(s);                    // list all the files in this directory now!
      dirLevel -= 2;               // remove the extra indentation
    }
  }
  sdErrorCheck();                  // are we doign OK?
}
/*
   play recursively - possible stack overflow if subdirectories too nested
*/
void play(FatReader &dir)
{
  FatReader file;
  while (dir.readDir(dirBuf) > 0)
  { // Read every file in the directory one at a time
    // skip . and .. directories
    if (dirBuf.name[0] == '.')
      continue;

    Serial.println();            // clear out a new line

    for (uint8_t i = 0; i < dirLevel; i++)
      Serial.print(' ');       // this is for prettyprinting, put spaces in front

    if (!file.open(vol, dirBuf))
    { // open the file in the directory
      Serial.println("file.open failed");  // something went wrong :(
      while (1);                           // halt
    }

    if (file.isDir())
    { // check if we opened a new directory
      putstring("Subdir: ");
      printName(dirBuf);
      dirLevel += 2;                       // add more spaces
      // play files in subdirectory
      play(file);                         // recursive!
      dirLevel -= 2;
    }
    else {
      // Aha! we found a file that isnt a directory
      putstring("Playing "); printName(dirBuf);       // print it out
      if (!wave.create(file))
      { // Figure out, is it a WAV proper?
        putstring(" Not a valid WAV");     // ok skip it
      } else
      {
        Serial.println();                  // Hooray it IS a WAV proper!
        wave.play();                       // make some noise!

        while (wave.isplaying)
        { // playing occurs in interrupts, so we print dots in realtime
          putstring(".");
          delay(100);
        }
        sdErrorCheck();                    // everything OK?
        //        if (wave.errors)Serial.println(wave.errors);     // wave decoding errors
      }
    }
  }
}

void playcomplete(char *name)
{
  // call our helper to find and play this name
  playfile(name);
  while (wave.isplaying)
  {
    // do nothing while its playing
  }
  // now its done playing
}
void playfile(char *name)
{
  // see if the wave object is currently doing something
  if (wave.isplaying) {// already playing something, so stop it!
    wave.stop(); // stop it
  }
  // look in the root directory and open the file
  if (!file.open(root, name))
  {
    putstring("Couldn't open file "); Serial.print(name); return;
  }
  // OK read the file and turn it into a wave object
  if (!wave.create(file))
  {
    putstring_nl("Not a valid WAV"); return;
  }

  // ok time to play! start playback
  wave.play();
}
