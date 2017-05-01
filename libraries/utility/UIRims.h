/*!
 * \file UIRims.h
 * \brief UIRims class declaration
 */

#ifndef UIRims_h
#define UIRims_h

#define KEYNONE 0
#define KEYUP 1
#define KEYDOWN 2
#define KEYLEFT 3
#define KEYRIGHT 4
#define KEYSELECT 5

#define LCDCOLUMNS 16
#define LCDROWS 2

#define NCTHERM 404

/// \brief Alarm frequency for no power alarm [Hz]
#define POWERFREQ 500
/// \brief Alarm frequency for ring method [Hz]
#define RINGFREQ 1000
/// \brief Alarm frequency for flow alarm [Hz]
#define FLOWFREQ 1500
/// \brief Alarm frequency for no not connected thermistor alarm [Hz]
#define NCTHERMFREQ 2000
/// \brief Alarm duration (except ring method) [mSec]
#define ALARMLENGTH 50

/// \brief Software key debounce time [mSec]
#define KEYDEBOUNCETIME 15

#include "Arduino.h"
#include "LiquidCrystal.h"


/*!
 * \brief Library to menage LCD and keypad for Rims library
 *
 * Made for :  
 * DFRobot LCD Keypad Shield For Arduino
 * http://www.dfrobot.com/index.php?route=product/product&product_id=51#.Ua1dHhXEnXQ
 *  
 * If anything else is used, lcd must be 16 cols 2 rows and keypad must be
 * associated to a analog ADC pin with a volage divider for each keys.
 * Needed keys : UP, DOWN, LEFT, RIGHT, SELECT
 *
 * \author Francis Gagnon
 */ 
class UIRims
{
	friend class UIRimsIdent;
	
public:
	
	UIRims(LiquidCrystal* lcd, byte pinKeysAnalog,
           byte pinLight=13,char pinSpeaker=-1);
	
	// === STD DIALOGS METHOD ===
	void showPumpWarning(float flow = 0.0);
	void showHeaterWarning(float state = false);
	void showTempScreen();
	void showTimeFlowScreen();
	void showMemAccessScreen();
	void switchScreen();
	
	// === VARIABLE SETTER ===
	void setTempSP(float tempCelcius);
	void setTempPV(float tempCelcius, boolean buzz = true);
	virtual void setTime(unsigned int timeSec);
	void setFlow(float flow, boolean buzz = true); //liter/min
	void setFlowBounds(float lowBound, float upBound);
	void setHeaterVoltState(boolean state, boolean buzz = true);
	
	// === KEYS READER ===
	byte readKeysADC(boolean waitNone = true);
	
	// === ALARM METHODS ===
	void timerRunningChar(boolean state);
	void ring(boolean state = true);
	void lcdLight(boolean state = true);
	
	// === SETUP DIALOGS ===
	float askSetPoint(float defaultVal); // Celsius
	unsigned int askTime(unsigned int defaultVal); // seconds
	byte askMashWater(int mashWaterValues[],byte defaultVal);

	
	
protected:
	
	byte _waitForKeyChange();
	void _waitTime(unsigned long timeInMilliSec);
	
	void _printStrLCD(String mess, byte col, byte row);
	void _printFloatLCD(float val, int width, int prec,
						byte col, byte row);
	void _setCursorPosition(byte col, byte row);
	float _celciusToFahrenheit(float celcius);
	void _moveCursorLR(byte begin, byte end, byte dotPosition,
					   byte row,boolean left);
	float _incDecValue(float value,byte dotPosition, 
					   boolean increase,
					   float lowerBound, float upperBound,
					   boolean timeFormat);
	float _askValue(byte begin, byte end, 
					byte dotPosition, byte row,
					float defaultVal,
				    float lowerBound, float upperBound,
					boolean timeFormat);
private:
	
	LiquidCrystal* _lcd;
	byte _cursorCol;
	byte _cursorRow;
	byte _pinKeysAnalog;
	byte _pinLight;
	char _pinSpeaker;
	
	boolean _waitNone;
	boolean _tempScreenShown;
	
	float _tempSP;
	float _tempPV;
	unsigned int _time;
	float _flow;
	
	float _flowLowBound;
	float _flowUpBound;
};


#endif
