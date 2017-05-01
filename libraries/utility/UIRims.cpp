/*!
 * \file UIRims.cpp
 * \brief UIRims class definition
 */

#include "Arduino.h"
#include "UIRims.h"

/*!
 * \brief Constructor
 * \param lcd : LiquidCrystal* (16x2 characters) that will be used during
                identification.
 * \param pinKeysAnalog : byte. Analog pin for keypad
 * \param pinLight : byte. Pin used for heater LED indicator
 * \param pinSpeaker : byte. Pin used for buzzer alarm
 */
UIRims::UIRims(LiquidCrystal* lcd, byte pinKeysAnalog,
			   byte pinLight,char pinSpeaker)
: _lcd(lcd), _pinKeysAnalog(pinKeysAnalog),_waitNone(true),
  _cursorCol(0), _cursorRow(0), _pinLight(pinLight), 
  _pinSpeaker(pinSpeaker),
  _tempSP(0), _tempPV(0), _time(0), _flow(0),
  _flowLowBound(-1),_flowUpBound(100)
{
	pinMode(pinLight,OUTPUT);
	if(pinSpeaker != -1) pinMode(pinSpeaker,OUTPUT);
	digitalWrite(pinLight,HIGH);
	_lcd->begin(LCDCOLUMNS,LCDROWS);
	_lcd->clear();
	byte okChar[8] = {
		B01110,
		B10001,
		B01110,
		B00000,
		B11111,
		B00100,
		B11011,
		B00000
	};
	byte upChar[8] = {
		B00100,
		B01110,
		B11111,
		B00100,
		B00100,
		B00100,
		B00100,
		B00000
	};
	byte downChar[8] = {
		B00100,
		B00100,
		B00100,
		B00100,
		B11111,
		B01110,
		B00100,
		B00000
	};
	_lcd->createChar(1,okChar);
	_lcd->createChar(2,upChar);
	_lcd->createChar(3,downChar);
}

/*!
 * \brief Show temperature (set point and process value) screen on _lcd
 */
void UIRims::showTempScreen()
{
	_tempScreenShown = true;
	_lcd->noBlink();
	_lcd->clear();
	_printStrLCD("SP:00.0\xdf""C(000\xdf""F)",0,0);
	_printStrLCD("PV:00.0\xdf""C(000\xdf""F)",0,1);
	this->setTempSP(_tempSP);
	this->setTempPV(_tempPV, false);
}

/*!
 * \brief Show remaining time and flow screen on _lcd
 */
void UIRims::showTimeFlowScreen()
{
	_tempScreenShown = false;
	_lcd->noBlink();
	_lcd->clear();
	_printStrLCD("time:000m00s   x",0 ,0);
	_printStrLCD(String("flow:00.0L/min \x01"),0,1);
	this->setTime(_time);
	this->setFlow(_flow,false);
}

/*!
 * \brief Toggle between tempScreen and timeFlowScreen on _lcd
 */
void UIRims::switchScreen()
{
	if(_tempScreenShown) this->showTimeFlowScreen();
	else this->showTempScreen();
}

/*!
 * \brief Read keys without software debouce. 
 * \param waitNone : boolean. If true, KEYNONE must be detected
 *        to return anything else than KEYNONE.
 * \return byte : KEYNONE, KEYUP, KEYDOWN, KEYLEFT, KEYRIGHT or KEYSELECT
 */
byte UIRims::readKeysADC(boolean waitNone)
{
	byte res;
	int adcKeyVal = analogRead(_pinKeysAnalog);  
	if (adcKeyVal > 1000) res = KEYNONE;
	else if (adcKeyVal < 50) res = KEYRIGHT;
	else if (adcKeyVal < 195) res = KEYUP;
	else if (adcKeyVal < 380) res = KEYDOWN;
	else if (adcKeyVal < 555) res = KEYLEFT;
	else if (adcKeyVal < 790) res = KEYSELECT;
	if(waitNone)
	{
		if(res==KEYNONE and _waitNone) _waitNone = false;
		else if(res!=KEYNONE)
		{
			if(_waitNone) res = KEYNONE;
			else _waitNone = true;
		}
	}
	else _waitNone = true;
	return res;
}

/*!
 * \brief Read keys with software debounce
 * \return byte : KEYNONE, KEYUP, KEYDOWN, KEYLEFT, KEYRIGHT or KEYSELECT
 */
byte UIRims::_waitForKeyChange()
{
	boolean keyConfirmed = false, keyDetected = false;
	byte lastKey = this->readKeysADC(false), currentKey;
	unsigned long refTime = millis(), currentTime;
	while(not keyConfirmed)
	{
		currentTime = millis();
		currentKey = this->readKeysADC(false);
		if(keyDetected)
		{
			if(currentTime - refTime >= KEYDEBOUNCETIME) keyConfirmed = true;
		}
		if(currentKey != lastKey)
		{
			keyDetected = true;
			refTime = currentTime;
			lastKey = currentKey;
		}
	}
	return currentKey;
}


/*!
 * \brief Show blinking char next to remaining time
 * \param state : boolean. If true, char is shown.
 */
void UIRims::timerRunningChar(boolean state)
{
	if(not _tempScreenShown)
	{
		_setCursorPosition(15,0);
		if(state == true)
		{
			_printStrLCD("\x01",15,0);
			_lcd->blink();
		}
		else
		{
			_printStrLCD("x",15,0);
			_lcd->noBlink();
		}
	}
}

/*!
 * \brief Ring with given speaker setted with the Constructor
 * \param state : boolean. If true, start buzz, if false stop buzz.
 */
void UIRims::ring(boolean state)
{
	if(_pinSpeaker != -1)
	{
		(state == true) ? tone(_pinSpeaker,RINGFREQ) : noTone(_pinSpeaker);
	}
}

/*!
 * \brief Turn on/off the LCD backlighting
 * \param state : boolean. If true, light on, else, light off.
 */
void UIRims::lcdLight(boolean state)
{
	digitalWrite(_pinLight,state);
}

/*!
 * \brief Pause the Arduino for the given timeInMilliSec
 * \param timeInMilliSec : unsigned long. Time that the Arduino
                           will be pause
 * \return byte : KEYNONE, KEYUP, KEYDOWN, KEYLEFT, KEYRIGHT or KEYSELECT
 */
void UIRims::_waitTime(unsigned long timeInMilliSec)
{
	unsigned long startTime = millis();
	while(millis() - startTime <= timeInMilliSec) continue;
}

/*!
 * \brief Show mess at column col and row row on _lcd
 * \param mess : String. Message to print
 * \param col : byte. starting column on _lcd
 * \param row : byte. starting row on _lcd
 */
void UIRims::_printStrLCD(String mess, byte col, byte row)
{
	_lcd->setCursor(col,row);
	_lcd->print(mess);
	_lcd->setCursor(_cursorCol,_cursorRow);
}

/*!
 * \brief Show a floating number at column col and row row
          on _lcd
 * \param val : float.
 * \param width : int. minumum width
 * \param prec : int. digit after point
 * \param col : byte. starting column on _lcd
 * \param row : byte. starting row on _lcd
 */
void UIRims::_printFloatLCD(float val, int width, int prec,
							byte col, byte row)
{
	char myFloatStr[17];
	String res(dtostrf(val,width,prec,myFloatStr));
	res.replace(' ','0');
	if(res.length() > width)
	{
		res = String(dtostrf(val,width,0,myFloatStr));
	}
	_lcd->setCursor(col,row);
	_lcd->print(res);
	_lcd->setCursor(_cursorCol,_cursorRow);
}

/*!
 * \brief Set _lcd cursor at given column col and given row row
 * \param col : byte. starting column on _lcd
 * \param row : byte. starting row on _lcd
 */
void UIRims::_setCursorPosition(byte col, byte row)
{
	_cursorCol = col;
	_cursorRow = row;
	_lcd->setCursor(col,row);
}

/*!
 * \brief Convert celcius temp to fahrenheit temp
 * \param celcius : float.
 * \return float : celcius param converted in fahrenheit.
 */
float UIRims::_celciusToFahrenheit(float celcius)
{
	return ((9.0/5.0)*celcius)+32.0;
}

/*!
 * \brief Set a new set point temperature.
 *
 * Set a new set point temperature. If tempScreen is 
 * shown, it will be updated on the lcd _lcd else
 * it will be memorized for when it will be shown.
 *
 * \param tempCelcius : float.
 */
void UIRims::setTempSP(float tempCelcius)
{
	_tempSP = tempCelcius;
	if(_tempScreenShown)
	{
		float tempFahren = _celciusToFahrenheit(tempCelcius);
		_printFloatLCD(tempCelcius,4,1,3,0);
		_printFloatLCD(tempFahren,3,0,10,0);
	}
}

/*!
 * \brief Set a new process value temperature.
 *
 * Set a new process value temperature. If tempScreen is 
 * shown, it will be updated on the lcd _lcd else
 * it will be memorized for when it will be shown.
 *
 * \param tempCelcius : float. If tempCelcius >= NCTHERM, thermistor is
 *                             considered unconnected.
 * \param buzz : boolean. Emit an alarm on speaker if thermistor is
 *                        disconnected.
 */
void UIRims::setTempPV(float tempCelcius, boolean buzz)
{
	_tempPV = tempCelcius;
	boolean thermNC = (tempCelcius >= NCTHERM);
	if(thermNC) tone(_pinSpeaker,NCTHERMFREQ,ALARMLENGTH);
	if(_tempScreenShown)
	{
		float tempFahren = _celciusToFahrenheit(tempCelcius);
		if(not thermNC)
		{
			_printFloatLCD(tempCelcius,4,1,3,1);
			_printFloatLCD(tempFahren,3,0,10,1);
		}
		else
		{
			_printStrLCD(" #NC",3,1);
			_printStrLCD("#NC",10,1);
		}
	}
}

/*!
 * \brief Set a new remaining time.
 *
 * Set a new remaining time. If timeFlowScreen is 
 * shown, it will be updated on the lcd _lcd else
 * it will be memorized for when it will be shown.
 *
 * \param timeSec : unsigned int.
 */
void UIRims::setTime(unsigned int timeSec)
{
	_time = timeSec;
	if(not _tempScreenShown)
	{
		int minutes = timeSec / 60;
		int seconds = timeSec % 60;
		_printFloatLCD(minutes,3,0,5,0);
		_printFloatLCD(seconds,2,0,9,0);
	}
}

/*!
 * \brief Set a new flow value.
 *
 * Set a new flow value. If timeFlowScreen is 
 * shown, it will be updated on the lcd _lcd else
 * it will be memorized for when it will be shown.
 *
 * \param flow : float.
 * \param buzz : boolean. Emit an alarm on speaker if flow is incorrect.
 */
void UIRims::setFlow(float flow, boolean buzz)
{
	_flow = flow;
	boolean flowOk = (flow >= _flowLowBound) and (flow <= _flowUpBound);
	if(not flowOk and buzz) tone(_pinSpeaker,FLOWFREQ,ALARMLENGTH);
	if(not _tempScreenShown)
	{
		_printFloatLCD(constrain(flow,0,99.9),4,1,5,1);
		if(flowOk) _printStrLCD("\x01",15,1);
		else if(flow<_flowLowBound) _printStrLCD("\x03",15,1);
		else _printStrLCD("\x02",15,1);
	}
}

/*! 
 * \brief Set bounds for accepted flow rate 
 * 
 * \param lowBound : float. Lower bound of flow rate [L/min]
 * \param upBound : float. Upper bound of flow rate [L/min]
 * 
 */
void UIRims::setFlowBounds(float lowBound, float upBound)
{
	_flowLowBound = lowBound; _flowUpBound = upBound;
}

/*!
 * \brief Increse or decrease a floating point value on _lcd->
          dotPosition give the position (column) of the point mark.
 * \param value : float.
 * \param dotPosition : byte. position (column) of the point mark
 * \param increase : boolean. If true, increment else, decrement.
 * \param lowerBound : float. decreasing limit of the value
 * \param upperBound : float. increasing limit of the value
 * \param timeFormat : boolean. treats the value (in sec) as a time
 *                     with minutes and secondes.
 * \return float : result of increment/decrement
 */
float UIRims::_incDecValue(float value,byte dotPosition, boolean increase,
						   float lowerBound, float upperBound,
						   boolean timeFormat)
{
	float res,constrainedRes;
	int digitPosition, way = (increase) ? (+1) : (-1);
	if(_cursorCol<dotPosition)
	{
		digitPosition = (dotPosition - _cursorCol) - 1;
	}
	else
	{
		digitPosition = (dotPosition - _cursorCol);
	}
	if(not timeFormat)
	{
		res = value + (way*pow(10,digitPosition));
	}
	else
	{
		if(digitPosition<0)
		{
			res = value + (way*pow(10,digitPosition + 2));
		}
		else
		{
			res = value + (way*60*pow(10,digitPosition));
		}
	}
	constrainedRes = (constrain(res,lowerBound,upperBound)!= res) ? \
	                 (value) : (res);
	if(not timeFormat) this->setTempSP(constrainedRes);
	else this->setTime(constrainedRes);
	return constrainedRes;
}

/*!
 * \brief Move the cursor left or right on _lcd.
 * \param begin : byte. indicate the position (column) of the
 *        beginning of the cursor bounds.
 * \param end : byte. indicate the position (column) of the ending of 
 *              the cursor bounds.
 * \param dotPosition : byte. position (column) of the dot to skip it.
 * \param row : byte. row indicate the row of the cursor on _lcd
 * \param left : boolean. If true, left movement else, right.
 */
void UIRims::_moveCursorLR(byte begin, byte end, byte dotPosition,
						   byte row,boolean left)
{
	int way = (left) ? (-1) : (+1);
	if((_cursorCol > begin and left) or \
	   (_cursorCol < end and not left))
	{
		if (_cursorCol == dotPosition - (1*way))
		{
			_setCursorPosition(
				_cursorCol + (2*way) , row);
		}
		else
		{
			_setCursorPosition(
				_cursorCol + (1*way) , row);
		}
	}
}

/*!
 * \brief Ask a value on _lcd
 * \param begin : byte. cursor bounds (columns) for the value.
 * \param end : byte. cursor bounds (columns) for the value.
 * \param dotPosition : byte. point mark position
 * \param row : byte. row on _lcd
 * \param defaultVal : float. starting value
 * \param lowerBound : float. value's limits
 * \param upperBound : float. value's limits
 * \param timeFormat : boolean. treats the value (in sec) as a time
 *                      with minutes and secondes.
 * \return float : selected value
 */
float UIRims::_askValue(byte begin, byte end, 
						byte dotPosition, byte row,
						float defaultVal,
						float lowerBound, float upperBound,
						boolean timeFormat)
{
	boolean valSelected = false;
	float value = defaultVal;
	timeFormat ? this->setTime(defaultVal) : \
	             this->setTempSP(defaultVal);
	_setCursorPosition(dotPosition-1,row);
	_lcd->blink();
	_waitTime(500);
	while(not valSelected)
	{
		switch(_waitForKeyChange())
		{
			case KEYNONE :
				break;
			case KEYUP :
				value = _incDecValue(value,dotPosition,true,
									lowerBound,upperBound,timeFormat);
				break;
			case KEYDOWN :
				value = _incDecValue(value,dotPosition,false,
									lowerBound,upperBound,timeFormat);
				break;
			case KEYLEFT :
				_moveCursorLR(begin,end,dotPosition,
									row,true);
				break;
			case KEYRIGHT :
				_moveCursorLR(begin,end,dotPosition,
									row,false);
				break;
			case KEYSELECT :
				valSelected = true;
				_lcd->noBlink();
				_setCursorPosition(0,0);
				break;
		}
	}
	return value;
}

/*!
 * \brief Ask set point temperature with the UI.
 * \param defaultVal : float.
 * \return float : selected value
 */
float UIRims::askSetPoint(float defaultVal)
{
	float res;
	this->showTempScreen();
	_printStrLCD("                 ",0,1);
	res = _askValue(3,6,5,0,defaultVal,0.0,99.9,false);
	return res;
}

/*!
 * \brief Ask timer time on the UI.
 * \param defaultVal : unsigned int. in sec
 * \return unsigned int : selected value in sec
 */
unsigned int UIRims::askTime(unsigned int defaultVal)
{
	unsigned int res;
	this->showTimeFlowScreen();
	_printStrLCD("                 ",0,1);
	res = _askValue(5,10,8,0,defaultVal,0,59999,true);
	return res;
}


/*!
 * \brief Ask mash water quantity (4 choices max).
 * \param mashWaterValues : int[4]. Array that contain different
 *                          choices for mash water quantity. If
 *                          less than 4 is needed, the rest should be
 *                          setted to -1. All values should be between
 *                          0 L and 99 L.
 * \param defaultVal : byte. Cursor starting position index.
 * \return byte : 0 to 4. Index of the selected mash water quantity.
 */
byte UIRims::askMashWater(int mashWaterValues[], byte defaultVal)
{
	boolean mashWaterSelected = false;
	byte mashWaterIndex = defaultVal, mashChoices = 0;
	byte keyPressed = KEYNONE;
	_printStrLCD("Mash water qty: ",0,0);
	for(int i=0;i<=3;i++)
	{
		if(mashWaterValues[i] != - 1)
		{
			_printStrLCD(" ",0,1);
			_printFloatLCD(mashWaterValues[i],2,0,(4*i)+1,1);
			_printStrLCD("L",(4*i)+3,1);
			mashChoices++;
		}
	}
	_printStrLCD("\x7e",mashWaterIndex*4,1);
	_waitTime(500);
	while(not mashWaterSelected)
	{
		keyPressed = _waitForKeyChange();
		if(keyPressed == KEYSELECT) mashWaterSelected = true;
		else
		{
			_printStrLCD(" ",mashWaterIndex*4,1);
			if(keyPressed == KEYUP or keyPressed == KEYRIGHT)
			{
				mashWaterIndex = constrain(mashWaterIndex+1,0,mashChoices-1);
			}
			else if(keyPressed == KEYDOWN or keyPressed == KEYLEFT)
			{
				mashWaterIndex = constrain(mashWaterIndex-1,0,mashChoices-1);
			}
			_printStrLCD("\x7e",mashWaterIndex*4,1);
		}
	}
	return mashWaterIndex;
}

/*!
 * \brief Show the pump switching warning.
 */
void UIRims::showPumpWarning(float flow)
{
	_lcd->clear();
	_printStrLCD("start pump  [OK]",0,0);
	_printStrLCD("flow:00.0L/min",0,1);
	setFlow(flow,false);
	_waitTime(500);
}

/*!
 * \brief Show the heater switching warning.
 */
void UIRims::showHeaterWarning(float state)
{
	_lcd->clear();
	_printStrLCD("start heater[OK]",0,0);
	_printStrLCD("state:off",0,1);
	setHeaterVoltState(state,false);
	_waitTime(500);
}

/*!
 * \brief Show the memory access mode screen.
 */
void UIRims::showMemAccessScreen()
{
	_lcd->clear();
	_printStrLCD("USB memory",0,0);
	_printStrLCD("access mode",0,1);
}

/*!
 * \brief Set voltage detection on heater under the heater warning.
 * \param state : boolean. If true, show on, else, off.
 * \param buzz : boolean. If true, speaker alarm is trigerred if no voltage.
 *               If false, state is shown on LCD.
 */
void UIRims::setHeaterVoltState(boolean state, boolean buzz)
{
	if(not buzz) state ? _printStrLCD("on ",6,1): _printStrLCD("off",6,1);
	else if(!state) tone(_pinSpeaker,POWERFREQ,ALARMLENGTH);
}