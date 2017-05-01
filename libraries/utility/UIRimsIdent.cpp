/*!
 * \file UIRimsIdent.cpp
 * \brief UIRimsIdent class definition
 */

#include "Arduino.h"
#include "UIRimsIdent.h"

/*!
 * \brief Constructor
 * \param lcd : LiquidCrystal* (16x2 characters) that will be used during
                identification.
 * \param pinKeysAnalog : byte analog pin for thermistor reading
 * \param pinLight : byte pin used for heater LED indicator
 * \param pinSpeaker : byte pin used for buzzer alarm
 */
UIRimsIdent::UIRimsIdent(LiquidCrystal* lcd, 
						 byte pinKeysAnalog,byte pinLight,int pinSpeaker)
: UIRims(lcd,pinKeysAnalog,pinLight,pinSpeaker)
{
}

/*!
 * \brief Show warning about Arduino Serial Monitor
 */
void UIRimsIdent::showSerialWarning()
{
	_lcd->clear();
	_printStrLCD("open serial [OK]",0,0);
	_printStrLCD("monitor",0,1);
	_waitTime(500);
}

/*!
 * \brief Show system identification screen on _lcd
 */
void UIRimsIdent::showIdentScreen()
{
	_lcd->clear();
	_printStrLCD("000% 000m00s    ",0,0);
	_printStrLCD("PV:00.0\xdf""C(000\xdf""F)",0,1);
	_tempScreenShown = true;
}

/*!
 * \brief Set ident CV on identScreen
 * \param controlValue : unsigned long. current CV between 0 to ssrWindow
 * \param ssrWindow : unsigned long. ssrWindow size in mSec.
 */
void UIRimsIdent::setIdentCV(unsigned long controlValue, 
							 unsigned long ssrWindow)
{
	_printFloatLCD(controlValue*100/ssrWindow,3,0,0,0);
}

/*!
 * \brief Set a new remaining time.
 *
 * If timeFlowScreen is 
 * shown, it will be updated on the lcd _lcd else
 * it will be memorized for when it will be shown.
 * If waitRefresh is true (default is true) it will
 * wait LCDREFRESHTIME mSec before updating _lcd.
 *
 * \param timeSec : unsigned int. current CV between 0 to ssrWindow
 */
void UIRimsIdent::setTime(unsigned int timeSec)
{
	_tempScreenShown = false;
	UIRims::setTime(timeSec);
	_tempScreenShown = true;
}