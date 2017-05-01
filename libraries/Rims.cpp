/*!
 * \file Rims.cpp
 * \brief Rims class definition
 */
 
#include "Arduino.h"
#include "math.h"
#include "utility/PID_v1mod.h"
#include "Rims.h"

/*
============================================================
Global Variable
============================================================
*/

///\brief Last time interrupt was called [µSec]
volatile unsigned long g_flowLastTime = 0;
///\brief Current time interrupt is called [µSec]
volatile unsigned long g_flowCurTime = 0;
///\brief ISR for flow sensor.
void isrFlow(); /// ISR for flow sensor
///\brief Header for csv printing on serial monitor
const char g_csvHeader[] = "time,sp,cv,pv,flow,timerRemaining";

/*
============================================================
RIMS class definition
============================================================
*/
/*!
 * \brief Constructor
 * \param uiRims : UIRims*. Pointer to UIRims instance
 * \param analogPinTherm : byte. Analog pin to thermistor
 * \param ssrPin : byte. Pin to control heater's solid state relay.
 * \param currentTemp : double*. Pointer to a double that 
 *                      will be used for current temparature
 * \param ssrControl : double*. Pointer to a double that 
 *                     will use to control SSR
 * \param settedTemp : double*. Pointer to a double that 
 *                     will be use to store setted temperature
 */
Rims::Rims(UIRims* uiRims, byte analogPinTherm, byte ssrPin, 
	       double* currentTemp, double* ssrControl, double* settedTemp)
: _ui(uiRims), _analogPinPV(analogPinTherm), _pinCV(ssrPin),
  _processValPtr(currentTemp), _controlValPtr(ssrControl), _setPointPtr(settedTemp),
  _myPID(currentTemp, ssrControl, settedTemp, 0, 0, 0, DIRECT),
  _pidQty(0), 
  _stopOnCriticalFlow(false), _rimsInitialized(false),
  _memConnected(false),
  _pinLED(13),_pinHeaterVolt(-1), _noPower(false)
{
	_steinhartCoefs[0] = DEFAULTSTEINHART0;
	_steinhartCoefs[1] = DEFAULTSTEINHART1;
	_steinhartCoefs[2] = DEFAULTSTEINHART2; 
	_steinhartCoefs[3] = DEFAULTSTEINHART3;
	_res1 = DEFAULTRES1;
	_fineTuneTemp = 0;
	for(int i=0;i<=3;i++)
	{
		_kps[i] = 0; _kis[i] = 0; _kds[i] = 0; _tauFilter[i] = 0; 
		_mashWaterValues[i] = -1;
	}
	_myPID.SetSampleTime(SAMPLETIME);
	_myPID.SetOutputLimits(0,SSRWINDOWSIZE);
	_settedTime = (unsigned long)DEFAULTTIME*1000;
	*(_setPointPtr) = DEFAULTSP;
	_currentPID = 0;
	pinMode(ssrPin,OUTPUT);
	pinMode(13,OUTPUT);
}

/*!
 * \brief Set thermistor parameters.
 *
 * @image html thermistor_circuit.png "Thermistor voltage divider circuit" 
 * \param steinhartCoefs : float[4]. Steinhart-hart equation coefficients in order
 *        of increasing power, i.e :
 *    	  \f[
 *	      \frac{1}{T[kelvin]}=C_{0}+C_{1}\ln(R)+C_{2}\ln(R)^2+C_{3}\ln(R)^3
 *   	  \f]
 *        for more information : http://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
 *
 * \param res1 : float. In ohm.
 * \param fineTuneTemp : float. optional (default=0). if you want to add a fine tune factor
          after the steinhart-hart temperature calculation.
 */
void Rims::setThermistor(float steinhartCoefs[], float res1, float fineTuneTemp)
{
	for(int i=0;i<=4;i++) _steinhartCoefs[i] = steinhartCoefs[i];
	_res1 = res1;
	_fineTuneTemp = fineTuneTemp;
}

/*!
 * \brief Set tuning for PID object.
 *
 * Algorithm is in parallel form i.e. :
 * \f[ 
 * G_{c}(s) = K_{p}+\frac{K_{i}}{s}+\frac{K_{d}s}{Ts+1}
 * \f]
 * Where \f$T=tauFilter[sec]\f$. It can be set to the same value as a PID output filter.
 * 
 * In practice, a good value of derivative filter is :
 * \f[ 
 * T = \frac{K_{d}}{10}
 * \f]
 * 
 * For more information : 
 * http://playground.arduino.cc/Code/PIDLibrary
 * 
 * For anti-windup, integration static saturation was replaced by
 * <b>integration clamping</b>. Basically, it stop integration at a smarter
 * time than "if limit is reached at the output of the integrator". More precisely, it
 * stop integration <b>if limit is reached at the ouput of the PID</b> <i>AND</i>
 * <b>if the temperature error and and the PID output have the same
 * sign.</b>
 * 
 * @image html pid.png "Full PID block diagram"
 *
 * @image html clamping.png "Integration clamping diagram"  
 *
 * \param Kp : float. Propotionnal gain
 * \param Ki : float. Integral gain.
 * \param Kd : float. Derivative gain.
 * \param tauFilter : float. Derivative filter time constant in sec.
 * \param mashWaterQty : int (default=-1). If multiple regulators is needed,
 *                       set this parameter to mash water volume in liter.
 *                       This mash water volume will be associated to this PID.
 *                       Total of 4 regulators is allowed (with different
 *                       mash water volume).
 */
void Rims::setTuningPID(double Kp, double Ki, double Kd, double tauFilter,
                        int mashWaterQty)
{
	if(mashWaterQty != -1) _currentPID = _pidQty;
	else _currentPID = 0;
	_kps[_currentPID] = Kp; _kis[_currentPID] = Ki; _kds[_currentPID] = Kd;
    _tauFilter[_currentPID] = tauFilter;
	_mashWaterValues[_currentPID] = mashWaterQty;
	_currentPID = 0;
	_pidQty++;
}

/*!
 * \brief Set interrupt function for flow sensor.
 * 
 * It is recommended to call pinMode(pinUsedForInterrupt,INPUT_PULLUP) 
 * in the setup() function of the main sketch. For exemple, on 
 * an Arduino UNO, if interrupt #1 is used for the flow sensor,
 * you should call in setup() : pinMode(3,INPUT_PULLUP);
 * 
 * \param interruptFlow : byte. Interrupt pin number connected to the
 *                        flow sensor. For more info :
 *                        http://arduino.cc/en/Reference/attachInterrupt
 * \param flowFactor : float. Factor used to calculate flow from the
 *                     input frequency, i.e. :
 *   				   \f[
 *  				   freq[Hz] = flowFactor * flow[L/min]
 *  				   \f]
 * \param lowBound : float. Lower bound used for accepted flow rate.
 * \param upBound : float. Upper bound used for accepted flow rate
 * \param stopOnCriticalFlow : boolean (default=true). If true and flow is
 *                             <= CRITICALFLOW on getFlow() call, 
 *                             heater is turn off. Else
 *                             flow mesurment doesn't influence heater action.   
 *                             
 */
void Rims::setInterruptFlow(byte interruptFlow, float flowFactor,
							float lowBound,float upBound,
							boolean stopOnCriticalFlow)
{
	attachInterrupt(interruptFlow,isrFlow,RISING);
	_flowFactor = flowFactor;
	_ui->setFlowBounds(lowBound,upBound);
	_stopOnCriticalFlow = stopOnCriticalFlow;
}

/*!
 * \brief Set pin for heater LED indicator
 * \param pinLED : byte
 */
void Rims::setPinLED(byte pinLED)
{
	pinMode(pinLED,OUTPUT);
	_pinLED = pinLED;
}

/*!
 * \brief Set pin to detect if there is voltage applied on heater.
 * 
 * If no voltage is detect on the heater, a speaker alarm is trigerred.
 * 
 * \warning DO NOT APPLY 120V OR 240V DIRECTLY ON ARDUINO PINS.
 * 
 * It can be easily and cheaply made with a +5V DC power supply in
 * parallel with the SSR and heater. It is recommended to
 * add a 10Kohm or so resistor in series with the power supply.
 * 
 * @image html power_circuit.png "Voltage detection circuit"  
 * 
 * I use it to detect if my breaker is trigerred or if I shut it off
 * manually with an external switch. If the PID is informed that the heater
 * is shut off, it will run smoother when the heater will be re-powered.
 * This feature is really not mandatory.
 * 
 * \param pinHeaterVolt : byte. 5V power supply pin.
 */
void Rims::setHeaterPowerDetect(char pinHeaterVolt)
{
	pinMode(pinHeaterVolt,INPUT);
	_pinHeaterVolt = pinHeaterVolt;
}

#ifdef WITH_W25QFLASH
	/*!
	 * \brief Set pin for flash memory chip select.
	 * 
	 * Code was built on Windbond W25Q80BV SPI flash memory.
	 * MOSI, MISO and CLK are fixed pins on all Arduino. 
	 * For ex, it's 11, 12, 13 for MOSI, MISO and CLK respectivly
	 * on Arduino UNO. For more information : 
	 * http://arduino.cc/en/Reference/SPI.
	 * 
	 * \warning WINBOND W25Q80BV WORKS IN 3.3V AND MOST ARDUINO
	 * WORKS IN 5V. IF IT'S YOUR CASE, DO NOT DIRECTLY CONNECT 
	 * ARDUINO MOSI, MISO, CLK AND CS ON THE MEMORY PINS.
	 * 
	 * If you have to convert 5 V signal to 3.3V signal,
	 * here's a simple and cheap circuit that works well.
	 * 
	 * @image html mem_circuit.png "Voltage convert flash memory"
	 * 
	 * \param csPin : byte. pin used for flash memory chip select.
	 */
	void Rims::setMemCSPin(byte csPin)
	{ 
		_myMem.setCSPin(csPin); 
		_memConnected = _myMem.verifyMem();
	}
	
	/*!
	 * \brief Check if enterring in USB memory access mode.
	 * 
	 * Must be called at the end of the setup() function of
	 * the main sketch. If KEYSELECT is pressed, entering
	 * in USB memory access mode. In this mode, you can
	 * dump brew session data on the serial port and erase
	 * the entire memory. A basic menu interface is 
	 * implemanted via serial communication.
	 * 
	 * The menu is :
	 * 
	 * -# dump brew session data
	 * -# calculate free space
	 * -# clear all memory
	 * -# exit
	 * 
	 */ 
	void Rims::checkMemAccessMode()
	{
		byte selectedMenu = 0;
		// if KEYSELECT is pressed, entering in memory dump mode
		if(_ui->readKeysADC(false) == KEYSELECT)
		{
			if(_memConnected)
			{
				_ui->showMemAccessScreen();
				do
				{
					Serial.println("MEMORY ACCESS MODE");
					Serial.println("<1> dump brew session data");
					Serial.println("<2> calculate free space");
					Serial.println("<3> clear all memory");
					Serial.println("<4> exit");
					while(not Serial.available());
					selectedMenu = Serial.parseInt();
					Serial.read(); // flush remaining '\n'
					Serial.write('>');Serial.println(selectedMenu);
					switch(selectedMenu)
					{
					case 1:
						_memDumpBrewData();
						break;
					case 2:
						_memFreeSpace();
						break;
					case 3:
						_memClearAll();
						break;
					case 4:
						Serial.println("EXIT");
						break;
					}
					// flush remaining '\n' :
					Serial.flush(); Serial.read(); 
				}
				while(selectedMenu != 4);
			}
			else Serial.println("MEM NOT FOUND!");
		}
	}
	
	/*!
	 * \brief Dump brew session data on USB serial port.
	 */
	void Rims::_memDumpBrewData()
	{
		byte readBuffer[BYTESPERDATA];
		unsigned int brewSession, brewSessionQty;
		unsigned long startingAddr, nextStartingAddr, curAddr;
		unsigned long sessionDataQty;
		float time, sp, pv, flow, timerRemaining;
		unsigned int cv;
		Serial.println("DUMP");
		Serial.print("Currently ");
		brewSessionQty = _memCountSessions();
		Serial.print(brewSessionQty);
		Serial.println(" brew sessions. Which one, starting at 1 ?");
		while(not Serial.available());
		brewSession = Serial.parseInt();
		Serial.write('>');Serial.println(brewSession);
		if(brewSession >= 1 and brewSession <= brewSessionQty)
		{
			Serial.println(g_csvHeader);
			_myMem.read(ADDRSESSIONTABLE + 4*(brewSession-1),
						readBuffer,4);
			memcpy(&startingAddr,readBuffer,4);
			if(brewSession == brewSessionQty) // last session
			{
				sessionDataQty = _memCountSessionData();
				nextStartingAddr = startingAddr+ 4 + \
				                   BYTESPERDATA*(sessionDataQty);
			}
			else
			{
				_myMem.read(ADDRSESSIONTABLE + 4*(brewSession),
							readBuffer,4);
				memcpy(&nextStartingAddr,readBuffer,4);
			}
			_myMem.read(startingAddr,readBuffer,4);
			memcpy(&sp,readBuffer,4); // set point at the beginning
			for(curAddr = startingAddr + 4;
				curAddr < nextStartingAddr;
				curAddr += BYTESPERDATA)
			{
				_myMem.read(curAddr,readBuffer,BYTESPERDATA);
				memcpy(&time,readBuffer,4);
				memcpy(&cv,readBuffer+4,2);
				memcpy(&pv,readBuffer+6,4);
				memcpy(&flow,readBuffer+10,4);
				memcpy(&timerRemaining,readBuffer+14,4);
				Serial.print(time,3);	Serial.write(',');
				Serial.print(sp,1);		Serial.write(',');
				Serial.print(cv);		Serial.write(',');
				Serial.print(pv,3);		Serial.write(',');
				Serial.print(flow,2);	Serial.write(',');
				Serial.println(timerRemaining,0);
			}
		}		
	}
	
	/*!
	 * \brief Show free memory on flash mem via USB serial port.
	 */
	void Rims::_memFreeSpace()
	{
		byte readBuffer[4];
		unsigned int brewSesQty = _memCountSessions();
		unsigned long freeBytes, lastSessionAddr, freePoints;
		Serial.println("FREE MEM");
		if(brewSesQty)
		{
			_myMem.read(ADDRSESSIONTABLE+4*(brewSesQty-1),
						readBuffer,4);
			memcpy(&lastSessionAddr,readBuffer,4);
		}
		else lastSessionAddr = ADDRBREWDATA - 4;
		freeBytes = MEMSIZEBYTES - \
		   (lastSessionAddr + 4 + BYTESPERDATA*_memCountSessionData());
		freePoints = freeBytes / BYTESPERDATA;
		Serial.print("Currently ");
		Serial.print(freeBytes); Serial.print(" free bytes or about ");
		Serial.print(freePoints); Serial.println(" data points");
	}
	
	/*!
	 * \brief Clear all memory via USB serial port.
	 */
	void Rims::_memClearAll()
	{
		char inputChar;
		Serial.println("CLEAR");
		Serial.println("Are you sure ? (y/n)");
		while(not Serial.available());
		inputChar = Serial.read();
		Serial.write('>');Serial.println(inputChar);
		if(inputChar == 'Y' or inputChar == 'y')
		{
			Serial.println("Clearing all memory...");
			_myMem.erase(0x000000,W25Q_ERASE_CHIP);
			_myMem.waitFree();
			Serial.println("Finished!");
		}
	}
	
	/*!
	 * \brief Count how many brew sessions were saved in flash mem.
	 * 
	 * Max is 1024 brew sessions. When new brew session 
	 * is started, the starting address of the datablock is saved in 
	 * the brew sessions table, starting at ADDRSESSIONTABLE or 0x000000.
	 * For exemple, if 2 brew session were done of 2 seconds each
	 * (so 4+18+18=40 bytes each), the memory map of the brew 
	 * sessions table would be :
	 * 
	 * Address  | Data       | Size 
	 * -------- | -----------| -------
	 * 0x000000 | 0x00001100 | 4 bytes
	 * 0x000004 | 0x00001128 | 4 bytes      
	 * 0x000008 | 0xFFFFFFFF | 4 bytes
	 * 0x00000C | 0xFFFFFFFF | 4 bytes
	 * ...      | ...        | ...
	 * 0x000FFF | 0xFFFFFFFF | 4 bytes
	 * 
	 */
	unsigned int Rims::_memCountSessions()
	{
		bool nullAddrFound = false;
		byte page, offset, readBuffer[256];
		for(page=0;page<16;page++)  // 16 pages per sector
		{
			_myMem.read(ADDRSESSIONTABLE+(256*page),readBuffer,256);
			offset = 0;
			do
			{
				if(readBuffer[offset] == 0xFF) 
				{
					nullAddrFound = true;
					break;
				}
				offset++;
			}
			while(offset > 0);
			if(nullAddrFound) break;
		}
		return (page*64)+(offset/4);
	}
	
	/*!
	 * \brief Count how many data point were taken.
	 * 
	 * When brew data is added in the memory, a sector is filled with "0"
	 * to remember how many data were taken. This sector is at the address
   * ADDRDATACOUNT or 0x001000 (second sector).
	 * If 12 brew data were taken since de beginning (or 12 seconds were
	 * ellapsed), the memory map would be like this :
	 * 
	 * Address  | Data           
	 * -------- | -----------
	 * 0x001000 | b00000000
	 * 0x001001 | b11110000         
	 * 0x001002 | b11111111
	 * 0x001003 | b11111111
	 * ...      | ...
	 * 0x001FFF | b11111111
	 * 
	 * I used this method because of a limitation of the flash memory.
	 * I could not use normal counting because freshly erased bits 
	 * can be cleared (set to "0") but cannot be set to "1" without 
	 * full sector erase.
	 * 
	 */
	unsigned long Rims::_memCountSessionData()
	{
		boolean freeBitFound = false;
		byte i,page,offset,readBuffer[256];
		for(page=0;page<16;page++) // 16 pages per sector
		{
			_myMem.read(ADDRDATACOUNT+(page*256),readBuffer,256);
			do
			{
				if(readBuffer[offset] & 0xFF)
				{
					freeBitFound = true;
					break;
				}
				offset++;
			}
			while(offset>0);
			if(freeBitFound) break;
		}
		for(i=0;i<8;i++) if((readBuffer[offset]>>i) & 0x01) break;
		return 8*((page*256)+offset)+i;
	}
	
	/*!
	 * \brief Initialize flash memory
	 * 
	 * Verify where to store the new datas and saved temperature
	 * setpoint at the beginning of the datablock.
	 * 
	 * \param sp : float. Temperature setpoint stored at the beginning
	 *                    of the datablock.
	 * 
	 */
	void Rims::_memInit(float sp)
	{
		unsigned int brewSesQty = _memCountSessions();
		unsigned long lastSesDataQty = _memCountSessionData();
		byte buffer[4];
		unsigned long lastStartingAddr;
		if(brewSesQty == 0) _memNextAddr = ADDRBREWDATA;
		else
		{
			_myMem.read(ADDRSESSIONTABLE+(4*(brewSesQty-1)),buffer,4);
			memcpy(&lastStartingAddr,buffer,4);
			_memNextAddr = lastStartingAddr+\
			               (BYTESPERDATA*lastSesDataQty) + 4;
		}
		memcpy(buffer,&_memNextAddr,4);
		_myMem.program(ADDRSESSIONTABLE+((brewSesQty*4)%1024),buffer,4);
		_myMem.erase(ADDRDATACOUNT,W25Q_ERASE_SECTOR);
		_memDataQty = 0;
		memcpy(buffer,&sp,4);
		_myMem.program(_memNextAddr,buffer,4);
		_memNextAddr += 4;
	}
	
	/*!
	 * \brief Add data point to the flash memory.
	 * 
	 * Five values is added at _memNextAddr, 18 bytes in total.
	 * Temperature setpoint (float : 4 bytes) is saved only
	 * once, at the beginning of the brew sessions in _memInit().
	 * For exemple, for the first data (starting at ADDRBREWDATA
	 * or 0x001100) of the first brew session, the memory map would be :
	 * 
	 * Address  | Data           | Size
	 * -------- | -------------- | -----
	 * 0x001100 | sp             | 4 bytes 
	 * 0x001104 | time           | 4 bytes
	 * 0x091108 | cv             | 2 bytes
	 * 0x09110A | pv             | 4 bytes
	 * 0x09110E | flow           | 4 bytes
	 * 0x091102 | timerRemaining | 4 bytes
	 * 
	 * \param time : float. time in sec of data point
	 * \param cv : unsigned int. SSR control value (mSec at ON state)
	 * \param pv : float. temperature in deg Celcius
	 * \param flow : float. flow in L/min
	 * \param timerRemaining : float. remaining time on timer
	 *                         in seconds.
	 */
	void Rims::_memAddBrewData(float time, unsigned int cv,
							   float pv, float flow,
							   float timerRemaining)
	{
		byte writeBuffer[BYTESPERDATA], dataCountMkr;
		memcpy(writeBuffer,&time,4);
		memcpy(writeBuffer+4,&cv,2);
		memcpy(writeBuffer+6,&pv,4);
		memcpy(writeBuffer+10,&flow,4);
		memcpy(writeBuffer+14,&timerRemaining,4);
		_myMem.program(_memNextAddr,writeBuffer,BYTESPERDATA);
		dataCountMkr = 0xFF << ((_memDataQty % 8)+1);
		_myMem.program(ADDRDATACOUNT+_memDataQty/8,&dataCountMkr,1);
		_memDataQty ++;
		_memNextAddr += BYTESPERDATA;
	}
#endif

/*!
 * \brief Start and run Rims instance. 
 *
 * Should be called in the loop() function of your sketchbook
 * First time : _initialize() is called
 * Remaining time : _iterate() is called
 */
void Rims::run()
{
	if(not _rimsInitialized) _initialize();
	else _iterate();
}

/*!
 * \brief Initialize a Rims instance before starting temperature regulation.
 *
 * Initialization procedure :
 * -# Ask Temperature set point
 * -# Ask Timer time
 * -# Ask Mash water qty (if setted)
 * -# Show pump switching warning
 * -# Show heater switching warning
 */
void Rims::_initialize()
{
	_timerElapsed = false;
	// === ASK SETPOINT ===
	*(_setPointPtr) = _ui->askSetPoint(*(_setPointPtr));
	// === ASK TIMER ===
	_settedTime = (unsigned long)_ui->askTime(_settedTime/1000)*1000;
	// === ASK MASH WATER ===
	if(_pidQty != 1) _currentPID = _ui->askMashWater(_mashWaterValues,
													 _currentPID);
	// === PUMP SWITCHING WARN ===
	_ui->showPumpWarning(this->getFlow());
	_currentTime = millis();
	unsigned long lastFlowRefresh = _currentTime - SAMPLETIME;
	while(_ui->readKeysADC()==KEYNONE)
	{
		_currentTime = millis();
		if(_currentTime - lastFlowRefresh >= SAMPLETIME)
		{
			_ui->setFlow(this->getFlow(),false);
			lastFlowRefresh = _currentTime;
		}
	}
	// === HEATER SWITCHING WARN ===
	_ui->showHeaterWarning(this->getHeaterVoltage());
	while(_ui->readKeysADC()==KEYNONE)
	{
		if(_pinHeaterVolt != -1)
		{
			_ui->setHeaterVoltState(this->getHeaterVoltage(),false);
		}
	}
#ifdef WITH_W25QFLASH
	// === MEM INIT ===
	if(_memConnected) _memInit(*_setPointPtr);
#endif
	Serial.println(g_csvHeader);
	_ui->showTempScreen();
	*(_processValPtr) = this->getTempPV();
	_ui->setTempSP(*(_setPointPtr));
	_ui->setTempPV(*(_processValPtr));
	_sumStoppedTime = true;
	_runningTime = _totalStoppedTime = _timerStopTime = 0;
	_buzzerState = false;
	stopHeating(true);
	_myPID.SetTunings(_kps[_currentPID],_kis[_currentPID],_kds[_currentPID]);
	_myPID.SetDerivativeFilter(_tauFilter[_currentPID]);
	*(_controlValPtr) = 0;
	stopHeating(false);
	_rimsInitialized = true;
	_currentTime = _windowStartTime = _timerStartTime = _rimsStartTime \
				 = _lastScreenSwitchTime = millis();
	_lastTimePID = _currentTime - SAMPLETIME;
}

/*!
 * \brief Main method called for temperature regulation at each iteration
 * 
 * At each PID calculation (at each SAMPLETIME sec), datas is
 * sent over Serial communication for logging purpose.
 *
 */
void Rims::_iterate()
{
	_currentTime = millis();
	if(_currentTime-_lastTimePID>=SAMPLETIME)
	{
		// === READ TEMPERATURE/FLOW ===
		*(_processValPtr) = getTempPV();
		_flow = this->getFlow();
		// === CRITCAL STATES ===
		stopHeating((_stopOnCriticalFlow and _criticalFlow) \
		            or _ncTherm or _noPower);
		// === REFRESH PID ===
		_myPID.Compute();
		// === REFRESH DISPLAY ===
		_refreshDisplay();
		// === DATA LOG ===
#ifdef WITH_W25QFLASH
		if(_memConnected)
		{
			_memAddBrewData((_currentTime-_rimsStartTime)/1000.0,
							*(_controlValPtr),
							*(_processValPtr),
							_flow,
							(_settedTime-_runningTime)/1000.0);
		}
#endif
		Serial.print(
	    (double)(_currentTime-_rimsStartTime)/1000.0,3);	Serial.write(',');
		Serial.print(*(_setPointPtr),1);					Serial.write(',');
		Serial.print(*(_controlValPtr),0);					Serial.write(',');
		Serial.print(*(_processValPtr),3);					Serial.write(',');
		Serial.print(_flow,2);								Serial.write(',');
		Serial.println((_settedTime-_runningTime)/1000.0,0);
		_lastTimePID += SAMPLETIME;
	}
	// === SSR CONTROL ===
	_refreshSSR();
	// === TIME REMAINING ===
	_refreshTimer();
	// === KEY CHECK ===
	int keyPressed = _ui->readKeysADC();
	if((keyPressed!=KEYNONE and _currentTime-_lastScreenSwitchTime>=500)\
	    or _currentTime-_lastScreenSwitchTime >= SCREENSWITCHTIME)
	{
		_ui->switchScreen();
		_ui->timerRunningChar(not(_sumStoppedTime or _timerElapsed));
		_lastScreenSwitchTime = _currentTime;
	}
	if(keyPressed == KEYSELECT and _timerElapsed)
	{
		stopHeating(true);
		_ui->lcdLight(true);
		_ui->ring(false);
		_rimsInitialized = false;
	}
}

/*!
 * \brief Refresh timer value.
 *
 * If error on temperature >= MAXTEMPVAR, timer will not count down.
 * \param verifyTemp : boolean. If true, error on current temperature 
 *					   should not be greater than MAXTEMPVAR to count down.
 *                     Else, current temperature is ignored.
 */
void Rims::_refreshTimer(boolean verifyTemp)
{
	_currentTime = millis();
	if(not _timerElapsed)
	{
		if(abs(*(_setPointPtr)-*(_processValPtr)) <= MAXTEMPVAR or not verifyTemp)
		{
			if(_sumStoppedTime)
			{
				_sumStoppedTime = false;
				_totalStoppedTime += (_timerStartTime - 
											_timerStopTime);
			}
			_runningTime = _currentTime - _totalStoppedTime;
			_timerStopTime = _currentTime;
		}
		else
		{
			_timerStartTime = _currentTime;
			if(not _sumStoppedTime) _sumStoppedTime = true;
		}
		if(_runningTime >= _settedTime) 
		{
			_timerElapsed = true;
			_runningTime = _settedTime;
		}
	}
}

/*!
 * \brief Refresh display used by UIRims instance
 */
void Rims::_refreshDisplay()
{
	_ui->setTempPV(*(_processValPtr));
	if(_timerElapsed)
	{
		_buzzerState = not _buzzerState;
		_ui->ring(_buzzerState);
		_ui->lcdLight(_buzzerState);
	}
	_ui->setTime((_settedTime-_runningTime)/1000);
	_ui->timerRunningChar((not _sumStoppedTime) and (not _timerElapsed));
	_ui->setFlow(_flow);
	_ui->setHeaterVoltState(!_noPower);
		
}

/*!
 * \brief Refresh solid state relay
 * SSR will be refreshed in function of _controlValPtr value. 
 */
void Rims::_refreshSSR()
{
	_currentTime = millis();
	if(_currentTime - _windowStartTime > SSRWINDOWSIZE)
	{
		_windowStartTime += SSRWINDOWSIZE;
	}
	if(_currentTime - _windowStartTime <= *(_controlValPtr))
	{
		digitalWrite(_pinCV,HIGH);
		digitalWrite(_pinLED,HIGH);
	}
	else
	{
		digitalWrite(_pinCV,LOW);
		digitalWrite(_pinLED,LOW);
	}
	_noPower = !this->getHeaterVoltage();
}

/*!
 * \brief Get temperature from thermistor
 *
 * Steinhart-hart equation will be applied here. If voltage is maximal
 * (i.e. ~=5V) it means that the thermistor is not connected and 
 * regulation and heating is stopped until reconnection.
 */
double Rims::getTempPV()
{
	double tempPV = NCTHERM;
	_ncTherm = true;
	int curTempADC = analogRead(_analogPinPV);
	if(curTempADC < 1021)  // connected thermistor
	{
		_ncTherm = false;
		double vin = ((double)curTempADC)/1024.0;
		double resTherm = (_res1*vin)/(1.0-vin);
		double logResTherm = log(resTherm);
		double invKelvin = _steinhartCoefs[0]+\
						_steinhartCoefs[1]*logResTherm+\
						_steinhartCoefs[2]*pow(logResTherm,2)+\
						_steinhartCoefs[3]*pow(logResTherm,3);
		tempPV = (1/invKelvin)-273.15+_fineTuneTemp;
	}
	return tempPV;
}

/*!
 * \brief Get flow from hall-effect flow sensor.
 */
float Rims::getFlow()
{
	float flow;
	if(g_flowCurTime == 0) flow = 0.0;
	else if(micros() - g_flowCurTime >= 5e06) flow = 0.0;
	else flow = (1e06 / (_flowFactor* (g_flowCurTime - g_flowLastTime)));
	_criticalFlow = (flow <= CRITICALFLOW);
	return constrain(flow,0,99.99);
}

/*!
 * \brief Check if heater is powered or not.
 */
boolean Rims::getHeaterVoltage()
{
	boolean res = true;
	if(_pinHeaterVolt != -1) res = digitalRead(_pinHeaterVolt);
	return res;
}

/*!
 * \brief Stop heater no matter what PID output
 * \param state : boolean. If true, heater is shut off. Else, heater is 
 *                         turned on.
 */
void Rims::stopHeating(boolean state)
{
	if(state == true)
	{
		_myPID.SetMode(MANUAL);
		*(_controlValPtr) = 0;
		_refreshSSR();
	}
	else _myPID.SetMode(AUTOMATIC);
}

/*
============================================================
ISR Definition
============================================================
*/
/*!
 * \brief ISR for flow sensor.
 *
 * ISR is used as a software capture mode. Time values are store in 
 * g_flowLastTime and g_flowCurTime.
 * 
 */
void isrFlow()
{
	g_flowLastTime = g_flowCurTime;
	g_flowCurTime = micros();
}
