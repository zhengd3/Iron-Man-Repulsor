//=====================================================
/*!
 * \mainpage Quick Links
 * 
 * - Rims main class
 * - UIRims user interface class
 *
 * Francis Gagnon
 *
 * This Library is licensed under a 
 * <a href="http://www.gnu.org/licenses/licenses.en.html">GPLv3 License</a>.
 *
 * <a href="http://code.google.com/p/rims-arduino-library/">
 * Return to Google Code page</a>
 */
//=====================================================
 
/*!
 * \file Rims.h
 * \brief Rims class declaration
 */

#ifndef Rims_h
#define Rims_h


// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
///\brief uncomment/comment to include/exclude flash memory
//#define WITH_W25QFLASH                                          
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

///\brief Sample time for PID. Same time used
///       for LCD refresh rate and data log rate [mSec]
#define SAMPLETIME 1000
///\brief solid state relay window size [mSec]
#define SSRWINDOWSIZE 5000

///\brief Default set point on UIRims [celcius]
#define DEFAULTSP 68
///\brief Default timer time on UIRims [sec]
#define DEFAULTTIME 5400

#define DEFAULTSTEINHART0 0.001
#define DEFAULTSTEINHART1 0.0002
#define DEFAULTSTEINHART2 -4e-7
#define DEFAULTSTEINHART3 1e-7
///\brief [ohm]
#define DEFAULTRES1 10000

///\brief Max temperature variation from set 
///       point before stopping timer count down [celcius]
#define MAXTEMPVAR 1.0 /// celsius
///\brief Time before tempScreen/timeFlowScreen 
///       is automatically shown[mSec]
#define SCREENSWITCHTIME 10000 /// mSec

/// \brief Default lower bound for accepted flow rate [L/min]
#define DEFAULTFLOWLOWBOUND 3.0
/// \brief Default Upper bound for accepted flow rate [L/min]
#define DEFAULTFLOWUPBOUND 5.0
///\brief If _stopOnCriticalFlow is activited, heater will be turn off
///       if flow is <= than this value.
#define CRITICALFLOW 1.0

///\brief Flash mem address for table of starting addr. of brew sessions
#define ADDRSESSIONTABLE	0x000000 // 1st sector
///\brief Flash mem address for counting data in current brew session
#define ADDRDATACOUNT		0x001000 // 2nd sector
///\brief Flash mem starting address for all brew datas
#define ADDRBREWDATA		0x002000 // 3rd sector
///\brief Total bytes used per data point (at each second)
#define BYTESPERDATA		18
///\brief Memory size in bytes (Winbond W25QW25Q80BV : 1 MByte)
#define MEMSIZEBYTES		1048576

#include "Arduino.h"
#include "utility/UIRims.h"
#include "utility/PID_v1mod.h"


#ifdef WITH_W25QFLASH
	#include "utility/w25qflash.h"
#endif


extern const char g_csvHeader[];

/*!
 * \brief Recirculation infusion mash system (RIMS) library for Arduino
 * \author Francis Gagnon 
 *
 * \bug Only one Rims (or child) instance is allowed with flow sensor
        because of static method _isrFlowSensor()
 */

class Rims
{
	friend class RimsIdent;
	
public:
	Rims(UIRims* uiRims, byte analogPinTherm, byte ssrPin, 
		 double* currentTemp, double* ssrControl, double* settedTemp);

	void setThermistor(float steinhartCoefs[],float res1, float fineTune = 0);
	void setPinLED(byte pinLED);
	void setInterruptFlow(byte interruptFlow, float flowFactor, 
						  float lowBound = DEFAULTFLOWLOWBOUND, 
						  float upBound = DEFAULTFLOWUPBOUND,
					      boolean stopOnCriticalFlow = true);
	void setHeaterPowerDetect(char pinHeaterVolt);
	
	void setTuningPID(double Kp, double Ki, double Kd, double tauFilter,
	                  int mashWaterQty = -1);
#ifdef WITH_W25QFLASH
	void setMemCSPin(byte csPin);
	void checkMemAccessMode();
#endif
	
	void run();
	
	double getTempPV();
	float getFlow();
	boolean getHeaterVoltage();
	
	void stopHeating(boolean state);
	
protected:
	
	virtual void _initialize();
	virtual void _iterate();
	
	void _refreshTimer(boolean verifyTemp = true);
	void _refreshDisplay();
	void _refreshSSR();
#ifdef WITH_W25QFLASH
	unsigned int  _memCountSessions();
	unsigned long _memCountSessionData();
	void          _memInit(float sp);
	void          _memAddBrewData(float time, unsigned int cv,
								  float pv, float flow,
								  float timerRemaining);
	void          _memDumpBrewData();
	void          _memFreeSpace();
	void          _memClearAll();
#endif
	
private:
	
	// ===GENERAL===
	UIRims* _ui;
	PIDmod _myPID;
	byte _analogPinPV;
	byte _pinCV;
	byte _pinLED;
	char _pinHeaterVolt;
#ifdef WITH_W25QFLASH
	W25QFlash _myMem;
#endif
	
	// ===STATE DATAS===
	boolean _rimsInitialized;
	boolean _stopOnCriticalFlow;
	boolean _criticalFlow;
	boolean _ncTherm;
	boolean _noPower;
	boolean _buzzerState;
	boolean _memConnected;
	
	// ===MULTIPLE PIDs===
	byte _pidQty;
	byte _currentPID;
	int _mashWaterValues[4];
	
	// ===THERMISTOR===
	float _steinhartCoefs[4];
	float _res1;
	float _fineTuneTemp;
	
	// ===PID I/O===
	double* _setPointPtr;
	double* _processValPtr;
	double* _controlValPtr; /// [0,SSRWINDOWSIZE]
	
	// ===PID PARAMS===
	double _kps[4];
	double _kis[4];
	double _kds[4];
	double _tauFilter[4];
	
	// ===TIMER===
	unsigned long _currentTime;             ///mSec
	unsigned long _settedTime;				///mSec
	unsigned long _rimsStartTime;           ///mSec
	unsigned long _windowStartTime;			///mSec
	unsigned long _runningTime;				///mSec
	unsigned long _totalStoppedTime;		///mSec
	unsigned long _timerStopTime;			///mSec
	unsigned long _timerStartTime;			///mSec
	unsigned long _lastScreenSwitchTime;    ///mSec
	unsigned long _lastTimePID;             ///mSec
	boolean _sumStoppedTime;
	boolean _timerElapsed;
	
	// ===FLOW SENSOR===
	float _flowFactor; /// freq[Hz] = flowFactor * flow[L/min]
	float _flow;
	
#ifdef WITH_W25QFLASH
	// ===FLASH MEM===
	unsigned long _memNextAddr;
	unsigned long _memDataQty;
#endif
	
};
#endif
