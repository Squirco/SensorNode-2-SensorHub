#include <Arduino.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include "EEPROMex.h"
#include "SparkFunBME280.h"
#include "Wire.h"
#include "SPI.h"
#include "VCNL4040.h"
#include "TimerOne.h"
#include "CmdMessenger.h"
#include "SoftReset.h"
#include "LED.h"

#define APP_FW_VER "1.0.0-rc.1"

#define SYS_STATUS_OK						0x090d
#define SYS_SETTINGS_SAVED			0x055d
#define SYS_STATUS_NO_CLIMATE		0x2bad
#define SYS_STATUS_NO_ALS				0x3bad
#define SYS_PS_CALIBRATED				0x355c
#define SYS_PS_CALIBRATE_FAIL		0x35fc
#define SYS_STATUS_NO_SENSORS		0xabad


#define LEDPIN 9

#define SYS_TASK_DEFAULT				0
#define SYS_TASK_PUSH_DATA			1
#define SYS_TASK_NL_CONTROL			2
#define SYS_TASK_SAVE_SETTINGS	3
#define SYS_TASK_CALIBRATE			4

#define NONE	0
#define OFF		0
#define ON		1

//system settings
uint32_t sysDataPushInterval;
uint8_t sysDataPushMode;
uint16_t sysStatus;
void sysLoadSettings(void);
void sysSaveSettings(void);
void sysLoadDefault(void);
uint16_t sysBootTest(void);

//System Tasker
void sysTaskTimer(void);
void sysTaskProcessor(void);
uint8_t sysTaskFlag;
uint32_t sysTaskCounter;

//sensors
VCNL4040 alsSensor;
BME280 climateSensor;
void sensorPsISR(void);

//sensor Flags and variables
bool sensorDataReady = false;
bool sensorDataPushed = false;
bool sensorPollALS = true;
bool sensorPollPS = true;
bool sensorPsCalibrated = false;

const uint8_t sensorALSTriggerL = 5;
const uint8_t sensorALSTriggerH = 7;
const uint8_t sensorPSTriggerL = 5;
const uint8_t sensorPSTriggerH = 7;

uint16_t temperature;
uint16_t humidity;
uint32_t pressure;
uint16_t lux;
uint16_t ps;
uint16_t psCal;

//night light
#define LED_CONTROL_MODE_RESTORE	0
#define LED_CONTROL_MODE_ALS			1
#define LED_CONTROL_MODE_PS				2
#define LED_CONTROL_MODE_ALS_PS		3
#define LED_CONTROL_MODE_BREATHE	4
#define LED_CONTROL_MODE_CMD			5
#define LED_CONTROL_MODE_OFF			6

bool ledPSTimeoutStart = false;
bool ledPSTimedout = false;
uint16_t ledPSTimeout = 30000;
uint8_t ledFadeTargetMin;
uint8_t ledFadeTargetMax;
uint8_t ledFade;
uint8_t ledControlMode;
uint8_t ledControlModeRestore;

void ledController(void);
void ledPSControl(void);
void ledFadeTo(uint8_t pin, uint8_t brightness);
bool ledFadeFlag = false;
uint8_t ledFadeTarget = 0;

//command messenger
CmdMessenger cmdMessenger = CmdMessenger(Serial);
void onReturnStatus(void);
void onReturnDeviceInfo(void);
void onSaveSettings(void);
void onDataSetPushMode(void);
void onDataReturnPushMode(void);
void onDataSetQueryInt(void);
void onDataReturnQueryInt(void);
void onReturnTempQuery(void);
void onReturnHumiQuery(void);
void onReturnPresQuery(void);
void onReturnLuxQuery(void);
void onReturnPSQuery(void);
void onLedSetMode(void);
void onLedReturnMode(void);
void onLedRestoreMode(void);
void onLedSetFadeLimits(void);
void onLedReturnFadeLimits(void);
void onLedFadeTo(void);
void onCalibratePS(void);
void onSoftReset(void);
//climate sensor
enum
{
	kQStatus,							//0
	kRStatus,							//1
	kQDeviceInfo,					//2
	kRDeviceInfo,					//3
	kSSaveSettings,				//4
	kSDataPushMode,				//5
	kQDataPushMode,				//6
	kRDataPushMode,				//7
	kSDataQueryInt,				//8
	kQDataQueryInt,				//9
	kRDataQueryInt,				//10
	kQTemp,								//11
	kRTemp,								//12
	kQHumi,								//13
	kRHumi,								//14
	kQPres,								//15
	kRPres,								//16
	kQLux,								//17
	kRLux,								//18
	kQPS,									//19
	kRPS,									//20
	kSLedMode,						//21
	kQLedMode,						//22
	kRLedMode,						//23
	kSLedModeRestore,			//24
	kSLedFadeMinMax,			//25
	kQLedFadeMinMax,			//26
	kRLedFadeMinMax,			//27
	kSLedCmdFadeTo,				//28
	kSCalibratePS,				//29
	kRCalibratePS,				//30
	kSReset								//31
};

void attachCommandCallbacks()
{
	cmdMessenger.attach(kQStatus, onReturnStatus);
	cmdMessenger.attach(kQDeviceInfo, onReturnDeviceInfo);
	cmdMessenger.attach(kSSaveSettings, onSaveSettings);
	cmdMessenger.attach(kSDataPushMode, onDataSetPushMode);
	cmdMessenger.attach(kQDataPushMode, onDataReturnPushMode);
	cmdMessenger.attach(kSDataQueryInt, onDataSetQueryInt);
	cmdMessenger.attach(kQDataQueryInt, onDataReturnQueryInt);
	cmdMessenger.attach(kQTemp, onReturnTempQuery);
	cmdMessenger.attach(kQHumi, onReturnHumiQuery);
	cmdMessenger.attach(kQPres, onReturnPresQuery);
	cmdMessenger.attach(kQLux, onReturnLuxQuery);
	cmdMessenger.attach(kQPS, onReturnPSQuery);
	cmdMessenger.attach(kSLedMode, onLedSetMode);
	cmdMessenger.attach(kQLedMode, onLedReturnMode);
	cmdMessenger.attach(kSLedModeRestore, onLedRestoreMode);
	cmdMessenger.attach(kSLedFadeMinMax, onLedSetFadeLimits);
	cmdMessenger.attach(kQLedFadeMinMax, onLedReturnFadeLimits);
	cmdMessenger.attach(kSLedCmdFadeTo, onLedFadeTo);
	cmdMessenger.attach(kSCalibratePS, onCalibratePS);
	cmdMessenger.attach(kSReset, onSoftReset);
}

//Tasker Functions
void sysTaskProcessor(void)
{
	switch (sysTaskFlag) {
		case SYS_TASK_DEFAULT:
		{
			sensorDataReady = false;
			if (!sensorDataReady) {
				if (sensorPollALS) {
					lux = alsSensor.lux();
				}
				if (sensorPollPS) {
					ps = alsSensor.ps();
				}
				temperature = climateSensor.readTempC();
				humidity = climateSensor.readHumidity();
				pressure = climateSensor.readPressure();
			}

			sensorDataReady = true;

			switch (ledControlMode){
				case LED_CONTROL_MODE_ALS:
					if(!sensorPollALS)
					{
						sensorPollALS = true;
					}
					if (lux <= sensorALSTriggerL)
					{
						ledFadeTarget = ledFadeTargetMax;
					}
					else if (lux >= sensorALSTriggerH)
					{
						ledFadeTarget = ledFadeTargetMin;
					}
					break;
				case LED_CONTROL_MODE_PS:
					ledPSControl();
					break;
        case LED_CONTROL_MODE_ALS_PS:
					if (lux <= sensorALSTriggerL)
					{
						ledPSControl();
					}
					break;
				case LED_CONTROL_MODE_OFF:
					ledFadeTarget = 0;
					break;
				default:
					break;
			}
			break;
		}
		case SYS_TASK_PUSH_DATA:
		{
			if (sensorDataPushed == false)
			{
				if (sensorDataReady) {
					sensorDataPushed = true;
					cmdMessenger.sendCmd(kRTemp, temperature);
					cmdMessenger.sendCmd(kRHumi, humidity);
					cmdMessenger.sendCmd(kRPres, pressure);
					cmdMessenger.sendCmd(kRLux, lux);
					if (sensorPsCalibrated) {
						cmdMessenger.sendCmd(kRPS, ps);
					}
				}
			}

			sysTaskFlag = SYS_TASK_DEFAULT;
			break;
		}
		case SYS_TASK_NL_CONTROL:
		{
			switch (ledControlMode)
			{
        case LED_CONTROL_MODE_PS:
          sensorPollALS = false;
          ledFadeTarget = ledFadeTargetMax;
          ledPSTimeoutStart = true;
          ledPSTimedout = false;
          break;
				case LED_CONTROL_MODE_ALS_PS:
					if (lux <= sensorALSTriggerL)
					{
						sensorPollALS = false;
						ledFadeTarget = ledFadeTargetMax;
            ledPSTimeoutStart = true;
            ledPSTimedout = false;
					}
					break;
				default:
					break;
			}
			sysTaskFlag = SYS_TASK_DEFAULT;
			break;
		}
		case SYS_TASK_SAVE_SETTINGS:
		{
			sysSaveSettings();
			cmdMessenger.sendCmd(kRStatus, SYS_SETTINGS_SAVED);
			sysTaskFlag = SYS_TASK_DEFAULT;
			break;
		}
		case SYS_TASK_CALIBRATE:
		{
			sensorPsCalibrated = false;
			uint16_t psCalFactor;
			if(alsSensor.psSetCanc(0))
			{
				delay(500);
				psCalFactor = alsSensor.psCalibrate();
				if (alsSensor.psSetCanc(psCalFactor)) {
					psCal = psCalFactor;
					sensorPsCalibrated = true;
					cmdMessenger.sendCmd(kRCalibratePS, SYS_PS_CALIBRATED);
					sysTaskFlag = SYS_TASK_SAVE_SETTINGS;
				}
				else
				{
					cmdMessenger.sendCmd(kRCalibratePS, SYS_PS_CALIBRATE_FAIL);
					sysTaskFlag = SYS_TASK_DEFAULT;
				}
			}
			else
			{
				cmdMessenger.sendCmd(kRCalibratePS, SYS_PS_CALIBRATE_FAIL);
				sysTaskFlag = SYS_TASK_DEFAULT;
			}
			break;
		}
		default:
		{
			sysTaskFlag = SYS_TASK_DEFAULT;
			break;
		}
	}
}

void sysTaskTimer(void)
{
  sysTaskCounter++;

	if (sysTaskCounter % 10 == 0) {
		ledFadeFlag = true;
		//sleep_disable();
	}

	if ((sysTaskCounter % sysDataPushInterval == 0) && (sysDataPushMode > 0)) {
		sensorDataPushed = false;
		sysTaskFlag = SYS_TASK_PUSH_DATA;
	}

  if ((sysTaskCounter % ledPSTimeout == 0) && (ledPSTimeoutStart == true)) {
    ledPSTimeoutStart = false;
    ledPSTimedout = true;
		//sleep_disable();
  }
}

//system settings
void sysLoadDefault(void)
{
  sysDataPushMode = ON;
  sysDataPushInterval = 20000;
  ledControlMode = LED_CONTROL_MODE_ALS_PS;
  ledFadeTargetMin = 0;
  ledFadeTargetMax = 100;
}

void sysLoadSettings(void)
{
	sysLoadDefault();
	uint8_t settingsSaved = EEPROM.read(0);
	if (settingsSaved==1) {
		sysDataPushMode = EEPROM.read(4);
	  sysDataPushInterval = EEPROM.readInt(6);
		ledControlMode = EEPROM.read(10);
	  ledFadeTargetMin = EEPROM.read(12);
	  ledFadeTargetMax = EEPROM.read(14);
		sensorPsCalibrated = EEPROM.read(16);
		psCal = EEPROM.readInt(18);
	}
	else
	{
		sysLoadDefault();
		sysSaveSettings();
	}
}

void sysSaveSettings(void)
{
	EEPROM.updateByte(4, sysDataPushMode);
	EEPROM.updateInt(6, sysDataPushInterval);
	EEPROM.updateByte(10, ledControlMode);
	EEPROM.updateByte(12, ledFadeTargetMin);
	EEPROM.updateByte(14, ledFadeTargetMax);
	EEPROM.updateByte(16, sensorPsCalibrated);
	EEPROM.updateInt(18, psCal);
	EEPROM.updateByte(0, 1);
}

uint16_t sysBootTest(void)
{
	alsSensor.id();
	climateSensor.id();
	if (!(alsSensor.id()==VCNL4040_ID_L)) {
		if (!(climateSensor.id()==BME280_ID)) {
			return SYS_STATUS_NO_SENSORS;
		}
		return	SYS_STATUS_NO_ALS;
	}

	if (!(climateSensor.id()==BME280_ID)) {
		return SYS_STATUS_NO_CLIMATE;
	}
	return SYS_STATUS_OK;
}

//ALS & Prox Functions
void ledPSControl()
{
	if (sensorPsCalibrated) {
		if (ps >= sensorPSTriggerH) {
			sensorPollALS = false;
			ledFadeTarget = ledFadeTargetMax;
			ledPSTimeoutStart = true;
			ledPSTimedout = false;
		}
		else if ((ps <= sensorPSTriggerL) && ledPSTimedout == true)
		{
			ledFadeTarget = ledFadeTargetMin;
			sensorPollALS = true;
		}
		else
		{
			ledPSTimeoutStart = true;
			ledPSTimedout = false;
		}
	}
}
//LED Control Functions
void ledController(void)
{
	if (ledFadeFlag==true)
	{
		if (ledFade < ledFadeTarget)
		{
			ledFade++;
		}
		else if (ledFade > ledFadeTarget)
		{
			ledFade--;
		}
		ledFadeTo(LEDPIN, ledFade);
		ledFadeFlag = false;
	}

	if (ledControlMode == LED_CONTROL_MODE_BREATHE) {
		if (ledFade==0) {
			ledFadeTarget = 100;
		}
		if (ledFade==100) {
			ledFadeTarget = 0;
		}
	}
}

void ledFadeTo(uint8_t pin, uint8_t brightness)
{
	if (brightness >=ledFadeTargetMin && brightness <=ledFadeTargetMax) {
		Timer1.setPwmDuty(pin, pgm_read_word(&(cie[brightness])));
	}
}

//Command Functions
void onReturnStatus()
{
	cmdMessenger.sendCmd(kRStatus, sysStatus);
}

void onReturnDeviceInfo()
{
	cmdMessenger.sendCmdStart(kRDeviceInfo);
	cmdMessenger.sendCmdArg(APP_FW_VER);
	cmdMessenger.sendCmdArg(EEPROM.readInt(64));
	cmdMessenger.sendCmdEnd();
}

void onSaveSettings()
{
	sysTaskFlag = SYS_TASK_SAVE_SETTINGS;
}

void onDataSetPushMode()
{
	sysDataPushMode = cmdMessenger.readBoolArg();
	onDataReturnPushMode();
}
void onDataReturnPushMode()
{
	cmdMessenger.sendCmd(kRDataPushMode, sysDataPushMode);
}
void onDataSetQueryInt()
{
	sysDataPushInterval = (uint32_t)cmdMessenger.readDoubleArg();
	onDataReturnQueryInt();
}
void onDataReturnQueryInt()
{
	cmdMessenger.sendCmd(kRDataQueryInt, sysDataPushInterval);
}

void onReturnTempQuery()
{
	cmdMessenger.sendCmd(kRTemp, temperature);
}
void onReturnHumiQuery()
{
	cmdMessenger.sendCmd(kRHumi, humidity);
}
void onReturnPresQuery()
{
	cmdMessenger.sendCmd(kRPres, pressure);
}
void onReturnLuxQuery()
{
	lux = alsSensor.lux();
	cmdMessenger.sendCmd(kRLux, lux);
}
void onReturnPSQuery(){
	cmdMessenger.sendCmd(kRPS, alsSensor.ps());
}

void onLedSetMode()
{
	uint8_t mode = (uint8_t)cmdMessenger.readInt16Arg();
	if (mode>=LED_CONTROL_MODE_RESTORE && mode<=LED_CONTROL_MODE_OFF)
	{
		ledControlModeRestore = ledControlMode;
		ledControlMode = mode;
		if (ledControlMode == LED_CONTROL_MODE_ALS_PS) {
			ledFadeTarget = ledFadeTargetMin;
			sensorPollALS = true;
		}
	}
	onLedReturnMode();
}

void onLedReturnMode()
{
	cmdMessenger.sendCmd(kRLedMode, ledControlMode);
}

void onLedRestoreMode()
{
	ledControlMode = ledControlModeRestore;
	onLedReturnMode();
}

void onLedSetFadeLimits()
{
	uint8_t lower = (uint8_t)cmdMessenger.readInt16Arg();
	uint8_t upper = (uint8_t)cmdMessenger.readInt16Arg();

	if (lower>= 0 && lower<= 100 && lower<=upper)
	{
		ledFadeTargetMin = lower;
	}
	if (upper>=0 && upper<=100 && lower<=upper)
	{
		ledFadeTargetMax = upper;
	}
	onLedReturnFadeLimits();
}
void onLedReturnFadeLimits()
{
	cmdMessenger.sendCmdStart(kRLedFadeMinMax);
	cmdMessenger.sendCmdArg(ledFadeTargetMin);
	cmdMessenger.sendCmdArg(ledFadeTargetMax);
	cmdMessenger.sendCmdEnd();
}
void onLedFadeTo()
{
	ledFadeTarget = (uint8_t)cmdMessenger.readInt16Arg();
}
void onCalibratePS()
{
	sysTaskFlag = SYS_TASK_CALIBRATE;
}
void onSoftReset()
{
	soft_restart();
}

void setup()
{
	delay(1000);
	Serial.begin(38400);
	sysStatus = sysBootTest();

	if (!((sysStatus==SYS_STATUS_NO_CLIMATE) || (sysStatus==SYS_STATUS_NO_SENSORS)))
	{
		climateSensor.settings.commInterface = I2C_MODE;
		climateSensor.settings.I2CAddress = 0x76;
		climateSensor.settings.runMode = 3;
		climateSensor.settings.tStandby = 5;
		climateSensor.settings.filter = 0;
		climateSensor.settings.tempOverSample = 1;
		climateSensor.settings.pressOverSample = 1;
		climateSensor.settings.humidOverSample = 1;
		climateSensor.begin();
	}

	if (!((sysStatus==SYS_STATUS_NO_ALS) || (sysStatus==SYS_STATUS_NO_SENSORS)))
	{
		alsSensor.alsConf(0x4C);
		alsSensor.psConf(0x0E, 0x08, 0, 0x07);
		alsSensor.ps();
		alsSensor.lux();
	}

	Timer1.initialize(400);
	Timer1.attachInterrupt(sysTaskTimer);
	Timer1.pwm(LEDPIN, 0);

	sysLoadSettings();
	if (sensorPsCalibrated) {
		alsSensor.psSetCanc(psCal);
		ps=alsSensor.ps();
		lux=alsSensor.lux();
	}
	cmdMessenger.printLfCr();
	attachCommandCallbacks();
	cmdMessenger.sendCmd(kRStatus,sysStatus);
}

void loop()
{
	cmdMessenger.feedinSerialData();
	ledController();
	sysTaskProcessor();
	//set_sleep_mode(SLEEP_MODE_IDLE);
	//sleep_enable();
}
