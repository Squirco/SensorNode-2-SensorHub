#include <Arduino.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include "EEPROMex.h"
#include "SparkFunBME280.h"
#include "Wire.h"
#include "SPI.h"
#include "VCNL4040.h"
#include "TimerOne.h"
#include "CmdMessenger.h"
#include "SoftReset.h"
#include "LED.h"

#define APP_FW_VER 35

#define SYS_STATUS_OK						0
#define SYS_STATUS_NO_CLIMATE		1
#define SYS_STATUS_NO_ALS				2
#define SYS_STATUS_NO_SENSORS		3

#define LEDPIN 9

#define SYS_TASK_DEFAULT				0
#define SYS_TASK_PUSH_DATA			1
#define SYS_TASK_NL_CONTROL			2
#define SYS_TASK_SAVE_SETTINGS	3

#define NONE	0
#define OFF		0
#define ON		1

//system settings
uint32_t sysDataPushInterval;
uint8_t sysDataPushMode;
uint8_t sysStatus;
void sysLoadSettings(void);
void sysSaveSettings(void);
void sysLoadDefault(void);
uint8_t sysBootTest(void);

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
//uint8_t sensorPSCanc;

const uint8_t sensorALSTriggerL = 5;
const uint8_t sensorALSTriggerH = 7;
const uint8_t sensorPSTriggerL = 3;
const uint8_t sensorPSTriggerH = 5;

uint16_t temperature;
uint16_t humidity;
uint32_t pressure;
uint16_t lux;
uint16_t ps;

//night light
#define LED_CONTROL_MODE_CMD			0
#define LED_CONTROL_MODE_ALS			1
#define LED_CONTROL_MODE_PS				2
#define LED_CONTROL_MODE_ALS_PS		3
#define LED_CONTROL_MODE_BREATHE	4
#define LED_CONTROL_MODE_OFF			5

bool ledPSTimeoutStart = false;
bool ledPSTimedout = false;
uint16_t ledPSTimeout = 35000;
uint8_t ledFadeTargetMin;
uint8_t ledFadeTargetMax;
uint8_t ledFade;
uint8_t ledControlMode;

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
	kSLedFadeMinMax,			//24
	kQLedFadeMinMax,			//25
	kRLedFadeMinMax,			//26
	kSLedCmdFadeTo,				//27
	kSCalibratePS,				//28
	kSReset,							//29
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
	cmdMessenger.attach(kSLedFadeMinMax, onLedSetFadeLimits);
	cmdMessenger.attach(kQLedFadeMinMax, onLedReturnFadeLimits);
	cmdMessenger.attach(kSLedCmdFadeTo, onLedFadeTo);
	cmdMessenger.attach(kSCalibratePS, onCalibratePS);
	cmdMessenger.attach(kSReset, onSoftReset);
}

void setup()
{
	Serial.begin(38400);
	//delay(100);
	//delay(1000);
	//noInterrupts();
	sysStatus = sysBootTest();

	if (!((sysStatus==SYS_STATUS_NO_ALS) || (sysStatus==SYS_STATUS_NO_SENSORS)))
	{
		alsSensor.alsConf(0x4C);
		alsSensor.psConf(0xFE, 0x08, 0, 0x08);
		//alsSensor.psSetIntThres(sensorPSTriggerH, sensorPSTriggerH);
		alsSensor.psCalibrate();

		lux = alsSensor.lux();
		ps = alsSensor.ps();
	}

if (!((sysStatus==SYS_STATUS_NO_CLIMATE) || (sysStatus==SYS_STATUS_NO_SENSORS)))
{
	climateSensor.settings.commInterface = I2C_MODE;
	climateSensor.settings.I2CAddress = 0x76;
	climateSensor.settings.runMode = 3;
	climateSensor.settings.tStandby = 0;
	climateSensor.settings.filter = 0;
	climateSensor.settings.tempOverSample = 1;
	climateSensor.settings.pressOverSample = 1;
	climateSensor.settings.humidOverSample = 1;
	climateSensor.begin();
}

	//attachInterrupt(digitalPinToInterrupt(2), sensorPsISR, FALLING);
	Timer1.initialize(400);
	Timer1.attachInterrupt(sysTaskTimer);
	Timer1.pwm(LEDPIN, 0);
	//pinMode(LEDPIN, OUTPUT);
	//digitalWrite(LEDPIN, HIGH);
	//interrupts();

	//alsSensor.intFlag();

	//temperature = climateSensor.readTempC();
	//humidity = climateSensor.readHumidity();
	//pressure = climateSensor.readPressure();

	sysLoadSettings();
	//ledControlMode = 4;


	//Serial.begin(115200);
	cmdMessenger.printLfCr();
	attachCommandCallbacks();
	cmdMessenger.sendCmd(kRStatus,sysStatus);
	//Serial.println("Boot");
	//Serial.println("----------");
}

void loop()
{
	cmdMessenger.feedinSerialData();
	ledController();
	sysTaskProcessor();
}

//Tasker Functions
void sysTaskProcessor(void)
{
	switch (sysTaskFlag) {
		case SYS_TASK_DEFAULT:
			sensorDataReady = false;
			if (!sensorDataReady) {
				if (sensorPollALS) {
					lux = alsSensor.lux();
				}
				if (sensorPollPS) {
					//alsSensor.intFlag();
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
		case SYS_TASK_PUSH_DATA:
			if (sensorDataPushed == false)
			{
				if (sensorDataReady) {
					sensorDataPushed = true;
					cmdMessenger.sendCmd(kRTemp, temperature);
					cmdMessenger.sendCmd(kRHumi, humidity);
					cmdMessenger.sendCmd(kRPres, pressure);
					cmdMessenger.sendCmd(kRLux, lux);
				}
			}

			//attachInterrupt(digitalPinToInterrupt(2), sensorPsISR, FALLING);
			sysTaskFlag = SYS_TASK_DEFAULT;
			break;
		case SYS_TASK_NL_CONTROL:
			//alsSensor.intFlag();
			switch (ledControlMode)
			{
        case LED_CONTROL_MODE_PS:
          sensorPollALS = false;
          //sensorPollPS = true;
          ledFadeTarget = ledFadeTargetMax;
          ledPSTimeoutStart = true;
          ledPSTimedout = false;
          break;
				case LED_CONTROL_MODE_ALS_PS:
					if (lux <= sensorALSTriggerL)
					{
						sensorPollALS = false;
						//sensorPollPS = true;
						ledFadeTarget = ledFadeTargetMax;
            ledPSTimeoutStart = true;
            ledPSTimedout = false;
					}
					break;
				default:
					break;
			}
			//attachInterrupt(digitalPinToInterrupt(2), sensorPsISR, FALLING);
			sysTaskFlag = SYS_TASK_DEFAULT;
			break;
		case SYS_TASK_SAVE_SETTINGS:
			//detachInterrupt(digitalPinToInterrupt(2));
			sysSaveSettings();
			//attachInterrupt(digitalPinToInterrupt(2), sensorPsISR, FALLING);
			sysTaskFlag = SYS_TASK_DEFAULT;
		default:
			sysTaskFlag = SYS_TASK_DEFAULT;
			break;
	}
}

void sysTaskTimer(void)
{
  sysTaskCounter++;

	if (sysTaskCounter % 10 == 0) {
		ledFadeFlag = true;
	}

	if ((sysTaskCounter % sysDataPushInterval == 0) && (sysDataPushMode > 0)) {
		//detachInterrupt(digitalPinToInterrupt(2));
		sensorDataPushed = false;
		//sysTaskCounter = 0;
		sysTaskFlag = SYS_TASK_PUSH_DATA;
	}

  if ((sysTaskCounter % ledPSTimeout == 0) && (ledPSTimeoutStart == true)) {
    ledPSTimeoutStart = false;
    ledPSTimedout = true;
  }
}

//system settings
void sysLoadDefault(void)
{
  sysDataPushMode = OFF;
  sysDataPushInterval = 30000;
  ledControlMode = LED_CONTROL_MODE_OFF;
  ledFadeTargetMin = 0;
  ledFadeTargetMax = 100;
}

void sysLoadSettings(void)
{
	uint8_t settingsSaved = EEPROM.read(0);
	if (settingsSaved!=0xFF) {
		sysDataPushMode = EEPROM.read(4);
	  sysDataPushInterval = EEPROM.readInt(6);
		ledControlMode = EEPROM.read(10);
	  ledFadeTargetMin = EEPROM.read(12);
	  ledFadeTargetMax = EEPROM.read(14);
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
	EEPROM.updateByte(0, 1);
	//cmdMessenger.sendCmd(kRStatus, "good");
}

uint8_t sysBootTest(void)
{
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

void sensorPsISR()
{
	detachInterrupt(digitalPinToInterrupt(2));
	//noInterrupts();
	sysTaskFlag = SYS_TASK_NL_CONTROL;
}

void ledPSControl()
{
	if (ps >= sensorPSTriggerH) {
		sensorPollALS = false;
		//sensorPollPS = true;
		ledFadeTarget = ledFadeTargetMax;
		ledPSTimeoutStart = true;
		ledPSTimedout = false;
	}
	else if ((ps <= sensorPSTriggerL) && ledPSTimedout == true)
	//if (ps <= sensorPSTriggerL)
	{
		ledFadeTarget = ledFadeTargetMin;
		//ledPSTimeoutStart = true;
		//ledPSTimedout = false;
		sensorPollALS = true;
	}
	else
	{
		ledPSTimeoutStart = true;
		ledPSTimedout = false;
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
void onReturnPSQuery(

){
	cmdMessenger.sendCmd(kRPS, alsSensor.ps());
}

void onLedSetMode()
{
	uint8_t mode = (uint8_t)cmdMessenger.readInt16Arg();
	if (mode>=LED_CONTROL_MODE_CMD && mode<=LED_CONTROL_MODE_OFF)
	{
		ledControlMode = mode;
	}
	onLedReturnMode();
}

void onLedReturnMode()
{
	cmdMessenger.sendCmd(kRLedMode, ledControlMode);
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
	alsSensor.psCalibrate();
}
void onSoftReset()
{
	soft_restart();
}
