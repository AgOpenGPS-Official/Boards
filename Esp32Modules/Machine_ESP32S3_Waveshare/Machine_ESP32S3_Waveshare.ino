// AgOpenGPS machine (section / hydraulics) module for the
// Waveshare ESP32-S3-ETH-8DI-8RO(-C) DIN-rail relay board.
//
// Arduino IDE board settings (esp32 core 2.0.x):
//   Board: ESP32S3 Dev Module, Flash Size: 16MB, PSRAM: OPI PSRAM,
//   Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS), USB CDC On Boot: Enabled
// or build with PlatformIO (platformio.ini in this folder).

#include <Wire.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Update.h>
#include "ETHClass.h"
#include "WebPage.h"

# define InoDescription "Machine_ESP32S3_Waveshare :  13-Sep-2026"
const uint16_t InoID = 13096;	// change to send defaults to eeprom, ddmmy, no leading 0
const uint8_t InoType = 5;		// 5 - ESP32-S3 Waveshare machine

#define MaxReadBuffer 100	// bytes
#define EEPROM_SIZE 128

// ---- Waveshare ESP32-S3-ETH-8DI-8RO pinout ----

// relays are driven through a TCA9554PWR I2C expander, EXIO1-8 = relay 1-8, high = energized
#define I2C_SDA_PIN 42
#define I2C_SCL_PIN 41	// bench-confirmed, the Waveshare diagram has SDA/SCL swapped
#define TCA9554_ADDR 0x20
#define TCA9554_REG_OUTPUT 0x01
#define TCA9554_REG_CONFIG 0x03

// W5500 ethernet on SPI
#define ETH_MISO_PIN 14
#define ETH_MOSI_PIN 13
#define ETH_SCLK_PIN 15
#define ETH_CS_PIN 16
#define ETH_INT_PIN 12
#define ETH_RST_PIN -1
#define ETH_SPI_MHZ 20

// optocoupled digital inputs, the GPIO is pulled low when the input is active
const uint8_t InputPins[8] = { 4, 5, 6, 7, 8, 9, 10, 11 };

#define RGB_LED_PIN 38	// WS2812
#define BUZZER_PIN 46

struct ModuleConfig
{
	uint8_t IP0 = 192;
	uint8_t IP1 = 168;
	uint8_t IP2 = 5;
	uint8_t IP3 = 123;
};

ModuleConfig MDL;

//Variables for config - 0 is false
struct Config {
	uint8_t raiseTime = 2;
	uint8_t lowerTime = 4;
	uint8_t enableToolLift = 0;
	uint8_t isRelayActiveHigh = 0; //if zero, active low (default)

	uint8_t user1 = 0; //user defined values set in machine tab
	uint8_t user2 = 0;
	uint8_t user3 = 0;
	uint8_t user4 = 0;
};
Config aogConfig;   //8 bytes

/*
* Functions as below assigned to relays (pin[0..7] = relay 1..8)
0: -
1 thru 16: Section 1 .. Section 16
17,18    Hyd lower, Hyd raise
19,20    Tram right, Tram left
21       Geo Stop
*/
uint8_t pin[24] = { 1,2,3,4,5,6,7,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

//read value from Machine data and set 1 or zero according to list
uint8_t relayState[24];

// network
WiFiUDP UDP_AGIO;
const uint16_t ListeningPortAGIO = 8888;	// to listen on
const uint16_t DestinationPortAGIO = 9999;	// to send to
WebServer server(80);

bool EthChipFound = false;
bool EthLinkUp = false;

// hardware state
bool RelayChipFound = false;
uint8_t relayOutputs = 0;		// bit per relay as last written to the TCA9554
uint32_t LastRelayWrite = 0;
uint8_t inputStates = 0;		// bit per digital input, 1 = active

// relay test from the web page, only allowed while AgOpenGPS is not sending
bool testMode = false;
uint8_t testOutputs = 0;

const uint16_t LoopTime = 50;      //in msec = 20hz
uint32_t LoopLast = LoopTime;

//Comm checks
uint8_t watchdogTimer = 20; //make sure we are talking to AOG
#define WatchdogTimeout 20	// 1 second

bool isRaise = false, isLower = false;

//The variables used for storage
uint8_t relayHi = 0, relayLo = 0, tramline = 0, uTurn = 0, hydLift = 0, geoStop = 0;
float gpsSpeed;
uint8_t raiseTimer = 0, lowerTimer = 0, lastTrigger = 0;

void loop()
{
	if (millis() - LoopLast >= LoopTime)
	{
		LoopLast = millis();

		//If connection lost to AgOpenGPS, the watchdog will count up
		if (watchdogTimer < 250) watchdogTimer++;

		if (watchdogTimer > WatchdogTimeout)
		{
			uint8_t off = aogConfig.isRelayActiveHigh ? 255 : 0;
			relayLo = off;
			relayHi = off;
			tramline = off;
		}

		//hydraulic lift
		if (hydLift != lastTrigger && (hydLift == 1 || hydLift == 2))
		{
			lastTrigger = hydLift;
			lowerTimer = 0;
			raiseTimer = 0;

			//50 msec per frame so 20 per second
			switch (hydLift)
			{
				//lower
			case 1:
				lowerTimer = aogConfig.lowerTime * 20;
				break;

				//raise
			case 2:
				raiseTimer = aogConfig.raiseTime * 20;
				break;
			}
		}

		//countdown if not zero, make sure up only
		if (raiseTimer)
		{
			raiseTimer--;
			lowerTimer = 0;
		}
		if (lowerTimer) lowerTimer--;

		//if anything wrong, shut off hydraulics, reset last
		if ((hydLift != 1 && hydLift != 2) || watchdogTimer > 10)
		{
			lowerTimer = 0;
			raiseTimer = 0;
			lastTrigger = 0;
		}

		if (aogConfig.isRelayActiveHigh)
		{
			isLower = isRaise = false;
			if (lowerTimer) isLower = true;
			if (raiseTimer) isRaise = true;
		}
		else
		{
			isLower = isRaise = true;
			if (lowerTimer) isLower = false;
			if (raiseTimer) isRaise = false;
		}

		ReadInputs();
		CheckRelays();
		UpdateLED();
	}

	ReceiveAGIO();

	server.handleClient();
}
